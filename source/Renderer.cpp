//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"
#include <ppl.h>

#include <future>
#include <thread>
#include <iostream>
using namespace dae;

#define ASYNC

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);

	m_Ar = float(m_Width) / float(m_Height);
}

void Renderer::Render(Scene* pScene) const
{

	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const uint32_t numPixels = static_cast<uint32_t>(m_Width * m_Height);

#if defined(ASYNC)
	//async logic
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};

	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixels = numPixels % numCores;
	uint32_t currPixelIndex{ 0 };

	for (uint32_t coreId{}; coreId < numCores; ++coreId)
	{
		uint32_t taskSize{ numPixelsPerTask };
		if (numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		async_futures.push_back(
			std::async(std::launch::async, [=, this]
				{
					const uint32_t pixelIndexEnd = currPixelIndex + taskSize;
					for (uint32_t pixelIndex{ currPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPixel(pScene, pixelIndex, camera.fov, m_Ar, camera, lights, materials);
					}
				})
		);
		currPixelIndex += taskSize;
	}

	//Wait for all tasks
	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}


#elif defined(PARALLEL_FOR)
	//Parrallel-For logic
	concurrency::parallel_for(0u, numPixels,
		[=, this](int i)
		{
			RenderPixel(pScene, i, camera.fov, m_Ar, camera, lights, materials);
		});

#else
	//Synchronus Logic (no threading)
	for (uint32_t i{}; i < numPixels; i++)
	{
		RenderPixel(pScene, i, camera.fov, m_Ar, camera, lights, materials);
	}
#endif
	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);

}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	m_LightingMode = static_cast<LightingMode>((int(m_LightingMode) + 1) % 4);

	switch (m_LightingMode)
	{
	case dae::Renderer::LightingMode::ObservedArea:
		std::cout << "ObservedArea \n";
		break;
	case dae::Renderer::LightingMode::Radiance:
		std::cout << "Radiance \n";
		break;
	case dae::Renderer::LightingMode::BRDF:
		std::cout << "BRDF \n";
		break;
	case dae::Renderer::LightingMode::Combined:
		std::cout << "Combined \n";
		break;
	default:
		break;
	}
}


void dae::Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights,
	const std::vector<Material*>& materials) const
{
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

	float rx = px + 0.5f;
	float ry = py + 0.5f;

	float cx = (2 * (rx / float(m_Width)) - 1) * aspectRatio * fov;
	float cy = (1 - (2 * (ry / float(m_Height)))) * fov;

	Vector3 rayDirection{ cx, cy, 1.f };
	//transform ray direction to the rotation of the camera
	rayDirection = camera.cameraToWorld.TransformVector(rayDirection);
	rayDirection.Normalize();

	//initialize viewray with the position of the camera and the raydirection
	Ray viewRay{ camera.origin, rayDirection };

	//initialize the color variable
	ColorRGB finalColor{};

	//initialize and get closest hit frome the scene
	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);


	if (closestHit.didHit)
	{
		finalColor = ColorRGB{ 0, 0, 0 };

		//check if the ray reaches a light
		for (auto& light : lights)
		{
			//get the direction from the closesthit to the light
			Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, closestHit.origin + 0.0001f * closestHit.normal) };
			float length{ lightDirection.Normalize() };

			//get ray from closestHit to light
			if (m_ShadowsEnabled)
			{
				//if ray hits an object, make it darker and go to the next light in the loop
				Ray ray{ closestHit.origin + 0.0001f * closestHit.normal, lightDirection , 0.001f, length };

				if (pScene->DoesHit(ray))continue;
			}

			//calculate observedarea
			float observedArea{ Vector3::Dot(closestHit.normal, lightDirection) };

			switch (m_LightingMode)
			{
			case dae::Renderer::LightingMode::ObservedArea:
				//if observed area is smaller than 0, don't apply it
				if (observedArea < 0)continue;
				finalColor += ColorRGB{ 1.f, 1.f, 1.f } *observedArea;
				break;
			case dae::Renderer::LightingMode::Radiance:
				finalColor += LightUtils::GetRadiance(light, closestHit.origin);
				break;
			case dae::Renderer::LightingMode::BRDF:
				finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection);
				break;
			case dae::Renderer::LightingMode::Combined:
				if (observedArea < 0) continue;
				finalColor += observedArea *
					LightUtils::GetRadiance(light, closestHit.origin) *
					materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection);
				break;
			default:
				break;
			}
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}
