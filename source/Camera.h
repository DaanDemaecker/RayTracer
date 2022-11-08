#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{ _fovAngle * TO_RADIANS}
		{
			fov = tanf(fovAngle * 0.5f);
			cameraToWorld = CalculateCameraToWorld();
		}


		Vector3 origin{};
		float fovAngle{90.f};
		float fov{};

		float cameraSpeed{ 10.f };
		float angularSpeed{ 0.25f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};


		Matrix CalculateCameraToWorld() const
		{
			Matrix worldMatrix;
			Vector3 right, up;
			right = Vector3::Cross(Vector3::UnitY, forward);
			up = Vector3::Cross(forward, right);

			worldMatrix[0] = { right, 0 };
			worldMatrix[1] = { up, 0 };
			worldMatrix[2] = { forward, 0 };
			worldMatrix[3] = { origin, 1 };
			
			return worldMatrix;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			origin += (pKeyboardState[SDL_SCANCODE_W] || pKeyboardState[SDL_SCANCODE_Z] || pKeyboardState[SDL_SCANCODE_UP]) * (forward * cameraSpeed * deltaTime);
			origin -= (pKeyboardState[SDL_SCANCODE_S] || pKeyboardState[SDL_SCANCODE_DOWN])* (forward * cameraSpeed * deltaTime);
			origin -= (pKeyboardState[SDL_SCANCODE_Q] || pKeyboardState[SDL_SCANCODE_A] || pKeyboardState[SDL_SCANCODE_LEFT])*(right * cameraSpeed * deltaTime);
			origin += (pKeyboardState[SDL_SCANCODE_D] || pKeyboardState[SDL_SCANCODE_RIGHT])*(right * cameraSpeed * deltaTime);

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			switch (mouseState)
			{
			case SDL_BUTTON_LEFT: //right click
				origin += forward *  float(mouseY) * deltaTime;
				totalYaw += float(mouseX) * deltaTime * angularSpeed;
				break;
			case SDL_BUTTON_X1: //left click
				totalPitch -= float(mouseY) * deltaTime * angularSpeed;
				totalYaw += float(mouseX) * deltaTime * angularSpeed;
				break;
			case SDL_BUTTON_X2: //both click
				origin.y += float(mouseY) * deltaTime;
			default:
				break;
			}


			Matrix rotation{};
			rotation = Matrix::CreateRotation(totalPitch, totalYaw, 0);
			forward = rotation.TransformVector(Vector3::UnitZ);
			up = rotation.TransformVector(Vector3::UnitY);
			right = rotation.TransformVector(Vector3::UnitX);

			cameraToWorld = CalculateCameraToWorld();
		}
	};
}
