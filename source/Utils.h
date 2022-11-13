#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <xmmintrin.h>

#define useMollerTrumbore 
//#define useGeometricSphereHit

namespace dae
{
	namespace Utils
	{
		inline float FastSqrt(float arg)
		{
			//source https://geometrian.com/programming/tutorials/fastsqrt/index.php
			return _mm_cvtss_f32(
				_mm_sqrt_ss(_mm_set_ps1(arg))
			);
		}

		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof())
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if (isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}

	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifdef useGeometricSphereHit

			const Vector3 raySphereVector{ ray.origin - sphere.origin };

			float a{}, b{}, c{}, t{}, discriminant{};

			a = Vector3::Dot(ray.direction, ray.direction);
			b = Vector3::Dot(2 * ray.direction, raySphereVector);
			c = Vector3::Dot(raySphereVector, raySphereVector) - Square(sphere.radius);

			discriminant = { Square(b) - 4 * a * c };
			if (discriminant >= 0)
			{
				float discriminantSquareRoot{ sqrtf(discriminant) };

				t = (-b - discriminantSquareRoot) / (2 * a);

				if (t < ray.min)
				{
					t = (-b + discriminantSquareRoot) / (2 * a);
				}

				if (t > ray.min && t < ray.max)
				{
					if (!ignoreHitRecord)
					{
						hitRecord.t = t;
						hitRecord.didHit = true;
						hitRecord.materialIndex = sphere.materialIndex;
						hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
						hitRecord.normal = hitRecord.origin - sphere.origin;
					}
					return true;
				}
			}
			return false;
#else
			//create a vector from the origin of the ray to the origin of the sphere
			const Vector3 tC{ sphere.origin - ray.origin };

			//create an imaginary right triangle with point p on the ray
			//calculate the distance from ray origin to imaginary point p
			const float dp{ Vector3::Dot(tC, ray.direction) };

			//calculate the squared distance from the origin of the sphere to point p
			const float tCPsqr{ Square(tC.Magnitude()) - Square(dp) };

			//if p is outside the sphere, return
			if (tCPsqr > Square(sphere.radius))
				return false;

			//create another imaginary triangle c, p, i
			//with c=sphereOrigin, p = p and i = intersectpoint
			//calculate the distance between p and i
			const float piDistance{ sqrt(Square(sphere.radius) - tCPsqr) };

			//calculate the distance from ray origin to i
			const float t = dp - piDistance;


			if (t < ray.min || t > ray.max)
				return false;

			if (!ignoreHitRecord)
			{
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
				hitRecord.normal = hitRecord.origin - sphere.origin;
			}
			return true;

#endif // useGeometricSphereHit
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion

#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float t{};

			if (Vector3::Dot(ray.direction, plane.normal) != 0)
			{
				t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal);

				if (t >= ray.min && t <= ray.max)
				{
					if (!ignoreHitRecord)
					{
						hitRecord.t = t;
						hitRecord.didHit = true;
						hitRecord.materialIndex = plane.materialIndex;
						hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
						hitRecord.normal = plane.normal;
					}
					return true;
				}
			}
			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float dot{ Vector3::Dot(triangle.normal, ray.direction) };
			TriangleCullMode cullMode{ triangle.cullMode };

			if (ignoreHitRecord)
			{
				if (cullMode == TriangleCullMode::BackFaceCulling)cullMode = TriangleCullMode::FrontFaceCulling;
				else if (cullMode == TriangleCullMode::FrontFaceCulling)cullMode = TriangleCullMode::BackFaceCulling;
			}

			if (dot == 0)return false;
			else if (cullMode == TriangleCullMode::BackFaceCulling && dot > 0)return false;
			else if (cullMode == TriangleCullMode::FrontFaceCulling && dot < 0)return false;

			float t{};


			if (Vector3::Dot(ray.direction, triangle.normal) != 0)
			{

				Vector3 vCenter{ (triangle.v0 + triangle.v1 + triangle.v2) / 3 };
				t = Vector3::Dot(vCenter - ray.origin, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal);
				if (t >= ray.min && t <= ray.max)
				{
					Vector3 p{ ray.origin + t * ray.direction };
					Vector3 pointToSide{};
					Vector3 edge{};

					pointToSide = p - triangle.v0;
					edge = triangle.v1 - triangle.v0;
					if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0)return false;

					pointToSide = p - triangle.v1;
					edge = triangle.v2 - triangle.v1;
					if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0)return false;

					pointToSide = p - triangle.v2;
					edge = triangle.v0 - triangle.v2;
					if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0)return false;

					if (!ignoreHitRecord)
					{
						hitRecord.t = t;
						hitRecord.didHit = true;
						hitRecord.materialIndex = triangle.materialIndex;
						hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
						hitRecord.normal = triangle.normal;
					}
					return true;
				}

			}
			return false;
		}

		inline bool HitTest_MollerTrumbore(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			TriangleCullMode cullMode{ triangle.cullMode };

			if (ignoreHitRecord)
			{
				if (cullMode == TriangleCullMode::BackFaceCulling)cullMode = TriangleCullMode::FrontFaceCulling;
				else if (cullMode == TriangleCullMode::FrontFaceCulling)cullMode = TriangleCullMode::BackFaceCulling;
			}

			float dot{ Vector3::Dot(triangle.normal, ray.direction) };
			if (dot == 0)return false;
			else if (cullMode == TriangleCullMode::BackFaceCulling && dot > 0)return false;
			else if (cullMode == TriangleCullMode::FrontFaceCulling && dot < 0)return false;

			Vector3 edge1{ triangle.v1 - triangle.v0 };
			Vector3 edge2{ triangle.v2 - triangle.v0 };

			Vector3 h{}, s{}, q{};
			float a{}, f{}, u{}, v{};

			h = Vector3::Cross(ray.direction, edge2);
			a = Vector3::Dot(edge1, h);
			if (a > -FLT_EPSILON && a < FLT_EPSILON)
			{
				//if a = 0, the ray is parallel to the triangle and doesn't hit
				return false;
			}

			f = 1.0 / a;
			s = ray.origin - triangle.v0;
			u = f * Vector3::Dot(s, h);
			if (u < 0 || u > 1)
			{
				return false;
			}

			q = Vector3::Cross(s, edge1);
			v = f * Vector3::Dot(ray.direction, q);
			if (v < 0 || u + v > 1)
			{
				return false;
			}

			float t{ f * Vector3::Dot(edge2, q) };
			if (t > ray.min && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.materialIndex = triangle.materialIndex;
					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
					hitRecord.normal = triangle.normal;
				}
				return true;
			}

			return false;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest(const Ray& ray, const Vector3& AABBMin, const Vector3& AABBMax)
		{
			float tx1 = (AABBMin.x - ray.origin.x) * ray.inverseDirection.x;
			float tx2 = (AABBMax.x - ray.origin.x) * ray.inverseDirection.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (AABBMin.y - ray.origin.y) * ray.inverseDirection.y;
			float ty2 = (AABBMax.y - ray.origin.y) * ray.inverseDirection.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (AABBMin.z - ray.origin.z) * ray.inverseDirection.z;
			float tz2 = (AABBMax.z - ray.origin.z) * ray.inverseDirection.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}

		inline void IntersectBVH(const TriangleMesh& mesh, Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool& hasHit, bool ignoreHitRecord, const size_t nodeIdx)
		{
			//get current node
			const BVHNode& node{ mesh.pBVHNodes[nodeIdx] };

			//do broad slabtest before continuing
			if (!SlabTest(ray, node.aabbMin, node.aabbMax)) return;

			//if the current node is not the end, search the child nodes
			if (!node.IsLeaf())
			{
				IntersectBVH(mesh, triangle, ray, hitRecord, hasHit, ignoreHitRecord, node.leftChild);
				IntersectBVH(mesh, triangle, ray, hitRecord, hasHit, ignoreHitRecord, node.leftChild + 1);
				return;
			}
			HitRecord tempRecord{};
			//for every triangle in the node
			for (size_t idx{}; idx < node.indicesCount; idx += 3)
			{
				// Set the position and normal of the current triangle to the triangle object
				triangle.v0 = mesh.transformedPositions[mesh.indices[node.firstIndice + idx]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[node.firstIndice + idx + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[node.firstIndice + idx + 2]];
				triangle.normal = mesh.transformedNormals[(node.firstIndice + idx) / 3];

				bool hit{};
#ifdef useMollerTrumbore 
				hit = HitTest_MollerTrumbore(triangle, ray, tempRecord, ignoreHitRecord);
#else
				hit = HitTest_Triangle(triangle, ray, tempRecord, ignoreHitRecord);
#endif // useMollerTrumbore 
				if (!hit)continue;

				hasHit = true;
				if (ignoreHitRecord)
					return;

				if (tempRecord.t < hitRecord.t)
				{
					hitRecord = tempRecord;
				}

			}
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifndef useBVH
			//slabtest
			if (!SlabTest_TraingleMesh(ray, mesh.minAABB, mesh.maxAABB))
			{
				return false;
			}
#endif // !useBVH
			HitRecord closestHit{};
			HitRecord tempRecord{};

			Triangle triangle{};
			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

			bool hasHit{ false };

#ifdef useBVH
			IntersectBVH(mesh, triangle, ray, hitRecord, hasHit, ignoreHitRecord, 0);
#else


			for (int i{}; i < mesh.normals.size(); i++)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[3 * i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[3 * i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[3 * i + 2]];
				triangle.normal = mesh.transformedNormals[i];

				bool hit{ false };

#ifdef useMollerTrumbore 
				hit = HitTest_MollerTrumbore(triangle, ray, tempRecord, ignoreHitRecord);
#else
				hit = HitTest_Triangle(triangle, ray, tempRecord, ignoreHitRecord);
#endif // useMollerTrumbore 

				
				if (hit)
				{
					hasHit = true;

					if (tempRecord.t < closestHit.t && tempRecord.t > ray.min)
					{
						closestHit = tempRecord;
					}
				}

				if (!ignoreHitRecord)hitRecord = closestHit;
			}
#endif // useBVH
			return hasHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return light.origin - origin;
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			switch (light.type)
			{
			case dae::LightType::Point:
				return light.color* light.intensity / Square((light.origin - target).Magnitude());
				break;
			case dae::LightType::Directional:
				return light.color* light.intensity;
				break;
			}
			return ColorRGB{};
		}
	}
}