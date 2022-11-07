#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 raySphereVector{ ray.origin - sphere.origin };

			float a{}, b{}, c{}, t{}, discriminant{};

			a = Vector3::Dot(ray.direction, ray.direction);
			b = Vector3::Dot(2 * ray.direction, raySphereVector);
			c = Vector3::Dot(raySphereVector, raySphereVector) - Square(sphere.radius);

			discriminant = { Square(b) - 4 * a * c };
			if (discriminant >= 0)
			{
				float discriminantSquareRoot{sqrtf(discriminant)};

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
					if(Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0)return false;

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

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TraingleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			float tx1 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::min(ty1, ty2);
			tmax = std::max(ty1, ty2);

			float tz1 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::min(tz1, tz2);
			tmax = std::max(tz1, tz2);

			return tmax > 0 && tmax >= tmin;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//slabtest
			if (!SlabTest_TraingleMesh(mesh, ray))
			{
				return false;
			}

			HitRecord closestHit{};
			HitRecord tempRecord{};

			Triangle triangle{};
			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

			for (int i{}; i < mesh.normals.size(); i++)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[3 * i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[3 * i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[3 * i + 2]];
				triangle.normal = mesh.transformedNormals[i];
				
				if (HitTest_Triangle(triangle, ray, tempRecord, ignoreHitRecord))
				{
					if (ignoreHitRecord)
					{
						return true;
					}

					if (tempRecord.t < closestHit.t && tempRecord.t > ray.min)
					{
						closestHit = tempRecord;
					}
				}

				if (!ignoreHitRecord)hitRecord = closestHit;
			}
			return closestHit.didHit;
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

	namespace Utils
	{
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

				if(isnan(normal.x))
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
}