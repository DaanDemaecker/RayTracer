#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

#define useBVH

namespace dae
{
#pragma region BVH
	struct BVHNode
	{
		Vector3 aabbMin{};
		Vector3 aabbMax{};
		unsigned int leftChild{};
		unsigned int firstIndice{};
		unsigned int indicesCount{};
		bool IsLeaf() const { return indicesCount > 0; };
	};
#pragma endregion bvh

#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal):
			v0{_v0}, v1{_v1}, v2{_v2}, normal{_normal.Normalized()}{}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		BVHNode* pBVHnode{};
		unsigned int rootBvhNodeIdx{};
		unsigned int bvhNodesUsed{};

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.clear();

			normals.resize(indices.size() / 3);

			for (int i{}; i < normals.size(); i++)
			{
				Vector3 a = positions[indices[3 * i + 1]] - positions[indices[3 * i]];
				Vector3 b = positions[indices[3 * i + 2]] - positions[indices[3 * i]];
				normals[i] = Vector3::Cross(a, b);
				normals[i].Normalize();
			}
		}

		void UpdateTransforms()
		{
			Matrix pointMatrix{ scaleTransform * rotationTransform * translationTransform };
			Matrix normalMatrix{ rotationTransform * translationTransform };

			transformedPositions.clear();
			transformedNormals.clear();

			transformedPositions.resize(positions.size());
			transformedNormals.resize(normals.size());
			
			for (int i{}; i < positions.size(); i++)
			{
				transformedPositions[i] = pointMatrix.TransformPoint(positions[i]);
			}

			for (int i{}; i < normals.size(); i++)
			{
				transformedNormals[i] = normalMatrix.TransformVector(normals[i]);
			}
#ifdef useBVH
			UpdateBVH();
#else
			UpdateTransformedAABB(pointMatrix);
#endif // useBVH
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];
				for (auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			//AABB update:be careful -> transform the 8 vertices of the aabb
			//an calculate new min and max
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			//(xmax, ymin, zmin)
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymin, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymin, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymax, zmin)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmax, ymax, zmin)
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmax, ymax, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymax, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}

		void UpdateBVH()
		{
			//maximum amount of nodes is needed is (amount of triangles * 2 - 1)
			//if (bvhNodes.size() == 0)bvhNodes.resize(normals.size() * 2 - 1);

			bvhNodesUsed = 0;

			BVHNode& root = pBVHnode[rootBvhNodeIdx];
			root.leftChild = 0;
			root.firstIndice = 0;
			root.indicesCount = static_cast<unsigned int>(indices.size());

			UpdateBVHNodeBounds(rootBvhNodeIdx);

			Subdivide(rootBvhNodeIdx);
		}

		inline void UpdateBVHNodeBounds(int nodeIdx)
		{
			BVHNode& node = pBVHnode[nodeIdx];
			node.aabbMin = Vector3{FLT_MAX, FLT_MAX, FLT_MAX };
			node.aabbMax = Vector3{ FLT_MIN, FLT_MIN, FLT_MIN };

			for (size_t i{ node.firstIndice }; i < node.firstIndice + node.indicesCount; ++i)
			{
				Vector3& curVertex{ transformedPositions[indices[i]] };
				node.aabbMin = Vector3::Min(node.aabbMin, curVertex);
				node.aabbMax = Vector3::Max(node.aabbMax, curVertex);
			}
		}

		inline void Subdivide(int nodeIdx)
		{
			// terminate recursion
			BVHNode& node = pBVHnode[nodeIdx];
			int maxTrianglesPerNode{ 2 };
			if (node.indicesCount <= maxTrianglesPerNode*3) return;

			Vector3 extent = node.aabbMax - node.aabbMin;
			int axis = 0;
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis]) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;


			int i = node.firstIndice;
			int j = i + node.indicesCount - 1;
			while (i <= j)
			{
				Vector3 centroid{ (transformedPositions[indices[i]] + transformedPositions[indices[i+2]] + transformedPositions[indices[i+2]]) };

				if (centroid[axis] < splitPos)
				{
					i += 3;
				}
				else
				{
					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i + 2], indices[j]);
					std::swap(normals[i / 3], normals[(j - 2) / 3]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) / 3]);

					j -= 3;
				}
			}
			//stop split if one of the sides is empty
			int leftCount = i - node.firstIndice;
			if (leftCount == 0 || leftCount == node.indicesCount) return;
			// create child nodes
			int leftChildIdx = ++bvhNodesUsed;
			int rightChildIdx = ++bvhNodesUsed;

			node.leftChild = leftChildIdx;

			pBVHnode[leftChildIdx].firstIndice = node.firstIndice;
			pBVHnode[leftChildIdx].indicesCount = leftCount;

			pBVHnode[rightChildIdx].firstIndice = i;
			pBVHnode[rightChildIdx].indicesCount = node.indicesCount - leftCount;

			node.indicesCount = 0;

			UpdateBVHNodeBounds(leftChildIdx);
			UpdateBVHNodeBounds(rightChildIdx);
			// recurse
			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};

		const float min{ 0.0001f };
		const float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}