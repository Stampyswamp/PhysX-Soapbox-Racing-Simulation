#pragma once

#include "BasicActors.h"
#include <vector>

namespace PhysicsEngine
{
    // mesh builder
    class MeshBuilder
    {
    public:
        std::vector<PxVec3> verts;
        std::vector<PxU32>  tris;

        // add vertex
        inline PxU32 AddVertex(const PxVec3& v)
        {
            verts.push_back(v);
            return (PxU32)(verts.size() - 1);
        }

        // add triangle
        inline void AddTriangle(PxU32 a, PxU32 b, PxU32 c)
        {
            tris.push_back(a);
            tris.push_back(b);
            tris.push_back(c);
        }

        // add strip
        inline void AddStrip(int rowA, int rowB, int vertsPerRow)
        {
            for (int c = 0; c < vertsPerRow - 1; c++)
            {
                PxU32 a0 = rowA + c;
                PxU32 a1 = rowA + c + 1;
                PxU32 b0 = rowB + c;
                PxU32 b1 = rowB + c + 1;

                AddTriangle(a0, b0, a1);
                AddTriangle(a1, b0, b1);
            }
        }

        // finalize mesh
        inline TriangleMesh* Finalize(const PxTransform& pose)
        {
            return new TriangleMesh(verts, tris, pose);
        }
    };
}
