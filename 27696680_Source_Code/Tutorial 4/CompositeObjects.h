#pragma once

#include "BasicActors.h"
#include "MeshBuilder.h"
#include "Noise.h"
#include "TrackConfig.h"
#include "Collisions.h"
#include <vector>
#include <functional>

namespace PhysicsEngine
{
    // static box
    class StaticBox : public StaticActor
    {
    public:
        StaticBox(const PxTransform& pose, PxVec3 dimensions, const MaterialSet& mats)
            : StaticActor(pose)
        {
            CreateShape(PxBoxGeometry(dimensions));

            PxShape* s = GetShape(0);
            ApplyMaterialAndGroup(
                s,
                mats.worldMat,
                CG_WORLD,
                CG_CAR | CG_TRACK
            );
        }
    };

    // composite base
    class CompositeObject
    {
    protected:
        std::vector<Actor*> parts;

    public:
        virtual ~CompositeObject() {}

        void AddToScene(Scene* scene)
        {
            for (auto* p : parts)
                scene->Add(p);
        }

        void Color(const PxVec3& col)
        {
            for (auto* p : parts)
                p->Color(col);
        }
    };

    // ramp
    class CompositeRamp : public CompositeObject
    {
    public:
        CompositeRamp(const PxVec3& pos, const RampConfig& cfg, float headingDeg, const MaterialSet& mats)
        {
            MeshBuilder mb;

            float halfW = cfg.width * 0.5f;
            float dz = cfg.length / cfg.segments;
            float stepX = cfg.width / (cfg.interiorCount + 1);

            for (int i = 0; i <= cfg.segments; i++)
            {
                float z = i * dz;
                float t = (float)i / (float)cfg.segments;
                float baseY = cfg.height * t;

                bool boundaryRow = (i == 0 || i == cfg.segments);

                mb.AddVertex(PxVec3(-halfW, baseY, z));

                for (int c = 1; c <= cfg.interiorCount; c++)
                {
                    float x = -halfW + c * stepX;
                    float noise = (!boundaryRow ? RampNoise(c, i, cfg.noiseAmp) : 0.0f);
                    mb.AddVertex(PxVec3(x, baseY + noise, z));
                }

                mb.AddVertex(PxVec3(halfW, baseY, z));
            }

            int vertsPerRow = cfg.interiorCount + 2;
            for (int i = 0; i < cfg.segments; i++)
            {
                int rowA = i * vertsPerRow;
                int rowB = (i + 1) * vertsPerRow;
                mb.AddStrip(rowA, rowB, vertsPerRow);
            }

            float r = headingDeg * PxPi / 180.0f;
            PxQuat q(r, PxVec3(0, 1, 0));

            TriangleMesh* rampMesh = mb.Finalize(PxTransform(pos, q));
            parts.push_back(rampMesh);

            PxShape* shape = rampMesh->GetShape(0);
            ApplyMaterialAndGroup(
                shape,
                mats.trackMat,
                CG_TRACK,
                CG_CAR | CG_WORLD
            );
        }
    };

    // turn
    class CompositeTurn : public CompositeObject
    {
    public:
        std::function<PxVec3(float)> SampleCenter;
        std::function<PxVec3(float)> SampleTangent;

        static PxVec3 RotateYFunc(const PxVec3& v, float deg)
        {
            float r = deg * PxPi / 180.0f;
            float c = cosf(r);
            float s = sinf(r);

            return PxVec3(
                v.x * c + v.z * s,
                v.y,
                -v.x * s + v.z * c
            );
        }

        CompositeTurn(const PxVec3& pos, const TurnConfig& cfg, bool leftTurn, float headingDeg, const MaterialSet& mats)
        {
            MeshBuilder mb;

            auto LocalCenter = [=](float t)
                {
                    float theta = t * PxPi * 0.5f;

                    float x = cfg.radius * sinf(theta);
                    float z = -cfg.radius * (1 - cosf(theta));

                    if (!leftTurn)
                        x = -x;

                    float y = t * cfg.height;

                    return PxVec3(x, y, z);
                };

            auto LocalTangent = [=](float t)
                {
                    float theta = t * PxPi * 0.5f;
                    float dtheta = PxPi * 0.5f;

                    float dx = cfg.radius * cosf(theta) * dtheta;
                    float dz = -cfg.radius * sinf(theta) * dtheta;

                    if (!leftTurn)
                        dx = -dx;

                    float dy = cfg.height;

                    return PxVec3(dx, dy, dz).getNormalized();
                };

            SampleCenter = [=](float t)
                {
                    PxVec3 local = LocalCenter(t);

                    if (leftTurn)
                        local = PxVec3(-local.z, local.y, local.x);
                    else
                        local = PxVec3(local.z, local.y, -local.x);

                    return RotateYFunc(local, headingDeg) + pos;
                };

            SampleTangent = [=](float t)
                {
                    PxVec3 local = LocalTangent(t);

                    if (leftTurn)
                        local = PxVec3(-local.z, local.y, local.x);
                    else
                        local = PxVec3(local.z, local.y, -local.x);

                    return RotateYFunc(local, headingDeg).getNormalized();
                };

            std::vector<PxU32> leftEdge, rightEdge;

            for (int i = 0; i <= cfg.samples; i++)
            {
                float t = (float)i / cfg.samples;

                PxVec3 C = LocalCenter(t);
                PxVec3 T = LocalTangent(t);

                if (leftTurn)
                {
                    C = PxVec3(-C.z, C.y, C.x);
                    T = PxVec3(-T.z, T.y, T.x);
                }
                else
                {
                    C = PxVec3(C.z, C.y, -C.x);
                    T = PxVec3(T.z, T.y, -T.x);
                }

                PxVec3 F(T.x, 0, T.z);
                F.normalize();

                PxVec3 N = PxVec3(0, 1, 0).cross(F).getNormalized();

                PxVec3 L = C - N * (cfg.width * 0.5f);
                PxVec3 R = C + N * (cfg.width * 0.5f);

                leftEdge.push_back(mb.AddVertex(L));
                rightEdge.push_back(mb.AddVertex(R));

                if (i > 0)
                {
                    int i0 = i - 1;
                    int i1 = i;

                    mb.AddTriangle(leftEdge[i0], leftEdge[i1], rightEdge[i0]);
                    mb.AddTriangle(leftEdge[i1], rightEdge[i1], rightEdge[i0]);
                }
            }

            float r = headingDeg * PxPi / 180.0f;
            PxQuat q(r, PxVec3(0, 1, 0));

            TriangleMesh* mesh = mb.Finalize(PxTransform(pos, q));
            parts.push_back(mesh);

            PxShape* shape = mesh->GetShape(0);
            ApplyMaterialAndGroup(
                shape,
                mats.trackMat,
                CG_TRACK,
                CG_CAR | CG_WORLD
            );
        }
    };

    // rails
    class RailMesh : public CompositeObject
    {
    public:
        RailMesh(
            std::function<PxVec3(float)> SampleCenter,
            std::function<PxVec3(float)> SampleTangent,
            float railOffset,
            float railHeight,
            int samples,
            const PxVec3& color,
            const MaterialSet& mats,
            float thickness = 0.3f)
        {
            MeshBuilder mb;

            bool flip = (railOffset < 0);

            auto Tri = [&](PxU32 a, PxU32 b, PxU32 c)
                {
                    if (flip) mb.AddTriangle(a, c, b);
                    else      mb.AddTriangle(a, b, c);
                };

            std::vector<PxU32> botOuter, botInner;
            std::vector<PxU32> topOuter, topInner;

            for (int i = 0; i <= samples; i++)
            {
                float t = (float)i / samples;

                PxVec3 C = SampleCenter(t);
                PxVec3 T = SampleTangent(t).getNormalized();

                PxVec3 F(T.x, 0, T.z);
                if (F.magnitudeSquared() < 1e-6f)
                    F = PxVec3(0, 0, 1);
                F.normalize();

                PxVec3 N = PxVec3(0, 1, 0).cross(F).getNormalized();

                PxVec3 baseOuter = C + N * railOffset;

                float inwardSign = (railOffset < 0 ? +1.0f : -1.0f);
                PxVec3 baseInner = baseOuter + N * (thickness * inwardSign);

                botOuter.push_back(mb.AddVertex(baseOuter));
                botInner.push_back(mb.AddVertex(baseInner));

                topOuter.push_back(mb.AddVertex(baseOuter + PxVec3(0, railHeight, 0)));
                topInner.push_back(mb.AddVertex(baseInner + PxVec3(0, railHeight, 0)));

                if (i > 0)
                {
                    int i0 = i - 1;
                    int i1 = i;

                    Tri(botOuter[i0], topOuter[i0], botOuter[i1]);
                    Tri(botOuter[i1], topOuter[i0], topOuter[i1]);

                    Tri(botInner[i1], topInner[i0], botInner[i0]);
                    Tri(topInner[i1], topInner[i0], botInner[i1]);

                    Tri(topOuter[i0], topInner[i0], topOuter[i1]);
                    Tri(topOuter[i1], topInner[i0], topInner[i1]);

                    Tri(botOuter[i1], botInner[i0], botOuter[i0]);
                    Tri(botInner[i1], botInner[i0], botOuter[i1]);
                }
            }

            {
                PxU32 bo = botOuter[0];
                PxU32 bi = botInner[0];
                PxU32 to = topOuter[0];
                PxU32 ti = topInner[0];

                Tri(bo, bi, to);
                Tri(to, bi, ti);
            }
            {
                PxU32 bo = botOuter[samples];
                PxU32 bi = botInner[samples];
                PxU32 to = topOuter[samples];
                PxU32 ti = topInner[samples];

                Tri(bo, to, bi);
                Tri(to, ti, bi);
            }

            TriangleMesh* mesh = mb.Finalize(PxTransform(PxIdentity));
            parts.push_back(mesh);
            Color(color);

            PxShape* shape = mesh->GetShape(0);
            ApplyMaterialAndGroup(
                shape,
                mats.railMat,
                CG_TRACK,
                CG_CAR | CG_WORLD
            );
        }
    };
}
