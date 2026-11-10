#pragma once

#include "TrackConfig.h"
#include "CompositeObjects.h"
#include "Collisions.h"
#include "HayBale.h"
#include "Trigger.h"

namespace PhysicsEngine
{
    enum SegmentType
    {
        SEG_RAMP,
        SEG_FLAT,
        SEG_LTURN,
        SEG_RTURN,
        SEG_TRIGGER
    };

    struct TrackSegment
    {
        SegmentType   type;
        RampConfig    ramp;
        TurnConfig    turn;
        HayBaleConfig hay;
    };

    class TrackBuilder
    {
    public:
        PxVec3 cursor = PxVec3(0.0f, 0.1f, 0.0f);
        float  headingDeg = 0.0f;

        const MaterialSet* mats = nullptr;

        TrackBuilder() {}
        TrackBuilder(const MaterialSet* m) : mats(m) {}

        void SetMaterials(const MaterialSet* m)
        {
            mats = m;
        }

        PxVec3 Forward() const
        {
            float r = headingDeg * PxPi / 180.0f;
            return PxVec3(sinf(r), 0.0f, cosf(r));
        }

        PxVec3 ComputeBallSpawn(float radius = 0.5f)
        {
            PxVec3 f = Forward();
            PxVec3 up(0, 1, 0);

            float upOffset = radius * 2.0f;
            float inOffset = radius * 2.0f;

            return cursor + up * upOffset - f * inOffset;
        }

    private:
        template<typename CenterFunc, typename TangentFunc>
        void BuildRails(Scene* scene, CenterFunc SC, TangentFunc ST, int samples)
        {
            RailConfig railCfg;

            RailMesh* leftRail = new RailMesh(
                SC, ST,
                railCfg.offsets[0],
                railCfg.heightOffset,
                samples,
                railCfg.color,
                *mats,
                railCfg.thickness
            );
            leftRail->AddToScene(scene);

            RailMesh* rightRail = new RailMesh(
                SC, ST,
                railCfg.offsets[1],
                railCfg.heightOffset,
                samples,
                railCfg.color,
                *mats,
                railCfg.thickness
            );
            rightRail->AddToScene(scene);
        }

    public:
        void BuildSegment(Scene* scene, const TrackSegment& seg)
        {
            if (!mats)
                return;

            switch (seg.type)
            {
            case SEG_RAMP:
            case SEG_FLAT:
            {
                const RampConfig& cfg = seg.ramp;

                CompositeRamp* ramp = new CompositeRamp(cursor, cfg, headingDeg, *mats);
                ramp->Color(PxVec3(0, 0, 0));
                ramp->AddToScene(scene);

                PxVec3 f = Forward();

                auto SC = [&](float t)
                    {
                        float s = t * cfg.length;
                        float y = t * cfg.height;
                        return cursor + PxVec3(f.x * s, y, f.z * s);
                    };

                auto ST = [&](float)
                    {
                        return PxVec3(f.x * cfg.length, cfg.height, f.z * cfg.length);
                    };

                BuildRails(scene, SC, ST, cfg.segments);

                cursor += PxVec3(f.x * cfg.length, cfg.height, f.z * cfg.length);
                break;
            }

            case SEG_LTURN:
            case SEG_RTURN:
            {
                const TurnConfig& cfg = seg.turn;
                bool leftTurn = (seg.type == SEG_LTURN);

                CompositeTurn* turn = new CompositeTurn(cursor, cfg, leftTurn, headingDeg, *mats);
                turn->Color(PxVec3(0, 0, 0));
                turn->AddToScene(scene);

                auto SC = turn->SampleCenter;
                auto ST = turn->SampleTangent;

                BuildRails(scene, SC, ST, cfg.samples);

                cursor = SC(1.0f);
                headingDeg += (leftTurn ? 90.0f : -90.0f);

                break;
            }

            case SEG_TRIGGER:
            {
                const HayBaleConfig& cfg = seg.hay;

                float bale = cfg.baleSize; // 1.0f
                int   cols = cfg.cols;
                int   rows = cfg.rows;

                PxVec3 center = cursor;

                // spacing between bales
                float gap = 0.2f;

                // visually perfect radius (tuned scale factor)
                float segment = bale + gap;
                float radius = ((segment * cols) / (2.0f * PxPi)) * 1.15f;

                for (int r = 0; r < rows; r++)
                {
                    float y = center.y + r * (bale * 1.1f);

                    // brick offset
                    float ringOffset = (r % 2 == 1) ? (PxTwoPi / cols) * 0.5f : 0.0f;

                    for (int i = 0; i < cols; i++)
                    {
                        float t = (float)i / (float)cols;
                        float ang = t * PxTwoPi + ringOffset;

                        float x = center.x + radius * cosf(ang);
                        float z = center.z + radius * sinf(ang);

                        PxVec3 pos(x, y, z);

                        // face inward
                        PxVec3 dir = (center - pos).getNormalized();
                        float yaw = atan2f(dir.x, dir.z);
                        PxQuat rot(yaw, PxVec3(0, 1, 0));

                        PxTransform tform(pos, rot);
                        HayBale* b = new HayBale(tform, mats->hayMat);
                        scene->Add(b);
                    }
                }

                // trigger in the middle
                PxVec3 half(0.5f, rows * bale * 0.5f, 0.5f);
                FinishTrigger* trig = new FinishTrigger(center, half);
                scene->Add(trig);

                // move cursor forward by full diameter + 1 bale
                cursor += Forward() * (radius * 2.0f + bale);
                break;
            }
            }
        }
    };
}
