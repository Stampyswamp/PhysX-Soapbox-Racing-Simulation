#pragma once

#include <vector>
#include "TrackConfig.h"
#include "TrackBuilder.h"

namespace PhysicsEngine
{
    inline unsigned int NextRand(unsigned int& seed)
    {
        seed = seed * 1664525u + 1013904223u;
        return seed;
    }

    inline int RandRange(unsigned int& seed, int min, int max)
    {
        return min + (NextRand(seed) % (max - min + 1));
    }

    class ProceduralTrackGenerator
    {
    public:
        RampConfig RandomRamp(unsigned int& seed)
        {
            RampConfig cfg;
            cfg.width = 4.0f;
            cfg.length = (float)RandRange(seed, 25, 60);
            cfg.height = (float)RandRange(seed, 15, 30);
            cfg.segments = (int)(cfg.length * 0.8f);
            return cfg;
        }

        RampConfig RandomFlat(unsigned int& seed)
        {
            RampConfig cfg;
            cfg.width = 4.0f;
            cfg.height = 0.0f;
            cfg.length = (float)RandRange(seed, 5, 10);
            cfg.segments = (int)(cfg.length * 0.2f);
            return cfg;
        }

        TurnConfig RandomTurn(unsigned int& seed)
        {
            TurnConfig cfg;
            cfg.width = 4.0f;
            cfg.height = (float)RandRange(seed, 10, 18);
            cfg.radius = (float)RandRange(seed, 20, 30);
            cfg.samples = RandRange(seed, 24, 80);
            return cfg;
        }

        // no triggers here; only ramp/flat/turn
        TrackSegment PickSegment(
            unsigned int& seed,
            int& sinceLastRamp,
            bool forceRamp = false)
        {
            if (forceRamp)
            {
                sinceLastRamp = 0;
                return { SEG_RAMP, RandomRamp(seed) };
            }

            if (sinceLastRamp >= 3)
            {
                sinceLastRamp = 0;
                return { SEG_RAMP, RandomRamp(seed) };
            }

            int type = RandRange(seed, 0, 3);

            TrackSegment seg;

            switch (type)
            {
            case 0:
                seg.type = SEG_RAMP;
                seg.ramp = RandomRamp(seed);
                sinceLastRamp = 0;
                break;

            case 1:
                seg.type = SEG_FLAT;
                seg.ramp = RandomFlat(seed);
                sinceLastRamp++;
                break;

            case 2:
                seg.type = SEG_LTURN;
                seg.turn = RandomTurn(seed);
                sinceLastRamp++;
                break;

            case 3:
                seg.type = SEG_RTURN;
                seg.turn = RandomTurn(seed);
                sinceLastRamp++;
                break;
            }

            return seg;
        }

        void GenerateTrack(Scene* scene, TrackBuilder& builder, unsigned int seed)
        {
            int sinceLastRamp = 0;
            int segmentCount = 8;

            // first segment is always trigger
            TrackSegment first;
            first.type = SEG_TRIGGER;
            first.hay = { 1.0f, 8, 6 };
            builder.BuildSegment(scene, first);

            for (int i = 1; i < segmentCount; i++)
            {
                bool isLast = (i == segmentCount - 1);

                TrackSegment seg = PickSegment(
                    seed,
                    sinceLastRamp,
                    isLast
                );

                builder.BuildSegment(scene, seg);
            }
        }
    };
}
