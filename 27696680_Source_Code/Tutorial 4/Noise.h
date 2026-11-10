#pragma once

#include <cmath>

namespace PhysicsEngine
{
    // smooth 2D noise
    inline float SmoothNoise2D(float x, float y)
    {
        float fx = x - floorf(x);
        float fy = y - floorf(y);

        float v00 = sinf((x + 0) * 21.0f + (y + 0) * 67.0f) * 690.0f;
        float v10 = sinf((x + 1) * 21.0f + (y + 0) * 67.0f) * 690.0f;
        float v01 = sinf((x + 0) * 21.0f + (y + 1) * 67.0f) * 690.0f;
        float v11 = sinf((x + 1) * 21.0f + (y + 1) * 67.0f) * 690.0f;

        v00 -= floorf(v00);
        v10 -= floorf(v10);
        v01 -= floorf(v01);
        v11 -= floorf(v11);

        float i1 = v00 * (1 - fx) + v10 * fx;
        float i2 = v01 * (1 - fx) + v11 * fx;

        return i1 * (1 - fy) + i2 * fy;
    }

    // ramp noise
    inline float RampNoise(int xIndex, int zIndex, float amplitude)
    {
        float nx = (float)xIndex * 0.35f;
        float nz = (float)zIndex * 0.15f;
        return (SmoothNoise2D(nx, nz) - 0.5f) * amplitude;
    }
}
