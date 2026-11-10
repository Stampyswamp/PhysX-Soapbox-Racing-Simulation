#pragma once

#include "BasicActors.h"
#include "Collisions.h"

namespace PhysicsEngine
{
    class HayBale : public Box
    {
    public:
        static constexpr float BaleSize = 1.0f;

        HayBale(const PxTransform& pose, PxMaterial* hayMat)
            : Box(pose, PxVec3(BaleSize * 0.5f, BaleSize * 0.5f, BaleSize * 0.5f))
        {
            // hay colour
            Color(PxVec3(0.85f, 0.70f, 0.30f));

            // shape ref
            PxShape* s = GetShape(0);

            // correct collision filtering
            ApplyMaterialAndGroup(s, hayMat, CG_WORLD, CG_CAR | CG_TRACK);

            // light mass
            PxRigidDynamic* body = (PxRigidDynamic*)Get();
            body->setMass(0.5f);
            body->setAngularDamping(0.2f);
            body->setLinearDamping(0.05f);
        }
    };
}
