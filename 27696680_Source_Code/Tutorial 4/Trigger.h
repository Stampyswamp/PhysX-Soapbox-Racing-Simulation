#pragma once

#include "BasicActors.h"
#include "Collisions.h"

namespace PhysicsEngine
{
    class FinishTrigger : public Actor
    {
    public:
        FinishTrigger(const PxVec3& center, const PxVec3& halfExtents)
        {
            PxRigidStatic* body = GetPhysics()->createRigidStatic(PxTransform(center));
            actor = body;

            PxShape* shape = body->createShape(
                PxBoxGeometry(halfExtents),
                *GetPhysics()->createMaterial(0, 0, 0)
            );

            shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
            shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);

            PxFilterData fd;
            fd.word0 = CG_TRIGGER;
            fd.word1 = CG_CAR;
            shape->setSimulationFilterData(fd);
        }
    };
}
