#pragma once

#include "BasicActors.h"
#include "Collisions.h"
#include "TrackConfig.h"

namespace PhysicsEngine
{
    class SoapBox : public Box
    {
    public:
        SoapBox(const PxTransform& pose, const MaterialSet& mats)
            : Box(pose, PxVec3(1.2f, 0.4f, 0.7f)) // rectangular chassis
        {
            PxRigidDynamic* body = (PxRigidDynamic*)Get();

            // chassis color
            Color(PxVec3(1.0f, 0.4f, 0.0f));

            PxShape* chassis = GetShape(0);

            ApplyMaterialAndGroup(
                chassis,
                mats.carMat,
                CG_CAR,
                CG_TRACK | CG_WORLD | CG_TRIGGER
            );

            // wheel radius
            float wheelR = 0.25f;

            // wheel positions
            PxVec3 wheelPos[4] =
            {
                PxVec3(0.9f, -0.4f,  0.55f), // front-right
                PxVec3(-0.9f, -0.4f,  0.55f), // front-left
                PxVec3(0.9f, -0.4f, -0.55f), // rear-right
                PxVec3(-0.9f, -0.4f, -0.55f)  // rear-left
            };

            for (int i = 0; i < 4; i++)
            {
                PxShape* wheel = body->createShape(
                    PxSphereGeometry(wheelR),
                    *mats.carMat
                );

                wheel->setLocalPose(PxTransform(wheelPos[i]));

                ApplyMaterialAndGroup(
                    wheel,
                    mats.carMat,
                    CG_CAR,
                    CG_TRACK | CG_WORLD | CG_TRIGGER
                );

                
                Color(PxVec3(1.0f, 0.3f, 0.3f));
            }

            body->setMass(40.0f);
            body->setAngularDamping(0.3f);
            body->setLinearDamping(0.05f);
        }

        void AddToScene(Scene* scene)
        {
            scene->Add(this);
        }
    };
}
