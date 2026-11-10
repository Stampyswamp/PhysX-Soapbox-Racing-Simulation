#pragma once

#include "BasicActors.h"
#include "TrackBuilder.h"
#include "TrackConfig.h"
#include "ProceduralTrackGenerator.h"
#include "Collisions.h"
#include "SoapBox.h"
#include <iostream>

namespace PhysicsEngine
{
    using namespace std;

    struct FilterGroup
    {
        enum Enum
        {
            ACTOR0 = (1 << 0),
            ACTOR1 = (1 << 1),
            ACTOR2 = (1 << 2)
        };
    };

    class MySimulationEventCallback : public PxSimulationEventCallback
    {
    public:
        bool trigger;

        MySimulationEventCallback() : trigger(false) {}

        virtual void onTrigger(PxTriggerPair* pairs, PxU32 count)
        {
            for (PxU32 i = 0; i < count; i++)
            {
                if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
                {
                    if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
                    {
                        cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
                        trigger = true;
                    }

                    if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
                    {
                        cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
                        trigger = false;
                    }
                }
            }
        }

        virtual void onContact(const PxContactPairHeader& pairHeader,
            const PxContactPair* pairs,
            PxU32 nbPairs)
        {
            cerr << "Contact found between "
                << pairHeader.actors[0]->getName() << " "
                << pairHeader.actors[1]->getName() << endl;

            for (PxU32 i = 0; i < nbPairs; i++)
            {
                if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
                    cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;

                if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
                    cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
            }
        }

        virtual void onConstraintBreak(PxConstraintInfo*, PxU32) {}
        virtual void onWake(PxActor**, PxU32) {}
        virtual void onSleep(PxActor**, PxU32) {}

#if PX_PHYSICS_VERSION >= 0x304000
        virtual void onAdvance(const PxRigidBody* const*, const PxTransform*, PxU32) {}
#endif
    };

    static PxFilterFlags CustomFilterShader(
        PxFilterObjectAttributes attributes0, PxFilterData filterData0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
        PxPairFlags& pairFlags, const void*, PxU32)
    {
        if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
        {
            pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
            return PxFilterFlags();
        }

        pairFlags = PxPairFlag::eCONTACT_DEFAULT;

        if ((filterData0.word0 & filterData1.word1) &&
            (filterData1.word0 & filterData0.word1))
        {
            pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
            pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
        }

        return PxFilterFlags();
    }

    class MyScene : public Scene
    {
    public:
        MyScene() : Scene(CustomFilterShader), my_callback(nullptr) {}

        bool IsGenerated = false;
        unsigned int Seed = 7018274;

    private:
        MySimulationEventCallback* my_callback;

        void SetVisualisation()
        {
            px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
            px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
        }

    public:
        virtual void CustomInit() override
        {
            SetVisualisation();

            my_callback = new MySimulationEventCallback();
            px_scene->setSimulationEventCallback(my_callback);

            Plane* plane = new Plane();
            plane->Color(PxVec3(0.3f, 0.3f, 0.3f));
            Add(plane);

            MaterialSet materials = CreateMaterials(GetPhysics());

            TrackBuilder builder;
            builder.SetMaterials(&materials);

            TrackSegment track[] =
            {
                { SEG_TRIGGER, {}, {}, {1.0f, 8, 6} },
                { SEG_FLAT,  {4.0f, 25.0f, 0.0f, 40, 0.1f, 5} },
                { SEG_RAMP,  {4.0f, 40.0f, 30.0f, 40, 0.1f, 5} },
                { SEG_RTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_RTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_RTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_RTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_LTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_LTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_LTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_LTURN, {}, {30.0f, 4.0f, 15.0f, 32} },
                { SEG_RAMP,  {4.0f, 70.0f, 30.0f, 40, 0.3f, 5} },
            };

            if (IsGenerated)
            {
                ProceduralTrackGenerator gen;
                gen.GenerateTrack(this, builder, Seed);
            }
            else
            {
                for (auto& seg : track)
                    builder.BuildSegment(this, seg);
            }

            PxVec3 spawn = builder.ComputeBallSpawn(1.0f);

            SoapBox* soapbox = new SoapBox(PxTransform(spawn), materials);
            soapbox->AddToScene(this);
        }

        virtual void CustomUpdate() override {}

        void ExampleKeyReleaseHandler()
        {
            cerr << "I am realeased!" << endl;
        }

        void ExampleKeyPressHandler()
        {
            cerr << "I am pressed!" << endl;
        }
    };
}
