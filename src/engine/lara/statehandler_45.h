#pragma once

#include "abstractstatehandler.h"
#include "engine/collisioninfo.h"


namespace engine
{
    namespace lara
    {
        class StateHandler_45 final : public AbstractStateHandler
        {
        public:
            explicit StateHandler_45(LaraNode& lara)
                : AbstractStateHandler(lara, LaraStateId::RollForward)
            {
            }


            void handleInput(CollisionInfo& /*collisionInfo*/) override
            {
            }


            void postprocessFrame(CollisionInfo& collisionInfo) override
            {
                setFalling(false);
                setFallSpeed(0);
                collisionInfo.facingAngle = getRotation().Y;
                setMovementAngle(collisionInfo.facingAngle);
                collisionInfo.policyFlags |= CollisionInfo::SlopesAreWalls;
                collisionInfo.badPositiveDistance = loader::HeightLimit;
                collisionInfo.badNegativeDistance = -core::ClimbLimit2ClickMin;
                collisionInfo.badCeilingDistance = 0;
                collisionInfo.initHeightInfo(getPosition(), getLevel(), core::ScalpHeight);

                if( stopIfCeilingBlocked(collisionInfo) )
                    return;

                if( tryStartSlide(collisionInfo) )
                    return;

                if( collisionInfo.mid.floor.distance <= 200 )
                {
                    applyShift(collisionInfo);
                    placeOnFloor(collisionInfo);
                    return;
                }

                setAnimIdGlobal(loader::AnimationId::FREE_FALL_FORWARD, 492);
                setTargetState(LaraStateId::JumpForward);
                setFallSpeed(0);
                setFalling(true);
            }
        };
    }
}
