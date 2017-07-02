#pragma once

#include "abstractstatehandler.h"
#include "engine/collisioninfo.h"


namespace engine
{
    namespace lara
    {
        class StateHandler_52 final : public AbstractStateHandler
        {
        public:
            explicit StateHandler_52(LaraNode& lara)
                : AbstractStateHandler(lara, LaraStateId::SwandiveBegin)
            {
            }


            void handleInput(CollisionInfo& collisionInfo) override
            {
                collisionInfo.policyFlags &= ~CollisionInfo::EnableSpaz;
                collisionInfo.policyFlags |= CollisionInfo::EnableBaddiePush;
                if( getFallSpeed() > core::FreeFallSpeedThreshold )
                    setTargetState(LaraStateId::SwandiveEnd);
            }


            void postprocessFrame(CollisionInfo& collisionInfo) override
            {
                collisionInfo.badPositiveDistance = loader::HeightLimit;
                collisionInfo.badNegativeDistance = -core::ClimbLimit2ClickMin;
                collisionInfo.badCeilingDistance = 192;
                collisionInfo.facingAngle = getRotation().Y;
                setMovementAngle(collisionInfo.facingAngle);
                collisionInfo.initHeightInfo(getPosition(), getLevel(), core::ScalpHeight);
                checkJumpWallSmash(collisionInfo);
                if( collisionInfo.mid.floor.distance > 0 || getFallSpeed() <= 0 )
                    return;

                setTargetState(LaraStateId::Stop);
                setFallSpeed(0);
                setFalling(false);
                placeOnFloor(collisionInfo);
            }
        };
    }
}
