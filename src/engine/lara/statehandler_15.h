#pragma once

#include "abstractstatehandler.h"
#include "engine/collisioninfo.h"
#include "engine/inputstate.h"
#include "level/level.h"


namespace engine
{
    namespace lara
    {
        class StateHandler_15 final : public AbstractStateHandler
        {
        public:
            explicit StateHandler_15(LaraNode& lara)
                : AbstractStateHandler(lara, LaraStateId::JumpPrepare)
            {
            }


            void handleInput(CollisionInfo& /*collisionInfo*/) override
            {
                if( getLevel().m_inputHandler->getInputState().zMovement == AxisMovement::Forward && getRelativeHeightAtDirection(getRotation().Y, 256) >= -core::ClimbLimit2ClickMin )
                {
                    setMovementAngle(getRotation().Y);
                    setTargetState(LaraStateId::JumpForward);
                }
                else if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Left && getRelativeHeightAtDirection(getRotation().Y - 90_deg, 256) >= -core::ClimbLimit2ClickMin )
                {
                    setMovementAngle(getRotation().Y - 90_deg);
                    setTargetState(LaraStateId::JumpRight);
                }
                else if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Right && getRelativeHeightAtDirection(getRotation().Y + 90_deg, 256) >= -core::ClimbLimit2ClickMin )
                {
                    setMovementAngle(getRotation().Y + 90_deg);
                    setTargetState(LaraStateId::JumpLeft);
                }
                else if( getLevel().m_inputHandler->getInputState().zMovement == AxisMovement::Backward && getRelativeHeightAtDirection(getRotation().Y + 180_deg, 256) >= -core::ClimbLimit2ClickMin )
                {
                    setMovementAngle(getRotation().Y + 180_deg);
                    setTargetState(LaraStateId::JumpBack);
                }

                if( getFallSpeed() > core::FreeFallSpeedThreshold )
                {
                    setTargetState(LaraStateId::FreeFall);
                }
            }


            void postprocessFrame(CollisionInfo& collisionInfo) override
            {
                setFallSpeed(0);
                setFalling(false);
                collisionInfo.badPositiveDistance = loader::HeightLimit;
                collisionInfo.badNegativeDistance = -loader::HeightLimit;
                collisionInfo.badCeilingDistance = 0;
                collisionInfo.facingAngle = getMovementAngle();
                collisionInfo.initHeightInfo(getPosition(), getLevel(), core::ScalpHeight);

                if( collisionInfo.mid.ceiling.distance <= -100 )
                    return;

                setAnimIdGlobal(loader::AnimationId::STAY_SOLID, 185);
                setTargetState(LaraStateId::Stop);
                setHorizontalSpeed(0);
                setPosition(collisionInfo.oldPosition);
            }
        };
    }
}
