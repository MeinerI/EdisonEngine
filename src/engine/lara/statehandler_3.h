#pragma once

#include "abstractstatehandler.h"
#include "engine/collisioninfo.h"
#include "engine/inputstate.h"
#include "level/level.h"


namespace engine
{
    namespace lara
    {
        class StateHandler_3 final : public AbstractStateHandler
        {
        public:
            explicit StateHandler_3(LaraNode& lara)
                : AbstractStateHandler(lara, LaraStateId::JumpForward)
            {
            }


            void handleInput(CollisionInfo& /*collisionInfo*/) override
            {
                if( getTargetState() == LaraStateId::SwandiveBegin || getTargetState() == LaraStateId::Reach )
                    setTargetState(LaraStateId::JumpForward);

                if( getTargetState() != LaraStateId::Death && getTargetState() != LaraStateId::Stop )
                {
                    if( getLevel().m_inputHandler->getInputState().action && getHandStatus() == 0 )
                        setTargetState(LaraStateId::Reach);

                    if( getLevel().m_inputHandler->getInputState().moveSlow && getHandStatus() == 0 )
                        setTargetState(LaraStateId::SwandiveBegin);

                    if( getFallSpeed() > core::FreeFallSpeedThreshold )
                        setTargetState(LaraStateId::FreeFall);
                }

                if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Left )
                {
                    subYRotationSpeed(2.25_deg, -3_deg);
                }
                else if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Right )
                {
                    addYRotationSpeed(2.25_deg, 3_deg);
                }
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

                if( applyLandingDamage() )
                {
                    setTargetState(LaraStateId::Death);
                }
                else if( getLevel().m_inputHandler->getInputState().zMovement != AxisMovement::Forward || getLevel().m_inputHandler->getInputState().moveSlow )
                {
                    setTargetState(LaraStateId::Stop);
                }
                else
                {
                    setTargetState(LaraStateId::RunForward);
                }

                setFallSpeed(0);
                setFalling(false);
                setHorizontalSpeed(0);
                placeOnFloor(collisionInfo);

                laraUpdateImpl();
            }
        };
    }
}
