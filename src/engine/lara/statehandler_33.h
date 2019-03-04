#pragma once

#include "statehandler_onwater.h"
#include "engine/cameracontroller.h"

namespace engine
{
namespace lara
{
class StateHandler_33 final
        : public StateHandler_OnWater
{
public:
    explicit StateHandler_33(LaraNode& lara)
            : StateHandler_OnWater{lara, LaraStateId::OnWaterStop}
    {
    }

    void handleInput(CollisionInfo& /*collisionInfo*/) override
    {
        getLara().m_state.fallspeed = std::max( 0_spd, getLara().m_state.fallspeed - 4_spd );

        if( getLara().m_state.health <= 0_hp )
        {
            setGoalAnimState( LaraStateId::WaterDeath );
            return;
        }

        if( getEngine().m_inputHandler->getInputState().freeLook )
        {
            getEngine().m_cameraController->setMode( CameraMode::FreeLook );
            getLara().addHeadRotationXY(
                    -FreeLookMouseMovementScale * (getEngine().m_inputHandler->getInputState().mouseMovement.y / 2000),
                    -40_deg, 40_deg,
                    FreeLookMouseMovementScale * (getEngine().m_inputHandler->getInputState().mouseMovement.x / 2000),
                    -50_deg, 50_deg
            );

            auto torsoRot = getLara().getTorsoRotation();
            torsoRot.X = 0_deg;
            torsoRot.Y = getLara().getHeadRotation().Y / 2;

            getLara().setTorsoRotation( torsoRot );

            return;
        }

        if( getEngine().m_cameraController->getMode() == CameraMode::FreeLook )
        {
            getEngine().m_cameraController->setMode( CameraMode::Chase );
        }

        if( getEngine().m_inputHandler->getInputState().xMovement == AxisMovement::Left )
        {
            getLara().m_state.rotation.Y -= 4_deg;
        }
        else if( getEngine().m_inputHandler->getInputState().xMovement == AxisMovement::Right )
        {
            getLara().m_state.rotation.Y += 4_deg;
        }

        if( getEngine().m_inputHandler->getInputState().zMovement == AxisMovement::Forward )
        {
            setGoalAnimState( LaraStateId::OnWaterForward );
        }
        else if( getEngine().m_inputHandler->getInputState().zMovement == AxisMovement::Backward )
        {
            setGoalAnimState( LaraStateId::OnWaterBackward );
        }

        if( getEngine().m_inputHandler->getInputState().stepMovement == AxisMovement::Left )
        {
            setGoalAnimState( LaraStateId::OnWaterLeft );
        }
        else if( getEngine().m_inputHandler->getInputState().stepMovement == AxisMovement::Right )
        {
            setGoalAnimState( LaraStateId::OnWaterRight );
        }

        if( !getEngine().m_inputHandler->getInputState().jump )
        {
            setSwimToDiveKeypressDuration( 0_frame );
            return;
        }

        addSwimToDiveKeypressDuration( 1_frame );
        if( getSwimToDiveKeypressDuration() != 10_frame )
        {
            // not yet allowed to dive; not that the keypress duration is always >10 when coming up from diving
            return;
        }

        setGoalAnimState( LaraStateId::UnderwaterForward );
        setAnimation( AnimationId::FREE_FALL_TO_UNDERWATER_ALTERNATE, 2041_frame );
        getLara().m_state.rotation.X = -45_deg;
        getLara().m_state.fallspeed = 80_spd;
        setUnderwaterState( UnderwaterState::Diving );
    }

    void postprocessFrame(CollisionInfo& collisionInfo) override
    {
        setMovementAngle( getLara().m_state.rotation.Y );
        commonOnWaterHandling( collisionInfo );
    }
};
}
}
