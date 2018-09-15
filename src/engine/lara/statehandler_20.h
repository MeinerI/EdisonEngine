#pragma once

#include "statehandler_standing.h"

namespace engine
{
namespace lara
{
class StateHandler_20 final
    : public StateHandler_Standing
{
public:
    explicit StateHandler_20(LaraNode& lara)
        : StateHandler_Standing(lara, LaraStateId::TurnFast)
    {
    }

    void handleInput(CollisionInfo& /*collisionInfo*/) override
    {
        if( getLara().m_state.health <= 0 )
        {
            setGoalAnimState( LaraStateId::Stop );
            return;
        }

        if( getYRotationSpeed() >= 0_deg )
        {
            setYRotationSpeed(8_deg);
            if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Right )
            {
                return;
            }
        }
        else
        {
            setYRotationSpeed(-8_deg);
            if( getLevel().m_inputHandler->getInputState().xMovement == AxisMovement::Left )
            {
                return;
            }
        }

        setGoalAnimState( LaraStateId::Stop );
    }
};
}
}
