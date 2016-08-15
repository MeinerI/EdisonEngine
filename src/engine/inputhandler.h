#pragma once

#include "inputstate.h"

#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>

namespace engine
{
class InputHandler final : public osgGA::GUIEventHandler
{
public:
    explicit InputHandler()
    {
    }

    void update()
    {
        m_inputState.setXAxisMovement(m_left, m_right);
        m_inputState.setZAxisMovement(m_backward, m_forward);
        m_inputState.setStepMovement(m_stepLeft, m_stepRight);
    }

    const InputState& getInputState() const
    {
        return m_inputState;
    }

    bool handle(osgGA::Event* evt, osg::Object* object, osg::NodeVisitor* nv) override
    {
        osgGA::GUIEventAdapter* adapter = evt->asGUIEventAdapter();
        BOOST_ASSERT(adapter != nullptr);

        switch(adapter->getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
            case osgGA::GUIEventAdapter::KEYUP:
                switch(adapter->getKey())
                {
                    case osgGA::GUIEventAdapter::KEY_A:
                        m_left = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_D:
                        m_right = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_Q:
                        m_stepLeft = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_E:
                        m_stepRight = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_W:
                        m_forward = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_S:
                        m_backward = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_Shift_L:
                    case osgGA::GUIEventAdapter::KEY_Shift_R:
                        m_inputState.moveSlow = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_Control_L:
                    case osgGA::GUIEventAdapter::KEY_Control_R:
                        m_inputState.action = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_Space:
                        m_inputState.jump = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        return true;
                    case osgGA::GUIEventAdapter::KEY_X:
                        m_inputState.roll = (adapter->getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
                        break;
                    default:
                        return false;
                }
                break;

            case osgGA::GUIEventAdapter::MOVE:
                m_inputState.mouseMovement.x() = adapter->getX() - (adapter->getXmax() - adapter->getXmin()) / 2;
                m_inputState.mouseMovement.y() = adapter->getY() - (adapter->getYmax() - adapter->getYmin()) / 2;
                //! @todo This is probably *not* warping the cursor.
                adapter->setX((adapter->getXmax() - adapter->getXmin()) / 2);
                adapter->setY((adapter->getYmax() - adapter->getYmin()) / 2);
                return true;

            default:
                break;
        }

        m_inputState.action = (adapter->getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) != 0;
        m_inputState.freeLook = (adapter->getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) != 0;


        return false;
    }


private:
    bool m_forward = false;
    bool m_backward = false;
    bool m_left = false;
    bool m_right = false;
    bool m_stepLeft = false;
    bool m_stepRight = false;

    InputState m_inputState;
};
}
