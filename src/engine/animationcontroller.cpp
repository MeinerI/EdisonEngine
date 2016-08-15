#include "animationcontroller.h"

#include "laracontroller.h"


namespace engine
{
    MeshAnimationController::MeshAnimationController(gsl::not_null<const level::Level*> level, const loader::AnimatedModel& model, const std::string& name)
        : AnimationController(level, name)
        , m_model(model)
        , m_currentAnimationId(model.animationIndex)
    {
        if( m_currentAnimationId >= model.animations.size() )
        {
            BOOST_LOG_TRIVIAL(error) << "No initial animation for " << name;
            return;
        }

        startAnimLoop(model.animations[m_currentAnimationId].firstFrame);
        m_targetState = getCurrentAnimState();
    }

    void MeshAnimationController::startAnimLoop(uint32_t localFrame)
    {
        BOOST_ASSERT(m_currentAnimationId < m_model.animations.size());
        m_model.animations[m_currentAnimationId].apply(m_model, localFrame);
    }

    void MeshAnimationController::advanceFrame()
    {
        BOOST_LOG_TRIVIAL(debug) << "Advance frame: current time=" << m_model.timeline->getCurrentTime() << ", duration=" << m_model.timeline->getDuration();
        if(m_model.timeline->getCurrentFrame() + 1 >= m_model.timeline->getNumFrames())
        {
            handleAnimationEnd();
        }
        else
        {
            m_model.timeline->gotoFrame(m_model.timeline->getCurrentFrame() + 1);
        }

        handleTRTransitions();
    }


    uint32_t MeshAnimationController::getCurrentFrame() const
    {
        BOOST_ASSERT(m_currentAnimationId < m_model.animations.size());

        return m_model.timeline->getCurrentFrame() + m_model.animations[m_currentAnimationId].firstFrame;
    }

    uint32_t MeshAnimationController::getAnimEndFrame() const
    {
        BOOST_ASSERT(m_currentAnimationId < m_model.animations.size());

        return m_model.animations[m_currentAnimationId].lastFrame;
    }

    osg::BoundingBoxImpl<osg::Vec3i> MeshAnimationController::getBoundingBox() const
    {
        BOOST_ASSERT(m_currentAnimationId < m_model.animations.size());

        return m_model.animations[m_currentAnimationId].getBoundingBox(getCurrentFrame());
    }

    uint32_t MeshAnimationController::getCurrentRelativeFrame() const
    {
        return m_model.timeline->getCurrentFrame();
    }

    uint16_t MeshAnimationController::getCurrentAnimState() const
    {
        BOOST_ASSERT(m_currentAnimationId < getLevel()->m_animations.size());
        const loader::Animation& currentAnim = getLevel()->m_animations[m_currentAnimationId];
        return currentAnim.state_id;
    }

    void MeshAnimationController::playGlobalAnimation(uint16_t anim, const boost::optional<uint32_t>& firstFrame)
    {
        if(m_currentAnimationId >= m_model.animations.size())
        {
            BOOST_LOG_TRIVIAL(error) << "No animation " << anim << " for " << getName();
            return;
        }

        m_currentAnimationId = anim;
        m_model.animations[m_currentAnimationId].apply(m_model, firstFrame.get_value_or(m_model.animations[m_currentAnimationId].firstFrame));
        //m_targetState = getCurrentState();

        BOOST_LOG_TRIVIAL(debug) << "Playing animation " << anim << ", state " << getCurrentAnimState();
    }

    bool MeshAnimationController::handleTRTransitions()
    {
        if( getCurrentAnimState() == m_targetState )
            return false;

        BOOST_ASSERT(m_currentAnimationId < getLevel()->m_animations.size());
        const loader::Animation& currentAnim = getLevel()->m_animations[m_currentAnimationId];
        const auto currentFrame = getCurrentFrame();

        for( size_t i = 0; i < currentAnim.transitionsCount; ++i )
        {
            auto tIdx = currentAnim.transitionsIndex + i;
            BOOST_ASSERT(tIdx < getLevel()->m_transitions.size());
            const loader::Transitions& tr = getLevel()->m_transitions[tIdx];
            if( tr.stateId != m_targetState )
                continue;

            for( auto j = tr.firstTransitionCase; j < tr.firstTransitionCase + tr.transitionCaseCount; ++j )
            {
                BOOST_ASSERT(j < getLevel()->m_transitionCases.size());
                const loader::TransitionCase& trc = getLevel()->m_transitionCases[j];

                if( currentFrame >= trc.firstFrame && currentFrame <= trc.lastFrame )
                {
                    m_currentAnimationId = trc.targetAnimation;
                    startAnimLoop(trc.targetFrame);
                    BOOST_LOG_TRIVIAL(debug) << getName() << " -- found transition to state " << m_targetState << ", new animation " << m_currentAnimationId << "/frame " << trc.targetFrame;
                    return true;
                }
            }
        }

        return false;
    }

    void MeshAnimationController::handleAnimationEnd()
    {
        BOOST_ASSERT(m_currentAnimationId < getLevel()->m_animations.size());
        const loader::Animation& currentAnim = getLevel()->m_animations[m_currentAnimationId];

        m_currentAnimationId = currentAnim.nextAnimation;
        startAnimLoop(currentAnim.nextFrame);

        setTargetState(getCurrentAnimState());
    }
}
