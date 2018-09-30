#include "skeletalmodelnode.h"

#include "level/level.h"

#include <stack>

namespace engine
{
SkeletalModelNode::SkeletalModelNode(const std::string& id,
                                     const gsl::not_null<const level::Level*>& lvl,
                                     const loader::SkeletalModelType& mdl)
        : Node{id}
        , m_level{lvl}
        , m_model{mdl}
{
    //setAnimation(mdl.animationIndex);
}

int SkeletalModelNode::calculateFloorSpeed(const items::ItemState& state, const int frameOffset)
{
    const auto scaled = state.anim->speed
                        + state.anim->acceleration * (state.frame_number - state.anim->firstFrame + frameOffset);
    return scaled / (1 << 16);
}

int SkeletalModelNode::getAcceleration(const items::ItemState& state)
{
    return state.anim->acceleration / (1 << 16);
}

SkeletalModelNode::InterpolationInfo SkeletalModelNode::getInterpolationInfo(const items::ItemState& state) const
{
    /*
     * == Animation Layout ==
     *
     * Each character in the timeline depicts a single frame.
     *
     * First frame                Last frame/end of animation
     * v                          v
     * |-----|-----|-----|-----|--x..|
     *       ^           <----->     ^
     *       Keyframe    Segment     Last keyframe
     */
    InterpolationInfo result;

    Expects( state.anim != nullptr );
    BOOST_ASSERT( state.anim->segmentLength > 0 );

    if( state.anim->firstFrame == state.anim->lastFrame )
    {
        // empty animation
        result.firstFrame = state.anim->frames;
        result.secondFrame = state.anim->frames;
        return result;
    }

    //BOOST_ASSERT( m_time >= startTime && m_time < endTime );
    Expects( state.frame_number >= state.anim->firstFrame && state.frame_number <= state.anim->lastFrame );
    const size_t firstKeyframeIndex = gsl::narrow<size_t>( state.frame_number - state.anim->firstFrame )
                                      / state.anim->segmentLength;
    Expects( firstKeyframeIndex < state.anim->getKeyframeCount() );

    result.firstFrame = state.anim->frames->next( firstKeyframeIndex );

    if( firstKeyframeIndex == state.anim->getKeyframeCount() - 1u )
    {
        // last keyframe
        result.secondFrame = result.firstFrame;
        result.bias = 0;
        return result;
    }

    result.secondFrame = result.firstFrame->next();

    auto segmentDuration = state.anim->segmentLength;
    const auto segmentFrame = (state.frame_number - state.anim->firstFrame) % segmentDuration;

    if( segmentFrame == 0 )
    {
        result.bias = 0;
        return result;
    }

    // If we are interpolating the last two keyframes, the real animation may be shorter
    // than the position of the last keyframe.  E.g., with a stretch factor of 10 and a length of 12,
    // the last segment would only be 2 frames long.  Frame 1 is interpolated with a bias of 0.1, but
    // frame 11 must be interpolated with a bias of 0.5 to compensate the shorter segment length.
    if( segmentDuration * (firstKeyframeIndex + 1) > state.anim->lastFrame )
        segmentDuration = gsl::narrow<uint8_t>( state.anim->lastFrame - segmentDuration * firstKeyframeIndex );

    result.bias = static_cast<float>(segmentFrame) / segmentDuration;
    BOOST_ASSERT( result.bias >= 0 && result.bias <= 1 );

    return result;
}

void SkeletalModelNode::updatePose(items::ItemState& state)
{
    if( getChildren().empty() )
        return;

    BOOST_ASSERT( getChildren().size() == m_model.meshes.size() );

    updatePose( getInterpolationInfo( state ) );
}

void SkeletalModelNode::updatePoseInterpolated(const InterpolationInfo& framePair)
{
    BOOST_ASSERT( !m_model.meshes.empty() );

    BOOST_ASSERT( framePair.bias > 0 );
    BOOST_ASSERT( framePair.secondFrame != nullptr );

    BOOST_ASSERT( framePair.firstFrame->numValues > 0 );
    BOOST_ASSERT( framePair.secondFrame->numValues > 0 );

    if( m_bonePatches.empty() )
        resetPose();
    BOOST_ASSERT( m_bonePatches.size() == getChildren().size() );

    const auto angleDataFirst = framePair.firstFrame->getAngleData();
    std::stack<glm::mat4> transformsFirst;
    transformsFirst.push( translate( glm::mat4{1.0f}, framePair.firstFrame->pos.toGl() )
                          * core::fromPackedAngles( angleDataFirst[0] ) * m_bonePatches[0] );

    const auto angleDataSecond = framePair.secondFrame->getAngleData();
    std::stack<glm::mat4> transformsSecond;
    transformsSecond.push( translate( glm::mat4{1.0f}, framePair.secondFrame->pos.toGl() )
                           * core::fromPackedAngles( angleDataSecond[0] ) * m_bonePatches[0] );

    BOOST_ASSERT( framePair.bias >= 0 && framePair.bias <= 2 );

    getChildren()[0]->setLocalMatrix( util::mix( transformsFirst.top(), transformsSecond.top(), framePair.bias ) );

    if( m_model.meshes.size() <= 1 )
        return;

    for( int i = 1; i < m_model.meshes.size(); ++i )
    {
        if( m_model.boneTree[i - 1].flags & 0x01 )
        {
            transformsFirst.pop();
            transformsSecond.pop();
        }
        if( m_model.boneTree[i - 1].flags & 0x02 )
        {
            transformsFirst.push( {transformsFirst.top()} ); // make sure to have a copy, not a reference
            transformsSecond.push( {transformsSecond.top()} ); // make sure to have a copy, not a reference
        }

        BOOST_ASSERT( (m_model.boneTree[i - 1].flags & 0x1c) == 0 );

        if( framePair.firstFrame->numValues < i )
            transformsFirst.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                     * m_bonePatches[i];
        else
            transformsFirst.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                     * core::fromPackedAngles( angleDataFirst[i] ) * m_bonePatches[i];

        if( framePair.firstFrame->numValues < i )
            transformsSecond.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                      * m_bonePatches[i];
        else
            transformsSecond.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                      * core::fromPackedAngles( angleDataSecond[i] ) * m_bonePatches[i];

        getChildren()[i]
                ->setLocalMatrix( util::mix( transformsFirst.top(), transformsSecond.top(), framePair.bias ) );
    }
}

void SkeletalModelNode::updatePoseKeyframe(const InterpolationInfo& framePair)
{
    BOOST_ASSERT( !m_model.meshes.empty() );

    BOOST_ASSERT( framePair.firstFrame->numValues > 0 );

    if( m_bonePatches.empty() )
        resetPose();
    BOOST_ASSERT( m_bonePatches.size() == getChildren().size() );

    const auto angleData = framePair.firstFrame->getAngleData();

    std::stack<glm::mat4> transforms;
    transforms.push( translate( glm::mat4{1.0f}, framePair.firstFrame->pos.toGl() )
                     * core::fromPackedAngles( angleData[0] ) * m_bonePatches[0] );

    getChildren()[0]->setLocalMatrix( transforms.top() );

    if( m_model.meshes.size() <= 1 )
        return;

    for( uint16_t i = 1; i < m_model.meshes.size(); ++i )
    {
        BOOST_ASSERT( (m_model.boneTree[i - 1].flags & 0x1c) == 0 );

        if( m_model.boneTree[i - 1].flags & 0x01 )
        {
            transforms.pop();
        }
        if( m_model.boneTree[i - 1].flags & 0x02 )
        {
            transforms.push( {transforms.top()} ); // make sure to have a copy, not a reference
        }

        if( framePair.firstFrame->numValues < i )
            transforms.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() ) * m_bonePatches[i];
        else
            transforms.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                * core::fromPackedAngles( angleData[i] ) * m_bonePatches[i];

        getChildren()[i]->setLocalMatrix( transforms.top() );
    }
}

loader::BoundingBox SkeletalModelNode::getBoundingBox(const items::ItemState& state) const
{
    const auto framePair = getInterpolationInfo( state );
    BOOST_ASSERT( framePair.bias >= 0 && framePair.bias <= 1 );

    if( framePair.secondFrame != nullptr )
    {
        return {framePair.firstFrame->bbox, framePair.secondFrame->bbox, framePair.bias};
    }
    return framePair.firstFrame->bbox;
}

bool SkeletalModelNode::handleStateTransitions(items::ItemState& state)
{
    Expects( state.anim != nullptr );
    if( state.anim->state_id == state.goal_anim_state )
        return false;

    for( const loader::Transitions& tr : state.anim->transitions )
    {
        if( tr.stateId != state.goal_anim_state )
            continue;

        for( const loader::TransitionCase& trc : tr.transitionCases )
        {
            if( state.frame_number >= trc.firstFrame && state.frame_number <= trc.lastFrame )
            {
                setAnimation( state, gsl::make_not_null( trc.targetAnimation ), trc.targetFrame );
                return true;
            }
        }
    }

    return false;
}

// ReSharper disable once CppMemberFunctionMayBeConst
void SkeletalModelNode::setAnimation(items::ItemState& state,
                                     const gsl::not_null<const loader::Animation*>& animation,
                                     uint16_t frame)
{
    BOOST_ASSERT( getChildren().empty() || animation->frames->numValues == getChildren().size() );

    if( frame < animation->firstFrame || frame > animation->lastFrame )
        frame = animation->firstFrame;

    state.anim = animation;
    state.frame_number = frame;
    state.current_anim_state = state.anim->state_id;
}

bool SkeletalModelNode::advanceFrame(items::ItemState& state)
{
    ++state.frame_number;
    if( handleStateTransitions( state ) )
    {
        state.current_anim_state = state.anim->state_id;
        if( state.current_anim_state == state.required_anim_state )
            state.required_anim_state = 0;
    }

    return state.frame_number > state.anim->lastFrame;
}

std::vector<SkeletalModelNode::Sphere> SkeletalModelNode::getBoneCollisionSpheres(const items::ItemState& state,
                                                                                  const loader::AnimFrame& frame,
                                                                                  const glm::mat4* baseTransform)
{
    BOOST_ASSERT( frame.numValues > 0 );
    BOOST_ASSERT( !m_model.meshes.empty() );

    if( m_bonePatches.empty() )
        resetPose();
    BOOST_ASSERT( m_bonePatches.size() == getChildren().size() );

    const auto angleData = frame.getAngleData();

    std::stack<glm::mat4> transforms;

    core::TRVec pos;

    if( baseTransform == nullptr )
    {
        pos = state.position.position;
        transforms.push( state.rotation.toMatrix() );
    }
    else
    {
        pos = core::TRVec( 0, 0, 0 );
        transforms.push( *baseTransform * state.rotation.toMatrix() );
    }

    transforms.top() = translate( transforms.top(), frame.pos.toGl() )
                       * core::fromPackedAngles( angleData[0] ) * m_bonePatches[0];

    std::vector<Sphere> result;
    result.emplace_back(
            translate( glm::mat4{1.0f}, pos.toRenderSystem() )
            + translate( transforms.top(), m_model.meshes[0]->center.toRenderSystem() ),
            m_model.meshes[0]->collision_size );

    for( int i = 1; i < m_model.meshes.size(); ++i )
    {
        BOOST_ASSERT( (m_model.boneTree[i - 1].flags & 0x1c) == 0 );

        if( m_model.boneTree[i - 1].flags & 0x01 )
        {
            transforms.pop();
        }
        if( m_model.boneTree[i - 1].flags & 0x02 )
        {
            transforms.push( {transforms.top()} ); // make sure to have a copy, not a reference
        }

        if( frame.numValues < i )
            transforms.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() ) * m_bonePatches[i];
        else
            transforms.top() *= translate( glm::mat4{1.0f}, m_model.boneTree[i - 1].toGl() )
                                * core::fromPackedAngles( angleData[i] ) * m_bonePatches[i];

        auto m = translate( transforms.top(), m_model.meshes[i]->center.toRenderSystem() );
        m[3] += glm::vec4( pos.toRenderSystem(), 0 );
        result.emplace_back( m, m_model.meshes[i]->collision_size );
    }

    return result;
}

void SkeletalModelNode::load(const YAML::Node& n)
{
    resetPose();

    if( n["patches"].IsDefined() )
    {
        Expects( n["patches"].IsSequence() );

        if( n["patches"].size() > 0 && !m_bonePatches.empty() )
        {
            Expects( n["patches"].size() == m_bonePatches.size() );
            for( size_t i = 0; i < m_bonePatches.size(); ++i )
                for( int x = 0, elem = 0; x < 4; ++x )
                    for( int y = 0; y < 4; ++y, ++elem )
                        m_bonePatches[i][x][y] = n["patches"][i][elem].as<float>();
        }
    }
}

YAML::Node SkeletalModelNode::save() const
{
    YAML::Node n;

    for( const glm::mat4& m : m_bonePatches )
    {
        YAML::Node mn;
        mn.SetStyle( YAML::EmitterStyle::Flow );
        for( int x = 0; x < 4; ++x )
            for( int y = 0; y < 4; ++y )
                mn.push_back( m[x][y] );
        n["patches"].push_back( mn );
    }

    return n;
}
}
