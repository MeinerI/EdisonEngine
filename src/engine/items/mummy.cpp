#include "mummy.h"

#include "engine/laranode.h"

namespace engine
{
namespace items
{
Mummy::Mummy(const gsl::not_null<level::Level*>& level, const gsl::not_null<const loader::Room*>& room,
             const loader::Item& item, const loader::SkeletalModelType& animatedModel)
        : ModelItemNode{level, room, item, true, animatedModel}
{
    m_state.health = level->m_scriptEngine["getObjectInfo"].call<sol::table>( m_state.type )["hit_points"];
}

void Mummy::update()
{
    if( m_state.current_anim_state == 1 )
    {
        auto head = core::Angle::fromAtan(
                getLevel().m_lara->m_state.position.position.X - m_state.position.position.X,
                getLevel().m_lara->m_state.position.position.Z - m_state.position.position.Z );
        head = util::clamp( head - m_state.rotation.Y, -90_deg, +90_deg );
        m_headRotation += util::clamp( head - m_headRotation, -5_deg, +5_deg );
        getSkeleton()->patchBone( 3, core::TRRotation{0_deg, m_headRotation, 0_deg}.toMatrix() );

        if( m_state.health <= 0 || m_state.touch_bits != 0 )
        {
            m_state.goal_anim_state = 2;
        }
    }

    ModelItemNode::update();

    if( m_state.triggerState == TriggerState::Deactivated )
    {
        deactivate();
        m_state.health = -16384;
    }
}

void Mummy::collide(LaraNode& lara, CollisionInfo& info)
{
    if( !isNear( lara, info.collisionRadius ) )
        return;

    if( !testBoneCollision( lara ) )
        return;

    if( !(info.policyFlags & CollisionInfo::EnableBaddiePush) )
        return;

    enemyPush( lara, info, false, true );
}
}
}