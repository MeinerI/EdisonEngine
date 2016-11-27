#pragma once

#include "audio/sourcehandle.h"
#include "core/interpolatedvalue.h"
#include "engine/skeletalmodelnode.h"

#include <chrono>


namespace loader
{
    struct Item;
}


namespace level
{
    class Level;
}


namespace engine
{
    class LaraNode;


    struct CollisionInfo;


    namespace items
    {
        struct InteractionLimits
        {
            gameplay::BoundingBox distance;
            core::TRRotation minAngle;
            core::TRRotation maxAngle;

            bool canInteract(const ItemNode& item, const LaraNode& lara) const;
        };


        class ItemNode : public SkeletalModelNode
        {
            core::RoomBoundPosition m_position;

            // needed for YPR rotation, because the scene node uses XYZ rotation
            core::TRRotation m_rotation;

            gsl::not_null<level::Level*> const m_level;

            core::InterpolatedValue<float> m_fallSpeed{0.0f};
            core::InterpolatedValue<float> m_horizontalSpeed{0.0f};

            bool m_falling = false; // flags2_08

            long m_floorHeight = 0;

            std::vector<std::weak_ptr<audio::SourceHandle>> m_sounds;

            void updateSounds();

        public:
            static constexpr const uint16_t Oneshot = 0x100;
            static constexpr const uint16_t ActivationMask = 0x3e00;
            static constexpr const uint16_t InvertedActivation = 0x4000;
            static constexpr const uint16_t Locked = 0x8000;

            uint16_t m_itemFlags;
            bool m_isActive = false;
            bool m_flags2_02_toggledOn = false;
            bool m_flags2_04_ready = false;
            bool m_flags2_10 = false;
            bool m_flags2_20 = true;
            bool m_flags2_40 = false;
            bool m_flags2_80 = false;
            std::chrono::microseconds m_triggerTimeout{0};

            const bool m_hasProcessAnimCommandsOverride;
            const uint8_t m_characteristics;


            enum class AnimCommandOpcode : uint16_t
            {
                SetPosition = 1,
                SetVelocity = 2,
                EmptyHands = 3,
                Kill = 4,
                PlaySound = 5,
                PlayEffect = 6,
                Interact = 7
            };


            ItemNode(const gsl::not_null<level::Level*>& level,
                     const std::string& name,
                     const gsl::not_null<const loader::Room*>& room,
                     gsl::not_null<loader::Item*> item,
                     bool hasProcessAnimCommandsOverride,
                     uint8_t characteristics,
                     const loader::AnimatedModel& animatedModel);

            virtual ~ItemNode() = default;


            const core::ExactTRCoordinates& getPosition() const noexcept
            {
                return m_position.position;
            }


            const core::TRRotation& getRotation() const noexcept
            {
                return m_rotation;
            }


            gsl::not_null<const loader::Room*> getCurrentRoom() const noexcept
            {
                return m_position.room;
            }


            long getFloorHeight() const noexcept
            {
                return m_floorHeight;
            }


            void setFloorHeight(long h) noexcept
            {
                m_floorHeight = h;
            }


            void setCurrentRoom(const loader::Room* newRoom);

            void applyTransform();


            void rotate(core::Angle dx, core::Angle dy, core::Angle dz)
            {
                m_rotation.X += dx;
                m_rotation.Y += dy;
                m_rotation.Z += dz;
            }


            void move(float dx, float dy, float dz)
            {
                m_position.position.X += dx;
                m_position.position.Y += dy;
                m_position.position.Z += dz;
            }


            void moveLocal(float dx, float dy, float dz)
            {
                const auto sin = getRotation().Y.sin();
                const auto cos = getRotation().Y.cos();
                m_position.position.X += dz * sin + dx * cos;
                m_position.position.Y += dy;
                m_position.position.Z += dz * cos - dx * sin;
            }


            void setPosition(const core::ExactTRCoordinates& pos)
            {
                m_position.position = pos;
            }


            void setXRotation(core::Angle x)
            {
                m_rotation.X = x;
            }


            void addXRotation(core::Angle x)
            {
                m_rotation.X += x;
            }


            void setYRotation(core::Angle y)
            {
                m_rotation.Y = y;
            }


            void addYRotation(core::Angle v)
            {
                m_rotation.Y += v;
            }


            void setZRotation(core::Angle z)
            {
                m_rotation.Z = z;
            }


            void addZRotation(core::Angle z)
            {
                m_rotation.Z += z;
            }


            const level::Level& getLevel() const
            {
                return *m_level;
            }


            level::Level& getLevel()
            {
                return *m_level;
            }


            const core::RoomBoundPosition& getRoomBoundPosition() const noexcept
            {
                return m_position;
            }


            bool isFalling() const noexcept
            {
                return m_falling;
            }


            void setFalling(bool falling) noexcept
            {
                m_falling = falling;
            }


            void setFallSpeed(const core::InterpolatedValue<float>& spd)
            {
                m_fallSpeed = spd;
            }


            const core::InterpolatedValue<float>& getFallSpeed() const noexcept
            {
                return m_fallSpeed;
            }


            void setHorizontalSpeed(const core::InterpolatedValue<float>& speed)
            {
                m_horizontalSpeed = speed;
            }


            const core::InterpolatedValue<float>& getHorizontalSpeed() const
            {
                return m_horizontalSpeed;
            }


            void dampenHorizontalSpeed(const std::chrono::microseconds& deltaTime, float f)
            {
                m_horizontalSpeed.sub( m_horizontalSpeed * f, deltaTime );
            }


            virtual void patchFloor(const core::TRCoordinates& /*pos*/, long& /*y*/)
            {
            }


            virtual void patchCeiling(const core::TRCoordinates& /*pos*/, long& /*y*/)
            {
            }


            virtual void onInteract(LaraNode& /*lara*/)
            {
                //BOOST_LOG_TRIVIAL(warning) << "Interaction not implemented: " << m_name;
            }


            void onFrameChanged(FrameChangeType frameChangeType) override;

            void activate();

            void deactivate();

            void update(const std::chrono::microseconds& deltaTime);

            virtual void updateImpl(const std::chrono::microseconds& deltaTime) = 0;


            core::InterpolatedValue<float>& getHorizontalSpeed()
            {
                return m_horizontalSpeed;
            }


            core::InterpolatedValue<float>& getFallSpeed() noexcept
            {
                return m_fallSpeed;
            }


            bool triggerSwitch(uint16_t arg)
            {
                if( !m_flags2_04_ready || m_flags2_02_toggledOn )
                {
                    return false;
                }

                m_flags2_04_ready = false;

                if( getCurrentState() != 0 || loader::isLastFloordataEntry( arg ) )
                {
                    deactivate();
                    m_flags2_02_toggledOn = false;
                }
                else
                {
                    m_triggerTimeout = std::chrono::milliseconds( gsl::narrow_cast<uint8_t>( arg ) );
                    if( m_triggerTimeout.count() != 1 )
                        m_triggerTimeout *= 1000;
                    m_flags2_02_toggledOn = true;
                }

                return true;
            }


            std::shared_ptr<audio::SourceHandle> playSoundEffect(int id);


            bool triggerPickUp()
            {
                if( !m_flags2_04_ready || !m_flags2_02_toggledOn )
                    return false;

                m_flags2_02_toggledOn = false;
                m_flags2_04_ready = true;
                return true;
            }


            bool triggerKey();

        protected:
            bool isInvertedActivation() const noexcept
            {
                return (m_itemFlags & InvertedActivation) != 0;
            }


            bool updateTriggerTimeout(const std::chrono::microseconds& deltaTime)
            {
                if( (m_itemFlags & ActivationMask) != ActivationMask )
                {
                    return isInvertedActivation();
                }

                if( m_triggerTimeout == std::chrono::microseconds::zero() )
                {
                    return !isInvertedActivation();
                }

                if( m_triggerTimeout < std::chrono::microseconds::zero() )
                {
                    return isInvertedActivation();
                }

                BOOST_ASSERT( deltaTime > std::chrono::microseconds::zero() );
                m_triggerTimeout -= deltaTime;
                if( m_triggerTimeout <= std::chrono::microseconds::zero() )
                    m_triggerTimeout = std::chrono::microseconds( -1 );

                return !isInvertedActivation();
            }
        };


        class StubItem final : public ItemNode
        {
        public:
            StubItem(const gsl::not_null<level::Level*>& level,
                         const std::string& name,
                         const gsl::not_null<const loader::Room*>& room,
                         const gsl::not_null<loader::Item*>& item,
                         const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, false, 0, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& /*deltaTime*/) override
            {
            }
        };


        class Item_55_Switch final : public ItemNode
        {
        public:
            Item_55_Switch(const gsl::not_null<level::Level*>& level,
                               const std::string& name,
                               const gsl::not_null<const loader::Room*>& room,
                               const gsl::not_null<loader::Item*>& item,
                               const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x30, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override
            {
                if( !updateTriggerTimeout( deltaTime ) )
                {
                    setTargetState( 1 );
                    m_triggerTimeout = std::chrono::microseconds::zero();
                }
            }


            void onInteract(LaraNode& lara) override;


            void onFrameChanged(FrameChangeType frameChangeType) override
            {
                m_itemFlags |= ActivationMask;

                ItemNode::onFrameChanged( frameChangeType );
            }
        };


        class Item_35_CollapsibleFloor final : public ItemNode
        {
        public:
            Item_35_CollapsibleFloor(const gsl::not_null<level::Level*>& level,
                                         const std::string& name,
                                         const gsl::not_null<const loader::Room*>& room,
                                         const gsl::not_null<loader::Item*>& item,
                                         const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x34, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& /*deltaTime*/) override
            {
            }


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void onFrameChanged(FrameChangeType frameChangeType) override;


            void patchFloor(const core::TRCoordinates& pos, long& y) override
            {
                if( pos.Y > getPosition().Y - 512 )
                    return;

                if( getCurrentState() != 0 && getCurrentState() != 1 )
                    return;

                y = std::lround( getPosition().Y - 512 );
            }


            void patchCeiling(const core::TRCoordinates& pos, long& y) override
            {
                if( pos.Y <= getPosition().Y - 512 )
                    return;

                if( getCurrentState() != 0 && getCurrentState() != 1 )
                    return;

                y = std::lround( getPosition().Y - 256 );
            }
        };


        class Item_41_TrapDoorUp final : public ItemNode
        {
        public:
            Item_41_TrapDoorUp(const gsl::not_null<level::Level*>& level,
                                   const std::string& name,
                                   const gsl::not_null<const loader::Room*>& room,
                                   const gsl::not_null<loader::Item*>& item,
                                   const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x30, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& /*deltaTime*/) override
            {
            }


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void onFrameChanged(FrameChangeType frameChangeType) override;


            void patchFloor(const core::TRCoordinates& pos, long& y) override
            {
                if( getCurrentState() != 1 || !possiblyOnTrapdoor( pos ) || pos.Y > getPosition().Y )
                    return;

                y = std::lround( getPosition().Y );
            }


            void patchCeiling(const core::TRCoordinates& pos, long& y) override
            {
                if( getCurrentState() != 1 || !possiblyOnTrapdoor( pos ) || pos.Y <= getPosition().Y )
                    return;

                y = std::lround( getPosition().Y + loader::QuarterSectorSize );
            }


        private:
            bool possiblyOnTrapdoor(const core::TRCoordinates& pos) const
            {
                auto sx = std::lround( std::floor( getPosition().X / loader::SectorSize ) );
                auto sz = std::lround( std::floor( getPosition().Z / loader::SectorSize ) );
                auto psx = pos.X / loader::SectorSize;
                auto psz = pos.Z / loader::SectorSize;
                auto axis = core::axisFromAngle( getRotation().Y, 0_au );
                BOOST_ASSERT( axis.is_initialized() );
                if( *axis == core::Axis::PosZ && sx == psx && sz - 1 == psz && sz - 2 == psz )
                {
                    return true;
                }
                if( *axis == core::Axis::NegZ && sx == psx && sz + 1 == psz && sz + 2 == psz )
                {
                    return true;
                }
                if( *axis == core::Axis::PosX && sz == psz && sx - 1 == psx && sx - 2 == psx )
                {
                    return true;
                }
                if( *axis != core::Axis::NegX || sz != psz || sx + 1 != psx || sx + 2 != psx )
                {
                    return false;
                }
                return true;
            }
        };


        class Item_TrapDoorDown final : public ItemNode
        {
        public:
            Item_TrapDoorDown(const gsl::not_null<level::Level*>& level,
                                  const std::string& name,
                                  const gsl::not_null<const loader::Room*>& room,
                                  const gsl::not_null<loader::Item*>& item,
                                  const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x30, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override
            {
                if( updateTriggerTimeout( deltaTime ) )
                {
                    if( getCurrentState() == 0 )
                        setTargetState( 1 );
                }
                else if( getCurrentState() == 1 )
                {
                    setTargetState( 0 );
                }
            }


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void patchFloor(const core::TRCoordinates& pos, long& y) override
            {
                if( getCurrentState() != 0 || !possiblyOnTrapdoorDown( pos ) || pos.Y > getPosition().Y
                    || y <= getPosition().Y )
                    return;

                y = std::lround( getPosition().Y );
            }


            void patchCeiling(const core::TRCoordinates& pos, long& y) override
            {
                if( getCurrentState() != 1 || !possiblyOnTrapdoorDown( pos ) || pos.Y <= getPosition().Y
                    || y > getPosition().Y )
                    return;

                y = std::lround( getPosition().Y + loader::QuarterSectorSize );
            }


        private:
            bool possiblyOnTrapdoorDown(const core::TRCoordinates& pos) const
            {
                auto sx = std::lround( std::floor( getPosition().X / loader::SectorSize ) );
                auto sz = std::lround( std::floor( getPosition().Z / loader::SectorSize ) );
                auto psx = pos.X / loader::SectorSize;
                auto psz = pos.Z / loader::SectorSize;
                auto axis = core::axisFromAngle( getRotation().Y, 1_au );
                BOOST_ASSERT( axis.is_initialized() );
                if( *axis == core::Axis::PosZ && sx == psx && sz + 1 == psz )
                {
                    return true;
                }
                if( *axis == core::Axis::NegZ && sx == psx && sz - 1 == psz )
                {
                    return true;
                }
                if( *axis == core::Axis::PosX && sz == psz && sx + 1 == psx )
                {
                    return true;
                }
                if( *axis != core::Axis::NegX || sz != psz || sx - 1 != psx )
                {
                    return false;
                }
                return true;
            }
        };


        class Item_Door final : public ItemNode
        {
        public:
            Item_Door(const gsl::not_null<level::Level*>& level,
                          const std::string& name,
                          const gsl::not_null<const loader::Room*>& room,
                          const gsl::not_null<loader::Item*>& item,
                          const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x30, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override
            {
                if( updateTriggerTimeout( deltaTime ) )
                {
                    if( getCurrentState() != 0 )
                    {
                        //! @todo Restore original sector data
                    }
                    else
                    {
                        setTargetState( 1 );
                    }
                }
                else
                {
                    if( getCurrentState() == 1 )
                    {
                        setTargetState( 0 );
                        return;
                    }
                    //! @todo Patch original sector data with blocking heights
                }
            }


            void onInteract(LaraNode& lara) override;
        };


        class Item_Block final : public ItemNode
        {
        public:
            Item_Block(const gsl::not_null<level::Level*>& level,
                           const std::string& name,
                           const gsl::not_null<const loader::Room*>& room,
                           const gsl::not_null<loader::Item*>& item,
                           const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x34, animatedModel )
            {
                if( !m_flags2_04_ready || !m_flags2_02_toggledOn )
                    loader::Room::patchHeightsForBlock( *this, -loader::SectorSize );
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override
            {
                if( updateTriggerTimeout( deltaTime ) )
                    setTargetState( 1 );
                else
                    setTargetState( 0 );
            }


            void onInteract(LaraNode& lara) override;

            void onFrameChanged(FrameChangeType frameChangeType) override;

        private:
            bool isOnFloor(int height) const;

            bool canPushBlock(int height, core::Axis axis) const;

            bool canPullBlock(int height, core::Axis axis) const;
        };


        class Item_TallBlock final : public ItemNode
        {
        public:
            Item_TallBlock(const gsl::not_null<level::Level*>& level,
                               const std::string& name,
                               const gsl::not_null<const loader::Room*>& room,
                               const gsl::not_null<loader::Item*>& item,
                               const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x34, animatedModel )
            {
                loader::Room::patchHeightsForBlock( *this, -2 * loader::SectorSize );
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override
            {
                if( updateTriggerTimeout( deltaTime ) )
                {
                    if( getCurrentState() == 0 )
                    {
                        loader::Room::patchHeightsForBlock( *this, 2 * loader::SectorSize );
                        setTargetState( 1 );
                    }
                }
                else
                {
                    if( getCurrentState() == 1 )
                    {
                        loader::Room::patchHeightsForBlock( *this, 2 * loader::SectorSize );
                        setTargetState( 0 );
                    }
                }
            }


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void onFrameChanged(FrameChangeType frameChangeType) override;
        };


        class Item_68_BridgeFlat final : public ItemNode
        {
        public:
            Item_68_BridgeFlat(const gsl::not_null<level::Level*>& level,
                                   const std::string& name,
                                   const gsl::not_null<const loader::Room*>& room,
                                   const gsl::not_null<loader::Item*>& item,
                                   const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, false, 0, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& /*deltaTime*/) override
            {
            }


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void onFrameChanged(FrameChangeType /*frameChangeType*/) override
            {
            }


            void patchFloor(const core::TRCoordinates& pos, long& y) override
            {
                if( pos.Y <= getPosition().Y )
                    y = std::lround( getPosition().Y );
            }


            void patchCeiling(const core::TRCoordinates& pos, long& y) override
            {
                if( pos.Y <= getPosition().Y )
                    return;

                y = std::lround( getPosition().Y + loader::QuarterSectorSize );
            }
        };


        class Item_SlopedBridge : public ItemNode
        {
        private:
            const int m_div;
        public:
            Item_SlopedBridge(const gsl::not_null<level::Level*>& level,
                                  const std::string& name,
                                  const gsl::not_null<const loader::Room*>& room,
                                  const gsl::not_null<loader::Item*>& item,
                                  const loader::AnimatedModel& animatedModel,
                                  int div)
                    : ItemNode( level, name, room, item, false, 0, animatedModel )
                    , m_div( div )
            {
            }


            void updateImpl(const std::chrono::microseconds& /*deltaTime*/) override final
            {
            }


            void onInteract(LaraNode& /*lara*/) override final
            {
            }


            void onFrameChanged(FrameChangeType /*frameChangeType*/) override
            {
            }


            void patchFloor(const core::TRCoordinates& pos, long& y) override final
            {
                auto tmp = std::lround( getPosition().Y + getBridgeSlopeHeight( pos ) / m_div );
                if( pos.Y <= tmp )
                    y = tmp;
            }


            void patchCeiling(const core::TRCoordinates& pos, long& y) override final
            {
                auto tmp = std::lround( getPosition().Y + getBridgeSlopeHeight( pos ) / m_div );
                if( pos.Y <= tmp )
                    return;

                y = tmp + loader::QuarterSectorSize;
            }


        private:
            long getBridgeSlopeHeight(const core::TRCoordinates& pos) const
            {
                auto axis = core::axisFromAngle( getRotation().Y, 1_deg );
                Expects( axis.is_initialized() );

                switch( *axis )
                {
                    case core::Axis::PosZ:return loader::SectorSize - 1 - pos.X % loader::SectorSize;
                    case core::Axis::PosX:return pos.Z % loader::SectorSize;
                    case core::Axis::NegZ:return pos.X % loader::SectorSize;
                    case core::Axis::NegX:return loader::SectorSize - 1 - pos.Z % loader::SectorSize;
                    default:return 0;
                }
            }
        };


        class Item_69_BridgeSlope1 final : public Item_SlopedBridge
        {
        public:
            Item_69_BridgeSlope1(const gsl::not_null<level::Level*>& level,
                                     const std::string& name,
                                     const gsl::not_null<const loader::Room*>& room,
                                     const gsl::not_null<loader::Item*>& item,
                                     const loader::AnimatedModel& animatedModel)
                    : Item_SlopedBridge( level, name, room, item, animatedModel, 4 )
            {
            }
        };


        class Item_70_BridgeSlope2 final : public Item_SlopedBridge
        {
        public:
            Item_70_BridgeSlope2(const gsl::not_null<level::Level*>& level,
                                     const std::string& name,
                                     const gsl::not_null<const loader::Room*>& room,
                                     const gsl::not_null<loader::Item*>& item,
                                     const loader::AnimatedModel& animatedModel)
                    : Item_SlopedBridge( level, name, room, item, animatedModel, 2 )
            {
            }
        };


        class Item_SwingingBlade final : public ItemNode
        {
        public:
            Item_SwingingBlade(const gsl::not_null<level::Level*>& level,
                                   const std::string& name,
                                   const gsl::not_null<const loader::Room*>& room,
                                   const gsl::not_null<loader::Item*>& item,
                                   const loader::AnimatedModel& animatedModel)
                    : ItemNode( level, name, room, item, true, 0x30, animatedModel )
            {
            }


            void updateImpl(const std::chrono::microseconds& deltaTime) override;


            void onInteract(LaraNode& /*lara*/) override
            {
            }


            void onFrameChanged(FrameChangeType frameChangeType) override;
        };
    }
}
