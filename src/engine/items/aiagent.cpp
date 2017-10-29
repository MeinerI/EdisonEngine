#include "aiagent.h"
#include "engine/heightinfo.h"
#include "level/level.h"

#include <boost/range/adaptors.hpp>


namespace engine
{
    namespace items
    {
        core::Angle AIAgent::rotateTowardsMoveTarget(const ai::Brain& creatureData, core::Angle maxRotationSpeed)
        {
            if( getHorizontalSpeed() == 0 || maxRotationSpeed == 0_au )
            {
                return 0_au;
            }

            const auto dx = creatureData.moveTarget.X - getPosition().X;
            const auto dz = creatureData.moveTarget.Z - getPosition().Z;
            auto rotation = core::Angle::fromAtan(dx, dz) - getRotation().Y;
            if( rotation > 90_deg || rotation < -90_deg )
            {
                // the move target is behind the NPC
                auto relativeSpeed = getHorizontalSpeed() * (1 << 14) / maxRotationSpeed.toAU();
                if( dx * dx + dz * dz < relativeSpeed * relativeSpeed )
                {
                    maxRotationSpeed /= 2;
                }
            }

            if( rotation > maxRotationSpeed )
            {
                rotation = maxRotationSpeed;
            }
            else if( rotation < -maxRotationSpeed )
            {
                rotation = -maxRotationSpeed;
            }

            addYRotation(rotation);
            return rotation;
        }


        bool AIAgent::isPositionOutOfReach(const core::TRCoordinates& testPosition, int currentBoxFloor, const ai::RoutePlanner& routePlanner) const
        {
            const auto sectorBoxIdx = getLevel().findRealFloorSector(testPosition, getCurrentRoom())->boxIndex;
            if( sectorBoxIdx == 0xffff )
            {
                return true;
            }

            const auto& sectorBox = getLevel().m_boxes[sectorBoxIdx];
            if( routePlanner.blockMask & sectorBox.overlap_index )
            {
                return true;
            }

            const auto testBoxFloor = currentBoxFloor - sectorBox.floor;
            if( testBoxFloor >= routePlanner.dropHeight && testBoxFloor <= routePlanner.stepHeight )
            {
                // We could step up to/drop down to the test sector.
                return routePlanner.flyHeight != 0 && testPosition.Y > routePlanner.flyHeight + sectorBox.floor;
            }

            return true;
        }


        bool AIAgent::anyMovingEnabledItemInReach() const
        {
            for( const std::shared_ptr<ItemNode>& item : getLevel().m_itemNodes | boost::adaptors::map_values )
            {
                if( !item->m_isActive || item.get() == this )
                {
                    continue;
                }

                if( item->m_triggerState == items::TriggerState::Enabled
                    && item->getHorizontalSpeed() != 0
                    && item->getPosition().distanceTo(getPosition()) < m_collisionRadius )
                {
                    return true;
                }
            }
            return false;
        }


        bool AIAgent::animateCreature(core::Angle rotationToMoveTarget, core::Angle roll)
        {
            if( m_triggerState == engine::items::TriggerState::Activated )
            {
                m_health = -16384;
                m_state.collidable = false;
                //! @todo disposeCreatureData();
                deactivate();
                return false;
            }

            ModelItemNode::update();

            const auto initialPos = getPosition();
            auto bboxTop = getSkeleton()->getBoundingBox().minY;
            const loader::Room* npcRoom = getCurrentRoom();
            auto npcSector = getLevel().findRealFloorSector(
                getPosition() + core::TRCoordinates(0, bboxTop, 0),
                &npcRoom);

            if( npcSector->boxIndex == 0xffff )
                return false;

            const auto npcBoxFloor = getLevel().m_boxes[npcSector->boxIndex].floor;

            const auto inSectorX = getPosition().X % loader::SectorSize;
            const auto inSectorZ = getPosition().Z % loader::SectorSize;
            int moveZ = 0;
            int moveX = 0;
            if( inSectorZ < m_collisionRadius )
            {
                // Clamp movement to -Z

                if( isPositionOutOfReach(
                    getPosition() + core::TRCoordinates(0, bboxTop, -m_collisionRadius),
                    npcBoxFloor,
                    m_brain.route) )
                {
                    moveZ = m_collisionRadius - inSectorZ;
                }

                if( inSectorX < m_collisionRadius )
                {
                    // Clamp movement to -X

                    if( isPositionOutOfReach(
                        getPosition() + core::TRCoordinates(-m_collisionRadius, bboxTop, 0),
                        npcBoxFloor,
                        m_brain.route) )
                    {
                        moveX = m_collisionRadius - inSectorX;
                    }
                    else if( moveZ == 0
                             && isPositionOutOfReach(
                                 getPosition() + core::TRCoordinates(-m_collisionRadius, bboxTop, -m_collisionRadius),
                                 npcBoxFloor,
                                 m_brain.route) )
                    {
                        // -X/-Z must be clamped. Clamp the lateral direction to allow better forward movement.
                        if( getRotation().Y > -135_deg && getRotation().Y < 45_deg )
                        {
                            // We're facing -X/+Z
                            moveZ = m_collisionRadius - inSectorZ;
                        }
                        else
                        {
                            moveX = m_collisionRadius - inSectorX;
                        }
                    }
                }
                else if( inSectorX > loader::SectorSize - m_collisionRadius )
                {
                    // Clamp movement to +X

                    if( isPositionOutOfReach(
                        getPosition() + core::TRCoordinates(m_collisionRadius, bboxTop, 0),
                        npcBoxFloor,
                        m_brain.route) )
                    {
                        moveX = loader::SectorSize - m_collisionRadius - inSectorX;
                    }
                    else if( moveZ == 0
                             && isPositionOutOfReach(
                                 getPosition() + core::TRCoordinates(m_collisionRadius, bboxTop, -m_collisionRadius),
                                 npcBoxFloor,
                                 m_brain.route) )
                    {
                        // +X/-Z
                        if( getRotation().Y > -45_deg && getRotation().Y < 135_deg )
                        {
                            moveZ = m_collisionRadius - inSectorZ;
                        }
                        else
                        {
                            moveX = loader::SectorSize - m_collisionRadius - inSectorX;
                        }
                    }
                }
            }
            else if( inSectorZ > loader::SectorSize - m_collisionRadius )
            {
                if( isPositionOutOfReach(
                    getPosition() + core::TRCoordinates(0, bboxTop, m_collisionRadius),
                    npcBoxFloor,
                    m_brain.route) )
                {
                    moveZ = loader::SectorSize - m_collisionRadius - inSectorZ;
                }

                if( inSectorX < m_collisionRadius )
                {
                    if( isPositionOutOfReach(
                        getPosition() + core::TRCoordinates(-m_collisionRadius, bboxTop, 0),
                        npcBoxFloor,
                        m_brain.route) )
                    {
                        moveX = m_collisionRadius - inSectorX;
                    }
                    else if( moveZ == 0
                             && isPositionOutOfReach(
                                 getPosition() + core::TRCoordinates(-m_collisionRadius, bboxTop, -m_collisionRadius),
                                 npcBoxFloor,
                                 m_brain.route) )
                    {
                        if( getRotation().Y > -45_deg && getRotation().Y < 135_deg )
                        {
                            moveX = m_collisionRadius - inSectorX;
                        }
                        else
                        {
                            moveZ = loader::SectorSize - m_collisionRadius - inSectorZ;
                        }
                    }
                }
                else if( inSectorX > loader::SectorSize - m_collisionRadius )
                {
                    if( isPositionOutOfReach(
                        getPosition() + core::TRCoordinates(m_collisionRadius, bboxTop, 0),
                        npcBoxFloor,
                        m_brain.route) )
                    {
                        moveX = loader::SectorSize - m_collisionRadius - inSectorX;
                    }
                    else if( moveZ == 0
                             && isPositionOutOfReach(
                                 getPosition() + core::TRCoordinates(m_collisionRadius, bboxTop, m_collisionRadius),
                                 npcBoxFloor,
                                 m_brain.route) )
                    {
                        if( getRotation().Y > -135_deg && getRotation().Y < 45_deg )
                        {
                            moveX = loader::SectorSize - m_collisionRadius - inSectorX;
                        }
                        else
                        {
                            moveZ = loader::SectorSize - m_collisionRadius - inSectorZ;
                        }
                    }
                }
            }
            else if( inSectorX < m_collisionRadius )
            {
                if( isPositionOutOfReach(
                    getPosition() + core::TRCoordinates(-m_collisionRadius, bboxTop, 0),
                    npcBoxFloor,
                    m_brain.route) )
                {
                    moveX = m_collisionRadius - inSectorX;
                }
            }
            else if( inSectorX > loader::SectorSize - m_collisionRadius )
            {
                if( isPositionOutOfReach(
                    getPosition() + core::TRCoordinates(m_collisionRadius, bboxTop, 0),
                    npcBoxFloor,
                    m_brain.route) )
                {
                    moveX = loader::SectorSize - m_collisionRadius - inSectorX;
                }
            }

            this->moveX(moveX);
            this->moveZ(moveZ);

            if( moveX != 0 || moveZ != 0 )
            {
                npcSector = getLevel().findRealFloorSector(
                    getPosition() + core::TRCoordinates(0, bboxTop, 0),
                    &npcRoom);
                auto effectiveCurveRoll = util::clamp(core::Angle(8 * roll.toAU()) - getRotation().Z, -3_deg, +3_deg);
                addYRotation(rotationToMoveTarget);
                addZRotation(effectiveCurveRoll);
            }

            if( anyMovingEnabledItemInReach() )
            {
                setPosition(initialPos);
                return true;
            }

            if( m_brain.route.flyHeight != 0 )
            {
                auto dy = util::clamp(m_brain.moveTarget.Y - getPosition().Y,
                                             -m_brain.route.flyHeight,
                                             m_brain.route.flyHeight);
                const auto currentFloor = engine::HeightInfo::fromFloor(npcSector,
                                                                        getPosition() + core::TRCoordinates(0, bboxTop, 0),
                                                                        getLevel().m_cameraController).distance;
                if( dy + getPosition().Y <= currentFloor )
                {
                    const auto currentCeiling = HeightInfo::fromCeiling(npcSector,
                                                                        getPosition() + core::TRCoordinates(0, bboxTop, 0),
                                                                        getLevel().m_cameraController).distance;
                    /** @fixme
                    if( npc is CrocodileInWater )
                    {
                    bboxTop = 0;
                    }
                    */
                    if( getPosition().Y + bboxTop + dy < currentCeiling )
                    {
                        if( getPosition().Y + bboxTop >= currentCeiling )
                        {
                            dy = 0;
                        }
                        else
                        {
                            setX(getPosition().X);
                            setZ(getPosition().Z);
                            dy = m_brain.route.flyHeight;
                        }
                    }
                }
                else if( currentFloor >= getPosition().Y )
                {
                    dy = 0;
                    setY(currentFloor);
                }
                else
                {
                    setX(getPosition().X);
                    setZ(getPosition().Z);
                    dy = -m_brain.route.flyHeight;
                }
                moveY(dy);
                const auto sector = getLevel().findRealFloorSector(
                    getPosition() + core::TRCoordinates(0, bboxTop, 0),
                    &npcRoom);
                setFloorHeight(HeightInfo::fromCeiling(sector,
                                                       getPosition() + core::TRCoordinates(0, bboxTop, 0),
                                                       getLevel().m_cameraController).distance);

                core::Angle flyAngle = 0_au;
                if( getHorizontalSpeed() != 0 )
                {
                    flyAngle = core::Angle::fromAtan(getHorizontalSpeed(), dy);
                }

                setXRotation(util::clamp(flyAngle, -1_deg, +1_deg));

                if( npcRoom != getCurrentRoom() )
                {
                    setCurrentRoom(npcRoom);
                }
                return true;
            }

            if( getPosition().Y - 1 >= getFloorHeight() )
            {
                setY(getFloorHeight());
            }
            else if( getFloorHeight() > 64 + getPosition().Y - 1 )
            {
                moveY(64);
            }

            setXRotation(0_au);
            const auto currentSector = getLevel().findRealFloorSector(
                getPosition(),
                &npcRoom);
            setFloorHeight(HeightInfo::fromFloor(currentSector, getPosition(), getLevel().m_cameraController).distance);
            if( npcRoom != getCurrentRoom() )
            {
                setCurrentRoom(npcRoom);
            }
            return true;
        }
    }
}
