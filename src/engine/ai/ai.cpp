#include "ai.h"

#include "level/level.h"
#include "engine/laranode.h"
#include "engine/items/aiagent.h"


namespace engine
{
    namespace ai
    {
        LookAhead::LookAhead(const items::ItemNode& npc, int pivotDistance)
        {
            const auto dx = npc.getLevel().m_lara->getPosition().X
                            - (npc.getPosition().X + pivotDistance * npc.getRotation().Y.sin());
            const auto dz = npc.getLevel().m_lara->getPosition().Z
                            - (npc.getPosition().Z + pivotDistance * npc.getRotation().Y.cos());
            const auto angle = core::Angle::fromAtan(dx, dz);
            pivotDistanceToLaraSq = std::lround(dx * dx + dz * dz);
            pivotAngleToLara = angle - npc.getRotation().Y;
            laraAngleToPivot = angle + 180_deg - npc.getLevel().m_lara->getRotation().Y;
            laraAhead = pivotAngleToLara > -90_deg && pivotAngleToLara < 90_deg;
            enemyFacing = false;
            if( laraAhead )
            {
                if( npc.getPosition().Y - loader::QuarterSectorSize < npc.getLevel().m_lara->getPosition().Y
                    && npc.getPosition().Y + loader::QuarterSectorSize > npc.getLevel().m_lara->getPosition().Y )
                {
                    enemyFacing = true;
                }
            }
        }


        void RoutePlanner::updateMood(Brain& brain,
                                      LookAhead& lookAhead,
                                      items::AIAgent& npc,
                                      bool ignoreProbabilities,
                                      uint16_t attackTargetUpdateProbability)
        {
            if(!npc.getCurrentBox().is_initialized())
                return;

            const auto laraZone = getZone(*npc.getLevel().m_lara);

            if( brain.mood != Mood::Attack
                && searchOverride.is_initialized()
                && !brain.isInsideZoneButNotInBox(npc, getZoneData(npc.getLevel()), *destinationBox) )
            {
                if(npc.getZone() == laraZone )
                {
                    brain.mood = Mood::Bored;
                }
                searchOverride.reset();
            }
            const auto originalMood = brain.mood;
            if( npc.getLevel().m_lara->getHealth() > 0 )
            {
                if( ignoreProbabilities )
                {
                    switch( brain.mood )
                    {
                        case Mood::Bored:
                        case Mood::Stalk:
                            if( npc.getZone() == laraZone )
                            {
                                brain.mood = Mood::Attack;
                            }
                            else if( npc.m_state.is_hit )
                            {
                                brain.mood = Mood::Escape;
                            }
                            break;
                        case Mood::Attack:
                            if(npc.getZone() != laraZone )
                            {
                                brain.mood = Mood::Bored;
                            }
                            break;
                        case Mood::Escape:
                            if(npc.getZone() == laraZone )
                            {
                                brain.mood = Mood::Attack;
                            }
                            break;
                    }
                }
                else
                {
                    switch( brain.mood )
                    {
                        case Mood::Bored:
                        case Mood::Stalk:
                            if( npc.m_state.is_hit
                                && ((std::rand() % 32768) < 2048 || npc.getZone() != laraZone) )
                            {
                                brain.mood = Mood::Escape;
                            }
                            else if(npc.getZone() == laraZone )
                            {
                                if( lookAhead.pivotDistanceToLaraSq >= util::square(3 * loader::SectorSize)
                                    && (brain.mood != Mood::Stalk || searchOverride.is_initialized()) )
                                {
                                    brain.mood = Mood::Stalk;
                                }
                                else
                                {
                                    brain.mood = Mood::Attack;
                                }
                            }
                            break;
                        case Mood::Attack:
                            if( npc.m_state.is_hit
                                && ((std::rand() % 32768) < 2048 || npc.getZone() != laraZone) )
                            {
                                brain.mood = Mood::Escape;
                            }
                            else if(npc.getZone() != laraZone )
                            {
                                brain.mood = Mood::Bored;
                            }
                            break;
                        case Mood::Escape:
                            if(npc.getZone() == laraZone && (std::rand() % 32768) < 256 )
                            {
                                brain.mood = Mood::Stalk;
                            }
                            break;
                    }
                }
            }
            else
            {
                brain.mood = Mood::Bored;
            }

            if( originalMood != brain.mood )
            {
                if( originalMood == Mood::Attack )
                {
                    setRandomSearchTarget(*destinationBox, npc);
                }
                searchOverride.reset();
            }

            uint16_t randomBox;

            switch( brain.mood )
            {
                case Mood::Attack:
                    if( (std::rand() % 32768) < attackTargetUpdateProbability )
                    {
                        searchTarget = npc.getLevel().m_lara->getPosition();
                        searchOverride = npc.getLevel().m_lara->getCurrentBox();
                        BOOST_ASSERT(searchOverride.is_initialized());
                        if( flyHeight != 0 && !npc.getLevel().m_lara->isInWater() )
                        {
                            searchTarget.Y += npc.getLevel().m_lara->getSkeleton()->getBoundingBox().minY;
                        }
                    }
                    break;
                case Mood::Bored:
                    randomBox = gsl::narrow_cast<uint16_t>(std::rand() % npc.getLevel().m_boxes.size());
                    if( brain.isInsideZoneButNotInBox(npc, getZoneData(npc.getLevel()), randomBox) )
                    {
                        if( stalkBox(npc, randomBox) )
                        {
                            setRandomSearchTarget(randomBox, npc);
                            brain.mood = Mood::Stalk;
                        }
                        else if( !searchOverride.is_initialized() )
                        {
                            setRandomSearchTarget(randomBox, npc);
                        }
                    }
                    break;
                case Mood::Stalk:
                    if( !searchOverride.is_initialized() || !stalkBox(npc, *searchOverride) )
                    {
                        randomBox = gsl::narrow_cast<uint16_t>(std::rand() % npc.getLevel().m_boxes.size());
                        if( brain.isInsideZoneButNotInBox(npc, getZoneData(npc.getLevel()), randomBox) )
                        {
                            if( stalkBox(npc, randomBox) )
                            {
                                setRandomSearchTarget(randomBox, npc);
                            }
                            else if( !searchOverride.is_initialized() )
                            {
                                setRandomSearchTarget(randomBox, npc);
                                if(npc.getZone() != laraZone )
                                {
                                    brain.mood = Mood::Bored;
                                }
                            }
                        }
                    }
                    break;
                case Mood::Escape:
                    randomBox = gsl::narrow_cast<uint16_t>(std::rand() % npc.getLevel().m_boxes.size());
                    if( brain.isInsideZoneButNotInBox(npc, getZoneData(npc.getLevel()), randomBox) && !searchOverride.is_initialized() )
                    {
                        if( inSameQuadrantAsBoxRelativeToLara(npc, randomBox) )
                        {
                            setRandomSearchTarget(randomBox, npc);
                        }
                        else if(npc.getZone() == laraZone && stalkBox(npc, randomBox) )
                        {
                            setRandomSearchTarget(randomBox, npc);
                            brain.mood = Mood::Stalk;
                        }
                    }
                    break;
            }
            if( !destinationBox.is_initialized() )
            {
                setRandomSearchTarget(*npc.getCurrentBox(), npc);
            }
            calculateTarget(brain.moveTarget, npc, npc.getLevel().m_lara);
        }


        void RoutePlanner::findPath(const items::ItemNode& npc, const items::ItemNode* enemy)
        {
            path.clear();

            if(enemy != nullptr)
            {
                searchTarget = enemy->getPosition();

                BOOST_ASSERT(enemy->getCurrentBox().is_initialized());

                destinationBox = *enemy->getCurrentBox() & ~0xc000;
            }

            const auto startBox = *npc.getCurrentBox() & ~0xc000;

            static constexpr const uint16_t maxDepth = 5;
            static constexpr const int UnsetBoxId = -1;

            std::vector<int> parents;

            std::vector<uint16_t> levelNodes;
            std::vector<uint16_t> nextLevel;
            levelNodes.emplace_back(startBox);

            std::vector<int> costs(startBox+1, UnsetBoxId);
            costs[startBox] = 0;

            for( uint16_t level = 1; level <= maxDepth; ++level )
            {
                nextLevel.clear();

                for( const auto levelBoxIdx : levelNodes )
                {
                    BOOST_ASSERT(levelBoxIdx < npc.getLevel().m_boxes.size());
                    const auto& levelBox = npc.getLevel().m_boxes[levelBoxIdx];
                    const auto overlapIdx = static_cast<uint16_t>( levelBox.overlap_index & ~0xc000 );
                    // examine edge levelBox --> childBox
                    for( auto childBox : getOverlaps(npc.getLevel(), overlapIdx) )
                    {
                        BOOST_ASSERT(childBox != levelBoxIdx);
                        if( !canTravelFromTo(npc.getLevel(), levelBoxIdx, childBox) )
                        {
                            continue;
                        }

                        if(costs.size() <= childBox)
                            costs.resize(childBox + 1, UnsetBoxId);

                        const bool unvisited = costs[childBox] == UnsetBoxId;

                        if( unvisited || level < costs[childBox])
                        {
                            costs[childBox] = level;
                            if(parents.size() <= childBox)
                                parents.resize(childBox+1, UnsetBoxId);
                            parents[childBox] = levelBoxIdx;
                        }

                        if( childBox == destinationBox )
                        {
                            auto current = *destinationBox;
                            BOOST_ASSERT(path.empty());
                            BOOST_ASSERT(current < npc.getLevel().m_boxes.size());
                            path.push_back(&npc.getLevel().m_boxes[current]);
                            while(parents.size() > current && parents[current] != UnsetBoxId )
                            {
                                current = gsl::narrow<uint16_t>(parents[current]);
                                BOOST_ASSERT(current < npc.getLevel().m_boxes.size());
                                path.push_back(&npc.getLevel().m_boxes[current]);
                            }

                            std::reverse(path.begin(), path.end());

                            BOOST_ASSERT(path.front() == &npc.getLevel().m_boxes[startBox]);

                            return;
                        }

                        if( unvisited )
                        {
                            nextLevel.emplace_back(childBox);
                        }
                    }
                }

                levelNodes.swap(nextLevel);
            }

            BOOST_ASSERT(path.empty());
        }


        gsl::span<const uint16_t> RoutePlanner::getOverlaps(const level::Level& lvl, uint16_t idx)
        {
            const uint16_t* first = &lvl.m_overlaps[idx];
            const uint16_t* last = first;
            const uint16_t* const endOfUniverse = &lvl.m_overlaps.back();

            while( last <= endOfUniverse && (*last & 0x8000) == 0 )
            {
                ++last;
            }

            return gsl::make_span(first, last);
        }


        bool RoutePlanner::canTravelFromTo(const level::Level& lvl, uint16_t from, uint16_t to) const
        {
            Expects(from < lvl.m_boxes.size());
            Expects(to < lvl.m_boxes.size());

            const auto& fromBox = lvl.m_boxes[from];
            const auto& toBox = lvl.m_boxes[to];
            if( (toBox.overlap_index & blockMask) != 0 )
            {
                return false;
            }

            const auto& zone = getZoneData(lvl);

            BOOST_ASSERT(from < zone.size());
            BOOST_ASSERT(to < zone.size());

            if( zone[from] != zone[to] )
            {
                return false;
            }

            const auto d = toBox.floor - fromBox.floor;
            return d >= dropHeight && d <= stepHeight;
        }


        const loader::ZoneData& RoutePlanner::getZoneData(const level::Level& lvl) const
        {
            if( flyHeight != 0 )
            {
                return lvl.roomsAreSwapped ? lvl.m_alternateZones.flyZone : lvl.m_baseZones.flyZone;
            }
            else if( stepHeight == loader::QuarterSectorSize )
            {
                return lvl.roomsAreSwapped ? lvl.m_alternateZones.groundZone1 : lvl.m_baseZones.groundZone1;
            }
            else
            {
                return lvl.roomsAreSwapped ? lvl.m_alternateZones.groundZone2 : lvl.m_baseZones.groundZone2;
            }
        }


        void RoutePlanner::setRandomSearchTarget(uint16_t boxIdx, const items::ItemNode& npc)
        {
            setRandomSearchTarget(boxIdx, npc.getLevel().m_boxes[boxIdx & ~0x8000]);
        }


        bool RoutePlanner::stalkBox(const items::ItemNode& npc, uint16_t box)
        {
            Expects(box < npc.getLevel().m_boxes.size());
            return stalkBox(npc, npc.getLevel().m_boxes[box]);
        }


        bool RoutePlanner::stalkBox(const items::ItemNode& npc, const loader::Box& box)
        {
            const auto laraToBoxDistX = (box.xmin + box.xmax) / 2 - npc.getLevel().m_lara->getPosition().X;
            const auto laraToBoxDistZ = (box.zmin + box.zmax) / 2 - npc.getLevel().m_lara->getPosition().Z;

            if( laraToBoxDistX > 3 * loader::SectorSize || laraToBoxDistX < -3 * loader::SectorSize || laraToBoxDistZ > 3 * loader::SectorSize ||
                laraToBoxDistZ < -3 * loader::SectorSize )
            {
                return false;
            }

            auto laraAxisBack = *core::axisFromAngle(npc.getLevel().m_lara->getRotation().Y + 180_deg, 45_deg);
            core::Axis laraToBoxAxis;
            if( laraToBoxDistZ > 0 )
            {
                if( laraToBoxDistX > 0 )
                {
                    laraToBoxAxis = core::Axis::PosX;
                }
                else
                {
                    laraToBoxAxis = core::Axis::NegZ;
                }
            }
            else if( laraToBoxDistX > 0 )
            {
                // Z <= 0, X > 0
                laraToBoxAxis = core::Axis::NegX;
            }
            else
            {
                // Z <= 0, X <= 0
                laraToBoxAxis = core::Axis::PosZ;
            }

            if( laraAxisBack == laraToBoxAxis )
            {
                return false;
            }

            core::Axis itemToLaraAxis;
            if( npc.getPosition().Z <= npc.getLevel().m_lara->getPosition().Z )
            {
                if( npc.getPosition().X <= npc.getLevel().m_lara->getPosition().X )
                {
                    itemToLaraAxis = core::Axis::PosZ;
                }
                else
                {
                    itemToLaraAxis = core::Axis::NegX;
                }
            }
            else
            {
                if( npc.getPosition().X > npc.getLevel().m_lara->getPosition().X )
                {
                    itemToLaraAxis = core::Axis::PosX;
                }
                else
                {
                    itemToLaraAxis = core::Axis::NegZ;
                }
            }

            if( laraAxisBack != itemToLaraAxis )
            {
                return true;
            }

            switch( laraAxisBack )
            {

                case core::Axis::PosZ:
                    return laraToBoxAxis == core::Axis::NegZ;
                case core::Axis::PosX:
                    return laraToBoxAxis == core::Axis::NegX;
                case core::Axis::NegZ:
                    return laraToBoxAxis == core::Axis::PosZ;
                case core::Axis::NegX:
                    return laraToBoxAxis == core::Axis::PosX;
            }

            BOOST_THROW_EXCEPTION(std::runtime_error("Unreachable code reached"));
        }


        bool RoutePlanner::inSameQuadrantAsBoxRelativeToLara(const items::ItemNode& npc, uint16_t box)
        {
            Expects(box < npc.getLevel().m_boxes.size());
            return inSameQuadrantAsBoxRelativeToLara(npc, npc.getLevel().m_boxes[box]);
        }


        bool RoutePlanner::inSameQuadrantAsBoxRelativeToLara(const items::ItemNode& npc, const loader::Box& box)
        {
            const auto laraToBoxX = (box.xmin + box.xmax) / 2 - npc.getLevel().m_lara->getPosition().X;
            const auto laraToBoxZ = (box.zmin + box.zmax) / 2 - npc.getLevel().m_lara->getPosition().Z;
            if( laraToBoxX <= -5 * loader::SectorSize
                || laraToBoxX >= 5 * loader::SectorSize
                || laraToBoxZ <= -5 * loader::SectorSize
                || laraToBoxZ >= 5 * loader::SectorSize )
            {
                const auto laraToNpcX = npc.getPosition().X - npc.getLevel().m_lara->getPosition().X;
                const auto laraToNpcZ = npc.getPosition().Z - npc.getLevel().m_lara->getPosition().Z;
                return ((laraToNpcZ > 0) == (laraToBoxZ > 0))
                       || ((laraToNpcX > 0) == (laraToBoxX > 0));
            }

            return false;
        }


        bool RoutePlanner::calculateTarget(core::TRCoordinates& targetPos, const items::ItemNode& npc, const items::ItemNode* enemy)
        {
            if(searchOverride.is_initialized())
            {
                destinationBox = searchOverride;
            }

            targetPos = npc.getPosition();

            findPath(npc, enemy);
            if( path.empty() )
            {
                return false;
            }

            static constexpr const uint16_t AllowNegX = (1 << 0);
            static constexpr const uint16_t AllowPosX = (1 << 1);
            static constexpr const uint16_t AllowNegZ = (1 << 2);
            static constexpr const uint16_t AllowPosZ = (1 << 3);
            static constexpr const uint16_t AllowAll = AllowNegX | AllowPosX | AllowNegZ | AllowPosZ;
            static constexpr const uint16_t StayInBox = (1 << 4);
            uint16_t reachable = AllowAll;

            // Defines the reachable area
            int minZ = npc.getPosition().Z, maxZ = npc.getPosition().Z, minX = npc.getPosition().X, maxX = npc.getPosition().X;
            for( const auto& currentBox : path )
            {
                if( flyHeight != 0 )
                {
                    if( currentBox->floor - loader::SectorSize < targetPos.Y )
                    {
                        targetPos.Y = currentBox->floor - loader::SectorSize;
                    }
                }
                else
                {
                    if( currentBox->floor < targetPos.Y )
                    {
                        targetPos.Y = currentBox->floor;
                    }
                }

                if( npc.getPosition().Z + 1 >= currentBox->zmin && npc.getPosition().Z <= currentBox->zmax - 1
                    && npc.getPosition().X + 1 >= currentBox->xmin && npc.getPosition().X <= currentBox->xmax - 1 )
                {
                    // initialize the reachable area to the box the NPC is in.
                    minZ = currentBox->zmin;
                    maxZ = currentBox->zmax;
                    maxX = currentBox->xmax;
                    minX = currentBox->xmin;
                }

                if( npc.getPosition().Z > currentBox->zmax )
                {
                    // need to travel to -Z
                    if( (reachable & AllowNegZ)
                        && npc.getPosition().X >= currentBox->xmin
                        && npc.getPosition().X <= currentBox->xmax )
                    {
                        // we may expand the line, as we have an overlap on the X axis,
                        // an are also allowed to travel towards -Z

                        if( currentBox->zmax - loader::SectorSize / 2 < targetPos.Z )
                        {
                            // travel as far as we can
                            targetPos.Z = currentBox->zmax - loader::SectorSize / 2;
                        }

                        if( reachable & StayInBox )
                        {
                            return true;
                        }

                        // clamp the reachable area on the X axis
                        if( currentBox->xmin > minX )
                        {
                            minX = currentBox->xmin;
                        }
                        if( currentBox->xmax < maxX )
                        {
                            maxX = currentBox->xmax;
                        }

                        // Now we only can travel towards -Z
                        reachable = AllowNegZ;
                    }
                    else if( reachable != AllowNegZ )
                    {
                        // We can't travel to -Z, but there are other axes we may travel to.
                        // So let's travel towards -Z as far as we can, stopping right before the reachable area limit.
                        targetPos.Z = minZ + loader::SectorSize / 2;
                        if( reachable != AllowAll )
                        {
                            // This is only the case if we're not examining the NPC's start box anymore.
                            // Thus, there's no use of searching further, as we need to reach a target
                            // which we can't reach due to the obstacles we encountered so far.
                            return true;
                        }
                        reachable |= StayInBox;
                    }
                }
                else if( npc.getPosition().Z < currentBox->zmin )
                {
                    if( (reachable & AllowPosZ)
                        && npc.getPosition().X >= currentBox->xmin
                        && npc.getPosition().X <= currentBox->xmax )
                    {
                        if( currentBox->zmin + loader::SectorSize / 2 > targetPos.Z )
                        {
                            targetPos.Z = currentBox->zmin + loader::SectorSize / 2;
                        }
                        if( reachable & StayInBox )
                        {
                            return true;
                        }
                        if( currentBox->xmin > minX )
                        {
                            minX = currentBox->xmin;
                        }
                        if( currentBox->xmax < maxX )
                        {
                            maxX = currentBox->xmax;
                        }
                        reachable = AllowPosZ;
                    }
                    else if( reachable != AllowPosZ )
                    {
                        targetPos.Z = maxZ - loader::SectorSize / 2;
                        if( reachable != AllowAll )
                        {
                            return true;
                        }
                        reachable |= StayInBox;
                    }
                }

                if( npc.getPosition().X > currentBox->xmax )
                {
                    if( (reachable & AllowNegX)
                        && npc.getPosition().Z >= currentBox->zmin
                        && npc.getPosition().Z <= currentBox->zmax )
                    {
                        if( currentBox->xmax - loader::SectorSize / 2 < targetPos.X )
                        {
                            targetPos.X = currentBox->xmax - loader::SectorSize / 2;
                        }
                        if( reachable & StayInBox )
                        {
                            return true;
                        }
                        if( currentBox->zmin > minZ )
                        {
                            minZ = currentBox->zmin;
                        }
                        if( currentBox->zmax < maxZ )
                        {
                            maxZ = currentBox->zmax;
                        }
                        reachable = AllowNegX;
                    }
                    else if( reachable != AllowNegX )
                    {
                        targetPos.X = minX + loader::SectorSize / 2;
                        if( reachable != AllowAll )
                        {
                            return true;
                        }
                        reachable |= StayInBox;
                    }
                }
                else if( npc.getPosition().X < currentBox->xmin )
                {
                    if( (reachable & AllowPosX)
                        && npc.getPosition().Z >= currentBox->zmin
                        && npc.getPosition().Z <= currentBox->zmax )
                    {
                        if( currentBox->xmin + loader::SectorSize / 2 > targetPos.X )
                        {
                            targetPos.X = currentBox->xmin + loader::SectorSize / 2;
                        }
                        if( reachable & StayInBox )
                        {
                            return true;
                        }
                        if( currentBox->zmin > minZ )
                        {
                            minZ = currentBox->zmin;
                        }
                        if( currentBox->zmax < maxZ )
                        {
                            maxZ = currentBox->zmax;
                        }
                        reachable = AllowPosX;
                    }
                    else if( reachable != AllowPosX )
                    {
                        targetPos.X = maxX - loader::SectorSize / 2;
                        if( reachable != AllowAll )
                        {
                            return true;
                        }
                        reachable |= StayInBox;
                    }
                }

                if( currentBox == path.back() )
                {
                    if( reachable & (AllowNegZ | AllowPosZ) )
                    {
                        targetPos.Z = searchTarget.Z;
                    }
                    if( reachable & (AllowNegX | AllowPosX) )
                    {
                        targetPos.X = searchTarget.X;
                    }
                    if( !(reachable & StayInBox) )
                    {
                        targetPos.X = util::clamp(targetPos.X, currentBox->xmin + loader::SectorSize / 2,
                                                 currentBox->xmax - loader::SectorSize / 2);
                        targetPos.Z = util::clamp(targetPos.Z, currentBox->zmin + loader::SectorSize / 2,
                                                 currentBox->zmax - loader::SectorSize / 2);
                    }

                    targetPos.Y = searchTarget.Y;
                    return true;
                }
            }

            const loader::Box* endBox = nullptr;
            if( !path.empty() )
            {
                endBox = path.back();
            }
            else
            {
                BOOST_ASSERT(*npc.getCurrentBox() < npc.getLevel().m_boxes.size());
                endBox = &npc.getLevel().m_boxes[*npc.getCurrentBox()];
            }

            if( reachable & (AllowNegZ | AllowPosZ) )
            {
                const auto v_centerZ = endBox->zmax - endBox->zmin - loader::SectorSize;
                targetPos.Z = (v_centerZ * std::rand() / RAND_MAX) + endBox->zmin + loader::SectorSize / 2;
            }
            else if( !(reachable & StayInBox) )
            {
                if( endBox->zmin + loader::SectorSize / 2 > targetPos.Z )
                {
                    targetPos.Z = endBox->zmin + loader::SectorSize / 2;
                }
                else if( endBox->zmax - loader::SectorSize / 2 < targetPos.Z )
                {
                    targetPos.Z = endBox->zmax - loader::SectorSize / 2;
                }
            }

            if( reachable & (AllowNegX | AllowPosX) )
            {
                const auto v_centerX = endBox->xmax - endBox->xmin - loader::SectorSize;
                targetPos.X = (v_centerX * std::rand() / RAND_MAX) + endBox->xmin + loader::SectorSize / 2;
            }
            else if( !(reachable & StayInBox) )
            {
                if( endBox->xmin + loader::SectorSize / 2 > targetPos.X )
                {
                    targetPos.X = endBox->xmin + loader::SectorSize / 2;
                }
                else if( endBox->xmax - loader::SectorSize / 2 < targetPos.X )
                {
                    targetPos.X = endBox->xmax - loader::SectorSize / 2;
                }
            }

            if( flyHeight != 0 )
            {
                targetPos.Y = endBox->floor - core::ClimbLimit2ClickMin;
            }
            else
            {
                targetPos.Y = endBox->floor;
            }

            return false;
        }


        bool Brain::isInsideZoneButNotInBox(const items::AIAgent& npc, const loader::ZoneData& zone, uint16_t boxIdx) const
        {
            Expects(boxIdx < zone.size());
            Expects(boxIdx < npc.getLevel().m_boxes.size());
            if( npc.getZone() != zone[boxIdx] )
            {
                return false;
            }

            const auto& box = npc.getLevel().m_boxes[boxIdx];
            if( (route.blockMask & box.overlap_index) != 0 )
            {
                return false;
            }

            return !(npc.getPosition().Z > box.zmin && npc.getPosition().Z < box.zmax && npc.getPosition().X > box.xmin && npc.getPosition().X < box.xmax);
        }
    }
}
