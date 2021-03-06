#include "aiagent.h"

#include "engine/particle.h"
#include "engine/script/reflection.h"
#include "laraobject.h"

#include <boost/range/adaptors.hpp>

namespace engine::objects
{
core::Angle AIAgent::rotateTowardsTarget(core::Angle maxRotationSpeed)
{
  if(m_state.speed == 0_spd || maxRotationSpeed == 0_au)
  {
    return 0_au;
  }

  const auto dx = m_state.creatureInfo->target.X - m_state.position.position.X;
  const auto dz = m_state.creatureInfo->target.Z - m_state.position.position.Z;
  auto turnAngle = angleFromAtan(dx, dz) - m_state.rotation.Y;
  if(turnAngle < -90_deg || turnAngle > 90_deg)
  {
    // the target is behind the current object, so we need a U-turn
    const auto relativeSpeed
      = m_state.speed * (90_deg).retype_as<core::Speed::type>() / maxRotationSpeed.retype_as<core::Speed::type>();
    if(util::square(dx) + util::square(dz) < util::square(relativeSpeed * 1_frame))
    {
      maxRotationSpeed /= 2;
    }
  }

  turnAngle = util::clamp(turnAngle, -maxRotationSpeed, maxRotationSpeed);

  m_state.rotation.Y += turnAngle;
  return turnAngle;
}

bool AIAgent::isPositionOutOfReach(const core::TRVec& testPosition,
                                   const core::Length& currentBoxFloor,
                                   const core::Length& nextBoxFloor,
                                   const ai::PathFinder& lotInfo) const
{
  const auto sectorBox = findRealFloorSector(testPosition, m_state.position.room)->box;
  if(sectorBox == nullptr)
    return true;

  if(!lotInfo.canVisit(*sectorBox))
    return true;

  const auto stepHeight = currentBoxFloor - sectorBox->floor;

  if(stepHeight > lotInfo.step || stepHeight < lotInfo.drop)
    return true;

  if(stepHeight < -lotInfo.step && sectorBox->floor > nextBoxFloor)
    return true;

  return lotInfo.fly != 0_len && testPosition.Y > lotInfo.fly + sectorBox->floor;
}

bool AIAgent::anyMovingEnabledObjectInReach() const
{
  for(const auto& object : getEngine().getObjectManager().getObjects() | boost::adaptors::map_values)
  {
    if(object.get().get() == this)
      break;

    if(!object->m_isActive || object.get().get() == &getEngine().getObjectManager().getLara())
      continue;

    if(object->m_state.triggerState == TriggerState::Active && object->m_state.speed != 0_spd
       && object->m_state.position.position.distanceTo(m_state.position.position) < m_collisionRadius)
    {
      return true;
    }
  }
  return false;
}

bool AIAgent::animateCreature(const core::Angle& angle, const core::Angle& tilt)
{
  if(m_state.creatureInfo == nullptr)
    return false;

  const auto& lotInfo = m_state.creatureInfo->pathFinder;

  const auto oldPosition = m_state.position.position;

  const auto boxFloor = m_state.box->floor;
  const auto zoneRef = loader::file::Box::getZoneRef(
    getEngine().roomsAreSwapped(), m_state.creatureInfo->pathFinder.fly, m_state.creatureInfo->pathFinder.step);
  ModelObject::update();
  if(m_state.triggerState == TriggerState::Deactivated)
  {
    m_state.health = -16384_hp;
    m_state.collidable = false;
    m_state.creatureInfo.reset();
    deactivate();
    return false;
  }

  const auto bbox = getSkeleton()->getBoundingBox();
  const auto bboxMinY = m_state.position.position.Y + bbox.minY;

  auto room = m_state.position.room;
  auto sector = findRealFloorSector(m_state.position.position + core::TRVec{0_len, bbox.minY, 0_len}, &room);

  if(sector->box == nullptr || m_state.box->*zoneRef != sector->box->*zoneRef
     || boxFloor - sector->box->floor > lotInfo.step || boxFloor - sector->box->floor < lotInfo.drop)
  {
    const auto newSectorX = m_state.position.position.X / core::SectorSize;
    const auto newSectorZ = m_state.position.position.Z / core::SectorSize;

    const auto oldSectorX = oldPosition.X / core::SectorSize;
    const auto oldSectorZ = oldPosition.Z / core::SectorSize;

    static const auto shoveMin = [](const core::Length& l) { return l - (l % core::SectorSize); };

    static const auto shoveMax = [](const core::Length& l) { return shoveMin(l) + core::SectorSize - 1_len; };

    if(newSectorX < oldSectorX)
      m_state.position.position.X = shoveMin(oldPosition.X);
    else if(newSectorX > oldSectorX)
      m_state.position.position.X = shoveMax(oldPosition.X);

    if(newSectorZ < oldSectorZ)
      m_state.position.position.Z = shoveMin(oldPosition.Z);
    else if(newSectorZ > oldSectorZ)
      m_state.position.position.Z = shoveMax(oldPosition.Z);

    sector
      = findRealFloorSector(core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z}, &room);
  }

  Expects(sector->box != nullptr);

  core::Length nextFloor = 0_len;
  if(lotInfo.nodes.at(sector->box).exit_box == nullptr)
  {
    nextFloor = sector->box->floor;
  }
  else
  {
    nextFloor = lotInfo.nodes.at(sector->box).exit_box->floor;
  }

  const auto basePosX = m_state.position.position.X;
  const auto basePosZ = m_state.position.position.Z;

  const auto inSectorX = basePosX % core::SectorSize;
  const auto inSectorZ = basePosZ % core::SectorSize;

  core::Length moveX = 0_len;
  core::Length moveZ = 0_len;

  const auto boundedMin = m_collisionRadius;
  const auto boundedMax = core::SectorSize - m_collisionRadius;
  const core::TRVec base{basePosX, bboxMinY, basePosZ};
  const core::TRVec testX{m_collisionRadius, 0_len, 0_len};
  const core::TRVec testZ{0_len, 0_len, m_collisionRadius};
  const auto minXMove = boundedMin - inSectorX;
  const auto maxXMove = boundedMax - inSectorX;
  const auto minZMove = boundedMin - inSectorZ;
  const auto maxZMove = boundedMax - inSectorZ;

  const auto cannotMoveTo = [this, floor = sector->box->floor, nextFloor = nextFloor, &lotInfo](
                              const core::TRVec& pos) { return isPositionOutOfReach(pos, floor, nextFloor, lotInfo); };

  if(inSectorZ < boundedMin)
  {
    const auto testBase = base - testZ;
    if(cannotMoveTo(testBase))
    {
      moveZ = minZMove;
    }

    if(inSectorX < boundedMin)
    {
      if(cannotMoveTo(base - testX))
      {
        moveX = minXMove;
      }
      else if(moveZ == 0_len && cannotMoveTo(testBase - testX))
      {
        switch(*axisFromAngle(m_state.rotation.Y, 45_deg))
        {
        case core::Axis::NegZ:
        case core::Axis::PosX: moveX = minXMove; break;
        case core::Axis::PosZ:
        case core::Axis::NegX: moveZ = minZMove; break;
        }
      }
    }
    else if(inSectorX > boundedMax)
    {
      if(cannotMoveTo(base + testX))
      {
        moveX = maxXMove;
      }
      else if(moveZ == 0_len && cannotMoveTo(testBase + testX))
      {
        switch(*axisFromAngle(m_state.rotation.Y, 45_deg))
        {
        case core::Axis::PosZ:
        case core::Axis::PosX: moveZ = minZMove; break;
        case core::Axis::NegZ:
        case core::Axis::NegX: moveX = maxXMove; break;
        }
      }
    }
  }
  else if(inSectorZ > boundedMax)
  {
    const auto testBase = base + testZ;
    if(cannotMoveTo(testBase))
    {
      moveZ = maxZMove;
    }

    if(inSectorX < boundedMin)
    {
      if(cannotMoveTo(base - testX))
      {
        moveX = minXMove;
      }
      else if(moveZ == 0_len && cannotMoveTo(testBase - testX))
      {
        switch(*axisFromAngle(m_state.rotation.Y, 45_deg))
        {
        case core::Axis::PosX:
        case core::Axis::NegZ: moveX = minXMove; break;
        case core::Axis::NegX:
        case core::Axis::PosZ: moveZ = maxZMove; break;
        }
      }
    }
    else if(inSectorX > boundedMax)
    {
      if(cannotMoveTo(base + testX))
      {
        moveX = maxXMove;
      }
      else if(moveZ == 0_len && cannotMoveTo(testBase + testX))
      {
        switch(*axisFromAngle(m_state.rotation.Y, 45_deg))
        {
        case core::Axis::PosZ:
        case core::Axis::NegX: moveX = maxXMove; break;
        case core::Axis::NegZ:
        case core::Axis::PosX: moveZ = maxZMove; break;
        }
      }
    }
  }
  else
  {
    if(inSectorX < boundedMin)
    {
      if(cannotMoveTo(base - testX))
      {
        moveX = minXMove;
      }
    }
    else if(inSectorX > boundedMax)
    {
      if(cannotMoveTo(base + testX))
      {
        moveX = maxXMove;
      }
    }
  }

  m_state.position.position.X += moveX;
  m_state.position.position.Z += moveZ;

  if(moveX != 0_len || moveZ != 0_len)
  {
    sector
      = findRealFloorSector(core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z}, &room);

    m_state.rotation.Y += angle;
    m_state.rotation.Z += util::clamp(8 * tilt - m_state.rotation.Z, -3_deg, +3_deg);
  }

  if(anyMovingEnabledObjectInReach())
  {
    m_state.position.position = oldPosition;
    return true;
  }

  if(lotInfo.fly != 0_len)
  {
    auto moveY = util::clamp(m_state.creatureInfo->target.Y - m_state.position.position.Y, -lotInfo.fly, lotInfo.fly);

    const auto currentFloor
      = HeightInfo::fromFloor(sector,
                              core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z},
                              getEngine().getObjectManager().getObjects())
          .y;

    if(m_state.position.position.Y + moveY > currentFloor)
    {
      // fly target is below floor

      if(m_state.position.position.Y > currentFloor)
      {
        // we're already below the floor, so fix it
        m_state.position.position.X = oldPosition.X;
        m_state.position.position.Z = oldPosition.Z;
        moveY = -lotInfo.fly;
      }
      else
      {
        m_state.position.position.Y = currentFloor;
        moveY = 0_len;
      }
    }
    else
    {
      const auto ceiling
        = HeightInfo::fromCeiling(sector,
                                  core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z},
                                  getEngine().getObjectManager().getObjects())
            .y;

      const auto y = m_state.type == TR1ItemId::CrocodileInWater ? 0_len : bbox.minY;

      if(m_state.position.position.Y + y + moveY < ceiling)
      {
        if(m_state.position.position.Y + y < ceiling)
        {
          m_state.position.position.X = oldPosition.X;
          m_state.position.position.Z = oldPosition.Z;
          moveY = lotInfo.fly;
        }
        else
          moveY = 0_len;
      }
    }

    m_state.position.position.Y += moveY;
    sector
      = findRealFloorSector(core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z}, &room);
    m_state.floor
      = HeightInfo::fromFloor(sector,
                              core::TRVec{m_state.position.position.X, bboxMinY, m_state.position.position.Z},
                              getEngine().getObjectManager().getObjects())
          .y;

    core::Angle yaw{0_deg};
    if(m_state.speed != 0_spd)
      yaw = angleFromAtan(-moveY, m_state.speed * 1_frame);

    if(yaw < m_state.rotation.X - 1_deg)
      m_state.rotation.X -= 1_deg;
    else if(yaw > m_state.rotation.X + 1_deg)
      m_state.rotation.X += 1_deg;
    else
      m_state.rotation.X = yaw;

    setCurrentRoom(room);

    return true;
  }

  if(m_state.position.position.Y > m_state.floor)
  {
    m_state.position.position.Y = m_state.floor;
  }
  else if(m_state.floor - m_state.position.position.Y > 64_len)
  {
    m_state.position.position.Y += 64_len;
  }
  else if(m_state.position.position.Y < m_state.floor)
  {
    m_state.position.position.Y = m_state.floor;
  }

  m_state.rotation.X = 0_au;

  sector = findRealFloorSector(m_state.position.position, &room);
  m_state.floor
    = HeightInfo::fromFloor(sector, m_state.position.position, getEngine().getObjectManager().getObjects()).y;

  setCurrentRoom(room);

  return true;
}

AIAgent::AIAgent(const gsl::not_null<Engine*>& engine,
                 const gsl::not_null<const loader::file::Room*>& room,
                 const loader::file::Item& item,
                 const gsl::not_null<const loader::file::SkeletalModelType*>& animatedModel)
    : ModelObject{engine, room, item, true, animatedModel}
{
  m_state.collidable = true;
  const core::Angle v = util::rand15(180_deg) * 2;
  m_state.rotation.Y += v;

  loadObjectInfo(false);
}

void AIAgent::collide(CollisionInfo& collisionInfo)
{
  if(!isNear(getEngine().getObjectManager().getLara(), collisionInfo.collisionRadius))
    return;

  if(!testBoneCollision(getEngine().getObjectManager().getLara()))
    return;

  if(!collisionInfo.policyFlags.is_set(CollisionInfo::PolicyFlags::EnableBaddiePush))
    return;

  const bool enableSpaz
    = m_state.health > 0_hp && collisionInfo.policyFlags.is_set(CollisionInfo::PolicyFlags::EnableSpaz);
  enemyPush(collisionInfo, enableSpaz, false);
}

bool AIAgent::canShootAtLara(const ai::AiInfo& aiInfo) const
{
  if(!aiInfo.ahead || aiInfo.distance >= util::square(7 * core::SectorSize))
  {
    return false;
  }

  const auto start = m_state.position;
  auto end = getEngine().getObjectManager().getLara().m_state.position;
  end.position.Y -= 768_len;
  return CameraController::clampPosition(start, end, getEngine().getObjectManager());
}

namespace
{
gsl::not_null<std::shared_ptr<Particle>> createGunFlare(Engine& engine,
                                                        const core::RoomBoundPosition& pos,
                                                        const core::Speed& /*speed*/,
                                                        const core::Angle& angle)
{
  auto particle = std::make_shared<GunflareParticle>(pos, engine, angle);
  setParent(particle, pos.room->node);
  return particle;
}
} // namespace

bool AIAgent::tryShootAtLara(ModelObject& object,
                             const core::Area& distance,
                             const core::TRVec& bonePos,
                             size_t boneIndex,
                             const core::Angle& angle)
{
  bool isHit = false;
  if(distance <= util::square(7 * core::SectorSize))
  {
    if(util::rand15() < (util::square(7 * core::SectorSize) - distance) / util::square(40_len) - 8192)
    {
      isHit = true;

      getEngine().getObjectManager().getLara().emitParticle(
        core::TRVec{},
        util::rand15(getEngine().getObjectManager().getLara().getSkeleton()->getBoneCount()),
        &createBloodSplat);

      if(!getEngine().getObjectManager().getLara().isInWater())
        getEngine().getObjectManager().getLara().playSoundEffect(TR1SoundId::BulletHitsLara);
    }
  }

  if(!isHit)
  {
    auto pos = getEngine().getObjectManager().getLara().m_state.position;
    pos.position.X += util::rand15s(core::SectorSize / 2);
    pos.position.Y = getEngine().getObjectManager().getLara().m_state.floor;
    pos.position.Z += util::rand15s(core::SectorSize / 2);
    getEngine().getObjectManager().getLara().playShotMissed(pos);
  }

  auto p = object.emitParticle(bonePos, boneIndex, &createGunFlare);
  p->angle.Y += angle;

  return isHit;
}
void AIAgent::loadObjectInfo(bool withoutGameState)
{
  m_collisionRadius
    = core::Length{getEngine().getScriptEngine()["getObjectInfo"].call<script::ObjectInfo>(m_state.type.get()).radius};

  if(!withoutGameState)
    m_state.loadObjectInfo(getEngine().getScriptEngine());
}

void AIAgent::hitLara(const core::Health& strength)
{
  getEngine().getObjectManager().getLara().m_state.is_hit = true;
  getEngine().getObjectManager().getLara().m_state.health -= strength;
}
} // namespace engine::objects
