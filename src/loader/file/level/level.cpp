#include "level.h"

#include "engine/objects/laraobject.h"
#include "render/textureanimator.h"
#include "tr1level.h"
#include "tr2level.h"
#include "tr3level.h"
#include "tr4level.h"
#include "tr5level.h"
#include "util/md5.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/range/adaptors.hpp>
#include <filesystem>

using namespace loader::file;
using namespace level;

Level::~Level() = default;

/// \brief reads the mesh data.
void Level::readMeshData(io::SDLReader& reader)
{
  const auto meshDataWords = reader.readU32();
  const auto basePos = reader.tell();

  const auto meshDataSize = meshDataWords * 2;
  reader.skip(meshDataSize);

  reader.readVector(m_meshIndices, reader.readU32());
  const auto endPos = reader.tell();

  m_meshes.clear();

  uint32_t meshDataPos = 0;
  for(uint32_t i = 0; i < m_meshIndices.size(); i++)
  {
    replace(m_meshIndices.begin(), m_meshIndices.end(), meshDataPos, i);

    reader.seek(basePos + std::streamoff(meshDataPos));

    if(gameToEngine(m_gameVersion) >= Engine::TR4)
      m_meshes.emplace_back(*Mesh::readTr4(reader));
    else
      m_meshes.emplace_back(*Mesh::readTr1(reader));

    for(auto pos : m_meshIndices)
    {
      if(pos > meshDataPos)
      {
        meshDataPos = pos;
        break;
      }
    }
  }

  reader.seek(endPos);
}

std::shared_ptr<Level> Level::createLoader(const std::filesystem::path& filename, Game gameVersion)
{
  std::filesystem::path sfxPath = filename;
  sfxPath.replace_filename("MAIN.SFX");

  io::SDLReader reader{filename};
  if(!reader.isOpen())
    return nullptr;

  if(gameVersion == Game::Unknown)
    gameVersion = probeVersion(reader, filename);
  if(gameVersion == Game::Unknown)
    return nullptr;

  reader.seek(0);
  return createLoader(std::move(reader), gameVersion, sfxPath);
}

std::shared_ptr<Level>
  Level::createLoader(io::SDLReader&& reader, Game game_version, const std::filesystem::path& sfxPath)
{
  if(!reader.isOpen())
    return nullptr;

  std::shared_ptr<Level> result;

  switch(game_version)
  {
  case Game::TR1: result = std::make_shared<TR1Level>(game_version, std::move(reader)); break;
  case Game::TR1Demo:
  case Game::TR1UnfinishedBusiness:
    result = std::make_shared<TR1Level>(game_version, std::move(reader));
    result->m_demoOrUb = true;
    break;
  case Game::TR2: result = std::make_shared<TR2Level>(game_version, std::move(reader)); break;
  case Game::TR2Demo:
    result = std::make_shared<TR2Level>(game_version, std::move(reader));
    result->m_demoOrUb = true;
    break;
  case Game::TR3: result = std::make_shared<TR3Level>(game_version, std::move(reader)); break;
  case Game::TR4:
  case Game::TR4Demo: result = std::make_shared<TR4Level>(game_version, std::move(reader)); break;
  case Game::TR5: result = std::make_shared<TR5Level>(game_version, std::move(reader)); break;
  default: BOOST_THROW_EXCEPTION(std::runtime_error("Invalid game version"));
  }

  result->m_sfxPath = sfxPath;
  return result;
}

Game Level::probeVersion(io::SDLReader& reader, const std::filesystem::path& filename)
{
  if(!reader.isOpen() || !std::filesystem::is_regular_file(filename))
    return Game::Unknown;

  const std::string ext = boost::algorithm::to_upper_copy(filename.extension().string());

  reader.seek(0);
  uint8_t check[4];
  reader.readBytes(check, 4);

  Game ret = Game::Unknown;
  if(ext == ".PHD")
  {
    if(check[0] == 0x20 && check[1] == 0x00 && check[2] == 0x00 && check[3] == 0x00)
    {
      ret = Game::TR1;
    }
  }
  else if(ext == ".TUB")
  {
    if(check[0] == 0x20 && check[1] == 0x00 && check[2] == 0x00 && check[3] == 0x00)
    {
      ret = Game::TR1UnfinishedBusiness;
    }
  }
  else if(ext == ".TR2")
  {
    if(check[0] == 0x2D && check[1] == 0x00 && check[2] == 0x00 && check[3] == 0x00)
    {
      ret = Game::TR2;
    }
    else if((check[0] == 0x38 || check[0] == 0x34) && check[1] == 0x00 && (check[2] == 0x18 || check[2] == 0x08)
            && check[3] == 0xFF)
    {
      ret = Game::TR3;
    }
  }
  else if(ext == ".TR4")
  {
    if(check[0] == 0x54 && // T
       check[1] == 0x52 && // R
       check[2] == 0x34 && // 4
       check[3] == 0x00)
    {
      ret = Game::TR4;
    }
    else if(check[0] == 0x54 && // T
            check[1] == 0x52 && // R
            check[2] == 0x34 && // 4
            check[3] == 0x63)   //
    {
      ret = Game::TR4;
    }
    else if(check[0] == 0xF0 && // T
            check[1] == 0xFF && // R
            check[2] == 0xFF && // 4
            check[3] == 0xFF)
    {
      ret = Game::TR4;
    }
  }
  else if(ext == ".TRC")
  {
    if(check[0] == 0x54 && // T
       check[1] == 0x52 && // R
       check[2] == 0x34 && // C
       check[3] == 0x00)
    {
      ret = Game::TR5;
    }
  }

  return ret;
}

const StaticMesh* Level::findStaticMeshById(const core::StaticMeshId meshId) const
{
  for(const auto& mesh : m_staticMeshes)
    if(mesh.id == meshId)
      return &mesh;

  return nullptr;
}

std::shared_ptr<render::scene::Mesh> Level::findStaticRenderMeshById(const core::StaticMeshId meshId) const
{
  for(const auto& mesh : m_staticMeshes)
  {
    if(mesh.isVisible() && mesh.id == meshId)
    {
      return mesh.renderMesh;
    }
  }

  return nullptr;
}

const std::unique_ptr<SkeletalModelType>& Level::findAnimatedModelForType(const core::TypeId type) const
{
  const auto it = m_animatedModels.find(type);
  if(it != m_animatedModels.end())
    return it->second;

  static const std::unique_ptr<SkeletalModelType> none;
  return none;
}

const std::unique_ptr<SpriteSequence>& Level::findSpriteSequenceForType(const core::TypeId type) const
{
  const auto it = m_spriteSequences.find(type);
  if(it != m_spriteSequences.end())
    return it->second;

  static const std::unique_ptr<SpriteSequence> none;
  return none;
}

void Level::convertTexture(ByteTexture& tex, Palette& pal, DWordTexture& dst)
{
  for(int y = 0; y < 256; y++)
  {
    for(int x = 0; x < 256; x++)
    {
      const auto col = tex.pixels[y][x];

      if(col > 0)
        dst.pixels[y][x] = {pal.colors[col].r, pal.colors[col].g, pal.colors[col].b, 255};
      else
        dst.pixels[y][x] = {0, 0, 0, 0};
    }
  }

  dst.md5 = util::md5(&tex.pixels[0][0], 256 * 256);
}

void Level::convertTexture(WordTexture& tex, DWordTexture& dst)
{
  for(int y = 0; y < 256; y++)
  {
    for(int x = 0; x < 256; x++)
    {
      const int col = tex.pixels[y][x];

      if((col & 0x8000) != 0)
      {
        const auto r = static_cast<uint8_t>((col & 0x00007c00) >> 7);
        const auto g = static_cast<uint8_t>((col & 0x000003e0) >> 2);
        const auto b = static_cast<uint8_t>((col & 0x0000001f) << 3);
        dst.pixels[y][x] = {r, g, b, 1};
      }
      else
      {
        dst.pixels[y][x] = {0, 0, 0, 0};
      }
    }
  }
}

void Level::updateRoomBasedCaches()
{
  for(Room& room : m_rooms)
  {
    for(Sector& sector : room.sectors)
    {
      sector.updateCaches(m_rooms, m_boxes, m_floorData);
    }
  }
}

void Level::postProcessDataStructures()
{
  BOOST_LOG_TRIVIAL(info) << "Post-processing data structures";

  updateRoomBasedCaches();

  Expects(m_baseZones.flyZone.size() == m_boxes.size());
  Expects(m_baseZones.groundZone1.size() == m_boxes.size());
  Expects(m_baseZones.groundZone2.size() == m_boxes.size());
  Expects(m_alternateZones.flyZone.size() == m_boxes.size());
  Expects(m_alternateZones.groundZone1.size() == m_boxes.size());
  Expects(m_alternateZones.groundZone2.size() == m_boxes.size());
  for(size_t i = 0; i < m_boxes.size(); ++i)
  {
    m_boxes[i].zoneFly = m_baseZones.flyZone[i];
    m_boxes[i].zoneGround1 = m_baseZones.groundZone1[i];
    m_boxes[i].zoneGround2 = m_baseZones.groundZone2[i];
    m_boxes[i].zoneFlySwapped = m_alternateZones.flyZone[i];
    m_boxes[i].zoneGround1Swapped = m_alternateZones.groundZone1[i];
    m_boxes[i].zoneGround2Swapped = m_alternateZones.groundZone2[i];
  }

  for(const std::unique_ptr<SkeletalModelType>& model : m_animatedModels | boost::adaptors::map_values)
  {
    if(model->pose_data_offset.index<decltype(m_poseFrames[0])>() >= m_poseFrames.size())
    {
      BOOST_LOG_TRIVIAL(warning) << "Pose frame data index "
                                 << model->pose_data_offset.index<decltype(m_poseFrames[0])>() << " out of range 0.."
                                 << m_poseFrames.size() - 1;
      continue;
    }
    model->frames = reinterpret_cast<const AnimFrame*>(&model->pose_data_offset.from(m_poseFrames));
    if(model->nMeshes > 1)
    {
      model->boneTree = gsl::make_span(reinterpret_cast<const BoneTreeEntry*>(&model->bone_index.from(m_boneTrees)),
                                       model->nMeshes - 1);
    }

    if(model->animation_index.index != 0xffff)
      model->animations = &model->animation_index.checkedFrom(m_animations);
  }

  for(Animation& anim : m_animations)
  {
    if(anim.poseDataOffset.index<decltype(m_poseFrames[0])>() >= m_poseFrames.size())
    {
      BOOST_LOG_TRIVIAL(warning) << "Pose frame data index " << anim.poseDataOffset.index<decltype(m_poseFrames[0])>()
                                 << " out of range 0.." << m_poseFrames.size() - 1;
      anim.frames = nullptr;
    }
    else
    {
      anim.frames = reinterpret_cast<const AnimFrame*>(&anim.poseDataOffset.from(m_poseFrames));
    }

    Expects(anim.nextAnimationIndex < m_animations.size());
    anim.nextAnimation = &m_animations[anim.nextAnimationIndex];

    Expects(gsl::narrow<size_t>(anim.animCommandIndex + anim.animCommandCount) <= m_animCommands.size());
    Expects(gsl::narrow<size_t>(anim.transitionsIndex + anim.transitionsCount) <= m_transitions.size());
    if(anim.transitionsCount > 0)
      anim.transitions = gsl::make_span(&anim.transitionsIndex.checkedFrom(m_transitions), anim.transitionsCount);
  }

  for(TransitionCase& transitionCase : m_transitionCases)
  {
    if(transitionCase.targetAnimationIndex.index < m_animations.size())
      transitionCase.targetAnimation = &transitionCase.targetAnimationIndex.from(m_animations);
    else
      BOOST_LOG_TRIVIAL(warning) << "Animation index " << transitionCase.targetAnimationIndex.index << " not less than "
                                 << m_animations.size();
  }

  for(Transitions& transition : m_transitions)
  {
    Expects(gsl::narrow<size_t>(transition.firstTransitionCase + transition.transitionCaseCount)
            <= m_transitionCases.size());
    if(transition.transitionCaseCount > 0)
      transition.transitionCases
        = gsl::make_span(&transition.firstTransitionCase.from(m_transitionCases), transition.transitionCaseCount);
  }

  for(const auto& sequence : m_spriteSequences | boost::adaptors::map_values)
  {
    Expects(sequence != nullptr);
    Expects(sequence->length <= 0);
    Expects(gsl::narrow<size_t>(sequence->offset - sequence->length) <= m_sprites.size());
    sequence->sprites = gsl::make_span(&m_sprites[sequence->offset], -sequence->length);
  }
}
