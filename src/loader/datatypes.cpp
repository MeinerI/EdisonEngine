#include "datatypes.h"

#include "level/level.h"
#include "render/textureanimator.h"
#include "util/vmath.h"

#include <boost/lexical_cast.hpp>
#include <boost/range/adaptors.hpp>

#include <osg/Image>
#include <osg/Billboard>
#include <osg/LightSource>

namespace loader
{

namespace
{
GLuint addVertex(gsl::not_null<osg::ref_ptr<osg::Geometry>> geom, uint16_t vertexIndex, const UVCoordinates& uvCoordinates, const std::vector<RoomVertex>& vertices)
{
    Expects(geom->getVertexArray()->getType() == osg::Array::Vec3ArrayType);
    auto posArray = static_cast<osg::Vec3Array*>(geom->getVertexArray());
    Expects(geom->getNormalArray()->getType() == osg::Array::Vec3ArrayType);
    auto normalsArray = static_cast<osg::Vec3Array*>(geom->getNormalArray());
    Expects(geom->getTexCoordArray(0)->getType() == osg::Array::Vec2ArrayType);
    auto uvArray = static_cast<osg::Vec2Array*>(geom->getTexCoordArray(0));
    Expects(geom->getColorArray()->getType() == osg::Array::Vec4ArrayType);
    auto colorArray = static_cast<osg::Vec4Array*>(geom->getColorArray());
    Expects(geom->getPrimitiveSet(0)->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType);
    auto idxArray = static_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));

    BOOST_ASSERT(vertexIndex < vertices.size());
    const GLuint geometryVertexIdx = gsl::narrow<GLuint>(geom->getVertexArray()->getDataSize());
    posArray->push_back( vertices[vertexIndex].vertex.toIrrlicht() );
    // TR5 only: iv.Normal = vertices[vertexIndex].normal.toIrrlicht();
    normalsArray->push_back( { 1,0,0 } );
    uvArray->push_back({
        uvCoordinates.xpixel / 255.0f,
        uvCoordinates.ypixel / 255.0f
    });
    colorArray->push_back(vertices[vertexIndex].color);
    idxArray->push_back(geometryVertexIdx);
    return geometryVertexIdx;
}
} // anonymous namespace

std::shared_ptr<render::Entity> Room::createSceneNode(int roomId, const level::Level& level, const loader::TextureLayoutProxy::MaterialMap& materials, const std::vector<osg::ref_ptr<osg::Texture2D>>& textures, const std::vector<osg::ref_ptr<osg::Geode>>& staticMeshes, render::TextureAnimator& animator)
{
    // texture => mesh buffer
    std::map<TextureLayoutProxy::TextureKey, osg::ref_ptr<osg::Geometry>> geometries;
    for(const QuadFace& quad : rectangles)
    {
        const TextureLayoutProxy& proxy = level.m_textureProxies.at(quad.proxyId);
        if(geometries.find(proxy.textureKey) == geometries.end())
        {
            auto geom = new osg::Geometry();
            geom->setVertexArray(new osg::Vec3Array());
            geom->setNormalArray(new osg::Vec3Array(), osg::Array::BIND_PER_VERTEX);
            geom->setColorArray(new osg::Vec4Array(), osg::Array::BIND_PER_VERTEX);
            geom->setTexCoordArray(0, new osg::Vec2Array(), osg::Array::BIND_PER_VERTEX);
            geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
            geometries[proxy.textureKey] = geom;
        }
        auto geom = geometries[proxy.textureKey];

        animator.registerVertex(quad.proxyId, geom, 0, addVertex(geom, quad.vertices[0], proxy.uvCoordinates[0], vertices));
        animator.registerVertex(quad.proxyId, geom, 1, addVertex(geom, quad.vertices[1], proxy.uvCoordinates[1], vertices));
        animator.registerVertex(quad.proxyId, geom, 2, addVertex(geom, quad.vertices[2], proxy.uvCoordinates[2], vertices));
        animator.registerVertex(quad.proxyId, geom, 0, addVertex(geom, quad.vertices[0], proxy.uvCoordinates[0], vertices));
        animator.registerVertex(quad.proxyId, geom, 2, addVertex(geom, quad.vertices[2], proxy.uvCoordinates[2], vertices));
        animator.registerVertex(quad.proxyId, geom, 3, addVertex(geom, quad.vertices[3], proxy.uvCoordinates[3], vertices));
    }
    for(const Triangle& poly : triangles)
    {
        const TextureLayoutProxy& proxy = level.m_textureProxies.at(poly.proxyId);
        if(geometries.find(proxy.textureKey) == geometries.end())
        {
            auto geom = new osg::Geometry();
            geom->setVertexArray(new osg::Vec3Array());
            geom->setNormalArray(new osg::Vec3Array(), osg::Array::BIND_PER_VERTEX);
            geom->setColorArray(new osg::Vec4Array(), osg::Array::BIND_PER_VERTEX);
            geom->setTexCoordArray(0, new osg::Vec2Array(), osg::Array::BIND_PER_VERTEX);
            geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
            geometries[proxy.textureKey] = geom;
        }
        auto geom = geometries[proxy.textureKey];

        for(int i = 0; i < 3; ++i)
            animator.registerVertex(poly.proxyId, geom, i, addVertex(geom, poly.vertices[i], proxy.uvCoordinates[i], vertices));
    }

    auto result = std::make_shared<render::Entity>();
    osg::ref_ptr<osg::Group> resultGrp{ new osg::Group() };
    for(auto& buffer : geometries)
    {
        auto it = materials.find(buffer.first);
        BOOST_ASSERT(it != materials.end());
        auto material = it->second;
        BOOST_LOG_TRIVIAL(debug) << "Darkness=" << darkness;
        if(isWaterRoom())
        {
            //! @fixme Clone me
            material->setMode(GL_FOG, osg::StateAttribute::ON);
        }
        buffer.second->setStateSet(material.get());
        result->addComponent(buffer.second.get());
    }

    for(Light& light : lights)
    {
        osg::ref_ptr<osg::LightSource> src{ new osg::LightSource() };

        light.node = new osg::Light();
        src->setLight(light.node);
        src->setLocalStateSetModes(osg::StateAttribute::ON);

        const auto pos = (light.position - position).toIrrlicht();
        switch(light.getLightType())
        {
            case LightType::Shadow:
                BOOST_LOG_TRIVIAL(debug) << "Light: Shadow";
                light.node->setPosition(osg::Vec4(pos, 1.0f));
                break;
            case LightType::Null:
            case LightType::Point:
                BOOST_LOG_TRIVIAL(debug) << "Light: Null/Point";
                light.node->setPosition(osg::Vec4(pos, 1.0f));
                break;
            case LightType::Spotlight:
                BOOST_LOG_TRIVIAL(debug) << "Light: Spot";
                light.node->setPosition(osg::Vec4(pos, 0.0f));
                break;
            case LightType::Sun:
                BOOST_LOG_TRIVIAL(debug) << "Light: Sun";
                light.node->setPosition(osg::Vec4(pos, 0.0f));
                break;
        }

        BOOST_LOG_TRIVIAL(debug) << "  - Position: " << light.position.X << "/" << light.position.Y << "/" << light.position.Z;
        BOOST_LOG_TRIVIAL(debug) << "  - Length: " << light.length;
        BOOST_LOG_TRIVIAL(debug) << "  - Color: " << light.color.a/255.0f << "/" << light.color.r/255.0f << "/" << light.color.g/255.0f << "/" << light.color.b/255.0f;
        BOOST_LOG_TRIVIAL(debug) << "  - Specular Fade: " << light.specularFade;
        BOOST_LOG_TRIVIAL(debug) << "  - Specular Intensity: " << light.specularIntensity;
        BOOST_LOG_TRIVIAL(debug) << "  - Inner: " << light.r_inner;
        BOOST_LOG_TRIVIAL(debug) << "  - Outer: " << light.r_outer;
        BOOST_LOG_TRIVIAL(debug) << "  - Intensity: " << light.intensity;

        const auto f = std::abs(light.specularIntensity) / 8191.0f;
        BOOST_ASSERT(f >= 0 && f <= 1);
        light.node->setSpecular(osg::Vec4{ light.color.r / 255.0f*f, light.color.g / 255.0f*f, light.color.b / 255.0f*f, light.color.a / 255.0f*f });
        light.node->setDiffuse(light.node->getSpecular());
        light.node->setAmbient(light.node->getSpecular());
        light.node->setDirection(light.dir.toIrrlicht());
        light.node->setSpotCutoff(light.specularFade * 2);
        light.node->setLinearAttenuation(1.0f / light.specularFade);
        result->addComponent(src.get());
    }

    for(const RoomStaticMesh& sm : this->staticMeshes)
    {
        auto idx = level.findStaticMeshIndexById(sm.meshId);
        BOOST_ASSERT(idx >= 0);
        BOOST_ASSERT(static_cast<size_t>(idx) < staticMeshes.size());
        auto smNode = std::make_shared<render::Entity>();
        smNode->addComponent(staticMeshes[idx].get());
        smNode->setRotation({0,util::auToDeg(sm.rotation),0});
        smNode->setPosition((sm.position - position).toIrrlicht());
        result->addChild(smNode);
    }
    result->setPosition(position.toIrrlicht());

    result->setName(("Room:" + boost::lexical_cast<std::string>(roomId)).c_str());

    for(const Sprite& sprite : sprites)
    {
        BOOST_ASSERT(sprite.vertex < vertices.size());
        BOOST_ASSERT(sprite.texture < level.m_spriteTextures.size());

        const SpriteTexture& tex = level.m_spriteTextures[sprite.texture];

        osg::ref_ptr<osg::Billboard> bb{ new osg::Billboard() };
        bb->setMode(osg::Billboard::AXIAL_ROT);
        bb->setAxis(osg::Y_AXIS);

        osg::ref_ptr<osg::Geometry> spriteGeometry = tex.buildGeometry(textures[tex.texture]);
        result->addComponent(spriteGeometry.get());
    }

    // resultNode->addShadowVolumeSceneNode();
    node = result;

    return result;
}

osg::ref_ptr<osg::Texture2D> DWordTexture::toTexture()
{
    osg::ref_ptr<osg::Image> img{new osg::Image()};
    img->setImage(256, 256, 1, GL_RGBA, GL_RGBA, GL_FLOAT, reinterpret_cast<unsigned char*>(&pixels[0][0]), osg::Image::AllocationMode::NO_DELETE);

    osg::ref_ptr<osg::Texture2D> tex{ new osg::Texture2D() };
    tex->setImage(img);
    return tex;
}

osg::BoundingBoxImpl<osg::Vec3i> StaticMesh::getCollisionBox(const core::TRCoordinates& pos, core::Angle angle) const
{
    auto result = collision_box;

    const auto axis = core::axisFromAngle(angle, 45_deg);
    switch(*axis)
    {
        case core::Axis::PosZ:
            // nothing to do
            break;
        case core::Axis::PosX:
            std::swap(result.xMin(), result.zMin());
            result.zMin() *= -1;
            std::swap(result.xMax(), result.zMax());
            result.zMax() *= -1;
            break;
        case core::Axis::NegZ:
            result.xMin() *= -1;
            result.zMin() *= -1;
            result.xMax() *= -1;
            result.zMax() *= -1;
            break;
        case core::Axis::NegX:
            std::swap(result.xMin(), result.zMin());
            result.xMin() *= -1;
            std::swap(result.xMax(), result.zMax());
            result.xMax() *= -1;
            break;
    }

    result._min = result._min + osg::Vec3i(pos.X, pos.Y, pos.Z);
    result._max = result._max + osg::Vec3i(pos.X, pos.Y, pos.Z);
    //! @fixme result.repair();
    return result;
}

void Room::patchHeightsForBlock(const engine::ItemController& ctrl, int height)
{
    core::RoomBoundPosition pos = ctrl.getRoomBoundPosition();
    //! @todo Ugly const_cast
    auto groundSector = const_cast<loader::Sector*>(ctrl.getLevel().findFloorSectorWithClampedPosition(pos).get());
    pos.position.Y += height - core::SectorSize;
    const auto topSector = ctrl.getLevel().findFloorSectorWithClampedPosition(pos);

    const auto q = height / core::QuarterSectorSize;
    if(groundSector->floorHeight == -127)
    {
        groundSector->floorHeight = topSector->ceilingHeight + q;
    }
    else
    {
        groundSector->floorHeight += q;
        if(groundSector->floorHeight == topSector->ceilingHeight)
            groundSector->floorHeight = -127;
    }

    if(groundSector->boxIndex == 0xffff)
        return;

    //! @todo Ugly const_cast
    loader::Box& box = const_cast<loader::Box&>(ctrl.getLevel().m_boxes[groundSector->boxIndex]);
    if((box.overlap_index & 0x8000) == 0)
        return;

    if(height >= 0)
        box.overlap_index &= ~0x4000;
    else
        box.overlap_index |= 0x4000;
}

}
