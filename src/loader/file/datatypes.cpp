#include "datatypes.h"

#include "engine/engine.h"
#include "render/textureanimator.h"
#include "util/helpers.h"
#include "level/level.h"
#include "render/scene/names.h"
#include "render/scene/MeshPart.h"
#include "render/scene/Material.h"
#include "render/scene/Sprite.h"
#include "render/gl/vertexarray.h"

#include <glm/gtc/type_ptr.hpp>

#include <boost/range/adaptors.hpp>

namespace loader
{
namespace file
{
namespace
{
#pragma pack(push, 1)


struct RenderVertex
{
    glm::vec3 position;
    glm::vec4 color;
    glm::vec3 normal{0.0f};

    static const render::gl::StructuredVertexBuffer::AttributeMapping& getFormat()
    {
        static const render::gl::StructuredVertexBuffer::AttributeMapping attribs{
                {VERTEX_ATTRIBUTE_POSITION_NAME, render::gl::VertexAttribute{&RenderVertex::position}},
                {VERTEX_ATTRIBUTE_NORMAL_NAME,   render::gl::VertexAttribute{&RenderVertex::normal}},
                {VERTEX_ATTRIBUTE_COLOR_NAME,    render::gl::VertexAttribute{&RenderVertex::color}}
        };

        return attribs;
    }
};


#pragma pack(pop)

struct MeshPart
{
    using IndexBuffer = std::vector<uint16_t>;
    static_assert( std::is_unsigned<IndexBuffer::value_type>::value, "Index buffer entries must be unsigned" );

    IndexBuffer indices;
    std::shared_ptr<render::scene::Material> material;
};


struct RenderModel
{
    std::vector<MeshPart> m_parts;

    std::shared_ptr<render::scene::Model> toModel(const gsl::not_null<std::shared_ptr<render::scene::Mesh>>& mesh)
    {
        for( const MeshPart& localPart : m_parts )
        {
#ifndef NDEBUG
            for( auto idx : localPart.indices )
            {
                BOOST_ASSERT( idx < mesh->getBuffers()[0]->getVertexCount() );
            }
#endif
            render::gl::VertexArrayBuilder builder;

            auto indexBuffer = std::make_shared<render::gl::IndexBuffer>();
            indexBuffer->setData( localPart.indices, true );
            builder.attach( indexBuffer );

            builder.attach( mesh->getBuffers() );

            auto part = std::make_shared<render::scene::MeshPart>(
                    builder.build( localPart.material->getShaderProgram()->getHandle() ) );
            part->setMaterial( localPart.material );
            mesh->addPart( part );
        }

        auto model = std::make_shared<render::scene::Model>();
        model->addMesh( mesh );

        return model;
    }
};
}

void Room::createSceneNode(
        const size_t roomId,
        const level::Level& level,
        const std::map<TextureKey, gsl::not_null<std::shared_ptr<render::scene::Material>>

        >& materials,
        const std::map<TextureKey, gsl::not_null<std::shared_ptr<render::scene::Material>>>& waterMaterials,
        const std::vector<gsl::not_null<std::shared_ptr<render::scene::Model>>>& staticMeshes,
        render::TextureAnimator& animator,
        const std::shared_ptr<render::scene::Material>& spriteMaterial
)
{
    RenderModel renderModel;
    std::map<TextureKey, size_t> texBuffers;
    std::vector<RenderVertex> vbuf;
    std::vector<glm::vec2> uvCoords;
    auto mesh = std::make_shared<render::scene::Mesh>( RenderVertex::getFormat(), false,
                                                       "Room:" + std::to_string( roomId ) );

    for(
        const QuadFace& quad
            : rectangles )
    {
        const TextureLayoutProxy& proxy = level.m_textureProxies.at( quad.proxyId.get() );

        if( texBuffers.find( proxy.textureKey ) == texBuffers.end() )
        {
            texBuffers[proxy.textureKey] = renderModel.m_parts.size();

            renderModel.m_parts.emplace_back();

            auto it = isWaterRoom() ? waterMaterials.at( proxy.textureKey ) : materials.at( proxy.textureKey );
            renderModel.m_parts.back().material = it;
        }
        const auto partId = texBuffers[proxy.textureKey];

        const auto firstVertex = vbuf.size();
        for( int i = 0; i < 4; ++i )
        {
            RenderVertex iv;
            iv.position = quad.vertices[i].from( vertices ).position.toRenderSystem();
            iv.color = quad.vertices[i].from( vertices ).color;
            uvCoords.push_back( proxy.uvCoordinates[i].toGl() );
            vbuf.push_back( iv );
        }

        animator.registerVertex( quad.proxyId, mesh, 0, firstVertex + 0 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 0 )
        );
        animator.registerVertex( quad.proxyId, mesh, 1, firstVertex + 1 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 1 )
        );
        animator.registerVertex( quad.proxyId, mesh, 2, firstVertex + 2 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 2 )
        );
        animator.registerVertex( quad.proxyId, mesh, 0, firstVertex + 0 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 0 )
        );
        animator.registerVertex( quad.proxyId, mesh, 2, firstVertex + 2 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 2 )
        );
        animator.registerVertex( quad.proxyId, mesh, 3, firstVertex + 3 );
        renderModel.m_parts[partId].indices.emplace_back(
                gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 3 )
        );
    }
    for( const Triangle& tri : triangles )
    {
        const TextureLayoutProxy& proxy = level.m_textureProxies.at( tri.proxyId.get() );

        if( texBuffers.find( proxy.textureKey ) == texBuffers.end() )
        {
            texBuffers[proxy.textureKey] = renderModel.m_parts.size();

            renderModel.m_parts.emplace_back();

            auto it = isWaterRoom() ? waterMaterials.at( proxy.textureKey ) : materials.at( proxy.textureKey );
            renderModel.m_parts.back().material = it;
        }
        const auto partId = texBuffers[proxy.textureKey];

        const auto firstVertex = vbuf.size();
        for( int i = 0; i < 3; ++i )
        {
            RenderVertex iv;
            iv.position = tri.vertices[i].from( vertices ).position.toRenderSystem();
            iv.color = tri.vertices[i].from( vertices ).color;
            uvCoords.push_back( proxy.uvCoordinates[i].toGl() );
            vbuf.push_back( iv );
        }

        animator.registerVertex( tri.proxyId, mesh, 0, firstVertex + 0 );
        renderModel.m_parts[partId].indices
                                   .emplace_back( gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 0 ) );
        animator.registerVertex( tri.proxyId, mesh, 1, firstVertex + 1 );
        renderModel.m_parts[partId].indices
                                   .emplace_back( gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 1 ) );
        animator.registerVertex( tri.proxyId, mesh, 2, firstVertex + 2 );
        renderModel.m_parts[partId].indices
                                   .emplace_back( gsl::narrow<MeshPart::IndexBuffer::value_type>( firstVertex + 2 ) );
    }

    mesh->getBuffers()[0]->assign( vbuf );

    static const render::gl::StructuredVertexBuffer::AttributeMapping attribs{
            {VERTEX_ATTRIBUTE_TEXCOORD_PREFIX_NAME, render::gl::VertexAttribute{
                    render::gl::VertexAttribute::SingleAttribute<glm::vec2>{}}}
    };

    mesh->addBuffer( attribs, true );
    mesh->getBuffers()[1]->assign( uvCoords );

    auto resModel = renderModel.toModel( mesh );
    resModel->getRenderState().setCullFace( true );
    resModel->getRenderState().setCullFaceSide( GL_BACK );

    node = std::make_shared<render::scene::Node>( "Room:" + std::to_string( roomId ) );
    node->setDrawable( resModel );
    node->addMaterialParameterSetter( "u_lightPosition", [](const render::scene::Node& /*node*/,
                                                            render::gl::Program::ActiveUniform& uniform
    ) {
        uniform.set( glm::vec3{0.0f} );
    } );
    node->addMaterialParameterSetter( "u_baseLight", [](const render::scene::Node& /*node*/,
                                                        render::gl::Program::ActiveUniform& uniform
    ) {
        uniform.set( 1.0f );
    } );
    node->addMaterialParameterSetter( "u_baseLightDiff", [](const render::scene::Node& /*node*/,
                                                            render::gl::Program::ActiveUniform& uniform
    ) {
        uniform.set( 1.0f );
    } );

    for(
        const RoomStaticMesh& sm
            : this->staticMeshes )
    {
        const auto idx = level.findStaticMeshIndexById( sm.meshId );
        if( idx < 0 )
            continue;

        BOOST_ASSERT( static_cast<size_t>(idx) < staticMeshes.size() );
        auto subNode = std::make_shared<render::scene::Node>( "staticMesh" );
        subNode->setDrawable( staticMeshes[idx].get() );
        subNode->setLocalMatrix( translate( glm::mat4{1.0f}, (sm.position - position).toRenderSystem() )
                                 * rotate( glm::mat4{1.0f}, sm.rotation.toRad(), glm::vec3{0, -1, 0} ) );

        subNode->addMaterialParameterSetter( "u_baseLight",
                                             [brightness = sm.getBrightness()](const render::scene::Node& /*node*/,
                                                                               render::gl::Program::ActiveUniform& uniform) {
                                                 uniform.set( brightness );
                                             } );
        subNode->addMaterialParameterSetter( "u_baseLightDiff", [](const render::scene::Node& /*node*/,
                                                                   render::gl::Program::ActiveUniform& uniform
        ) {
            uniform.set( 0.0f );
        } );
        subNode->addMaterialParameterSetter( "u_lightPosition", [](const render::scene::Node& /*node*/,
                                                                   render::gl::Program::ActiveUniform& uniform
        ) {
            uniform.set( glm::vec3{std::numeric_limits<float>::quiet_NaN()} );
        } );
        addChild( node, subNode );
    }
    node->setLocalMatrix( translate( glm::mat4{1.0f}, position.toRenderSystem() )
    );

    for( const SpriteInstance& spriteInstance : sprites )
    {
        BOOST_ASSERT( spriteInstance.vertex.get() < vertices.size() );

        const Sprite& sprite = level.m_sprites.at( spriteInstance.id.get() );

        const auto model = std::make_shared<render::scene::Sprite>( sprite.x0, -sprite.y0,
                                                                    sprite.x1, -sprite.y1,
                                                                    sprite.t0, sprite.t1,
                                                                    spriteMaterial,
                                                                    render::scene::Sprite::Axis::Y );

        auto spriteNode = std::make_shared<render::scene::Node>( "sprite" );
        spriteNode->setDrawable( model );
        const RoomVertex& v = vertices.at( spriteInstance.vertex.get() );
        spriteNode->setLocalMatrix( translate( glm::mat4{1.0f}, v.position.toRenderSystem() ) );
        spriteNode->addMaterialParameterSetter( "u_diffuseTexture",
                                                [texture = sprite.texture](const render::scene::Node & /*node*/,
                render::gl::Program::ActiveUniform & uniform
        ) { uniform.set( *texture ); } );
        spriteNode->addMaterialParameterSetter( "u_baseLight",
                                                [brightness = v.getBrightness()](const render::scene::Node& /*node*/,
                                                                                 render::gl::Program::ActiveUniform& uniform
                                                ) { uniform.set( brightness ); } );

        addChild( node, spriteNode );
    }
}

core::BoundingBox StaticMesh::getCollisionBox(const core::TRVec& pos, const core::Angle angle) const
{
    auto result = collision_box;

    const auto axis = axisFromAngle( angle, 45_deg );
    switch( *axis )
    {
        case core::Axis::PosZ:
            // nothing to do
            break;
        case core::Axis::PosX:
            result.min.X = collision_box.min.Z;
            result.max.X = collision_box.max.Z;
            result.min.Z = -collision_box.max.X;
            result.max.Z = -collision_box.min.X;
            break;
        case core::Axis::NegZ:
            result.min.X = -collision_box.max.X;
            result.max.X = -collision_box.min.X;
            result.min.Z = -collision_box.max.Z;
            result.max.Z = -collision_box.min.Z;
            break;
        case core::Axis::NegX:
            result.min.X = -collision_box.max.Z;
            result.max.X = -collision_box.min.Z;
            result.min.Z = collision_box.min.X;
            result.max.Z = collision_box.max.X;
            break;
    }

    result.min += pos;
    result.max += pos;
    return result;
}

void Room::patchHeightsForBlock(const engine::items::ItemNode& item, const core::Length height)
{
    auto room = item.m_state.position.room;
    //! @todo Ugly const_cast
    auto groundSector = const_cast<Sector*>(loader::file::findRealFloorSector( item.m_state.position.position, &room ));
    BOOST_ASSERT( groundSector != nullptr );
    const auto topSector = loader::file::findRealFloorSector(
            item.m_state.position.position + core::TRVec{0_len, height - core::SectorSize, 0_len}, &room );

    if( groundSector->floorHeight == -core::HeightLimit )
    {
        groundSector->floorHeight = topSector->ceilingHeight + height;
    }
    else
    {
        groundSector->floorHeight = topSector->floorHeight + height;
        if( groundSector->floorHeight == topSector->ceilingHeight )
            groundSector->floorHeight = -core::HeightLimit;
    }

    Expects( groundSector->box != nullptr );

    if( !groundSector->box->isBlockable() )
        return;

    if( height >= 0_len )
        groundSector->box->unblock();
    else
        groundSector->box->block();
}

const Sector* findRealFloorSector(const core::TRVec& position, const gsl::not_null<gsl::not_null<const Room*>*>& room)
{
    const Sector* sector;
    // follow portals
    while( true )
    {
        sector = (*room)->findFloorSectorWithClampedIndex( (position.X - (*room)->position.X) / core::SectorSize,
                                                           (position.Z - (*room)->position.Z) / core::SectorSize );
        if( sector->portalTarget == nullptr )
        {
            break;
        }

        *room = sector->portalTarget;
    }

    // go up/down until we are in the room that contains our coordinates
    Expects( sector != nullptr );
    if( sector->floorHeight > position.Y )
    {
        while( sector->ceilingHeight > position.Y && sector->roomAbove != nullptr )
        {
            *room = sector->roomAbove;
            sector = (*room)->getSectorByAbsolutePosition( position );
            if( sector == nullptr )
                return nullptr;
        }
    }
    else
    {
        while( sector->floorHeight < position.Y && sector->roomBelow != nullptr )
        {
            *room = sector->roomBelow;
            sector = (*room)->getSectorByAbsolutePosition( position );
            if( sector == nullptr )
                return nullptr;
        }
    }

    return sector;
}
}
}
