#include "names.h"
#include "Mesh.h"
#include "MeshPart.h"
#include "Material.h"

#include <boost/log/trivial.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace render
{
namespace scene
{
gsl::not_null<std::shared_ptr<Mesh>> Mesh::createQuadFullscreen(const float width,
                                                                const float height,
                                                                const gl::Program& program,
                                                                const bool invertY)
{
    struct Vertex
    {
        glm::vec2 pos;

        glm::vec2 uv;
    };

    const Vertex vertices[]{
            {{0.0f,  0.0f},   {0.0f, invertY ? 0.0f : 1.0f}},
            {{width, 0.0f},   {1.0f, invertY ? 0.0f : 1.0f}},
            {{width, height}, {1.0f, invertY ? 1.0f : 0.0f}},
            {{0.0f,  height}, {0.0f, invertY ? 1.0f : 0.0f}}
    };

    static const gl::StructuredVertexBuffer::AttributeMapping attribs{
            {VERTEX_ATTRIBUTE_POSITION_NAME,        gl::VertexAttribute{&Vertex::pos}},
            {VERTEX_ATTRIBUTE_TEXCOORD_PREFIX_NAME, gl::VertexAttribute{&Vertex::uv}}
    };

    auto mesh = std::make_shared<Mesh>( attribs, false );
    mesh->getBuffers()[0]->assign<Vertex>( &vertices[0], 4 );

    static const uint16_t indices[6] =
            {
                    0, 1, 2,
                    0, 2, 3
            };

    gl::VertexArrayBuilder builder;

    auto indexBuffer = std::make_shared<gl::IndexBuffer>();
    indexBuffer->setData( gsl::not_null<const uint16_t*>( &indices[0] ), 6, false );
    builder.attach( indexBuffer );
    builder.attach( mesh->getBuffers() );

    const auto part = std::make_shared<MeshPart>( builder.build( program ) );
    mesh->addPart( part );

    return mesh;
}

void Mesh::draw(RenderContext& context)
{
    context.pushState( m_renderState );
    for( const auto& part : m_parts )
    {
        part->draw( context );
    }
    context.popState();
}
}
}
