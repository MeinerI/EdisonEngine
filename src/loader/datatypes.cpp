#include "datatypes.h"

#include "level/level.h"
#include "render/textureanimator.h"
#include "util/vmath.h"

#include <glm/gtc/type_ptr.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/range/adaptors.hpp>


namespace loader
{
    namespace
    {
#pragma pack(push,1)
        struct RenderVertex
        {
            glm::vec3 position;
            glm::vec4 color;
            glm::vec3 normal{0.0f};


            static const gameplay::gl::StructuredVertexBuffer::AttributeMapping& getFormat()
            {
                static const gameplay::gl::StructuredVertexBuffer::AttributeMapping attribs{
                    { VERTEX_ATTRIBUTE_POSITION_NAME, gameplay::gl::VertexAttribute{ &RenderVertex::position } },
                    { VERTEX_ATTRIBUTE_NORMAL_NAME, gameplay::gl::VertexAttribute{ &RenderVertex::normal } },
                    { VERTEX_ATTRIBUTE_COLOR_NAME, gameplay::gl::VertexAttribute{ &RenderVertex::color } }
                };

                return attribs;
            }
        };
#pragma pack(pop)

        struct MeshPart
        {
            using IndexBuffer = std::vector<uint16_t>;
            static_assert(std::is_unsigned<IndexBuffer::value_type>::value, "Index buffer entries must be unsigned");

            IndexBuffer indices;
            std::shared_ptr<gameplay::Material> material;
        };


        struct RenderModel
        {
            std::vector<MeshPart> m_parts;


            std::shared_ptr<gameplay::Model> toModel(const gsl::not_null<std::shared_ptr<gameplay::Mesh>>& mesh)
            {
                for( const MeshPart& localPart : m_parts )
                {
#ifndef NDEBUG
                    for( auto idx : localPart.indices )
                    {
                        BOOST_ASSERT(idx < mesh->getBuffer(0)->getVertexCount());
                    }
#endif
                    gameplay::gl::VertexArrayBuilder builder;

                    auto indexBuffer = std::make_shared<gameplay::gl::IndexBuffer>();
                    indexBuffer->setData(localPart.indices, true);
                    builder.attach(indexBuffer);

                    builder.attach(mesh->getBuffers());

                    auto part = std::make_shared<gameplay::MeshPart>(builder.build(localPart.material->getShaderProgram()->getHandle()));
                    part->setMaterial(localPart.material);
                    mesh->addPart(part);
                }

                auto model = std::make_shared<gameplay::Model>();
                model->addMesh(mesh);

                return model;
            }
        };
    }


    std::shared_ptr<gameplay::Node> Room::createSceneNode(gameplay::Game* game,
                                                          size_t roomId,
                                                          const level::Level& level,
                                                          const std::vector<std::shared_ptr<gameplay::gl::Texture> >& textures,
                                                          const std::map<loader::TextureLayoutProxy::TextureKey, std::shared_ptr<gameplay::Material>>& materials,
                                                          const std::map<loader::TextureLayoutProxy::TextureKey, std::shared_ptr<gameplay::Material>>& waterMaterials,
                                                          const std::vector<std::shared_ptr<gameplay::Model>>& staticMeshes,
                                                          render::TextureAnimator& animator)
    {
        RenderModel renderModel;
        std::map<TextureLayoutProxy::TextureKey, size_t> texBuffers;
        std::vector<RenderVertex> vbuf;
        std::vector<glm::vec2> uvCoords;
        auto mesh = std::make_shared<gameplay::Mesh>(RenderVertex::getFormat(), false, "Room:" + boost::lexical_cast<std::string>(roomId));

        for( const QuadFace& quad : rectangles )
        {
            const TextureLayoutProxy& proxy = level.m_textureProxies.at(quad.proxyId);

            if( texBuffers.find(proxy.textureKey) == texBuffers.end() )
            {
                texBuffers[proxy.textureKey] = renderModel.m_parts.size();
                renderModel.m_parts.emplace_back();
                auto it = isWaterRoom() ? waterMaterials.find(proxy.textureKey) : materials.find(proxy.textureKey);
                Expects(it != (isWaterRoom() ? waterMaterials.end() : materials.end()));
                renderModel.m_parts.back().material = it->second;
            }
            const auto partId = texBuffers[proxy.textureKey];

            const auto firstVertex = vbuf.size();
            for( int i = 0; i < 4; ++i )
            {
                RenderVertex iv;
                iv.position = vertices[quad.vertices[i]].position.toRenderSystem();
                iv.color = vertices[quad.vertices[i]].color;
                uvCoords.push_back( proxy.uvCoordinates[i].toGl() );
                vbuf.push_back(iv);
            }

            animator.registerVertex(quad.proxyId, mesh, 0, firstVertex + 0);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 0));
            animator.registerVertex(quad.proxyId, mesh, 1, firstVertex + 1);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 1));
            animator.registerVertex(quad.proxyId, mesh, 2, firstVertex + 2);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 2));
            animator.registerVertex(quad.proxyId, mesh, 0, firstVertex + 0);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 0));
            animator.registerVertex(quad.proxyId, mesh, 2, firstVertex + 2);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 2));
            animator.registerVertex(quad.proxyId, mesh, 3, firstVertex + 3);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 3));
        }
        for( const Triangle& tri : triangles )
        {
            const TextureLayoutProxy& proxy = level.m_textureProxies.at(tri.proxyId);

            if( texBuffers.find(proxy.textureKey) == texBuffers.end() )
            {
                texBuffers[proxy.textureKey] = renderModel.m_parts.size();
                renderModel.m_parts.emplace_back();
                auto it = isWaterRoom() ? waterMaterials.find(proxy.textureKey) : materials.find(proxy.textureKey);
                Expects(it != (isWaterRoom() ? waterMaterials.end() : materials.end()));
                renderModel.m_parts.back().material = it->second;
            }
            const auto partId = texBuffers[proxy.textureKey];

            const auto firstVertex = vbuf.size();
            for( int i = 0; i < 3; ++i )
            {
                RenderVertex iv;
                iv.position = vertices[tri.vertices[i]].position.toRenderSystem();
                iv.color = vertices[tri.vertices[i]].color;
                uvCoords.push_back(proxy.uvCoordinates[i].toGl());
                vbuf.push_back(iv);
            }

            animator.registerVertex(tri.proxyId, mesh, 0, firstVertex + 0);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 0));
            animator.registerVertex(tri.proxyId, mesh, 1, firstVertex + 1);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 1));
            animator.registerVertex(tri.proxyId, mesh, 2, firstVertex + 2);
            renderModel.m_parts[partId].indices.emplace_back(gsl::narrow<uint16_t>(firstVertex + 2));
        }

        mesh->getBuffer(0)->assign(vbuf);

        static const gameplay::gl::StructuredVertexBuffer::AttributeMapping attribs{
            { VERTEX_ATTRIBUTE_TEXCOORD_PREFIX_NAME, gameplay::gl::VertexAttribute{ gameplay::gl::VertexAttribute::SingleAttribute<glm::vec2>{} } }
        };

        mesh->addBuffer(attribs, true);
        mesh->getBuffer(1)->assign(uvCoords);

        auto resModel = renderModel.toModel(mesh);
        node = std::make_shared<gameplay::Node>("Room:" + boost::lexical_cast<std::string>(roomId));
        node->setDrawable(resModel);
        node->addMaterialParameterSetter("u_lightPosition", [](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
        {
            uniform.set(glm::vec3{0.0f});
        });
        node->addMaterialParameterSetter("u_baseLight", [](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
        {
            uniform.set(1.0f);
        });
        node->addMaterialParameterSetter("u_baseLightDiff", [](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
        {
            uniform.set(1.0f);
        });

        for( const RoomStaticMesh& sm : this->staticMeshes )
        {
            auto idx = level.findStaticMeshIndexById(sm.meshId);
            BOOST_ASSERT(idx >= 0);
            BOOST_ASSERT(static_cast<size_t>(idx) < staticMeshes.size());
            auto subNode = std::make_shared<gameplay::Node>("");
            subNode->setDrawable(staticMeshes[idx]);
            subNode->setLocalMatrix(glm::translate(glm::mat4{1.0f}, (sm.position - position).toRenderSystem()) * glm::rotate(glm::mat4{1.0f}, util::auToRad(sm.rotation), glm::vec3{0,-1,0}));

            float brightness = 1 - (sm.darkness - 4096) / 8192.0f;

            subNode->addMaterialParameterSetter("u_baseLight", [brightness](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
            {
                uniform.set(brightness);
            });
            subNode->addMaterialParameterSetter("u_baseLightDiff", [](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
            {
                uniform.set(0.0f);
            });
            subNode->addMaterialParameterSetter("u_lightPosition", [](const gameplay::Node& /*node*/, gameplay::gl::Program::ActiveUniform& uniform)
            {
                uniform.set(glm::vec3{ std::numeric_limits<float>::quiet_NaN() });
            });
            node->addChild(subNode);
        }
        node->setLocalMatrix(glm::translate(glm::mat4{1.0f}, position.toRenderSystem()));

        for( const Sprite& sprite : sprites )
        {
            BOOST_ASSERT(sprite.vertex < vertices.size());
            BOOST_ASSERT(sprite.texture < level.m_spriteTextures.size());

            const SpriteTexture& tex = level.m_spriteTextures[sprite.texture];

            //! @fixme auto spriteNode = std::make_shared<gameplay::Sprite>(game, textures[tex.texture], tex.right_side - tex.left_side + 1, tex.bottom_side - tex.top_side + 1, tex.buildSourceRectangle(), 1, nullptr, "u_texture");
            // spriteNode->setBlendMode(gameplay::Sprite::BLEND_ADDITIVE);

            //auto n = std::make_shared<gameplay::Node>("");
            //n->setDrawable(spriteNode);
            //n->setLocalMatrix(glm::translate(glm::mat4{1.0f}, (vertices[sprite.vertex].position - core::TRCoordinates{0, tex.bottom_side / 2, 0}).toRenderSystem()));

            //node->addChild(n);
        }

        return node;
    }


    core::BoundingBox StaticMesh::getCollisionBox(const core::TRCoordinates& pos, core::Angle angle) const
    {
        auto result = collision_box;

        const auto axis = core::axisFromAngle(angle, 45_deg);
        switch( *axis )
        {
            case core::Axis::PosZ:
                // nothing to do
                break;
            case core::Axis::PosX:
                std::swap(result.min.x, result.min.z);
                result.min.z *= -1;
                std::swap(result.max.x, result.max.z);
                result.max.z *= -1;
                break;
            case core::Axis::NegZ:
                result.min.x *= -1;
                result.min.z *= -1;
                result.max.x *= -1;
                result.max.z *= -1;
                break;
            case core::Axis::NegX:
                std::swap(result.min.x, result.min.z);
                result.min.x *= -1;
                std::swap(result.max.x, result.max.z);
                result.max.x *= -1;
                break;
        }

        result.min += glm::vec3(pos.X, pos.Y, pos.Z);
        result.max += glm::vec3(pos.X, pos.Y, pos.Z);
        return result;
    }


    void Room::patchHeightsForBlock(const engine::items::ItemNode& ctrl, int height)
    {
        core::RoomBoundPosition pos = ctrl.getRoomBoundPosition();
        //! @todo Ugly const_cast
        auto groundSector = const_cast<loader::Sector*>(ctrl.getLevel().findRealFloorSector(pos).get());
        pos.position.Y += height - loader::SectorSize;
        const auto topSector = ctrl.getLevel().findRealFloorSector(pos);

        const auto q = height / loader::QuarterSectorSize;
        if( groundSector->floorHeight == -127 )
        {
            groundSector->floorHeight = topSector->ceilingHeight + q;
        }
        else
        {
            groundSector->floorHeight += q;
            if( groundSector->floorHeight == topSector->ceilingHeight )
                groundSector->floorHeight = -127;
        }

        if( groundSector->boxIndex == 0xffff )
            return;

        //! @todo Ugly const_cast
        loader::Box& box = const_cast<loader::Box&>(ctrl.getLevel().m_boxes[groundSector->boxIndex]);
        if( (box.overlap_index & 0x8000) == 0 )
            return;

        if( height >= 0 )
            box.overlap_index &= ~0x4000;
        else
            box.overlap_index |= 0x4000;
    }
}
