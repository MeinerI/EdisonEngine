#include "meshes.h"

#include "render/textureanimator.h"
#include "primitives.h"

#include <gsl.h>

#include <boost/lexical_cast.hpp>
#include <boost/range/adaptors.hpp>

#include <osg/Geometry>
#include <osg/Geode>

namespace loader
{
    namespace
    {
        GLuint addVertex(const gsl::not_null<osg::ref_ptr<osg::Geometry>>& geom, uint16_t srcVertexIndex, const UVCoordinates* uvCoordinates, const std::vector<core::TRCoordinates>& vertices, const std::vector<core::TRCoordinates>& normals)
        {
            Expects(geom->getVertexArray()->getType() == osg::Array::Vec3ArrayType);
            auto posArray = static_cast<osg::Vec3Array*>(geom->getVertexArray());
            Expects(geom->getNormalArray()->getType() == osg::Array::Vec3ArrayType);
            auto normalsArray = static_cast<osg::Vec3Array*>(geom->getNormalArray());
            Expects(geom->getTexCoordArray(0)->getType() == osg::Array::Vec3ArrayType);
            auto uvArray = static_cast<osg::Vec2Array*>(geom->getTexCoordArray(0));
            Expects(geom->getPrimitiveSet(0)->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType);
            auto idxArray = static_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));

            BOOST_ASSERT(srcVertexIndex < vertices.size());
            const GLuint geometryVertexIdx = gsl::narrow<GLuint>(geom->getVertexArray()->getDataSize());
            posArray->push_back(vertices[srcVertexIndex].toIrrlicht());
            if( !normals.empty() )
                normalsArray->push_back( normals[srcVertexIndex].toIrrlicht() );
            else
                normalsArray->push_back( {0, 0, 1} );
            if( uvCoordinates != nullptr )
            {
                uvArray->push_back({ uvCoordinates->xpixel / 255.0f, uvCoordinates->ypixel / 255.0f });
            }
            idxArray->push_back(geometryVertexIdx);
            return geometryVertexIdx;
        }
    }

    osg::ref_ptr<osg::Geometry> Mesh::createMesh(const std::vector<TextureLayoutProxy>& textureProxies,
                                        const loader::TextureLayoutProxy::MaterialMap& materials,
                                        const std::vector<osg::ref_ptr<osg::StateSet>>& colorMaterials,
                                        render::TextureAnimator& animator) const
    {
        BOOST_ASSERT(colorMaterials.size() == 256);

        // texture => mesh buffer
        std::map<TextureLayoutProxy::TextureKey, osg::ref_ptr<osg::Geometry>> geometries;
        for( const QuadFace& quad : textured_rectangles )
        {
            const TextureLayoutProxy& proxy = textureProxies.at(quad.proxyId);
            if(geometries.find(proxy.textureKey) == geometries.end())
            {
                auto geom = new osg::Geometry();
                geom->setVertexArray(new osg::Vec3Array());
                geom->setNormalArray(new osg::Vec3Array(), osg::Array::BIND_PER_VERTEX);
                geom->setTexCoordArray(0, new osg::Vec2Array(), osg::Array::BIND_PER_VERTEX);
                geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
                geometries[proxy.textureKey] = geom;
            }
            auto geom = geometries[proxy.textureKey];

            animator.registerVertex(quad.proxyId, geom, 0, addVertex(geom, quad.vertices[0], &proxy.uvCoordinates[0], vertices, normals));
            animator.registerVertex(quad.proxyId, geom, 1, addVertex(geom, quad.vertices[1], &proxy.uvCoordinates[1], vertices, normals));
            animator.registerVertex(quad.proxyId, geom, 2, addVertex(geom, quad.vertices[2], &proxy.uvCoordinates[2], vertices, normals));
            animator.registerVertex(quad.proxyId, geom, 0, addVertex(geom, quad.vertices[0], &proxy.uvCoordinates[0], vertices, normals));
            animator.registerVertex(quad.proxyId, geom, 2, addVertex(geom, quad.vertices[2], &proxy.uvCoordinates[2], vertices, normals));
            animator.registerVertex(quad.proxyId, geom, 3, addVertex(geom, quad.vertices[3], &proxy.uvCoordinates[3], vertices, normals));
        }
        for( const QuadFace& quad : colored_rectangles )
        {
            TextureLayoutProxy::TextureKey tk;
            tk.blendingMode = BlendingMode::Solid;
            tk.flags = 0;
            tk.tileAndFlag = 0;
            tk.colorId = quad.proxyId & 0xff;

            if( geometries.find(tk) == geometries.end() )
            {
                auto geom = new osg::Geometry();
                geom->setVertexArray(new osg::Vec3Array());
                geom->setNormalArray(new osg::Vec3Array(), osg::Array::BIND_PER_VERTEX);
                geom->setTexCoordArray(0, new osg::Vec2Array(), osg::Array::BIND_PER_VERTEX);
                geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
                geometries[tk] = geom;
            }
            auto geom = geometries[tk];

            addVertex(geom, quad.vertices[0], nullptr, vertices, normals);
            addVertex(geom, quad.vertices[1], nullptr, vertices, normals);
            addVertex(geom, quad.vertices[2], nullptr, vertices, normals);
            addVertex(geom, quad.vertices[0], nullptr, vertices, normals);
            addVertex(geom, quad.vertices[2], nullptr, vertices, normals);
            addVertex(geom, quad.vertices[3], nullptr, vertices, normals);
        }
        for( const Triangle& poly : textured_triangles )
        {
            const TextureLayoutProxy& proxy = textureProxies.at(poly.proxyId);
            if( geometries.find(proxy.textureKey) == geometries.end() )
            {
                auto geom = new osg::Geometry();
                geom->setVertexArray(new osg::Vec3Array());
                geom->setNormalArray(new osg::Vec3Array());
                geom->setTexCoordArray(0, new osg::Vec2Array());
                geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0));
                geometries[proxy.textureKey] = geom;
            }
            auto geom = geometries[proxy.textureKey];

            for( int i = 0; i < 3; ++i )
                animator.registerVertex(poly.proxyId, geom, i, addVertex(geom, poly.vertices[i], &proxy.uvCoordinates[i], vertices, normals));
        }

        for( const Triangle& poly : colored_triangles )
        {
            TextureLayoutProxy::TextureKey tk;
            tk.blendingMode = BlendingMode::Solid;
            tk.flags = 0;
            tk.tileAndFlag = 0;
            tk.colorId = poly.proxyId & 0xff;

            if( geometries.find(tk) == geometries.end() )
            {
                auto geom = new osg::Geometry();
                geom->setVertexArray(new osg::Vec3Array());
                geom->setNormalArray(new osg::Vec3Array(), osg::Array::BIND_PER_VERTEX);
                geom->setTexCoordArray(0, new osg::Vec2Array(), osg::Array::BIND_PER_VERTEX);
                geom->addPrimitiveSet(new osg::DrawElementsUInt());
                geometries[tk] = geom;
            }
            auto buf = geometries[tk];

            for( int i = 0; i < 3; ++i )
                addVertex(buf, poly.vertices[i], nullptr, vertices, normals);
        }

        osg::ref_ptr<osg::Geode> result{new osg::Geode()};
        for( auto& geom : geometries )
        {
            auto it = materials.find(geom.first);
            if( it != materials.end() )
            {
                geom.second->setStateSet(it->second);
            }
            else if( geom.first.colorId >= 0 && geom.first.colorId <= 255 )
            {
                geom.second->setStateSet(colorMaterials[geom.first.colorId]);
            }
            else
            {
                BOOST_LOG_TRIVIAL(error) << "Invalid mesh material";
            }

            result->addDrawable(geom.second);
        }

        return result;
    }
}
