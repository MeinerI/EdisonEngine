#pragma once

#include "loader/texture.h"

#include <gsl.h>
#include <boost/assert.hpp>
#include <osg/Geometry>

#include <deque>
#include <map>
#include <set>
#include <vector>

namespace render
{
    class TextureAnimator
    {
        std::vector<osg::ref_ptr<osg::Geometry>> m_meshBuffers;

        struct Sequence
        {
            struct VertexReference
            {
                const int bufferIndex;
                const int sourceIndex;
                size_t queueOffset = 0;

                VertexReference(int bufferIdx, int sourceIdx)
                    : bufferIndex(bufferIdx)
                      , sourceIndex(sourceIdx)
                {
                    Expects(sourceIdx >= 0 && sourceIdx < 4);
                }

                bool operator<(const VertexReference& rhs) const noexcept
                {
                    return bufferIndex < rhs.bufferIndex;
                }

                bool operator==(const VertexReference& rhs) const noexcept
                {
                    return bufferIndex == rhs.bufferIndex;
                }
            };

            std::vector<uint16_t> proxyIds;
            std::map<osg::ref_ptr<osg::Geometry>, std::set<VertexReference>> affectedVertices;

            void rotate()
            {
                BOOST_ASSERT(!proxyIds.empty());
                auto first = proxyIds.front();
                proxyIds.erase(proxyIds.begin(), std::next(proxyIds.begin()));
                proxyIds.emplace_back(first);
            }

            void registerVertex(gsl::not_null<osg::ref_ptr<osg::Geometry>> buffer, VertexReference vertex, uint16_t proxyId)
            {
                auto it = std::find(proxyIds.begin(), proxyIds.end(), proxyId);
                Expects(it != proxyIds.end());
                vertex.queueOffset = std::distance(proxyIds.begin(), it);
                affectedVertices[buffer].insert(vertex);
            }

            void updateCoordinates(const std::vector<loader::TextureLayoutProxy>& proxies)
            {
                BOOST_ASSERT(!proxyIds.empty());

                for( const auto& bufferAndVertices : affectedVertices )
                {
                    osg::ref_ptr<osg::Geometry> buffer = bufferAndVertices.first;
                    Expects(buffer->getTexCoordArray(0)->getType() == osg::Array::Vec2ArrayType);
                    const std::set<VertexReference>& vertices = bufferAndVertices.second;
                    for( const VertexReference& vref : vertices )
                    {
                        BOOST_ASSERT(vref.bufferIndex < buffer->getTexCoordArray(0)->getNumElements());
                        auto uvArray = static_cast<osg::Vec2Array*>(buffer->getTexCoordArray(0));

                        BOOST_ASSERT(vref.queueOffset < proxyIds.size());
                        const loader::TextureLayoutProxy& proxy = proxies[proxyIds[vref.queueOffset]];

                        uvArray->at(vref.bufferIndex).set(
                                                          proxy.uvCoordinates[vref.sourceIndex].xpixel / 255.0f,
                                                          proxy.uvCoordinates[vref.sourceIndex].ypixel / 255.0f
                                                         );
                    }
                }
            }
        };

        std::vector<Sequence> m_sequences;
        std::map<uint16_t, size_t> m_sequenceByProxyId;

    public:
        explicit TextureAnimator(const std::vector<uint16_t>& data)
        {
            /*
             * We have N rotating sequences, each consisting of M+1 proxy ids.
             */

            const uint16_t* ptr = data.data();
            const auto sequenceCount = *ptr++;

            for( size_t i = 0; i < sequenceCount; ++i )
            {
                Sequence sequence;
                const auto n = *ptr++;
                for( size_t j = 0; j <= n; ++j )
                {
                    BOOST_ASSERT(ptr <= &data.back());
                    const auto proxyId = *ptr++;
                    sequence.proxyIds.emplace_back(proxyId);
                    m_sequenceByProxyId.insert(std::make_pair(proxyId, m_sequences.size()));
                }
                m_sequences.emplace_back(std::move(sequence));
            }
        }

        void registerVertex(uint16_t proxyId, const gsl::not_null<osg::ref_ptr<osg::Geometry>>& geometry, int sourceIndex, int bufferIndex)
        {
            if( m_sequenceByProxyId.find(proxyId) == m_sequenceByProxyId.end() )
                return;

            const size_t sequenceId = m_sequenceByProxyId[proxyId];
            Expects(sequenceId < m_sequences.size());
            m_sequences[sequenceId].registerVertex(geometry, Sequence::VertexReference(bufferIndex, sourceIndex), proxyId);
        }

        void updateCoordinates(const std::vector<loader::TextureLayoutProxy>& proxies)
        {
            for( Sequence& sequence : m_sequences )
            {
                sequence.rotate();
                sequence.updateCoordinates(proxies);
            }
        }
    };
}
