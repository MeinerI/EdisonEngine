#pragma once

#include "loader/datatypes.h"


namespace render
{
    struct PortalTracer
    {
        osg::BoundingBox boundingBox{{-1, -1, 0}, {1, 1, 0}};
        const loader::Portal* lastPortal = nullptr;


        bool checkVisibility(const loader::Portal* portal, const osg::Camera& camera, const osg::Vec3& cameraPosition)
        {
            if( portal->normal.toIrrlicht() * (portal->vertices[0].toIrrlicht() - cameraPosition) >= 0 )
            {
                return false; // wrong orientation (normals must face the camera)
            }

            int numBehind = 0, numTooFar = 0;
            std::pair<osg::Vec3, bool> screen[4];

            osg::BoundingBox portalBB{{0, 0, 0}, {0, 0, 0}};
            portalBB._min = {1,1,0};
            portalBB._max = {-1,-1,0};
            BOOST_ASSERT(!portalBB.valid());
            {
                for( int i = 0; i < 4; ++i )
                {
                    screen[i] = projectOnScreen(portal->vertices[i].toIrrlicht(), camera, numBehind, numTooFar);
                    if( !screen[i].second )
                        continue;

                    portalBB.expandBy(screen[i].first.x(), screen[i].first.y(), 0);
                }
            }

            if( numBehind == 4 || numTooFar == 4 )
                return false;

            BOOST_ASSERT(portalBB.valid());

            if( numBehind == 0 )
            {
                boundingBox = boundingBox.intersect(portalBB);
                lastPortal = portal;

                return (boundingBox.xMax() - boundingBox.xMin()) > 0 && (boundingBox.yMax() - boundingBox.yMin()) > 0;
            }

            BOOST_ASSERT(numBehind >= 1 && numBehind <= 3);

            // consider everything is visible if the camera is in the midst of a portal

            lastPortal = portal;

            return (boundingBox.xMax() - boundingBox.xMin()) > 0 && (boundingBox.yMax() - boundingBox.yMin()) > 0;
        }


        uint16_t getLastDestinationRoom() const
        {
            return getLastPortal()->adjoining_room;
        }


        const loader::Portal* getLastPortal() const
        {
            Expects(lastPortal != nullptr);
            return lastPortal;
        }


    private:
        static std::pair<osg::Vec3, bool> projectOnScreen(osg::Vec3 vertex,
                                                          const osg::Camera& camera,
                                                          int& numBehind,
                                                          int& numTooFar)
        {
            const auto Near = 20;
            const auto Far = 20480;

            vertex = camera.getViewMatrix() * vertex;
            if( vertex.z() <= Near )
                ++numBehind;
            else if( vertex.z() > Far )
                ++numTooFar;

            osg::Vec4 tmp{vertex, 1.0f};
            tmp = camera.getProjectionMatrix() * tmp;

            osg::Vec3 screen{tmp.x() / tmp.w(), tmp.y() / tmp.w(), vertex.z()};
            return {screen, vertex.z() > Near};
        }
    };
}
