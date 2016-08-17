#pragma once

#include "core/angle.h"

#include <gsl.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osgAnimation/Animation>
#include <osgAnimation/RigGeometry>

#include <memory>


namespace engine
{
    class ItemController;
}


namespace render
{
    class Entity : public std::enable_shared_from_this<Entity>
    {
    private:
        std::weak_ptr<Entity> m_parent;
        std::set<std::shared_ptr<Entity>> m_children;

        osg::ref_ptr<osg::PositionAttitudeTransform> m_transform = new osg::PositionAttitudeTransform();
        osg::ref_ptr<osg::Geode> m_geode = new osg::Geode();

        std::shared_ptr<engine::ItemController> m_controller = nullptr;

    public:
        explicit Entity()
        {
            m_transform->addChild(m_geode);
        }


        explicit Entity(const std::weak_ptr<Entity>& parent)
            : m_parent{parent}
        {
            m_transform->addChild(m_geode);

            if( !m_parent.expired() )
            {
                auto p = m_parent.lock();
                p->addChild(shared_from_this());
            }
        }


        virtual ~Entity() = default;


        osg::Group* getGroup() const noexcept
        {
            return m_transform->asGroup();
        }


        void setName(const std::string& name)
        {
            m_geode->setName(name);
            m_transform->setName(name);
        }


        void addChild(const std::shared_ptr<Entity>& newChild)
        {
            if( !newChild->m_parent.expired() )
            {
                newChild->m_parent.lock()->removeChild(newChild);
            }

            m_children.insert(newChild);

            if( !newChild->m_parent.expired() )
            {
                newChild->m_parent = shared_from_this();
                m_transform->addChild(newChild->m_transform);
            }
        }


        void removeChild(const std::shared_ptr<Entity>& child)
        {
            m_children.erase(child);
            m_transform->removeChild(child->m_transform);
        }


        void setParent(const std::shared_ptr<Entity>& newParent)
        {
            if( !m_parent.expired() )
                m_parent.lock()->removeChild(shared_from_this());

            m_parent = newParent;

            if( !m_parent.expired() )
                m_parent.lock()->addChild(shared_from_this());
        }


        void addDrawable(const gsl::not_null<osg::Drawable*>& drawable)
        {
            m_geode->addDrawable(drawable);
        }


        void setPosition(const osg::Vec3& position)
        {
            m_transform->setPosition(position);
        }


        void setRotation(const osg::Vec3& rotation)
        {
            const auto q = core::xyzToQuat(rotation);
            m_transform->setAttitude(q);
        }


        void setRotation(float y)
        {
            setRotation({0, y, 0});
        }


        const std::string& getName() const
        {
            return m_transform->getName();
        }


        void setVisible(bool visible)
        {
            m_geode->setNodeMask(visible ? osg::Node::NodeMask(0) : ~osg::Node::NodeMask(0));
        }
    };
}
