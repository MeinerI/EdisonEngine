#pragma once

#include "core/angle.h"
#include "core/magic.h"

#include <gsl.h>

#include <osg/PositionAttitudeTransform>
#include <osgAnimation/Animation>
#include <osgAnimation/Bone>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/RigTransformSoftware>
#include <osgAnimation/Skeleton>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/UpdateBone>

#include <memory>
#include <vector>


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
        osg::ref_ptr<osg::Group> m_group = new osg::Group();

        std::shared_ptr<engine::ItemController> m_controller = nullptr;

    public:
        explicit Entity()
        {
        }


        explicit Entity(const std::weak_ptr<Entity>& parent)
            : m_parent{parent}
        {
            m_parent.lock()->addChild(shared_from_this());
        }


        virtual ~Entity() = default;


        void setName(const std::string& name)
        {
            m_group->setName(name);
        }


        void setParent(const std::shared_ptr<Entity>& entity)
        {
            if( !m_parent.expired() )
                m_parent.lock()->m_children.erase(shared_from_this());

            m_parent = entity;

            if( entity != nullptr )
                entity->m_children.insert(shared_from_this());
        }


        void addChild(const std::shared_ptr<Entity>& child)
        {
            if( !child->m_parent.expired() )
                child->m_parent.lock()->m_children.erase(child);

            m_children.insert(child);
            child->m_parent = shared_from_this();
        }


        void addComponent(const gsl::not_null<osg::Node*>& drawable)
        {
            m_group->addChild(drawable);
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
            return m_group->getName();
        }


        void setVisible(bool visible)
        {
            m_group->setNodeMask(visible ? osg::Node::NodeMask(0) : ~osg::Node::NodeMask(0));
        }
    };
}
