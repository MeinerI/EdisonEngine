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


    struct Animation
    {
        osg::ref_ptr<osgAnimation::Animation> animation = new osgAnimation::Animation();


        explicit Animation() = default;


        void freeze(int frames)
        {
            animation->setPlayMode(osgAnimation::Animation::PlayMode::ONCE);
            animation->setDuration(static_cast<double>(frames) / core::FrameRate);
        }


        void setFrame(int frame)
        {
            animation->update(static_cast<double>(frame) / core::FrameRate);
        }


        void setTime(double time)
        {
            animation->update(time);
        }
    };


    class Bone final
    {
        gsl::not_null<osg::ref_ptr<osgAnimation::Bone>> m_bone;
        const osg::ref_ptr<osgAnimation::QuatSphericalLinearChannel> m_rotation = new osgAnimation::QuatSphericalLinearChannel();
        const osg::ref_ptr<osgAnimation::Vec3LinearChannel> m_translation = new osgAnimation::Vec3LinearChannel();

    public:
        Bone(Animation& anim, const std::string& name, const osg::Vec3& initialPosition, const osg::Quat& initialRotation, gsl::not_null<osg::Group*> parent)
            : m_bone{new osgAnimation::Bone(name)}
        {
            Expects(dynamic_cast<osgAnimation::Skeleton*>(parent.get()) != nullptr || dynamic_cast<osgAnimation::Bone*>(parent.get()) != nullptr);

            parent->addChild(m_bone.get());

            osg::ref_ptr<osgAnimation::UpdateBone> updateCallback = new osgAnimation::UpdateBone();

            osg::ref_ptr<osgAnimation::StackedQuaternionElement> quatTransform = new osgAnimation::StackedQuaternionElement(initialRotation);
            updateCallback->getStackedTransforms().push_back(quatTransform);
            m_rotation->setTarget(quatTransform->getOrCreateTarget());
            anim.animation->addChannel(m_rotation);

            osg::ref_ptr<osgAnimation::StackedTranslateElement> vecTransform = new osgAnimation::StackedTranslateElement(initialPosition);
            updateCallback->getStackedTransforms().push_back(vecTransform);
            m_translation->setTarget(vecTransform->getOrCreateTarget());
            anim.animation->addChannel(m_translation);

            m_bone->setUpdateCallback(updateCallback);
        }


        void addKeyframe(int frame, const osg::Quat& rot, const osg::Vec3& pos)
        {
            m_rotation->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::QuatKeyframe(static_cast<double>(frame) / core::FrameRate, rot));
            m_translation->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::Vec3Keyframe(static_cast<double>(frame) / core::FrameRate, pos));
        }


        const gsl::not_null<osg::ref_ptr<osgAnimation::Bone>>& getBone() const noexcept
        {
            return m_bone;
        }


        void setName(const std::string& name)
        {
            m_bone->setName(name);
        }


        void addChild(const Bone& bone)
        {
            m_bone->addChild(bone.getBone().get());
        }
    };


    class SkeletalMesh : public Entity
    {
        osg::ref_ptr<osgAnimation::Skeleton> m_skeleton = new osgAnimation::Skeleton();
        osg::ref_ptr<osgAnimation::RigGeometry> m_geometry = new osgAnimation::RigGeometry();
        std::vector<std::shared_ptr<Bone>> m_bones;
        std::vector<Animation> m_animations;

    public:
        explicit SkeletalMesh(const gsl::not_null<osg::ref_ptr<osg::Geometry>>& geometry)
        {
            m_geometry->setSkeleton(m_skeleton);
            m_geometry->setSourceGeometry(geometry.get());
            m_geometry->setRigTransformImplementation(new osgAnimation::RigTransformSoftware());
            m_geometry->setInfluenceMap(new osgAnimation::VertexInfluenceMap());
        }


        const std::shared_ptr<Bone>& getBone(size_t id) const
        {
            if( id >= m_bones.size() )
                return nullptr;

            return m_bones[id];
        }


        const std::shared_ptr<Bone>& createBone(Animation& animation, const std::string& name, const osg::Vec3& initialTranslation, const osg::Quat& initialRotation, Bone* parent)
        {
            osg::Group* realParent = parent ? parent->getBone().get() : nullptr;
            if( realParent == nullptr )
                realParent = m_skeleton;

            m_bones.emplace_back(std::make_shared<Bone>(
                animation,
                name,
                initialTranslation,
                initialRotation,
                realParent
            ));

            return m_bones.back();
        }


        Animation* createAnimation()
        {
            m_animations.emplace_back();
            return &m_animations.back();
        }


        gsl::not_null<osgAnimation::VertexInfluenceMap*> getInfluenceMap() const
        {
            return m_geometry->getInfluenceMap();
        }


        void setVertexWeight(const Bone& bone, int vertexIndex, float weight)
        {
            (*getInfluenceMap())[bone.getBone()->getName()].push_back(osgAnimation::VertexIndexWeight(vertexIndex, weight));
        }
    };
}
