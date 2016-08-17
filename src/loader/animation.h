#pragma once

#include "io/sdlreader.h"
#include "util/vmath.h"

#include <osgAnimation/Animation>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Bone>
#include <osgAnimation/RigTransformSoftware>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/Timeline>
#include <osgAnimation/UpdateBone>

#include <gsl.h>
#include <boost/lexical_cast.hpp>

#include <map>
#include <stack>


namespace loader
{
    /** \brief animation->
    *
    * This describes each individual animation; these may be looped by specifying
    * the next animation to be itself. In TR2 and TR3, one must be careful when
    * parsing frames using the FrameSize value as the size of each frame, since
    * an animation's frame range may extend into the next animation's frame range,
    * and that may have a different FrameSize value.
    */
    struct Animation
    {
        uint32_t poseDataOffset; // byte offset into Frames[] (divide by 2 for Frames[i])
        uint8_t stretchFactor; // Slowdown factor of this animation
        uint8_t poseDataSize; // number of bit16's in Frames[] used by this animation
        uint16_t state_id;

        int32_t speed;
        int32_t accelleration;

        int32_t lateralSpeed; // new in TR4 -->
        int32_t lateralAccelleration; // lateral speed and acceleration.

        uint16_t firstFrame; // first frame in this animation
        uint16_t lastFrame; // last frame in this animation (numframes = (End - Start) + 1)
        uint16_t nextAnimation;
        uint16_t nextFrame;

        uint16_t transitionsCount;
        uint16_t transitionsIndex; // offset into StateChanges[]
        uint16_t animCommandCount; // How many of them to use.
        uint16_t animCommandIndex; // offset into AnimCommand[]

        constexpr size_t getKeyframeCount() const
        {
            return (lastFrame - firstFrame + stretchFactor) / stretchFactor;
        }

        constexpr size_t getFrameCount() const
        {
            return lastFrame - firstFrame + 1;
        }

        /// \brief reads an animation definition.
        static std::unique_ptr<Animation> readTr1(io::SDLReader& reader)
        {
            return read(reader, false);
        }

        static std::unique_ptr<Animation> readTr4(io::SDLReader& reader)
        {
            return read(reader, true);
        }

    private:
        static std::unique_ptr<Animation> read(io::SDLReader& reader, bool withLateral)
        {
            std::unique_ptr<Animation> animation{new Animation()};
            animation->poseDataOffset = reader.readU32();
            animation->stretchFactor = reader.readU8();
            if( animation->stretchFactor == 0 )
                animation->stretchFactor = 1;
            animation->poseDataSize = reader.readU8();
            animation->state_id = reader.readU16();

            animation->speed = reader.readI32();
            animation->accelleration = reader.readI32();
            if( withLateral )
            {
                animation->lateralSpeed = reader.readI32();
                animation->lateralAccelleration = reader.readI32();
            }
            else
            {
                animation->lateralSpeed = 0;
                animation->lateralAccelleration = 0;
            }

            animation->firstFrame = reader.readU16();
            animation->lastFrame = reader.readU16();
            animation->nextAnimation = reader.readU16();
            animation->nextFrame = reader.readU16();

            animation->transitionsCount = reader.readU16();
            animation->transitionsIndex = reader.readU16();
            animation->animCommandCount = reader.readU16();
            animation->animCommandIndex = reader.readU16();
            return animation;
        }
    };

    /** \brief State Change.
    *
    * Each one contains the state to change to and which animation dispatches
    * to use; there may be more than one, with each separate one covering a different
    * range of frames.
    */
    struct Transitions
    {
        uint16_t stateId;
        uint16_t transitionCaseCount; // number of ranges (seems to always be 1..5)
        uint16_t firstTransitionCase; // Offset into AnimDispatches[]

        /// \brief reads an animation state change.
        static std::unique_ptr<Transitions> read(io::SDLReader& reader)
        {
            std::unique_ptr<Transitions> state_change{new Transitions()};
            state_change->stateId = reader.readU16();
            state_change->transitionCaseCount = reader.readU16();
            state_change->firstTransitionCase = reader.readU16();
            return state_change;
        }
    };

    /** \brief Animation Dispatch.
    *
    * This specifies the next animation and frame to use; these are associated
    * with some range of frames. This makes possible such specificity as one
    * animation for left foot forward and another animation for right foot forward.
    */
    struct TransitionCase
    {
        uint16_t firstFrame; // Lowest frame that uses this range
        uint16_t lastFrame; // Highest frame (+1?) that uses this range
        uint16_t targetAnimation; // Animation to dispatch to
        uint16_t targetFrame; // Frame offset to dispatch to

        /// \brief reads an animation dispatch.
        static std::unique_ptr<TransitionCase> read(io::SDLReader& reader)
        {
            std::unique_ptr<TransitionCase> anim_dispatch{new TransitionCase()};
            anim_dispatch->firstFrame = reader.readU16();
            anim_dispatch->lastFrame = reader.readU16();
            anim_dispatch->targetAnimation = reader.readU16();
            anim_dispatch->targetFrame = reader.readU16();
            return anim_dispatch;
        }
    };

    struct AnimatedModel
    {
        uint32_t type; // Item Identifier (matched in Items[])
        uint16_t meshCount; // number of meshes in this object
        uint16_t firstMesh; // starting mesh (offset into MeshPointers[])
        uint32_t boneTreeIndex; // offset into MeshTree[]
        uint32_t meshPositionOffset; // byte offset into Frames[] (divide by 2 for Frames[i])
        uint16_t animationIndex; // offset into Animations[]

        /** \brief reads a moveable definition.
        *
        * some sanity checks get done which throw a exception on failure.
        * frame_offset needs to be corrected later in TR_Level::read_tr_level.
        */
        static std::unique_ptr<AnimatedModel> readTr1(io::SDLReader& reader)
        {
            std::unique_ptr<AnimatedModel> moveable{new AnimatedModel()};
            moveable->type = reader.readU32();
            moveable->meshCount = reader.readU16();
            moveable->firstMesh = reader.readU16();
            moveable->boneTreeIndex = reader.readU32();
            moveable->meshPositionOffset = reader.readU32();
            moveable->animationIndex = reader.readU16();
            return moveable;
        }

        static std::unique_ptr<AnimatedModel> readTr5(io::SDLReader& reader)
        {
            std::unique_ptr<AnimatedModel> moveable = readTr1(reader);
            if( reader.readU16() != 0xFFEF )
            BOOST_LOG_TRIVIAL(warning) << "TR5 Moveable: filler has wrong value";
            return moveable;
        }

        struct BoneWrapper
        {
            osg::ref_ptr<osgAnimation::UpdateBone> updateCallback = new osgAnimation::UpdateBone();
            osg::ref_ptr<osgAnimation::Bone> osgBone = new osgAnimation::Bone();
            osg::ref_ptr<osgAnimation::StackedQuaternionElement> quatTransform = new osgAnimation::StackedQuaternionElement();
            osg::ref_ptr<osgAnimation::StackedTranslateElement> vecTransform = new osgAnimation::StackedTranslateElement();
            std::vector<osg::ref_ptr<osgAnimation::RigGeometry>> geometries;

            explicit BoneWrapper()
            {
                updateCallback->getStackedTransforms().push_back(quatTransform);
                updateCallback->getStackedTransforms().push_back(vecTransform);

                osgBone->setUpdateCallback(updateCallback);
            }
        };

        struct AnimationWrapper
        {
            osg::ref_ptr<osgAnimation::Animation> animation = new osgAnimation::Animation();
            std::map<uint32_t, osg::BoundingBoxImpl<osg::Vec3i>> bboxes;
            uint16_t firstFrame;
            uint16_t lastFrame;

            osg::BoundingBoxImpl<osg::Vec3i> getBoundingBox(uint32_t localFrame) const
            {
                BOOST_ASSERT(localFrame >= firstFrame && localFrame <= lastFrame);
                localFrame -= firstFrame;
                auto it = bboxes.lower_bound(localFrame);
                if(it == bboxes.end())
                    return std::prev(it)->second;

                if(it->first == localFrame || it == bboxes.begin())
                    return it->second;

                // the iterator points behind the searched frame
                auto before = std::prev(it);
                auto dist = it->first - before->first;
                BOOST_ASSERT(dist > 0);
                auto lambda = float(localFrame - before->first) / dist;

                auto lerpInt = [](int a, int b, float d) -> int
                {
                    return static_cast<int>(a * (1.0f - d) + b * d);
                };

                auto lerpIVec = [&lerpInt](const osg::Vec3i& a, const osg::Vec3i& b, float d) -> osg::Vec3i
                {
                    return osg::Vec3i{
                        lerpInt(a.x(), b.x(), d),
                        lerpInt(a.y(), b.y(), d),
                        lerpInt(a.z(), b.z(), d)
                    };
                };

                // aabbox's getInterpolated does wrong rounding for ints, so we need to do it manually
                osg::BoundingBoxImpl<osg::Vec3i> interp(lerpIVec(before->second._min, it->second._min, lambda), lerpIVec(before->second._max, it->second._max, lambda));
                return interp;
            }

            void apply(const AnimatedModel& model, uint32_t localFrame) const
            {
                BOOST_ASSERT(localFrame >= firstFrame && localFrame <= lastFrame);

                const auto realOffset = (localFrame - firstFrame);
                const auto realLast = lastFrame - firstFrame;

                model.animatioManager->stopAll();
                model.animatioManager->playAnimation(animation);

                model.timeline->gotoFrame(realOffset);
                model.timeline->setLoop(0);

                // BOOST_LOG_TRIVIAL(debug) << "  - Frame loop (" << node->getName() << ") " << realFirst << ".." << realLast << " @ " << realOffset;
                //node->setCurrentFrame(gsl::narrow_cast<float>(realOffset));
            }
        };

        std::vector<BoneWrapper> bones;
        osg::ref_ptr<osgAnimation::Skeleton> skeleton = nullptr;
        std::vector<AnimationWrapper> animations;
        osg::ref_ptr<osgAnimation::BasicAnimationManager> animatioManager = new osgAnimation::BasicAnimationManager();
        osg::ref_ptr<osgAnimation::Timeline> timeline = new osgAnimation::Timeline();

        AnimatedModel()
        {
            timeline->setAnimationManager(animatioManager);
        }

        void buildSkeleton(const std::vector<int32_t>& boneTrees, const std::vector<osg::ref_ptr<osg::Geode>>& meshes, const std::vector<uint32_t>& meshIndices)
        {
            Expects(bones.empty());
            Expects(skeleton == nullptr);

            skeleton = new osgAnimation::Skeleton();

            std::stack<osg::ref_ptr<osgAnimation::Bone>> parentStack;

            for(size_t boneId = 0; boneId < meshCount; ++boneId)
            {
                bones.emplace_back();

                bones.back().osgBone->setName("bone:" + boost::lexical_cast<std::string>(boneId));

                if(type == 0)
                {
                    if(boneId == 7)
                        bones.back().osgBone->setName("chest");
                    else if(boneId == 0)
                        bones.back().osgBone->setName("hips");
                }

                if(boneId == 0)
                {
                    parentStack.push(bones.back().osgBone);
                    skeleton->addChild(bones.front().osgBone);
                    continue;
                }

                auto pred = bones[boneId - 1].osgBone;

                osg::ref_ptr<osgAnimation::Bone> parent = nullptr;
                BOOST_ASSERT(boneTreeIndex + 4 * boneId <= boneTrees.size());
                const int32_t* boneTreeData = &boneTrees[boneTreeIndex + (boneId - 1) * 4];

                switch(boneTreeData[0])
                {
                    case 0: // use predecessor
                        parent = pred;
                        parent->addChild(bones.back().osgBone);
                        break;
                    case 2: // push
                        parent = pred;
                        parent->addChild(bones.back().osgBone);
                        parentStack.push(parent);
                        break;
                    case 1: // pop
                        if(parentStack.empty())
                            BOOST_THROW_EXCEPTION(std::runtime_error("Invalid skeleton stack operation: cannot pop from empty stack"));
                        parent = parentStack.top();
                        parent->addChild(bones.back().osgBone);
                        parentStack.pop();
                        break;
                    case 3: // top
                        if(parentStack.empty())
                            BOOST_THROW_EXCEPTION(std::runtime_error("Invalid skeleton stack operation: cannot take top of empty stack"));
                        parent = parentStack.top();
                        parent->addChild(bones.back().osgBone);
                        break;
                    default:
                        BOOST_THROW_EXCEPTION(std::runtime_error("Invalid skeleton stack operation"));
                }

                BOOST_ASSERT(firstMesh + boneId < meshIndices.size());
                const auto meshIndex = meshIndices[firstMesh + boneId];
                BOOST_ASSERT(meshIndex < meshes.size());

                for(size_t i=0; i<meshes[meshIndex]->getNumDrawables(); ++i)
                {
                    auto srcGeo = meshes[meshIndex]->getDrawable(i)->asGeometry();

                    osg::ref_ptr<osgAnimation::RigGeometry> rigGeo = new osgAnimation::RigGeometry();
                    bones.back().geometries.emplace_back(rigGeo);

                    rigGeo->setSkeleton(skeleton);
                    rigGeo->setSourceGeometry(srcGeo);
                    rigGeo->setRigTransformImplementation(new osgAnimation::RigTransformSoftware());

                    osg::ref_ptr<osgAnimation::VertexInfluenceMap> influenceMap = new osgAnimation::VertexInfluenceMap();
                    osgAnimation::VertexInfluence& influence = (*influenceMap)[bones.back().osgBone->getName()];

                    auto vertexCount = srcGeo->getVertexArray()->getNumElements();
                    for(decltype(vertexCount) j = 0; j < vertexCount; ++j)
                        influence.emplace_back(j, 1.0f);

                    rigGeo->setInfluenceMap(influenceMap);
                }
            }
        }

        void loadAnimation(const Animation& animation, const std::vector<int16_t>& poseData, const std::vector<int32_t>& boneTrees)
        {
            Expects(skeleton != nullptr);
            Expects(!bones.empty());

            animations.emplace_back();

            animatioManager->registerAnimation(animations.back().animation);

            animations.back().firstFrame = animation.firstFrame;
            animations.back().lastFrame = animation.lastFrame;
            animations.back().animation->setPlayMode(osgAnimation::Animation::LOOP);
            animations.back().animation->setStartTime(0);

            std::vector<osg::ref_ptr<osgAnimation::QuatSphericalLinearChannel>> rotationChannels;
            std::vector<osg::ref_ptr<osgAnimation::Vec3LinearChannel>> translationChannels;

            for(const BoneWrapper& bone : bones)
            {
                const osg::ref_ptr<osgAnimation::QuatSphericalLinearChannel> rotationChannel = new osgAnimation::QuatSphericalLinearChannel();
                rotationChannel->setTarget(bone.quatTransform->getOrCreateTarget());
                rotationChannel->getOrCreateSampler();
                animations.back().animation->addChannel(rotationChannel);
                rotationChannels.emplace_back(rotationChannel);

                const osg::ref_ptr<osgAnimation::Vec3LinearChannel> translationChannel = new osgAnimation::Vec3LinearChannel();
                translationChannel->setTarget(bone.vecTransform->getOrCreateTarget());
                translationChannel->getOrCreateSampler();
                animations.back().animation->addChannel(translationChannel);
                translationChannels.emplace_back(translationChannel);
            }
            Expects(bones.size() == rotationChannels.size());
            Expects(bones.size() == translationChannels.size());

            BOOST_ASSERT(animation.poseDataOffset % 2 == 0);
            gsl::not_null<const int16_t*> pData = &poseData[animation.poseDataOffset / 2];

            uint32_t frame = 0;
            for(uint32_t i = 0; i <= gsl::narrow<uint32_t>(animation.lastFrame - animation.firstFrame); i += animation.stretchFactor)
            {
                osg::BoundingBoxImpl<osg::Vec3i> bbox;

                uint16_t angleSetOfs = 10;

                for(size_t boneId = 0; boneId < meshCount; boneId++)
                {
                    osg::Vec3 pos;
                    if(boneId == 0)
                    {
                        bbox._min = { pData[0], pData[2], pData[4] };
                        bbox._max = { pData[1], pData[3], pData[5] };
                        pos = osg::Vec3(pData[6], static_cast<float>(-pData[7]), pData[8]);
                    }
                    else
                    {
                        BOOST_ASSERT(boneTreeIndex + 4 * boneId <= boneTrees.size());
                        const int32_t* boneTreeData = &boneTrees[boneTreeIndex + (boneId - 1) * 4];
                        pos = osg::Vec3(static_cast<float>(boneTreeData[1]), static_cast<float>(-boneTreeData[2]), static_cast<float>(boneTreeData[3]));
                    }

                    auto temp2 = pData[angleSetOfs++];
                    auto temp1 = pData[angleSetOfs++];

                    osg::Vec3 rot;
                    rot.x() = static_cast<float>((temp1 & 0x3ff0) >> 4);
                    rot.y() = -static_cast<float>(((temp1 & 0x000f) << 6) | ((temp2 & 0xfc00) >> 10));
                    rot.z() = static_cast<float>(temp2 & 0x03ff);
                    rot *= 360 / 1024.0;

                    translationChannels[boneId]->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::Vec3Keyframe(static_cast<double>(frame) / core::FrameRate, pos));
                    rotationChannels[boneId]->getOrCreateSampler()->getOrCreateKeyframeContainer()->push_back(osgAnimation::QuatKeyframe(static_cast<double>(frame) / core::FrameRate, util::trRotationToQuat(rot)));
                }

                animations.back().bboxes.insert({ i, bbox });
                frame += animation.stretchFactor;
                pData = pData.get() + animation.poseDataSize;
            }

            animations.back().animation->computeDuration();
        }
    };
}
