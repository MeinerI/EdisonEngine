#include "Sprite.h"

#include "Camera.h"
#include "RenderContext.h"
#include "Scene.h"

#include <glm/gtc/quaternion.hpp>


namespace gameplay
{
    Sprite::Sprite(Game* game, const std::shared_ptr<gl::Texture>& texture, float width, float height, const Rectangle& source, unsigned frameCount, const std::shared_ptr<ShaderProgram>& shaderProgram)
        : Drawable()
        , _width{width}
        , _height{height}
        , _offset(OFFSET_BOTTOM_LEFT)
        , _anchor(glm::vec2(0.5f, 0.5f))
        , _flipFlags(FLIP_NONE)
        , _frames(frameCount)
        , _frameStride(0)
        , _framePadding(1)
        , _frameIndex(0)
        , _batch{ std::make_shared<SpriteBatch>(game, texture, shaderProgram) }
        , _opacity(1.0f)
        , _color{ 1,1,1,1 }
        , _blendMode(BLEND_ALPHA)
    {
        BOOST_ASSERT(texture != nullptr);
        BOOST_ASSERT(width >= -1 && height >= -1);
        BOOST_ASSERT(source.width >= -1 && source.height >= -1);
        BOOST_ASSERT(frameCount > 0);

        _batch->getTexture()->set(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        _batch->getTexture()->set(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        _batch->getTexture()->set(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        _batch->getTexture()->set(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        _batch->getStateBlock()->setDepthWrite(false);
        _batch->getStateBlock()->setDepthTest(true);

        auto imageWidth = _batch->getTexture()->getWidth();
        auto imageHeight = _batch->getTexture()->getHeight();
        if(width == -1)
            _width = imageWidth;
        if(height == -1)
            _height = imageHeight;

        _frames[0] = source;
        if(_frames[0].width == -1.0f)
            _frames[0].width = imageWidth;
        if(_frames[0].height == -1.0f)
            _frames[0].height = imageHeight;
    }


    Sprite::~Sprite() = default;


    float Sprite::getWidth() const
    {
        return _width;
    }


    float Sprite::getHeight() const
    {
        return _height;
    }


    void Sprite::setOffset(Sprite::Offset offset)
    {
        _offset = offset;
    }


    Sprite::Offset Sprite::getOffset() const
    {
        return _offset;
    }


    void Sprite::setAnchor(const glm::vec2& anchor)
    {
        _anchor = anchor;
    }


    const glm::vec2& Sprite::getAnchor() const
    {
        return _anchor;
    }


    void Sprite::setFlip(int flipFlags)
    {
        _flipFlags = flipFlags;
    }


    int Sprite::getFlip() const
    {
        return _flipFlags;
    }


    // ReSharper disable once CppMemberFunctionMayBeConst
    void Sprite::setFrameSource(size_t frameIndex, const Rectangle& source)
    {
        BOOST_ASSERT(frameIndex < _frames.size());

        _frames[frameIndex] = source;
    }


    const Rectangle& Sprite::getFrameSource(size_t frameIndex) const
    {
        BOOST_ASSERT(frameIndex < _frames.size());

        return _frames[frameIndex];
    }


    void Sprite::computeFrames(unsigned int frameStride, unsigned int framePadding)
    {
        _frameStride = frameStride;
        _framePadding = framePadding;

        if( _frames.size() < 2 )
            return;
        unsigned int imageWidth = _batch->getTexture()->getWidth();
        unsigned int imageHeight = _batch->getTexture()->getHeight();

        // Mark the start as reference
        float x = _frames[0].x;
        float y = _frames[0].y;

        // Compute frames 1+
        for( unsigned int frameIndex = 1; frameIndex < _frames.size(); frameIndex++ )
        {
            _frames[frameIndex].x = x;
            _frames[frameIndex].y = y;
            _frames[frameIndex].width = _width;
            _frames[frameIndex].height = _height;

            x += _frames[frameIndex].width + (float)_framePadding;
            if( x >= imageWidth )
            {
                y += _frames[frameIndex].height + (float)_framePadding;
                if( y >= imageHeight )
                {
                    y = 0.0f;
                }
                x = 0.0f;
            }
        }
    }


    size_t Sprite::getFrameCount() const
    {
        return _frames.size();
    }


    unsigned int Sprite::getFramePadding() const
    {
        return _framePadding;
    }


    unsigned int Sprite::getFrameStride() const
    {
        return _frameStride;
    }


    void Sprite::setFrameIndex(unsigned int index)
    {
        _frameIndex = index;
    }


    unsigned int Sprite::getFrameIndex() const
    {
        return _frameIndex;
    }


    void Sprite::setOpacity(float opacity)
    {
        _opacity = opacity;
    }


    float Sprite::getOpacity() const
    {
        return _opacity;
    }


    void Sprite::setColor(const glm::vec4& color)
    {
        _color = color;
    }


    const glm::vec4& Sprite::getColor() const
    {
        return _color;
    }


    Sprite::BlendMode Sprite::getBlendMode() const
    {
        return _blendMode;
    }


    void Sprite::setBlendMode(BlendMode mode)
    {
        switch( mode )
        {
            case BLEND_NONE:
                _batch->getStateBlock()->setBlend(false);
                break;
            case BLEND_ALPHA:
                _batch->getStateBlock()->setBlend(true);
                _batch->getStateBlock()->setBlendSrc(RenderState::BLEND_SRC_ALPHA);
                _batch->getStateBlock()->setBlendDst(RenderState::BLEND_ONE_MINUS_SRC_ALPHA);
                break;
            case BLEND_ADDITIVE:
                _batch->getStateBlock()->setBlend(true);
                _batch->getStateBlock()->setBlendSrc(RenderState::BLEND_SRC_ALPHA);
                _batch->getStateBlock()->setBlendDst(RenderState::BLEND_ONE);
                break;
            case BLEND_MULTIPLIED:
                _batch->getStateBlock()->setBlend(true);
                _batch->getStateBlock()->setBlendSrc(RenderState::BLEND_ZERO);
                _batch->getStateBlock()->setBlendDst(RenderState::BLEND_SRC_COLOR);
                break;
            default:
                BOOST_LOG_TRIVIAL(error) << "Unsupported blend mode (" << mode << ").";
                break;
        }
    }


    std::shared_ptr<RenderState::StateBlock> Sprite::getStateBlock() const
    {
        return _batch->getStateBlock();
    }


    const std::shared_ptr<Material>& Sprite::getMaterial() const
    {
        return _batch->getMaterial();
    }


    void Sprite::draw(RenderContext& context)
    {
        // Apply scene camera projection and translation offsets
        glm::vec3 position{ 0,0,0 };
        if( context.getCurrentNode() && context.getCurrentNode()->getScene() )
        {
            auto activeCamera = context.getCurrentNode()->getScene()->getActiveCamera();
            if( activeCamera )
            {
                // Scene projection
                glm::mat4 projectionMatrix;
                projectionMatrix = context.getCurrentNode()->getProjectionMatrix();
                _batch->setProjectionMatrix(projectionMatrix);

                // Camera translation offsets
                position.x -= activeCamera->getInverseViewMatrix()[3].x;
                position.y -= activeCamera->getInverseViewMatrix()[3].y;
            }

            // Apply node translation offsets
            glm::vec3 translation = context.getCurrentNode()->getTranslationWorld();
            position.x += translation.x;
            position.y += translation.y;
            position.z += translation.z;
        }

        // Apply local offset translation offsets
        if( (_offset & OFFSET_HCENTER) == OFFSET_HCENTER )
            position.x -= _width * 0.5f;
        if( (_offset & OFFSET_RIGHT) == OFFSET_RIGHT )
            position.x -= _width;
        if( (_offset & OFFSET_VCENTER) == OFFSET_VCENTER )
            position.y -= _height * 0.5f;
        if( (_offset & OFFSET_TOP) == OFFSET_TOP )
            position.y -= _height;
        if( (_offset & OFFSET_ANCHOR) == OFFSET_ANCHOR )
        {
            position.x -= _width * _anchor.x;
            position.y -= _height * _anchor.y;
        }

        // Apply node scale and rotation
        float rotationAngle = 0.0f;
        glm::vec2 scale = glm::vec2(_width, _height);
        if( context.getCurrentNode() != nullptr )
        {
            // Apply node rotation
            const glm::quat rot = glm::quat_cast(context.getCurrentNode()->getLocalMatrix());
            if(rot.x != 0.0f || rot.y != 0.0f || rot.z != 0.0f)
                rotationAngle = glm::angle(rot);
        }

        // Apply flip flags
        if( (_flipFlags & FLIP_HORIZONTAL) == FLIP_HORIZONTAL )
        {
            position.x += scale.x;
            scale.x = -scale.x;
        }
        if( (_flipFlags & FLIP_VERTICAL) == FLIP_VERTICAL )
        {
            position.y += scale.y;
            scale.y = -scale.y;
        }

        // TODO: Proper batching from cache based on batching rules (image, layers, etc)
        _batch->start();
        _batch->draw(position, _frames[_frameIndex], scale, glm::vec4(_color.x, _color.y, _color.z, _color.w * _opacity),
                     _anchor, rotationAngle);
        _batch->finishAndDraw(context);
    }
}
