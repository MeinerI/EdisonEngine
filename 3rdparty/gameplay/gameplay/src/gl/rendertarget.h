#pragma once

#include "bindableresource.h"

namespace gameplay
{
namespace gl
{
class RenderTarget : public BindableResource
{
protected:
    explicit RenderTarget(const std::function<Allocator>& allocator, const std::function<Binder>& binder,
                          const std::function<Deleter>& deleter)
            : BindableResource{allocator, binder, deleter}
    {
    }

    explicit RenderTarget(const std::function<Allocator>& allocator, const std::function<Binder>& binder,
                          const std::function<Deleter>& deleter, GLenum identifier, const std::string& label)
            : BindableResource{allocator, binder, deleter, identifier, label}
    {
    }

public:
    virtual GLint getWidth() const noexcept = 0;

    virtual GLint getHeight() const noexcept = 0;
};
}
}
