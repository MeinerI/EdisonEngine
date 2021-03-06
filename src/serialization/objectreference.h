#pragma once

#include "engine/engine.h"
#include "ptr.h"

#include <type_traits>

namespace serialization
{
template<typename T>
struct ObjectReference final
{
  static_assert(std::is_base_of_v<engine::objects::Object, T>);
  std::shared_ptr<T>& ptr;

  explicit ObjectReference(std::shared_ptr<T>& ptr)
      : ptr{ptr}
  {
  }

  void save(const Serializer& ser)
  {
    if(ptr == nullptr)
    {
      ser.node = YAML::Node{YAML::NodeType::Null};
    }
    else
    {
      ser.tag("objectref");
      for(const auto& obj : ser.engine.getObjectManager().getObjects())
      {
        if(obj.second.get() == ptr)
        {
          engine::ObjectId tmp = obj.first;
          ser(S_NV("id", tmp));
        }
      }
    }
  }

  void load(const Serializer& ser)
  {
    if(ser.node.IsNull())
    {
      ptr = nullptr;
    }
    else
    {
      ser.lazy([pptr = &ptr](const Serializer& ser) {
        ser.tag("objectref");
        engine::ObjectId id = 0;
        ser(S_NV("id", id));
        auto tmp = ser.engine.getObjectManager().getObjects().at(id).get();
        Expects(tmp != nullptr);
        *pptr = std::dynamic_pointer_cast<T>(tmp);
      });
    }
  }
};

} // namespace serialization
