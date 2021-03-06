#include "serialization.h"

#include "engine/engine.h"

#include <boost/stacktrace.hpp>

namespace serialization
{
Exception::Exception(const gsl::czstring msg)
    : std::runtime_error{msg}
{
  BOOST_LOG_TRIVIAL(fatal) << "Serialization exception: " << msg;
  BOOST_LOG_TRIVIAL(fatal) << "Stacktrace:\n" << boost::stacktrace::stacktrace();
}

void Serializer::save(const std::string& filename, engine::Engine& engine)
{
  std::ofstream file{filename, std::ios::out | std::ios::trunc};
  Expects(file.is_open());
  Serializer ser{YAML::Node{}, engine, false, nullptr};
  access::callSerializeOrSave(engine, ser);
  ser.processQueues();
  file << ser.node;
}

void Serializer::load(const std::string& filename, engine::Engine& engine)
{
  std::ifstream file{filename, std::ios::in};
  Expects(file.is_open());
  Serializer ser{YAML::Load(file), engine, true, nullptr};
  access::callSerializeOrLoad(engine, ser);
  ser.processQueues();
}

void Serializer::processQueues()
{
  while(!m_nextQueue->empty())
  {
    BOOST_LOG_TRIVIAL(debug) << "Processing serialization queue...";
    auto current = std::exchange(m_nextQueue, std::make_shared<NextQueue>());
    while(!current->empty())
    {
      current->front()();
      current->pop();
    }
  }
}

// ReSharper disable once CppMemberFunctionMayBeConst
void Serializer::applyTag()
{
  if(m_tag.empty())
    return;

  if(loading)
  {
    if(node.Tag() != m_tag)
      SERIALIZER_EXCEPTION("Expected tag \"" + m_tag + "\", but got \"" + node.Tag() + "\"");
  }
  else
    node.SetTag(m_tag);

  m_tag.clear();
}
} // namespace serialization
