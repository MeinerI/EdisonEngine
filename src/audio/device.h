#pragma once

#include "filterhandle.h"
#include "sourcehandle.h"
#include "stream.h"

#include <AL/alc.h>
#include <gsl/gsl>
#include <set>
#include <thread>


namespace audio
{

class Device final : public boost::noncopyable
{
public:
    explicit Device();
    ~Device();

    void update()
    {
        removeStoppedSources();
        updateStreams();
    }

    const std::shared_ptr<FilterHandle>& getUnderwaterFilter() const
    {
        return m_underwaterFilter;
    }

    void registerSource(const gsl::not_null<std::shared_ptr<SourceHandle>>& src)
    {
        m_sources.insert(src);
    }

    void registerStream(const gsl::not_null<std::shared_ptr<Stream>>& stream)
    {
        m_streams.insert(stream);
    }

    void removeStream(const std::shared_ptr<Stream>& stream)
    {
        m_streams.erase(stream);
    }

    void removeStoppedSources()
    {
        std::set<std::shared_ptr<SourceHandle>> cleaned;
        std::copy_if(m_sources.begin(), m_sources.end(), std::inserter(cleaned, cleaned.end()), [](const std::shared_ptr<SourceHandle>& src)
        {
            return !src->isStopped();
        });

        m_sources = std::move(cleaned);
    }

    void setListenerTransform(const glm::vec3& pos, const glm::vec3& front, const glm::vec3& up)
    {
        alListener3f(AL_POSITION, pos.x, pos.y, pos.z);
        DEBUG_CHECK_AL_ERROR();

        const ALfloat o[6] = {
            front.x, front.y, front.z,
            up.x, up.y, up.z
        };
        alListenerfv(AL_ORIENTATION, o);
        DEBUG_CHECK_AL_ERROR();
    }

    void applyDirectFilterToAllSources(const std::shared_ptr<FilterHandle>& filter)
    {
        for(const auto& src : m_sources)
            src->setDirectFilter(filter);
    }

private:
    ALCdevice* m_device = nullptr;
    ALCcontext* m_context = nullptr;
    std::shared_ptr<FilterHandle> m_underwaterFilter = nullptr;
    std::set<std::shared_ptr<SourceHandle>> m_sources;
    std::set<std::shared_ptr<Stream>> m_streams;
    std::thread m_streamUpdater;
    bool m_shutdown = false;

    void updateStreams()
    {
        for(const auto& stream : m_streams)
            stream->update();
    }
};

}
