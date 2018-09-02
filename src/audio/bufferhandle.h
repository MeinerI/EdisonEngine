#pragma once

#include "utils.h"
#include "sndfile/helpers.h"

#include <AL/al.h>
#include <boost/noncopyable.hpp>
#include <boost/log/trivial.hpp>

#include <vector>

namespace audio
{
class BufferHandle final : public boost::noncopyable
{
    const ALuint m_handle;

    static ALuint createHandle()
    {
        ALuint handle;
        alGenBuffers( 1, &handle );
        DEBUG_CHECK_AL_ERROR();

        Expects( alIsBuffer( handle ) );

        return handle;
    }

public:
    explicit BufferHandle()
            : m_handle( createHandle() )
    {
    }

    ~BufferHandle()
    {
        alDeleteBuffers( 1, &m_handle );
        DEBUG_CHECK_AL_ERROR();
    }

    ALuint get() const noexcept
    {
        return m_handle;
    }

    void fill(const int16_t* samples, size_t sampleCount, int channels, int sampleRate)
    {
        alBufferData( m_handle, channels == 2 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16, samples,
                      gsl::narrow<ALsizei>( sampleCount * sizeof( samples[0] ) ), sampleRate );
        DEBUG_CHECK_AL_ERROR();
    }

    bool fillFromWav(const uint8_t* data)
    {
        Expects( data[0] == 'R' && data[1] == 'I' && data[2] == 'F' && data[3] == 'F' );
        Expects( data[8] == 'W' && data[9] == 'A' && data[10] == 'V' && data[11] == 'E' );

        sndfile::MemBufferFileIo wavMem( data, *reinterpret_cast<const uint32_t*>(data + 4) + 8 );
        SF_INFO sfInfo;
        memset( &sfInfo, 0, sizeof( sfInfo ) );
        SNDFILE* sfFile = sf_open_virtual( &wavMem, SFM_READ, &sfInfo, &wavMem );

        if( sfFile == nullptr )
        {
            BOOST_LOG_TRIVIAL( error ) << "Failed to load WAV sample from memory: " << sf_strerror( sfFile );
            return false;
        }

        BOOST_ASSERT( sfInfo.frames >= 0 );

        std::vector<short> pcm( sfInfo.frames );
        sf_readf_short( sfFile, pcm.data(), sfInfo.frames );

        alBufferData( m_handle, sfInfo.channels == 2 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16, pcm.data(),
                      gsl::narrow<ALsizei>( pcm.size() * sizeof( pcm[0] ) ), sfInfo.samplerate );
        DEBUG_CHECK_AL_ERROR();

        sf_close( sfFile );

        return true;
    }
};
}
