#pragma once

#include <osg/Math>
#include <osg/Vec3>
#include <osg/Quat>

#include <cmath>

#include <gsl.h>

#include <boost/optional.hpp>


namespace core
{
    namespace detail
    {
        struct UnsignedRawAngle;
    }


    class Angle final
    {
        friend struct detail::UnsignedRawAngle;

        int32_t m_value;
        static const int32_t Scale = 1 << 16;


        struct RawTag
        {
        };


        constexpr explicit Angle(int32_t val, const RawTag&) noexcept
            : m_value{val}
        {
        }


    public:
        [[implicit]]


        constexpr Angle() noexcept
            : m_value{0}
        {
        }


        explicit Angle(int16_t val) noexcept
            : m_value{gsl::narrow_cast<int32_t>(val * Scale)}
        {
        }


        constexpr Angle(const Angle&) = default;


        static Angle fromRad(float r)
        {
            return Angle{gsl::narrow_cast<int32_t>(r / 2 / osg::PI * 65536 * Scale), RawTag()};
        }


        static Angle fromDegrees(float val)
        {
            return Angle{gsl::narrow_cast<int32_t>(std::lround(val * Scale)), RawTag()};
        }


        constexpr float toDegrees() const noexcept
        {
            return m_value * 360.0f / Scale / 65536;
        }


        float toRad() const noexcept
        {
            return m_value * osg::PI * 2 / Scale / 65536;
        }


        float sin() const noexcept
        {
            return std::sin(toRad());
        }


        float cos() const noexcept
        {
            return std::cos(toRad());
        }


        constexpr int16_t toAU() const noexcept
        {
            return m_value / Scale;
        }


        Angle operator-(const Angle& rhs) const noexcept
        {
            return Angle{gsl::narrow_cast<int32_t>(m_value - rhs.m_value), RawTag()};
        }


        Angle& operator-=(const Angle& rhs) noexcept
        {
            m_value -= rhs.m_value;
            return *this;
        }


        Angle operator+(const Angle& rhs) const noexcept
        {
            return Angle{gsl::narrow_cast<int32_t>(m_value + rhs.m_value), RawTag()};
        }


        Angle& operator+=(const Angle& rhs) noexcept
        {
            m_value += rhs.m_value;
            return *this;
        }


        Angle operator*(float v) const
        {
            return Angle{gsl::narrow_cast<int32_t>(std::lround(m_value * v)), RawTag()};
        }


        Angle& operator*=(float v)
        {
            m_value = gsl::narrow_cast<int32_t>(v);
            return *this;
        }


        Angle operator-() const
        {
            return Angle{-m_value, RawTag()};
        }


        constexpr bool operator==(const Angle& rhs) const noexcept
        {
            return m_value == rhs.m_value;
        }


        constexpr bool operator<(const Angle& rhs) const noexcept
        {
            return m_value < rhs.m_value;
        }
    };


    constexpr bool operator>(const Angle& a, const Angle& b) noexcept
    {
        return b < a;
    }


    constexpr bool operator<=(const Angle& a, const Angle& b) noexcept
    {
        return a == b || a < b;
    }


    constexpr bool operator>=(const Angle& a, const Angle& b) noexcept
    {
        return a == b || a > b;
    }


    constexpr bool operator!=(const Angle& a, const Angle& b) noexcept
    {
        return !(a == b);
    }


    namespace detail
    {
        /**
         * @brief A simple helper to provide negation of unsigned values created by operator""_au
         */
        struct UnsignedRawAngle final
        {
            const uint32_t value;


            explicit UnsignedRawAngle(unsigned long long val)
                : value{gsl::narrow<uint32_t>(val * Angle::Scale)}
            {
                Expects(value <= 32768U * Angle::Scale);
            }


            explicit UnsignedRawAngle(long double val)
                : value{gsl::narrow<uint32_t>(std::llround(val * Angle::Scale))}
            {
                Expects(value <= 32768U * Angle::Scale);
            }


            Angle operator-() const
            {
                return Angle{-gsl::narrow_cast<int32_t>(value), Angle::RawTag()};
            }


            Angle operator+() const
            {
                return static_cast<Angle>(*this);
            }


            Angle operator-(const Angle& rhs) const
            {
                return static_cast<Angle>(*this) - rhs;
            }


            Angle operator+(const Angle& rhs) const
            {
                return static_cast<Angle>(*this) + rhs;
            }


            operator Angle() const
            {
                return Angle{gsl::narrow_cast<int32_t>(value), Angle::RawTag()};
            }
        };
    }
}


inline core::detail::UnsignedRawAngle operator"" _au(unsigned long long v) noexcept
{
    Expects(v <= 32768);
    return core::detail::UnsignedRawAngle{v};
}


inline core::detail::UnsignedRawAngle operator"" _deg(unsigned long long v) noexcept
{
    Expects(v <= 180);
    return core::detail::UnsignedRawAngle{v * 65536 / 360};
}


inline core::detail::UnsignedRawAngle operator"" _deg(long double v) noexcept
{
    Expects(v <= 180);
    return core::detail::UnsignedRawAngle{v * 65536 / 360};
}


namespace core
{
    enum class Axis
    {
        PosZ,
        PosX,
        NegZ,
        NegX
    };


    inline boost::optional<Axis> axisFromAngle(const core::Angle& angle, const core::Angle& margin)
    {
        Expects(margin >= 0_deg && margin <= 45_deg);
        if( angle <= 0_deg + margin && angle >= 0_deg - margin )
            return Axis::PosZ;
        if( angle <= 90_deg + margin && angle >= 90_deg - margin )
            return Axis::PosX;
        if( angle <= -90_deg + margin && angle >= -90_deg - margin )
            return Axis::NegX;
        if( angle >= 180_deg - margin || angle <= -180_deg + margin )
            return Axis::NegZ;

        return {};
    }


    inline core::Angle alignRotation(const Axis& axis)
    {
        switch( axis )
        {
            case Axis::PosZ: return core::Angle(0_deg);
            case Axis::PosX: return core::Angle(90_deg);
            case Axis::NegZ: return core::Angle(-180_deg);
            case Axis::NegX: return core::Angle(-90_deg);
            default: return 0_deg;
        }
    }


    inline boost::optional<core::Angle> alignRotation(const core::Angle& angle, const core::Angle& margin)
    {
        auto axis = axisFromAngle(angle, margin);
        if( !axis )
            return {};

        return alignRotation(*axis);
    }


    class TRRotation final
    {
    public:
        Angle X;
        Angle Y;
        Angle Z;

        TRRotation() = default;


        TRRotation(const Angle& x, const Angle& y, const Angle& z)
            : X{x}
            , Y{y}
            , Z{z}
        {
        }


        osg::Vec3f toDegrees() const
        {
            return {
                X.toDegrees(),
                Y.toDegrees(),
                Z.toDegrees()
            };
        }


        osg::Vec3f toRad() const
        {
            return {
                X.toRad(),
                Y.toRad(),
                Z.toRad()
            };
        }

        TRRotation operator-(const TRRotation& rhs) const
        {
            return{ X - rhs.X, Y - rhs.Y, Z - rhs.Z };
        }
    };


    inline osg::Quat xyzToQuat(const TRRotation& r)
    {
        //! @todo This is horribly inefficient code, but it properly converts ZXY angles to XYZ angles.
        osg::Quat q;
        q *= osg::Quat(r.Y.toRad(), osg::Vec3f{0,1,0});
        q *= osg::Quat(r.X.toRad(), osg::Vec3f{-1,0,0});
        q *= osg::Quat(r.Z.toRad(), osg::Vec3f{0,0,-1});
        return q;
    }


    inline osg::Quat xyzToQuat(const osg::Vec3& r)
    {
        //! @todo This is horribly inefficient code, but it properly converts ZXY angles to XYZ angles.
        osg::Quat q;
        q *= osg::Quat(r.y(), osg::Vec3f{0,1,0});
        q *= osg::Quat(r.x(), osg::Vec3f{-1,0,0});
        q *= osg::Quat(r.z(), osg::Vec3f{0,0,-1});
        return q;
    }


    inline osg::Vec3f xyzToYprRad(const TRRotation& r)
    {
        auto q = xyzToQuat(r);

        double limit = 0.499999;
        double sqx = q.x() * q.x();
        double sqy = q.y() * q.y();
        double sqz = q.z() * q.z();

        double t = q.x() * q.y() + q.z() * q.w();

        if( t > limit ) // gimbal lock ?
        {
            return osg::Vec3f{
                gsl::narrow_cast<float>(2 * atan2(q.x(), q.w())),
                gsl::narrow_cast<float>(osg::PI_2),
                0
            };
        }
        else if( t < -limit )
        {
            return osg::Vec3f{
                gsl::narrow_cast<float>(-2 * atan2(q.x(), q.w())),
                gsl::narrow_cast<float>(-osg::PI_2),
                0
            };
        }
        else
        {
            return osg::Vec3f{
                gsl::narrow_cast<float>(atan2(2 * q.y() * q.w() - 2 * q.x() * q.z(), 1 - 2 * sqy - 2 * sqz)),
                gsl::narrow_cast<float>(asin(2 * t)),
                gsl::narrow_cast<float>(atan2(2 * q.x() * q.w() - 2 * q.y() * q.z(), 1 - 2 * sqx - 2 * sqz))
            };
        }
    }
}
