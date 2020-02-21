#ifndef MACRO_H
#define MACRO_H

namespace XBot { namespace Cartesian { namespace detail {

struct Version
{
    int major = -1;
    int minor = -1;
    int patch = -1;

    Version(int a_major, int a_minor, int a_patch)
    {
        major = a_major;
        minor = a_minor;
        patch = a_patch;
    }

    bool isCompatible(Version other)
    {
        return major == other.major && minor == other.minor;
    }
};

inline bool operator==(Version a, Version b)
{
    return a.major == b.major &&
            a.minor == b.minor &&
            a.patch == b.patch;
}

} } }

#define CARTESIO_ABI_VERSION (2, 1, 0)

#define CARTESIO_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr; \

#define NOTIFY_OBSERVERS(FieldName) \
    bool notify_success = true; \
    for(auto obs_weak: _observers) \
    { \
        if(auto obs = obs_weak.lock()) \
        {   \
            if(!obs->on##FieldName##Changed()) notify_success = false; \
        } \
    }






#endif // MACRO_H
