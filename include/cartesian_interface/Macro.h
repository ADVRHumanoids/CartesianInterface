#ifndef MACRO_H
#define MACRO_H

#define CARTESIO_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr; \

#define NOTIFY_OBSERVERS(FieldName) \
    for(auto obs_weak: _observers) \
    { \
        if(auto obs = obs_weak.lock()) obs->on##FieldName##Changed(); \
    }


#endif // MACRO_H
