#ifndef MACRO_H
#define MACRO_H

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
