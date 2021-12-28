#ifndef OPENSOTUTILS_H
#define OPENSOTUTILS_H

#include <OpenSoT/Task.h>

// if version is exported, we are using a recent enough opensot -> std::make_shared
// otherwise, we are using an old one -> boost::make_shared
#ifdef OPENSOT_VERSION_MAJOR

#include <memory>
namespace XBot { namespace Cartesian { namespace SotUtils {
    using std::make_shared;
    using std::shared_ptr;
} } }

#else

#include <boost/make_shared.hpp>
namespace XBot { namespace Cartesian { namespace SotUtils {
    using boost::make_shared;
    using boost::shared_ptr;
} } }

#endif

#endif // OPENSOTUTILS_H
