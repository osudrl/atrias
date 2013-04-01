#ifndef NOTNULLPTR_H
#define NOTNULLPTR_H

/**
  * @file notNullPtr.hpp
  * @author Ryan Van Why
  * @brief Contains templates testing if a data type is nullptr_t
  * This is utilized by the ATC class to detect data types not in use.
  */

// For nullptr_t
#include <cstddef>

namespace atrias {
namespace shared {

/**
  * @brief Returns true if this is not of type nullptr_t, false if it is.
  */
template <typename T>
inline bool notNullPtr() {
	return true;
}

// Utilize template specialization to detect the difference
template <>
inline bool notNullPtr<std::nullptr_t>() {
	return false;
}

}
}

#endif // NOTNULLPTR_H

// vim: noexpandtab
