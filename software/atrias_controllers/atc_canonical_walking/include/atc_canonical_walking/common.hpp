#ifndef COMMON_HPP
#define COMMON_HPP

/**
  * @file This contains definitions common to both the controller and GUI.
  */

// The namespaces for all controllers
namespace atrias {
namespace controller {

/**
  * @brief This defines the source of the tau signal
  */
typedef int8_t TauSource_t;
enum class TauSource: TauSource_t {
	STANCE_LEG_ANGLE = 0, // The normal value, stance leg angle
	GUI                   // The GUI controls tau.
};

// End namespaces
}
}

#endif // COMMON_HPP
