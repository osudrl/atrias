#ifndef ToSubstituteClassName_HPP
#define ToSubstituteClassName_HPP

/**
 * @file ToSubstituteClassName.hpp
 * @author Ryan Van Why
 * @brief ToSubstituteDescription
 */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type.
#include "ToSubstitutePackageName/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "ToSubstitutePackageName/controller_input.h"
// The type transmitted from the controller to the GUI
#include "ToSubstitutePackageName/controller_status.h"

// Include subcontrollers here

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ToSubstituteClassName : public ATC<
	ToSubstitutePackageName::controller_log_data,
	ToSubstitutePackageName::controller_input,
	ToSubstitutePackageName::controller_status>
{
	public:
		/** 
		 * @brief The constructor for this controller.
		 * @param name The name of this component.
		 * Every top-level controller will have this name parameter,
		 * just like current controllers.
		 */
		ToSubstituteClassName(string name);

	private:
		/** 
		 * @brief This is the main function for the top-level controller.
		 */
		void controller();

		// Include subcontrollers and variables here
};

}
}

#endif // ToSubstituteClassName_HPP
