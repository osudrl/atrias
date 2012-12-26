#ifndef GUICOMMS_H
#define GUICOMMS_H

/** @file GuiComms.h
  * @brief This class handles all communication with the GUI
  */

namespace atrias {
	namespace rtOps {
		class GuiComms;
	}
}

// Orocos
#include <rtt/InputPort.hpp>

// ATRIAS
#include "atrias_rt_ops/RTOps.h"
#include "atrias_shared/globals.h"

namespace atrias {

namespace rtOps {

class GuiComms {
	public:
		/** @brief Initializes this GuiComms instance.
		  * @param rt_ops A pointer to the RTOps instance.
		  */
		GuiComms(RTOps *rt_ops);

		/** @brief Retrieves and returns the Gui-commanded state.
		  * @return The state commanded by the GUI
		  */
		RtOpsState getGuiCmd();

		/** @brief Tells this class to check for a new GUI command.
		  */
		void updateHook();
	
	private:
		/** @brief Signifies whether or not we need to check for a new GUI command.
		  * Volatile so compiler optimizations don't cause race conditions.
		  * We do the check in the controller thread so as to avoid needing a lock
		  * for thread-safety
		  */
		volatile bool checkNeeded;

		/** @brief This is the current GUI-requested state.
		  * Note: may only be called from the controller thread.
		  */
		RtOpsState                 guiIn;

		/** @brief This is the port over which the GUI sends state requests.
		  */
		RTT::InputPort<RtOpsState> guiInPort;

		/** @brief This points to the main Rt Ops instance.
		  */
		RTOps                      *rtOps;
};

}

}

#endif // GUICOMMS_H

// vim: noexpandtab
