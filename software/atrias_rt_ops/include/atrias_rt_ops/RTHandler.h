#ifndef RTHANDLER_H
#define RTHANDLER_H

/** @file
  * @brief Handles anything needed to maintain realtime in RT Ops.
  */

// Orocos
#include <rtt/Logger.hpp>

#include <signal.h>
#include <sys/mman.h>

namespace atrias {

namespace rtOps {

class RTHandler {
	public:
		/** @brief Handles RT-related stuff that needs to occur on component load.
		  */
		RTHandler();
		
		/** @brief Enters realtime execution.
		  * Not realtime safe itself.
		  */
		void beginRT();
		
		/** @brief Leaves realtime execution.
		  * Not realtime safe itself.
		  */
		void endRT();
};

}

}

#endif // RTHANDLER_H
