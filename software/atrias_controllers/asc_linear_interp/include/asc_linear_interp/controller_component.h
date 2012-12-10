#ifndef __ASC_LINEAR_INTERP_H__
#define __ASC_LINEAR_INTERP_H__

/*! \file controller_component.h
 *  \author Ryan Van Why
 *  \brief Orocos Component header for the asc_linear_interp subcontroller.
 */

// Orocos
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>

#include <robot_invariant_defs.h>

// Datatypes
#include <asc_linear_interp/controller_log_data.h>

using namespace RTT;
using namespace Orocos;
using namespace asc_linear_interp;

namespace atrias {
namespace controller {

class ASCLinearInterp : public TaskContext {
	private:
		// The data points.
		double *values;

		// This points to any values we've dynamically allocated.
		double *valuesOnHeap;
		int    numValues;

		// The interval in which the interpolation occurs.
		double a;
		double b;

		// Operations
		double runController(double input);

		/** @brief Inputs the array of points from which to sample.
		  * @param samples    The samples.
		  * @param numSamples The number of samples in \a values[]
		  * @param start      The start of the interval.
		  * @param end        The end of the interval.
		  * @param copy       Whether or not to make a copy of the data -- will break realtime,
		  *                   but is necessary if samples[] won't be available later on.
		  */
		void   inputPoints(double samples[], int numSamples, double start, double end, bool copy);
		
		// Logging
		OutputPort<controller_log_data> logPort;
		
	public:
		// Constructor
		ASCLinearInterp(std::string name);
		
		// Standard Orocos hooks
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
}

#endif
