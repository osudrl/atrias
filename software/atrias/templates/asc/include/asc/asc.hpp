#ifndef __ASC_COMPONENT_H__
#define __ASC_COMPONENT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the asc_component subcontroller.
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>
// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_pd/controller_log_data.h"

// Namespaces we're using
using namespace std;
using namespace asc_pd;

// Our namespaces
namespace atrias {
namespace controller {

class ASCComponent : public AtriasController {
     public:
          /** 
            * @brief The constructor for this subcontroller
            * @param parent The instantiating, "parent" controller.
            * @param name   The name for this controller (such as "pdLeftA")
            */
          ASCPD(AtriasController *parent, string name);

          /** 
            * @brief The main function for this controller.
            * @return An example output
            *
            * We're overloading this operator to make this class a functor. This makes
            * calling this controller more convenient.
            */
          double operator()(double example);

		// Put local variables here.
		double example_local_variable;

     private:
          /** 
            * @brief This is our logging port.
            * You may have as many of these as you'd like of various types.
            */
          LogPort<controller_log_data> log_out;
};

}
}

#endif
