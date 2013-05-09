#ifndef __ToSubstituteClassName_HPP__
#define __ToSubstituteClassName_HPP__

/*! \file ToSubstituteClassName.hpp
 *  \brief ToSubstitutePackageName subcontroller
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>
// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "ToSubstitutePackageName/controller_log_data.h"

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ToSubstituteClassName : public AtriasController {
     public:
          /** 
            * @brief The constructor for this subcontroller
            * @param parent The instantiating, "parent" controller.
            * @param name   The name for this controller (such as "pdLeftA")
            */
          ToSubstituteClassName(AtriasController *parent, string name);

          /** 
            * @brief The main function for this controller.
            * @return An example output
            *
            * We're overloading this operator to make this class a functor. This makes
            * calling this controller more convenient.
            */
          double operator()();

          // Put functions here

		// Put local variables here.

     private:
          /** 
            * @brief This is our logging port.
            * You may have as many of these as you'd like of various types.
            */
          LogPort<ToSubstitutePackageName::controller_log_data_> log_out;
};

}
}

#endif // __ToSubstituteClassName_HPP__
