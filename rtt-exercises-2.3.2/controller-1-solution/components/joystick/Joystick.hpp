/*
 * Joystick.hpp
 *
 *  Created on: 28-okt-2008
 *      Author: sspr
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <rtt/TaskContext.hpp>

#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>
#include <rtt/OperationCaller.hpp>

namespace UseCase
{
	using namespace RTT;

	class Joystick
		: public TaskContext {
	protected:
		double jog_value;
		double scale;
		OutputPort<double> output;

		void setPosition(double d) {
			jog_value = d;
		}
	public:
		Joystick(const std::string& name) :
			TaskContext(name, PreOperational),
			jog_value(0.0),
			scale(1.0)
		{
			this->addProperty("scale", scale).doc("Scale applied to the value given by setPosition.");
			this->addOperation( "setPosition", &Joystick::setPosition, this ).doc("Set new joystick position.").arg("d", "Argument");
			this->addPort( "output", output );
            output.keepLastWrittenValue(true);
		}

		bool configureHook() {
			if ( output.connected() == false ) {
				log(Error) << "Joystick Output port not connected!"<<endlog();
				return false;
			}
			return true;
		}

		bool startHook() {
			return true;
		}

		void updateHook() {
            double steer = scale * jog_value ;
			output.write( steer );
		}

		void errorHook() {
            double steer = scale * jog_value ;
	          if ( steer < 10.0 && steer > -10.0 ) {
	                 this->recover();
	            }
		}

		void stopHook() {
		}

		void cleanupHook() {
		}

	};

}

#endif /* JOYSTICK_HPP_ */
