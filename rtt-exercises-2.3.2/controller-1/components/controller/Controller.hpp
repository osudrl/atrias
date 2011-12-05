/*
 * Controller.hpp
 *
 *  Created on: 29-okt-2008
 *      Author: sspr
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <rtt/TaskContext.hpp>

#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>

namespace UseCase
{
	using namespace RTT;
	class Controller
		: public RTT::TaskContext {
	protected:

		double Kp;
		OutputPort<double> steer;
		InputPort<double> target;
		InputPort<double> sense;

	public:
		Controller(const std::string& name) :
			TaskContext(name, PreOperational),
			Kp(50.0)
		{
			this->provides()->addProperty("Kp", Kp).doc("Proportional Gain");
			this->provides()->addPort( "Steer", steer ).doc("The output signal of this component.");
			this->provides()->addPort( "Target", target ).doc("Target signal");
			this->provides()->addPort( "Sense", sense ).doc("Measurement signal");
		}

		bool configureHook() {
			/**
			 * Exercise: The Kp property should have been read from an XML
			 * file 'Controller.cpf', using the Deployment Component
			 * (see deployment/application.cpf).
			 * Check here if it contains a valid value.
			 * If Kp is smaller or equal to zero, return false, otherwise true.
			 */
			return true;
		}

		bool startHook() {
			return true;
		}

		void updateHook() {
            double target_sample, sense_sample;
            if ( target.read(target_sample) && sense.read(sense_sample) )
                steer.write( Kp * (target_sample - sense_sample ) );
		}

		void stopHook() {
		}

		void cleanupHook() {
		}

	};

}

#endif /* CONTROLLER_HPP_ */
