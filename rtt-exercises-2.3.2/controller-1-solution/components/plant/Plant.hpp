/*
 * Plant.hpp
 *
 *  Created on: 29-okt-2008
 *      Author: sspr
 */

#ifndef PLANT_HPP_
#define PLANT_HPP_

#include <rtt/TaskContext.hpp>

#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>
#include <rtt/OperationCaller.hpp>

namespace UseCase
{
	using namespace RTT;
	class Plant
		: public RTT::TaskContext {
	protected:
        double current;
		Property<double> inertia;
		InputPort<double> input;
		OutputPort<double> output;

	public:
		Plant(const std::string& name)
			: TaskContext(name, PreOperational),
            current(0),
			inertia("inertia","Inertia of the plant.", 10.0),
			input("Input"),
			output("Output")
		{
			this->properties()->addProperty( inertia);
			this->ports()->addPort( input );
			this->ports()->addPort( output );
		}

		bool configureHook() {
			return true;
		}

		bool startHook() {
			return true;
		}

		void updateHook() {
            double input_sample;
            if ( input.read(input_sample) ) {
                current += this->engine()->getActivity()->getPeriod()*input_sample /inertia.value();
            }
            output.write( current );
		}

		void stopHook() {
		}

		void cleanupHook() {
		}

};

}

#endif /* PLANT_HPP_ */
