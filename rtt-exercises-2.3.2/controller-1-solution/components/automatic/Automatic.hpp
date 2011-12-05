/*
 * Automatic.hpp
 *
 *  Created on: 28-okt-2008
 *      Author: sspr
 */

#ifndef AUTOMATIC_HPP_
#define AUTOMATIC_HPP_

#include <rtt/TaskContext.hpp>

#include <rtt/Port.hpp>
#include <rtt/OperationCaller.hpp>

namespace UseCase
{
	using namespace RTT;

	class Automatic
		: public RTT::TaskContext
	{
	protected:
		double target, step, current;
        double baseStep;
		bool target_reached;
		Operation<bool(double)> move;
		Operation<bool(void)> moveDone;
		OutputPort<double> atposition;
		OutputPort<double> output;
		InputPort<double> input;

		bool move_impl(double d) {

            if ( input.read( current) ) {
                step = baseStep;
                target = current + d;
            } else {
                log(Error) <<"Move not accepted: input port has no data."<<endlog();
                return false;
            }

			// step and d must have same sign.
			if ( d * step < 0)
				step = -step;

            if ( fabs(d) < fabs(step) ) {
				output.write( target );
                current = target;
				log(Info) << "Did instant move to target "<< target <<endlog();
				return true;
			}

			target_reached = false;
			log(Info) << "Moving to target " << target << endlog();
			return true;
		}

		bool atpos_impl() {
            if ( fabs(current - target) < baseStep )
				return true;
			return false;
		}

	public:
		Automatic(const std::string& name) :
			TaskContext(name, PreOperational),
            target(0.0), step(0.0), current(0.0),
            baseStep(0.0), target_reached(true),
			move("move",&Automatic::move_impl, this, OwnThread),
			moveDone("moveDone", &Automatic::atpos_impl, this),
			atposition("atposition"),
			output("output"),
			input("input")
		{
			this->addOperation( move ).doc("Move over a distance").arg("d", "Distance to move");
            this->addOperation( moveDone ).doc("Is the current move command done ?");
			this->ports()->addPort( atposition ).doc("Written when the position is reached");
			this->ports()->addPort( output ).doc("Sends the interpolated target position.");
			this->ports()->addPort( input ).doc("Reads in the current position.");
		}

		bool configureHook() {
			Logger::In in("Automatic::configureHook()");
			if ( output.connected() == false ) {
				log(Error) << "Automatic Output port not connected!"<<endlog();
				return false;
			}
			if ( input.connected() == false ) {
				log(Error) << "Automatic Input port not connected!"<<endlog();
				return false;
			}

			if ( this->getPeriod() >= 1 ) {
				log(Error) << "Please choose an execution period < 1."<<endlog();
				return false;
			}

			return true;
		}

		bool startHook() {
			// set our interpolation step to 1 unit/s.
            baseStep = this->getPeriod();
            step = 0.0;

			return true;
		}

		void updateHook() {
            current += step;
			output.write( current );
			if ( atpos_impl() && !target_reached) {
				output.write( target );
                current = target;
				log(Info) << "Position reached at " << target <<endlog();
				atposition.write(target);
				step = 0.0;
				target_reached = true;
			}
		}

		void stopHook() {
			step = 0.0;
		}

		void cleanupHook() {
		}

	};
}

#endif /* AUTOMATIC_HPP_ */
