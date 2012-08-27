/*
 * EventManager-activity.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: Michael Anderson
 */

#include <atrias_controller_manager/EventManager-activity.h>

namespace atrias {

namespace controllerManager {

EventManager::EventManager(ControllerManager *manager) :
                Activity() {
    cManager = manager;
}

void EventManager::loop() {
    while (!done) {
    	bool process = false;
    	RtOpsEvent event;
    	{
    		//Make sure that eventCallback is not running
    		os::MutexLock lock(incomingEventsLock);

			if (!incomingEvents.empty()) {
				event = incomingEvents.front();
				incomingEvents.pop_front();
				process = true;
			}
    	}
    	if (process) {
            if (event == eventBeingWaitedOn) {
                cManager->commandPending = false;
                switch (event) {
                    case RtOpsEvent::ACK_DISABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_STOPPED);
                        break;
                    }
                    case RtOpsEvent::ACK_ENABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_RUNNING);
                        break;
                    }
                    case RtOpsEvent::ACK_E_STOP: {
                        cManager->setState(ControllerManagerState::CONTROLLER_ESTOPPED);
                        break;
                    }
                    case RtOpsEvent::ACK_NO_CONTROLLER_LOADED: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED);
                        break;
                    }
                    case RtOpsEvent::ACK_RESET: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED);
                        break;
                    }
                }
            }
            else {
                switch (event) {
                    case RtOpsEvent::CM_COMMAND_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                    case RtOpsEvent::CONTROLLER_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                    case RtOpsEvent::MEDULLA_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                }
            }
            {
            	//Make sure eventCallback isn't running before we continue
            	os::MutexLock lock(incomingEventsLock);

				if (incomingEvents.empty()) {
					//We're out of events to process, so re-lock the signaller
					eventsWaitingSignaller.lock();
				}
            }
        }
        if (cManager->commandPending) {
        	//Wait for a limited time for an event to come in
        	os::MutexTimedLock timedLock(eventsWaitingSignaller,
        			os::TimeService::Instance()->secondsSince(0) +
        			((RTT::Seconds)RT_OPS_WAIT_TIMEOUT_SECS));

        	//If no event has come in shut down with a message
            ASSERT(timedLock.isSuccessful(),
            		"ERROR! Timed out waiting for RT Ops to acknowledge a command!\n");

            //If we made it this far we've got a message and it's time to process it
        }
        else {
        	//Block until an event comes in
        	os::MutexLock lock(eventsWaitingSignaller);
        }
    }
}

void EventManager::eventCallback(RtOpsEvent event) {
	os::MutexLock lock(incomingEventsLock);
    incomingEvents.push_back(event);
    eventsWaitingSignaller.unlock();
}

void EventManager::setEventWait(RtOpsEvent event) {
    eventBeingWaitedOn = event;
    cManager->commandPending = true;
    os::MutexLock lock(incomingEventsLock);
}

bool EventManager::breakLoop() {
    done = true;
    eventsWaitingSignaller.unlock();
    return done;
}

bool EventManager::initialize() {
    done = false;
    return !done;
}

}

}
