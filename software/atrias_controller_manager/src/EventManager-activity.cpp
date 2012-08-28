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
                Activity(),
                eventsWaitingSignal(0) {
    cManager = manager;
    done = false;
}

void EventManager::loop() {
    printf("Entered new thread!\n");
    while (!done) {
        printf("Starting the loop!\n");
    	bool process = false;
    	RtOpsEvent event;
    	{
    		//Make sure that eventCallback is not running
            printf("Locking incomingEventsLock!\n");
    		os::MutexLock lock(incomingEventsLock);
    		printf("Locked incomingEventsLock!\n");

			if (!incomingEvents.empty()) {
				event = incomingEvents.front();
				incomingEvents.pop_front();
				process = true;
	            printf("Process is true!\n");
			}
    	}
    	if (process) {
            if (event == eventBeingWaitedOn) {
                cManager->commandPending = false;
                switch (event) {
                    case RtOpsEvent::ACK_DISABLE: {
                        printf("Ack disable!\n");
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
    	}

        if (eventsWaitingSignal.value() == 0 || incomingEvents.empty()) {
            if (cManager->commandPending) {
                //Wait for a limited time for an event to come in
                ASSERT(eventsWaitingSignal.waitUntil(
                        os::TimeService::Instance()->secondsSince(0) +
                        ((RTT::Seconds)RT_OPS_WAIT_TIMEOUT_SECS)),
                        "ERROR! Timed out waiting for RT Ops to acknowledge a command!\n");
                //If we made it this far we've got a message and it's time to process it
            }
            else {
                //Block until an event comes in
                printf("Waiting for eventsWaitingSignaller lock!\n");
                eventsWaitingSignal.wait();
            }
    	}
    }
}

void EventManager::eventCallback(RtOpsEvent event) {
	os::MutexLock lock(incomingEventsLock);
    incomingEvents.push_back(event);
    if (eventsWaitingSignal.value() == 0)
        eventsWaitingSignal.signal();
}

void EventManager::setEventWait(RtOpsEvent event) {
    os::MutexLock lock(incomingEventsLock);
    eventBeingWaitedOn = event;
    cManager->commandPending = true;
}

bool EventManager::breakLoop() {
    done = true;
    eventsWaitingSignal.signal();
    return done;
}

bool EventManager::initialize() {
    done = false;
    return !done;
}

}

}
