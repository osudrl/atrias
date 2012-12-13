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
    while (!done) {
    	bool process = false;
    	rtOps::RtOpsEvent event;
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
                switch (event) {
                    case rtOps::RtOpsEvent::ACK_DISABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_STOPPED);
                        break;
                    }
                    case rtOps::RtOpsEvent::ACK_ENABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_RUNNING);
                        break;
                    }
                    case rtOps::RtOpsEvent::ACK_E_STOP: {
                        cManager->setState(ControllerManagerState::CONTROLLER_ESTOPPED);
                        break;
                    }
                    case rtOps::RtOpsEvent::ACK_NO_CONTROLLER_LOADED: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED);
                        break;
                    }
                    case rtOps::RtOpsEvent::ACK_RESET: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED, true);
                        break;
                    }
                }

                {
                    os::MutexLock lock(cManager->commandPendingLock);
                    cManager->commandPending = false;
                }

                // Attempt to process all commands in queue
                while (cManager->tryProcessCommand());
            }
            else {
                switch (event) {
                    case rtOps::RtOpsEvent::CM_COMMAND_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                    case rtOps::RtOpsEvent::CONTROLLER_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                    case rtOps::RtOpsEvent::MEDULLA_ESTOP: {
                        cManager->throwEstop(false);
                        break;
                    }
                }
            }
    	}

        if (eventsWaitingSignal.value() == 0 || incomingEvents.empty()) {
            /*bool pending;
            {
                os::MutexLock lock(cManager->commandPendingLock);
                pending = cManager->commandPending;
            }
            if (pending) {
                //Wait for a limited time for an event to come in
                printf("[CManager] Going into eventsWaitingSignal wait!\n");
                ASSERT(eventsWaitingSignal.waitUntil(
                        os::TimeService::Instance()->secondsSince(0) +
                        ((RTT::Seconds)RT_OPS_WAIT_TIMEOUT_SECS)),
                        "ERROR! Timed out waiting for RT Ops to acknowledge a command!\n");
                //If we made it this far we've got a message and it's time to process it
            }
            else*/ {
                //Block until an event comes in
                eventsWaitingSignal.wait();
            }
    	}
    }
}

void EventManager::eventCallback(rtOps::RtOpsEvent event) {
    os::MutexLock lock(incomingEventsLock);
    incomingEvents.push_back(event);
    if (eventsWaitingSignal.value() == 0)
        eventsWaitingSignal.signal();
}

void EventManager::setEventWait(rtOps::RtOpsEvent event) {
    {
        os::MutexLock lock(incomingEventsLock);
        eventBeingWaitedOn = event;
    }
    if (eventsWaitingSignal.value() == 0)
        eventsWaitingSignal.signal();
    os::MutexLock commandLock(cManager->commandPendingLock);
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

// vim: expandtab:sts=4
