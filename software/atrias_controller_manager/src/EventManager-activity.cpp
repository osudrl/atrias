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
                eventSignal(0) {
    cManager = manager;
}

void EventManager::loop() {
    while (!done) {
printf("START LOOP!\n");
        if (!incomingEvents.empty()) {
            RtOpsEvent event = incomingEvents.front();
            incomingEvents.pop_front();
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
        }
printf("Event signal value before: %i\n", eventSignal.value());
        if (cManager->commandPending) {
printf("STARTING TO WAIT FOR COMMAND!\n");
            ASSERT(eventSignal.waitUntil(os::TimeService::Instance()->secondsSince(0) + ((RTT::Seconds)RT_OPS_WAIT_TIMEOUT_SECS)),
                "ERROR! Timed out waiting for RT Ops to acknowledge a command!\n");
        }
        else {
            while (true) {
                if 
        }

printf("Event signal value after: %i\n", eventSignal.value());
    }
}

void EventManager::eventCallback(RtOpsEvent event) {
    incomingEvents.push_back(event);
printf("Signalling event! Signal current count: %i!\n", eventSignal.value());
    eventSignal.signal();
printf("Once signalled, eventSignal value is: %i\n", eventSignal.value());
}

void EventManager::setEventWait(RtOpsEvent event) {
    eventBeingWaitedOn = event;
    cManager->commandPending = true;
    os::MutexLock lock(incomingEventsLock);
}

bool EventManager::breakLoop() {
    done = true;
    eventSignal.signal();
    return done;
}

bool EventManager::initialize() {
    done = false;
    return !done;
}

}

}
