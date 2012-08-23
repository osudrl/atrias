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
        if (!incomingEvents.empty()) {
            RtOpsEvent event = incomingEvents.front();
            incomingEvents.pop_back();
            if (event == eventBeingWaitedOn) {
                switch (event) {
                    case RtOpsEvent::ACK_DISABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_STOPPED, true);
                        break;
                    }
                    case RtOpsEvent::ACK_ENABLE: {
                        cManager->setState(ControllerManagerState::CONTROLLER_RUNNING, true);
                        break;
                    }
                    case RtOpsEvent::ACK_E_STOP: {
                        cManager->setState(ControllerManagerState::CONTROLLER_ESTOPPED, true);
                        break;
                    }
                    case RtOpsEvent::ACK_NO_CONTROLLER_LOADED: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED, true);
                        break;
                    }
                    case RtOpsEvent::ACK_RESET: {
                        cManager->setState(ControllerManagerState::NO_CONTROLLER_LOADED, true);
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
        eventSignal.wait();
    }
}

void EventManager::cycleLoop() {
    RTT::os::MutexLock lock(signalLock);
    if (!eventSignal.value())
        eventSignal.signal();
}

void EventManager::eventCallback(RtOpsEvent event) {
    RTT::os::MutexLock lock(signalLock);
    incomingEvents.push_back(event);
    eventSignal.signal();
}

void EventManager::setEventWait(RtOpsEvent event) {
    eventBeingWaitedOn = event;
    cManager->commandPending = true;
}

bool EventManager::breakLoop() {
    done = true;
    cycleLoop();
    return done;
}

bool EventManager::initialize() {
    done = false;
    return !done;
}

}

}
