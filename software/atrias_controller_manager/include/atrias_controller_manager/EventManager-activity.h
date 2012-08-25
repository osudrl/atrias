/*
 * EventManager-activity.h
 *
 *  Created on: Aug 21, 2012
 *      Author: Michael Anderson
 */

#ifndef EventManager_ACTIVITY_H_
#define EventManager_ACTIVITY_H_

//C++
#include <list>
#include <map>

// Orocos
#include <rtt/Activity.hpp>
#include <rtt/os/Semaphore.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

// Atrias
#include <atrias_shared/globals.h>
#include <atrias_controller_manager/ControllerManager-component.h>

using namespace RTT;
using namespace std;

namespace atrias {

namespace controllerManager {
class ControllerManager;

class EventManager: public Activity {
private:
    ControllerManager *cManager;

    /** @brief Used by \a breakLoop() to signal \a loop() to exit.
      */
    volatile bool done;

    /** @brief Protects access to incomingEvents
     */
    os::Mutex incomingEventsLock;

    list<RtOpsEvent> incomingEvents;
    RtOpsEvent       eventBeingWaitedOn;

    public:
        /** @brief Initializes this EventManager.
         */
        EventManager(ControllerManager *manager);

        /** @brief Run by Orocos. Is the main loop for the controllers..
         */
        void loop();

        /**
         * @brief Informs the loop about an incoming event message from RT Ops.
         * @param event The event that RT Ops sent.
         */
        void eventCallback(RtOpsEvent event);

        /**
         * @brief Sets the event manager to perform the associated action when
         * the specified event occurs.
         * @param event The event to watch for.
         */
        void setEventWait(RtOpsEvent event);

        /** @brief Run by Orocos to shut down the main loop.
         * @return Success.
         */
        bool breakLoop();

        /** @brief Initializes the EventManager. Run before \a loop().
         * @return Success.
         */
        bool initialize();
};

}

}

#endif /* EventManager_ACTIVITY_H_ */
