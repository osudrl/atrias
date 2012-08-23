/*
 * GuiPublishTimer.h
 *
 *  Created on: Aug 9, 2012
 *      Author: Michael Anderson
 */

#ifndef GUIPUBLISHTIMER_H_
#define GUIPUBLISHTIMER_H_

#include <rtt/os/TimeService.hpp>

#include <robot_invariant_defs.h>

#define MILLISECOND_IN_NANOSECONDS 1000000LL

namespace atrias {
namespace shared {

class GuiPublishTimer {
private:
    RTT::os::TimeService::nsecs lastTime;
    RTT::os::TimeService::nsecs timeOut;

public:
    GuiPublishTimer(uint32_t msecPerCall) {
        timeOut = ((RTT::os::TimeService::nsecs)(msecPerCall - 1)) * MILLISECOND_IN_NANOSECONDS;
        lastTime = 0;
    }
    virtual ~GuiPublishTimer() { }

    bool readyToSend() {
        RTT::os::TimeService::nsecs thisTime = RTT::os::TimeService::Instance()->getNSecs();
        if ((thisTime - lastTime) > timeOut) {
            lastTime = thisTime;
            return true;
        }
        return false;
    }
};

} /* namespace shared */
} /* namespace atrias */
#endif /* GUIPUBLISHTIMER_H_ */
