/*****************************************************************************
        EtherCAT Master

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file
    @brief OSAL interface: Time-related functions. */
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_TIMER_H
#define ECATSLAVE_OSAL_TIMER_H

#include "ecatbasictypes.h"

/*! @addtogroup ecatslave_osal_timer
    @{ */

#ifdef __cplusplus
extern "C" {
#endif

// /*-! @brief Should delay the execution of the calling task for a specified time interval.
//     @param TimeNs time interval measured in nanoseconds. */
// void EcatTimerNsSleep(ECAT_TIME_NS TimeNs);

/** @brief Should get current system time measured in nanoseconds.
    @param pTimeNs pointer to variable that receives value of current system time.
    @return #ECAT_TRUE on success, #ECAT_FALSE otherwise. */
extern ECAT_BOOL EcatSlave_TimerGetCurTimeStampNs(ECAT_TIME_NS* pTimeNs);


/*! @brief Should delay the execution of the calling task for a specified time interval.
    @param timeToSleepNs Time interval measured in nanoseconds. */
extern void EcatSlave_TimerSleepNs(ECAT_TIME_NS timeToSleepNs);


// /*-! @brief Should return number of nanoseconds passed since 01 Jan 2000 00:00:00 GMT.
//     @return 64-bit value: number of nanoseconds passed since 01 Jan 2000 00:00:00 GMT. */
// ECAT_ULONGLONG EcatGetSystemTimeNs(void);
//
// /*-! @brief Should get clock's minimal resolution i.e. smallest possible execution delay.
//     @param pTimeMinResolution Pointer to value that receives minimal
//     clock resolution measured in microseconds.
//     @return #ECAT_TRUE if succeeded, #ECAT_FALSE otherwise. */
// ECAT_BOOL EcatTimerGetMinClockResolution(
//     ECAT_TIME_US* pTimeMinResolution);

#ifdef __cplusplus
}
#endif

/*! @} marks end of group */

#endif /* ECATSLAVE_OSAL_TIMER_H */

