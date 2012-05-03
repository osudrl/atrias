/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/** @file
    @brief KPA EtherCAT Slave: abstraction layer for SDK. */
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_AUX_H
#define ECATSLAVE_OSAL_AUX_H

#include "ecatslavetypes.h"

/** @addtogroup ecatslave_osal_aux
    @{ */

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------------*/
/** @brief Function that prints a formatted message to somewhere defined by a developer.
    This function may be equivalent to printf (or printk for linux kernel).
    @return Similar to printf. */
extern int EcatSlave_Printf(char* format, ...);


/*--------------------------------------------------------------------------*/
/*! @brief Should divide signed 64-bit values (n = n/base).
    @param n - initially it contains dividend value,
    after return it contains result of division.
    @param base Divisor.
    @return Remainder of division. */
extern ECAT_INT64 EcatSlave_DoDivSigned(ECAT_INT64* n, ECAT_INT64 base);


/*--------------------------------------------------------------------------*/
/*! @brief Should start a custom thread that periodically polls ESC and
    processes information received while polling.
    This function is called inside startEcatSlaveMS() only
    if PollTime used for setEcatSlavePollTimeMS() is not 0.
    See OSAL implementation sample for how it should be implemented.
    @param BoardNumber Board number.
    @param PollTimeMcs Poll time (nanoseconds).
    @return @ref ECAT_RESULT "Result code". */
extern ECAT_RESULT EcatSlave_StartPollThread(UINT8 BoardNumber, UINT32 PollTimeMcs);

/*! @brief Should stop a custom thread started by EcatSlave_StartPollThread().
    This function is called inside stopEcatSlaveMS() only
    if PollTime used for setEcatSlavePollTimeMS() is not 0.
    See OSAL implementation sample for how it should be implemented.
    @param BoardNumber Board number.
    @return @ref ECAT_RESULT "Result code". */
extern ECAT_RESULT EcatSlave_StopPollThread(UINT8 BoardNumber);


#ifdef __cplusplus
}
#endif

/*! @} marks end of group */

#endif /* ECATSLAVE_OSAL_AUX_H */
