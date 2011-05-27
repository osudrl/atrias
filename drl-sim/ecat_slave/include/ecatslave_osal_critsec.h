/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/** @file
    @brief KPA EtherCAT Slave: process synchronization functions. */
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_CRITSEC_H
#define ECATSLAVE_OSAL_CRITSEC_H

#include "ecatslavetypes.h"

/*! @addtogroup ecatslave_osal_critsec
    @{ */

#ifdef __cplusplus
extern "C" {
#endif

typedef void* ECAT_CRITSEC_PTR;

/*! @brief Should return size (in bytes) of memory needed to
    store "critical section" object. For example it may be implemented so:
    @code

    typedef struct stEcatCriticalSection
    {
        // patform specific data
        pthread_mutex_t     CritSec;
        ECAT_CHAR*          pszName;
    } ECAT_CRITSEC;

    ECAT_WORD EcatSlave_CritSecGetMemSize(void)
    {
        return sizeof(struct stEcatCriticalSection);
    }
    @endcode
    @note Previous name of this function: @anchor EcatGetMemSizeOfCriticalSection */
extern ECAT_WORD EcatSlave_CritSecGetMemSize(void);

/*! @brief Should initialize critical section.
    @param pCritSec Pointer to critical section.
    @param pszName Name of critical section.
    @return @ref ECAT_RESULT "Result code".
    @note Previous name of this function: @anchor EcatInitializeCriticalSection */
extern ECAT_RESULT EcatSlave_CritSecInit(
    void*               pCritSec,
    const ECAT_CHAR*    pszName);

/*! @brief Should destroy critical section.
    @param pCritSec Pointer to critical section.
    @note Previous name of this function: @anchor EcatDeleteCriticalSection */
extern void EcatSlave_CritSecDelete(void* pCritSec);

/*! @brief Should lock critical section. If the CS is currently unlocked, it becomes
    locked and owned by the calling thread, and EcatEnterCriticalSection returns
    immediately. If the CS is already locked by another thread,
    EcatEnterCriticalSection suspends the calling thread until the CS is unlocked.
    @param pCritSec Pointer to critical section.
    @note Previous name of this function: @anchor EcatEnterCriticalSection */
extern void EcatSlave_CritSecEnter(void* pCritSec);

/*! @brief Similar EcatEnterCriticalSection but it should not block the calling thread.
    @param pCritSec Pointer to critical section.
    @return ECAT_TRUE - if entered to critical section.
    @note Previous name of this function: @anchor EcatTryEnterCriticalSection */
extern ECAT_BOOL EcatSlave_CritSecTryEnter(void* pCritSec);

/*! @brief Should unlock critical section.
    @param pCritSec Pointer to critical section.
    @note Previous name of this function: @anchor EcatLeaveCriticalSection */
extern void EcatSlave_CritSecLeave(void* pCritSec);

#ifdef __cplusplus
}
#endif

/*! @} marks end of group */

#endif /* ECATSLAVE_OSAL_CRITSEC_H */
