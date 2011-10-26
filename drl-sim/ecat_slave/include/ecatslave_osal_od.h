/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/** @file
    @brief KPA EtherCAT Slave: abstraction layer for Object Dictionary. */
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_OD_H
#define ECATSLAVE_OSAL_OD_H

#include "ecatslavetypes.h"

/*! @addtogroup ecatslave_osal_od
    @{ */


#ifdef __cplusplus
extern "C" {
#endif


/* These dynamic memory management functions must be implemented by a developer */

/** @brief Should allocate memory from a pool used by Object Dictionary.
    In a simplest case these function may be just wrappers for
    @ref ecatslave_osal_mem "dynamic memory management functions for
    slave's main memory pool".
    @param Size Number of bytes to allocate. */
extern ECAT_LPVOID EcatSlave_OdMemAlloc(ECAT_DWORD Size);

/** @brief Should free memory space previously allocated by
    EcatSlave_OdMemAlloc().
    @param pVoid This pointer should point to memory space to be freed. */
extern void EcatSlave_OdMemFree(ECAT_LPVOID pVoid);

/*! @brief Should change the size of the memory block.
    The contents MUST be unchanged to the minimum of the old and new sizes;
    newly allocated memory will be uninitialized.
    If "pVoid" is #ECAT_NULL then the call is equivalent to allocating of
    "NewSize" bytes. If "NewSize" is equal to zero then the call
    is equivalent to freeing of memory space pointed to by "pVoid".
    @param pVoid Pointer to old memory block.
    @param NewSize New memory size.
    @param OldSize Old memory size.
    @return Pointer to the new allocated memory or #ECAT_NULL if the request fails.*/
extern void* EcatSlave_OdMemRealloc(ECAT_LPVOID pVoid, ECAT_DWORD NewSize, ECAT_DWORD OldSize);

/** @brief This function should clear Object Dictionary memory pool
    associated with a slave board number "BoardNumber",
    i.e. turn it to initial unfragmented state.
    This function is called from within stopEcatSlaveOdMS() when
    all memory allocated from OD memory pool was completely freed but
    not yet allocated again.\n
    That means also that this function may be used as a nitifier signalling
    that a memory pool associated with "BoardNumber" can be unfragmented.\n
    This is a developer's decision whether to unfragment(free) memory pool or
    ignore this event and do nothing in this function.
    @param BoardNumber Ecat Slave board number (starts from 0). */
extern void EcatSlave_OdMemPoolFree(UINT8 BoardNumber);

/** @brief Should allocate memory from a pool other than
    the one used by EcatSlave_OdMemAlloc(), EcatSlave_OdMemFree().
    That pool is needed if EcatSlave_OdMemPoolFree() is not a stub function
    and really frees OD memory pool.
    @param Size Number of bytes to allocate. */
extern ECAT_LPVOID EcatSlave_OdExtMemAlloc(ECAT_DWORD Size);

/** @brief Should free memory space previously allocated by
    EcatSlave_OdMemExtMalloc().
    @param pVoid This pointer should point to memory space to be freed. */
extern void EcatSlave_OdExtMemFree(ECAT_LPVOID pVoid);


#ifdef __cplusplus
}
#endif

/*! @} marks end of group */

#endif /* ECATSLAVE_OSAL_OD_H */
