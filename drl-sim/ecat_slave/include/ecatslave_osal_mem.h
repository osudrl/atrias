/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file
    @brief This file contains declarations of dynamic memory management functions.
 */
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_MEM_H
#define ECATSLAVE_OSAL_MEM_H

#include "ecatbasictypes.h"

/*! @addtogroup ecatslave_osal_mem
    @{ */

#ifdef __cplusplus
extern "C" {
#endif

/* These dynamic memory management functions must be implemented by a developer */

/*--------------------------------------------------------------------------*/
/*! @brief Should allocate "size" bytes and return a pointer to the allocated memory.
    @param Size Memory size.
    @return Pointer to the allocated memory or ECAT_NULL if the request fails.*/
extern ECAT_LPVOID EcatSlave_MemAlloc(ECAT_DWORD Size);

/*--------------------------------------------------------------------------*/
/*! @brief Should free the memory space pointed to by "pVoid".
    @param pVoid Pointer to memory space. */
extern void EcatSlave_MemFree(ECAT_LPVOID pVoid);

/*--------------------------------------------------------------------------*/
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
extern ECAT_LPVOID EcatSlave_MemRealloc(ECAT_LPVOID pVoid, ECAT_DWORD NewSize, ECAT_DWORD OldSize);

#ifdef __cplusplus
}
#endif

/*! @} marks end of group */


#endif /* ECATSLAVE_OSAL_MEM_H */
