/** @file
    @brief Helper functions. */
#ifndef ECATSLAVE_DRV_FUNCS_H
#define ECATSLAVE_DRV_FUNCS_H

#include "ecatbasictypes.h"
#include "ecatslave_osal_aux.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Functions for internal use only */
void*       DRV_GetSlvHandleFromPool(UINT8 SlvIndex);
ECAT_IOMEM  DRV_GetSlvIobaseAddr(void* pSlvHandle);
void        DRV_SetSlvIobaseAddr(void* pSlvHandle, ECAT_IOMEM ioBase);
//void      DRV_EnableXXXX (void* pSlvHandle, ECAT_BOOL UseXXXX);
ECAT_RESULT DRV_AttachSlvBoard(UINT8 BoardNumber, void* pSlvHandle);
ECAT_RESULT DRV_DetachSlvBoard(UINT8 BoardNumber);

#ifdef __cplusplus
}
#endif


#endif /* ECATSLAVE_DRV_FUNCS_H */
