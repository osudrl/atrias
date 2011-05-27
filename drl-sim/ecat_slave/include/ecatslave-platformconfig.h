/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2008
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/** @file
    @brief Global configuration settings for EtherCAT Slave. */
/****************************************************************************/

#ifndef ECATSLAVE_PLATFORMCONFIG_H
#define ECATSLAVE_PLATFORMCONFIG_H

#include "ecatslave_kpapciboard_defs.h"

/************************* System Params ************************************/
// TODO: Debug this...
// #if _BIG_ENDIAN == 1
//     #define _BIG_ENDIAN_PLATFORM_
//     #warning Compiling for BIG ENDIAN PLATFORM
// #elif __LITTLE_ENDIAN == 1
//     #define _LITTLE_ENDIAN_PLATFORM_
//     #warning Compiling for LITTLE ENDIAN PLATFORM
// #else
//     #error
// #endif

#define _LITTLE_ENDIAN_PLATFORM_

#ifdef _BIG_ENDIAN
#error
#endif
//_LITTLE_ENDIAN

#define ESC_INTERFACE_WIDTH                 16   /*  8 or 16 bits    */


#define __ESC_CPU_SYSTEM__                  _X86_32BIT_

#define ECATSLAVE_IRQ_HANDLER               int (*)(void* pContextData, int ContextDataSize)

/************************** Board Params **************************************/
//#define _USE_PCI_                   1
#define ECATSLAVE_BOARDS_MAX        1


/************************* Chip Params **************************************/
#define MAX_DC_LATCH_SUPORTED 2

/************************* Other ********************************************/
#define ECAT_SLAVE_STACK_SIZE   5000

#define ECATSLAVE_CONFIG_MAX_DMA_BUFF_SIZE                      0x2000 /* 8192 */
//#define _DEBUG        // on/off debug messages
//#define _DEBUG_BUFFER_  // on/off syncmanager debug buffer

#define _COE_REQUEST_MESSAGES_
#define _COE_CHANGED_MESSAGES_
#define _COE_CHANGED_MESSAGES_WAIT_RETURN_

#define _COE_PROCESS_IMAGE_MESSAGES_                            0
#define _COE_GENERATE_PDO_MAP_                                  0

#if defined(_COE_REQUEST_MESSAGES_) || defined(_COE_CHANGED_MESSAGES_)
#define _COE_SDO_EVENTS_FILTER_                                 0
#endif
#if _COE_SDO_EVENTS_FILTER_ == 1
#define _COE_DISABLE_SDO_MESSAGES_FOR_PDO_MAPPING_OBJECTS_      0
#endif

#define GLOBAL_DEF_MEMORY_POOL_SIZE             (1 * 1024 * 1024) /* 1 Mb */

//if you disable OD memory pool you'll need to increase global memory pool size
#define OD_USES_OWN_MEMORY_POOL                                 1
#define OD_MEMORY_POOL_SIZE                     (5 * 1024 * 1024) /* 5 Mb */

// max number of objects to be cached
#define MAX_CACHED_OBJ_POINTERS                 300

// pointers to SubIndexes [1...MAX_SUB_INDEX_FOR_CACHE] of each cached Object will be cached
// SubIndexes 0 is not cached
#define MAX_SUB_INDEX_FOR_CACHE                 32



//#define ECAT_SLAVE_MODULE_LICENSE   "GPL"
#define ECAT_SLAVE_MODULE_LICENSE   "Proprietary"

/*! Defines period of timer. Used only in RTAI version of EtherCAT Slave. */
#define ECATSLAVE_RTAI_TIMER_PERIOD_NS      100000 /* 100 microsec */


//#define ECATSLAVE_CONFIG_SEPARATE_BOARD_DRIVER      1  /* internal use only*/
#define ECATSLAVE_CONFIG_NEW_IRQ_HANDLING           1  /* internal use only*/
//#define ECATSLAVE_CONFIG_DMA_TASK_SCHEDULER_ENABLED 1  /* internal use only*/

// #ifdef  ECATSLAVE_CONFIG_NEW_IRQ_HANDLING
// #   warning ECATSLAVE_CONFIG_NEW_IRQ_HANDLING  defined!
// #else
// #   warning ECATSLAVE_CONFIG_NEW_IRQ_HANDLING not defined. Using old irq handling
// #endif


/*****************************************************************************
    Below are switches for EcatSlave features.
    !!! Allowed values are "0" and "1" ONLY !!!
    1 means that a feature is enabled
    0 means that a feature is disabled
*****************************************************************************/
#define ECATSLAVE_CONFIG_ENABLE_COE         1
#define ECAT_MAILBOX_COE_DEFAULT_BUFFER_SIZE    256  /*!< Default buffer size for CoE data requests/responses(Rx&Tx buffers) */
#define ECAT_MAILBOX_COE_MAX_IN_OBJECTS         2    /*!< Length of a queue for incoming requests from master */
#define ECAT_MAILBOX_COE_MAX_OUT_OBJECTS        2    /*!< Length of a queue for outgoing responses to master  */


#define ECATSLAVE_CONFIG_ENABLE_EOE         1
#define ECATSLAVE_CONFIG_ENABLE_EOE_API     1

#define ECATSLAVE_CONFIG_ENABLE_VOE         1
#define ECATSLAVE_CONFIG_ENABLE_FOE         1


#define ECAT_LOGGER_ENABLE                  1
#define ECAT_LOGGER_PRINT_TO_STDOUT         1
#define ECAT_LOGGER_MAX_MSG_SIZE            256
#define ECAT_LOGGER_RING_SIZE               200

//#define KPADEBUG_PRINT_LOCATION EcatSlave_Printf("File:%s, line:%d, func:%s\n", __FILE__, __LINE__, __func__);

#endif /* ECATSLAVE_PLATFORMCONFIG_H */
