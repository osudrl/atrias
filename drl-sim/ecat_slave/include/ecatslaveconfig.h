/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2007
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

******************************************************************************

    Global configuration settings for EtherCAT Slave.

*****************************************************************************/

#ifndef ECATSLAVECONFIG_H
#define ECATSLAVECONFIG_H

/* ECATSLAVE must be _ALWAYS_ defined if you build ecatslave!!! */
#ifndef ECATSLAVE
#define ECATSLAVE
#endif

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   ALL PLATFORM SETTINGS MUST RESIDE
   IN THE FILE "ecatslave-platformconfig.h"
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ecatslave-platformconfig.h"
/* ***************************************************************************
 Here we check whether all configuration parameters are EXPLICITLY set
 in "ecatslave-platformconfig.h" and nothing is forgotten.
 Such a way simplifies porting to other platforms:
 new config parameters arise at compilation stage as soon as they were introduced.
 THIS CHECK IS THE FIRST PLACE WHERE THE NEW CONFIG PARAMETER MUST BE INTRODUCED!
*****************************************************************************/
#ifndef ECATSLAVE_CONFIG_ENABLE_COE
#error  ECATSLAVE_CONFIG_ENABLE_COE: This configuration parameter is unspecified!
#endif

#ifndef ECATSLAVE_CONFIG_ENABLE_EOE
#error  ECATSLAVE_CONFIG_ENABLE_EOE: This configuration parameter is unspecified!
#endif
/* TODO: Add check for all EcatSlave configuration switches !!! */


// /************************* System Params *********************************/
// //TODO
// #ifdef _BIG_ENDIAN
//  #define _BIG_ENDIAN_PLATFORM_
// #else
//  #define _LITTLE_ENDIAN_PLATFORM_
// #endif
//
//
// #define ESC_INTERFACE_WIDTH                 8   /*  8 or 16 bits    */

#define _WIN_CE_8BIT_                       1   /*driver level*/
#define _PPC_8BIT_                          2
#define _PPC_16BIT_                         3
#define _X86_32BIT_                         4
#define _xPC_Target_x86_PCI_16BIT_          5
#define _WIN_CE_PC104_16BIT_                6   /*application level*/


/*! Maximal number of EtherCAT Slave boards supported by EtherCAT Slave software. */
//#define ECATSLAVE_BOARDS_MAX                2 - moved to "ecatslave-platformconfig.h"

// /************************* Chip Params ***********************************/
// #define MAX_DC_LATCH_SUPORTED 2
//
//
//
//
// /************************* Other *****************************************/
// #define ECAT_SLAVE_STACK_SIZE   5000
//
// //#define _DEBUG        // on/off debug messages
// //#define _DEBUG_BUFFER_  // on/off syncmanager debug buffer

#endif /* ECATSLAVECONFIG_H */
