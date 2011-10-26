/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatslaveapi.h
    @brief EtherCAT Slave API declarations (same for user-space and for
        kernel's real-time context currently). */
/****************************************************************************/

#ifndef ECATSLAVEAPI_H
#define ECATSLAVEAPI_H

// Show/don't show not implemented functions in API description.
// This macro is to be defined ONLY in doxygen configuration file.
//#define DONT_SHOW_NOTIMPL_FUNCTIONS

#include "ecatslavetypes.h"
#include "ecatcmn.h"
//#include "ecatslavem2mapi.h"  /* note: include this file explicitly in a place that needs M2M api! */
#include "ecatslave_to_MS_api.h"
#include "ApiObjectDictionary.h"

#ifdef __cplusplus
extern "C" {
#endif


/*! @defgroup ecat_slave EtherCAT Slave API*/

/*!**************************************************************************/
/*! @defgroup ecat_slave_init Init/Config/Release functions
    @brief SyncManager settings, Slave callbacks set/reset, etc.
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_io_functions I/O functions: PI update, etc.
    @brief SyncManager buffers I/O update, etc.
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_mem I/O functions: EC slave memory r/w, etc.
    @brief EC slave memory read/write, EEPROM read/write, etc.
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_state AL/DL status and control
    @brief
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_fsm Slave states handling
    @brief
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_irq IRQ-mode related functions
    @brief Functions to set/reset IRQ handler, to set/reset IRQ masks, etc.
    @ingroup ecat_slave */

#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*! @defgroup ecat_slave_statistics_diagnostics Statistic & diagnostic functions
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_logging Log functions
    @ingroup ecat_slave */
#endif

/*! @defgroup ecat_slave_dc DC functions
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_mailbox_coe CoE protocol API
    @see ecat_slave_od
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_mailbox_voe VoE protocol API
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_mailbox_eoe EoE protocol API
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_mailbox_foe FoE protocol API
    @ingroup ecat_slave
    @see ecat_slave_foe_osal */

/*! @defgroup ecat_slave_logger Ecat slave logger api
    @ingroup ecat_slave */

/*! @defgroup ecat_slave_aux Auxiliary functions
    @ingroup ecat_slave */

/*!**************************************************************************/


/*!**************************************************************************/
/*! @addtogroup ecat_slave_init
    @{
*****************************************************************************/

#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*! @brief Sets initial parameters of EtherCAT Slave: hardware addresses, sizes, names, modes, etc.
    @note Implementation of this function is specific for each platform used.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pData Pointer to additional data.
           First 2 bytes must contain full size of data(including these 2 bytes) that pContextData points to.
    @param DataSize Size of the data that pData pointer points to (it will be compared with that first 2 bytes). */
ECAT_RESULT setEcatSlaveParametersMS(
    IN UINT8        BoardNumber,
    IN UINT8*       pData,
    IN UINT32       DataSize);
#endif /* ifndef DONT_SHOW_NOTIMPL_FUNCTIONS */

/*! @note Implementation of this function dependends on using platform.
    @brief Initialize hardware and system (if necessary),
    runs KPA EtherCAT slave and loads Object Dictionary from file
    pointed by pOdFileName.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pOdFileName Pointer to OD filename string.
    @return @ref ECAT_RESULT "Result code".
    @see setKpaEcatPc104BaseAdr, checkKPAEcatSlaveRunning, getKPAOdRunning,
    KPAEcatSlaveMainLoopStep, stopKPAEcatSlave*/
ECAT_RESULT runKpaEcatSlave (
    IN UINT8        BoardNumber,
    IN ECAT_CHAR*   pOdFileName);

/*! @brief Returns EtherCAT Slave controller type (e.g. 0x11 for ET1100).
    @ingroup ecat_slave_aux
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pEscType Pointer to a byte that receives ESC register 0x0000: Type of EtherCAT slave controller. *
    @return @ref ECAT_RESULT "Result code". */
ECAT_RESULT getEcatSlaveChipType(
    IN UINT8    BoardNumber,
    OUT UINT8*  pEscType);

/*! @brief Returns TRUE if slave is running, otherwise FALSE.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see runKpaEcatSlave */
ECAT_BOOL checkKPAEcatSlaveRunning(IN UINT8 BoardNumber);

/*! @note Implementation of this function depends on the used platform.
    @ingroup ecat_slave_aux
    @brief Sets base address for KPA PC104 Ecat slave board and checks Ecat chip type.
           The function uses IO port address 0x80 for writing.
    @note  If this function fails for some reason for a slave that was working
           before then one of possible reasons may be incorrect checksum in
           slave's EEPROM: therefore try to write (via EtherCAT) definitely
           correct EEPROM file into  slave controller EEPROM again.
    @param adr Base address. Must be multiple of 0x8000.
    @return ECAT_S_OK if Ecat type read from chip is equal to 0x11 (ET1100).
    @see runKpaEcatSlave*/
ECAT_RESULT setKpaEcatPc104BaseAdr (UINT32 adr);

/*! @brief Sets properties of slave's SyncManager.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SmId ID of a SyncManager (0,1,...).
    @param SmType Type of SyncManager (see #tEcatSmType).
    @param SmMinLen Minimum length of SyncManager memory measured in bytes.
    @param SmMaxLen Maximum length of SyncManager memory measured in bytes.
    @param SmMinAddr Minimum physical start address of SyncManager.
    @param SmMaxAddr Maximum physical start address of SyncManager.
    @return  Return value #ECAT_E_INSUFFICIENT_BUF_SIZE means that mailbox SM size
             is more then maximum value supported of FoE for current platform
             (see #ECAT_MAILBOX_FOE_DEFAULT_BUFFER_SIZE).
             @ref ECAT_RESULT "Result code".
    @see getEcatSlaveSmConfigurationMS */
ECAT_RESULT setEcatSlaveSyncManagerMS(
    IN UINT8 BoardNumber,
    IN UINT16  SmId,
    IN tEcatSmType   SmType,
    IN UINT32  SmMinLen,
    IN UINT32  SmMaxLen,
    IN UINT32  SmMinAddr,
    IN UINT32  SmMaxAddr);

/*! @note Implementation of this function depends on the used platform.
    @ingroup ecat_slave_io_functions
    @brief Handles KPA ECAT slave and performs Mailbox exchange.
    The function should be called cyclically.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see runKpaEcatSlave, checkKPAEcatSlaveRunning*/
void KPAEcatSlaveMainLoopStep (UINT8 BoardNumber);

/*! @note Implementation of this function depends on the used platform.
    @brief Stops Ecat slave and Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see runKpaEcatSlave */
ECAT_RESULT stopKPAEcatSlave(IN UINT8 BoardNumber);


/*! @brief Returns EtherCAT slave stack version.
    @ingroup ecat_slave_aux
    @param pMajor Pointer to major part of version.
    @param pMinor Pointer to minor part of version.
    @param pSubLevel Pointer to sublevel of version.
    @param pPrivateNumber Pointer to private number of version.
    @return @ref ECAT_RESULT "Result code"*/
ECAT_RESULT getEcatSlaveStackVersion(
    OUT UINT8* pMajor,
    OUT UINT8* pMinor,
    OUT UINT8* pSubLevel,
    OUT UINT8* pPrivateNumber);

/*! @note Implementation of this function depends on the used platform.
    @ingroup ecat_slave_aux
    @brief Returns Ecat board hardware version.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nVersion Pointer to variable for board hardware version.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT getEcatSlaveBoardVersion(IN UINT8 BoardNumber, OUT UINT16* nVersion);

/*! @note Implementation of this function depends on the used platform.
    @ingroup ecat_slave_aux
    @brief Returns positions of board rotary switches.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nSwitchesPosition Pointer to variable for positions of board rotary switches.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT getEcatRotarySwitches(IN UINT8 BoardNumber, OUT UINT8* nSwitchesPosition);

/*! @note Implementation of this function depends on the used platform.
    @ingroup ecat_slave_aux
    @brief Switches LED on (if onLED == ECAT_TRUE) or off (if onLED == ECAT_FALSE).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param onLED Requested state of LED.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT setEcatLED (IN UINT8 BoardNumber, IN ECAT_BOOL onLED);


/*! @note Implementation and parameters of this function depends on the used platform.
    @brief Sets IRQ handler and enables IRQ using functions of OS.
           The IRQ must be then enabled by #selectEcatSlaveBoardIrqSources and #setSmEventMask (if necessary).
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param IrqHandler IRQ handler function. Type of this function defined as IRQ_HANDLER_TYPE.
    @param Priority IRQ thread priority. Depends on used platform.
    @param pContext Additional platform depended data. See the example for corresponding platform.
    @param pIrqNumber The IRQ line number.
            Input: Irq number to request (for example PC104 EC slave may need this input value)
            Output: Assigned  irq number (for example PCI EC slave returns IRQ number assigned to EC slave board).
    @see releaseEcatSlaveIrqHandler, selectEcatSlaveBoardIrqSources */
ECAT_RESULT setEcatSlaveIrqHandler(
    IN UINT8        BoardNumber,
    IN void*        IrqHandler,
    IN UINT32       Priority,
    IN OUT void*    pContext,
    IN OUT UINT16*  pIrqNumber);


/*! @note Implementation and parameters of this function depends on the used platform.
    @brief Detaches IRQ handler and disabled IRQ using functions of OS.
           The IRQ must be then disabled by #selectEcatSlaveBoardIrqSources and #setSmEventMask (if necessary).
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pContext Additional platform depended data. See the example for corresponding platform.
    @see setEcatSlaveIrqHandler, selectEcatSlaveBoardIrqSources */
ECAT_RESULT releaseEcatSlaveIrqHandler(IN UINT8 BoardNumber, IN OUT void *pContext);


/*! @note Implementation and parameters of this function depends on the used platform.
    @brief Selects source of IRQ request of slave board.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nIrqSrcEn IRQ source specifier\n
           See @ref ECATSLAVE_IRQMASK "EtherCAT Slave board IRQ source masks" \n
    @param nIrqSrcCtrl IRQ control specifier.
            Bit value '0' means rising edge,\n
            bit value '1' means falling edge.

    Control bits can be masked by:\n
    @ref ECATSLAVE_IRQMASK "EtherCAT Slave board IRQ source masks"
    @see setEcatSlaveIrqHandler */
ECAT_RESULT selectEcatSlaveBoardIrqSources (IN UINT8 BoardNumber, UINT16 nIrqSrcEn ,UINT16 nIrqSrcCtrl);


/*! @note Implementation and parameters of this function depends on the used platform.
    @brief Returns Ecat board IRQ status register.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pnBoardIrqStatus Pointer to variable for Ecat board IRQ status register.\n
    This value can be combined with IRQ source masks:\n
           @ref ECATSLAVE_IRQMASK "EtherCAT Slave board IRQ source masks" \n
    @see setEcatSlaveIrqHandler */
ECAT_RESULT getEcatSlaveBoardIrqStatusReg(IN UINT8 BoardNumber, OUT UINT16* pnBoardIrqStatus);

/*! @brief Sets period of I/O polling.
    If PollTimeMicrosec equals 0 then I/O updates are initiated by application.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param PollTimeMicrosec Period of polling in microseconds.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT setEcatSlavePollTimeMS(
    IN UINT8  BoardNumber,
    IN UINT32 PollTimeMicrosec);

/*! @brief Gets period of I/O polling.
    If PollTimeMicrosec equals 0 then I/O updates are initiated by application.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pPollTimeMicrosec Period of polling in microseconds.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlavePollTimeMS(
    IN  UINT8   BoardNumber,
    OUT UINT32* pPollTimeMicrosec);

/*! @brief Sets heartbeat interval.
    If HeartbeatInterval equals 0 then heartbeat is disabled.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param HeartbeatIntervalMillisec Interval of heartbeats.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription*/
ECAT_RESULT setEcatSlaveHeartbeatIntervalMS(IN UINT8 BoardNumber,
    IN UINT32 HeartbeatIntervalMillisec);

/*! @brief Initializes slave.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT initEcatSlaveMS(UINT8 BoardNumber);

/*! @brief Releases slave.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT releaseEcatSlaveMS(UINT8 BoardNumber);

/*! @brief Starts slave.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT startEcatSlaveMS(IN UINT8 BoardNumber);

/*! @brief Stops slave.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT stopEcatSlaveMS(IN UINT8 BoardNumber);

/*! @brief Sets slave's callback function.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to a @link #tEcatSlaveCallbackFunction callback function @endlink which will be added to the callback list.
    @param pContextData Pointer to data which are passed as argument in callback function.
    Both setEcatSlaveCallbackFunctionMS() and removeEcatSlaveCallbackFunctionMS()
    must use pointer to the same context data otherwise
    removeEcatSlaveCallbackFunctionMS will fail!
    @param EventMask Mask for supported @ref tEcatSlaveDriverEvent "events"
        (events that will be  processed by the callback function).
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveCallbackFunctionMS\n
         tEcatSlaveDriverEvent*/
ECAT_RESULT setEcatSlaveCallbackFunctionMS(
    IN UINT8 BoardNumber,
    IN tEcatSlaveCallbackFunction pFunc,
    IN void* pContextData,
    IN UINT32 EventMask);

/*! @brief Remove slave's callback function.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to @link #tEcatSlaveCallbackFunction callback function @endlink which will be removed from the callback list.
    @param pContextData Pointer to data which are passed as argument in callback function.
    Both setEcatSlaveCallbackFunctionMS() and removeEcatSlaveCallbackFunctionMS()
    must use pointer to the same context data otherwise
    removeEcatSlaveCallbackFunctionMS() will fail!
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveCallbackFunctionMS*/
ECAT_RESULT removeEcatSlaveCallbackFunctionMS(
    IN UINT8 BoardNumber,
    IN tEcatSlaveCallbackFunction pFunc,
    IN void* pContextData);

#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*! @brief [obsolete] Sets slave's on-state-change callback function.
    This function is obsolete, use #setEcatSlaveCallbackFunction instead.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to EcatSlave
    @link #tEcatSlaveOnStateChangeCallbackFunction on-state-change callback function @endlink.
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveOnStateChangeCallbackFunction */
ECAT_RESULT setEcatSlaveOnStateChangeCallbackFunctionMS(IN UINT8 BoardNumber,
    IN tEcatSlaveOnStateChangeCallbackFunction pFunc);

/*! @brief [obsolete] Removes slave's on-state-change callback function.
    This function is obsolete, use #removeEcatSlaveCallbackFunctionMS instead.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to EcatSlave
    @link #tEcatSlaveOnStateChangeCallbackFunction on-state-change callback function @endlink.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOnStateChangeCallbackFunctionMS */
ECAT_RESULT removeEcatSlaveOnStateChangeCallbackFunctionMS(
    IN UINT8 BoardNumber,
    IN tEcatSlaveOnStateChangeCallbackFunction pFunc);

/*! @brief Sets maximum size of SyncManager buffer.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of a SyncManager (0,1,...).
    @param MaxBytesCount Maximum size of SyncManager buffer
        measured in bytes.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT setEcatSlaveMaxIoSizeMS(
    IN UINT8  BoardNumber,
    IN UINT16 SyncManagerId,
    IN UINT16 MaxBytesCount);
#endif // #ifndef DONT_SHOW_NOTIMPL_FUNCTIONS

/*! @} (marks end of group "ecat_slave_init") */


/*!**************************************************************************/
/*! @addtogroup ecat_slave_io_functions
    @{
*****************************************************************************/
/*! @brief Returns ECAT_TRUE if Object Dictionary has been started successfully.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see runKpaEcatSlave*/
ECAT_BOOL getKPAOdRunning(IN UINT8 BoardNumber);

/*! @brief Sends values of input objects from Object Dictionary to
    Ecat chip process image on cyclic communication.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wSMIndex SM number of the corresponding process image.
    @return @ref ECAT_RESULT "Result code"
    @see runKpaEcatSlave, setEcatSlaveOdObjectEntryValueMS, checkEcatSlavePiIsAvailable*/
ECAT_RESULT updateOdTxInputsViaPI (
    IN UINT8    BoardNumber,
    IN UINT16   wSMIndex);

/*! @brief Sends values from Ecat chip process image to Object Dictionary
    output objects on cyclic communication.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wSMIndex SM number of the corresponding process image.
    @return @ref ECAT_RESULT "Result code"
    @see runKpaEcatSlave, getEcatSlaveOdObjectEntryValueMS,
    checkEcatSlavePiIsAvailable, checkRxOutputsSmIsFull*/
ECAT_RESULT updateOdRxOutputsViaPI (
    IN UINT8    BoardNumber,
    IN UINT16   wSMIndex);

/*! @brief Returns ECAT_S_OK if SM buffer has been written by Ecat Master
    since latest slave read operation, otherwise ECAT_E_BUF_EMPTY.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nSMNumber SM number to be used.
    @see checkEcatSlavePiIsAvailable*/
ECAT_RESULT checkRxOutputsSmIsFull (
    IN UINT8    BoardNumber,
    UINT16      nSMNumber);

/*! @brief Enables (if mask is ECAT_TRUE) or disables (otherwise) interrupt by SM's buffer read/write.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nSMNumber SM number for which the mask is set.
    @param Mask If this parameter is ECAT_TRUE then interrupt by SM's buffer
           read/write is enabled, otherwise disabled.
    @see handlerEcatSlaveEscIst, clearAlEventMasks */
ECAT_RESULT setSmEventMask (
    IN UINT8    BoardNumber,
    ECAT_WORD   nSMNumber,
    ECAT_BOOL   Mask);

/*! @brief Clears all AL Event Masks.\n
    Used for clearing interrupt requests.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see setSmEventMask, getEcatSlaveAlEventMasks, setEcatSlaveAlEventMasks */
ECAT_RESULT clearAlEventMasks (IN UINT8 BoardNumber);

/*! @brief [obsolete] Gets AL Event Masks.
    @ingroup ecat_slave_irq
    This function is obsolete, use #getEcatSlaveAlEventMaskMS instead.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pMask Pointer to variable for saving the mask.
    @see setEcatSlaveAlEventMasks, clearAlEventMasks, setSmEventMask */
ECAT_RESULT getEcatSlaveAlEventMasks (IN UINT8 BoardNumber, UINT32* pMask);

/*! @brief [obsolete] Sets AL Event Masks. This function is obsolete,
           use #setEcatSlaveAlEventMaskMS instead.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param mask The mask value.
    @see getEcatSlaveAlEventMasks, clearAlEventMasks, setSmEventMask */
ECAT_RESULT setEcatSlaveAlEventMasks (IN UINT8 BoardNumber, UINT32 mask);

/*! @brief Gets value of AL Event Mask Register.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pMask Pointer to variable for saving the mask.
    @see setEcatSlaveAlEventMaskMS */
ECAT_RESULT getEcatSlaveAlEventMaskMS(
    IN UINT8    BoardNumber,
    OUT UINT32* pMask);

/*! @brief Sets value of AL Event Mask Register.
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param Mask The mask value.
    @see getEcatSlaveAlEventMaskMS */
ECAT_RESULT setEcatSlaveAlEventMaskMS(
    IN UINT8    BoardNumber,
    IN UINT32   Mask);

/*! @brief Updates slave's transmit-data.
    This function updates data that are transmitted to EtherCAT.
    From the EtherCAT master's side this data are seen as slave's inputs.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of SyncManager (0,1,...).
    @param pTxInputs Data that slave gives to EtherCAT (for example
        values acquired from physical inputs).
        The size of pTxInputs buffer must be the same as returned
        by #getEcatSlaveConfiguredIoSizeMS.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT updateEcatSlaveTxInputsMS(
    IN UINT8    BoardNumber,
    IN UINT16   SyncManagerId,
    IN UINT8*   pTxInputs);

/*! @brief Updates slave's Process Data with received data.
    This function writes values obtained from EtherCAT to
    the specified output buffer.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of SyncManager (0,1,...).
    @param pRxOutputs Data that slave obtains from EtherCAT (for example
        values that are to be set to physical outputs).
        The size of pRxOutputs buffer must be the same as returned
        by #getEcatSlaveConfiguredIoSizeMS.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT updateEcatSlaveRxOutputsMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  SyncManagerId,
    OUT UINT8*  pRxOutputs);

/*! @brief Performs asynchronous mailbox update.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT performEcatSlaveMailboxUpdateMS(IN UINT8 BoardNumber);

/*! @brief Gets SyncManagers size configured by the Master.
    @ingroup ecat_slave_init
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of a SyncManager (0,1,...).
    @param pIoBufferSize Pointer to variable that receives the size.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveConfiguredIoSizeMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  SyncManagerId,
    OUT UINT16* pIoBufferSize);

/*! @brief Gets SyncManager debug buffer. For internal use only.
    @ingroup ecat_slave_aux
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of a SyncManager (0,1,...).
    @param pData Pointer to output buffer where SyncManager data will be copied.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveSmDebugBufferMS. */
ECAT_RESULT getEcatSlaveSmDebugBufferMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  SyncManagerId,
    OUT UINT8*  pData);


/*! @brief Enable Process Data watchdog expire handling.
    If enabled, slave will change it state from OP to SO with error
    by Process Data watchdog expiring.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param bEnable Enable (if ECAT_TRUE) or disable (if ECAT_FALSE) the handling.
    @see   enableEcatSlaveIoTimeout*/
ECAT_RESULT enableEcatSlaveWdPd(
    IN  UINT8   BoardNumber,
    ECAT_BOOL   bEnable);


/*! @brief Enable handlings of Local Input Error (input data are to be sent to the Master are out-of-date,
    e.g. ADC or local bus error) and Local Output Error (it was failed to put output
    data received from the Master to slave's outputs, e.g. ADC or local bus error).
    If enabled, slave will change it state from OP to SO with error by a timeout expiring.
    In case of Local Input Error input SM will be disabled.
    Two timeout timers (for input and output) are used. The timers are refreshed
    in functions #updateOdTxInputsViaPI() and #updateOdRxOutputsViaPI()
    if they finished successfully.
    The handlings are disabled by default.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param bInputEnable Enable (if ECAT_TRUE) or disable (if ECAT_FALSE) the handlings for inputs.
    @param bOutputEnable Enable (if ECAT_TRUE) or disable (if ECAT_FALSE) the handlings for outputs.
    @see  setEcatSlaveIoTimeoutInterval, handlerEcatSlaveLocalInputError,
    handlerEcatSlaveLocalOutputError, enableEcatSlaveWdPd*/
ECAT_RESULT enableEcatSlaveIoTimeout(
    IN  UINT8       BoardNumber,
    IN  ECAT_BOOL   bInputEnable,
    IN  ECAT_BOOL   bOutputEnable);


/*! @brief Set time intervals for Local Input and Output timeouts.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param msInput Time interval for Local Input timeout (in ms).
    @param msOutput Time interval for Local Output timeout (in ms).
    @see   enableEcatSlaveIoTimeout */
ECAT_RESULT setEcatSlaveIoTimeoutInterval(
    IN  UINT8       BoardNumber,
    IN ECAT_TIME_MS msInput,
    IN ECAT_TIME_MS msOutput);


/*! @brief Calls handler for Local Input Error (slave state change from OP
           to SO and disabling input SM).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see   enableEcatSlaveIoTimeout, handlerEcatSlaveLocalOutputError */
ECAT_RESULT handlerEcatSlaveLocalInputError(IN  UINT8   BoardNumber);


/*! @brief Calls handler for Local OutputError (slave state change from OP to SO).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see   enableEcatSlaveIoTimeout, handlerEcatSlaveLocalInputError */
ECAT_RESULT handlerEcatSlaveLocalOutputError(IN  UINT8   BoardNumber);

/*! @} (marks end of group "ecat_slave_io_functions") */



/*! @brief Gets slave's Application Layer status.
    @ingroup ecat_slave_state
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pALStatus Pointer to variable that receives
        slave's @ref tECAT_AL_STATUS "Application Layer(AL) status".
    @return @ref ECAT_RESULT "Result code".
    @see tECAT_AL_STATUS,\n
         getEcatSlaveRequestedStateMS,\n
         setEcatSlaveStateMS */
ECAT_RESULT getEcatSlaveALStatusMS(
    IN  UINT8   BoardNumber,
    OUT UINT8*  pALStatus);

/*! @brief Gets requested (by EtherCAT) slave's state.
    @ingroup ecat_slave_fsm
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pRequestedState Pointer to variable that receives previously
        requested (by EtherCAT) @ref tECAT_AL_STATUS "slave's state".
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveALStatusMS, setEcatSlaveStateMS */
ECAT_RESULT getEcatSlaveRequestedStateMS(
    IN  UINT8   BoardNumber,
    OUT UINT8*  pRequestedState);

/*! @brief Sets state of the slave.
    @ingroup ecat_slave_fsm
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param State Required @ref tECAT_AL_STATUS "slave's state".
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveRequestedStateMS, getEcatSlaveALStatusMS */
ECAT_RESULT setEcatSlaveStateMS(
    IN UINT8 BoardNumber,
    IN  UINT8 State);

/*! @brief Sets error status of a slave.
    @ingroup ecat_slave_fsm
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ErrorInd Variable that contains value of error status.
        There may be an error status value which means that there is no
        error occured.
    @return @ref ECAT_RESULT "Result code".
    @see  getEcatSlaveALStatusMS */
ECAT_RESULT setEcatSlaveErrorStatusMS(
    IN  UINT8   BoardNumber,
    IN  UINT8   ErrorInd);

/*! @brief Retrieves configuration of a specified SyncManager channel of
    a slave.
    @ingroup ecat_slave_init
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncManagerId ID of a SyncManager (0,1,...).
    @param pSmChannel Pointer to SyncManagerChannel structure.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveSmConfigurationMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  SyncManagerId,
    OUT struct stSyncManagerChannel* pSmChannel);

/*! @brief Retrieves configuration of a specified FMMU channel of a slave.
    @ingroup ecat_slave_init
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param FmmuId ID of an FMMU channel (0,1,...).
    @param pFmmuChannel Pointer to FmmuChannel structure.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveFmmuConfigurationMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  FmmuId,
    OUT struct stFmmuChannel* pFmmuChannel);

/*! @brief Gets value of DL Status register.
    @ingroup ecat_slave_state
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pDlStatus Pointer to structure that represents DL Status register.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveDlStatusMS(
    IN UINT8 BoardNumber,
    OUT struct stEscDlStatus* pDlStatus);

/*! @brief Gets value of DL Control register.
    @ingroup ecat_slave_state
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pDlControl Pointer to structure that represents DL Control
        register.
    @return@ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveDlControlMS(
    IN UINT8 BoardNumber,
    OUT struct stEscDlControl* pDlControl);

/*! @brief Gets slave's Watchdog Process data status.
    @ingroup ecat_slave_io_functions
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pWdPdStatus Pointer to variable that receives
        slave's Watchdog Process data status.
        If bit0 == 0 WD expired.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT getEcatWdPdStatusMS(
    IN  UINT8   BoardNumber,
    OUT UINT16* pWdPdStatus);

/*! @brief Request Slave State. It can only turn down the state.
    Don't use this function in *_REQ callbacks.
    @ingroup ecat_slave_fsm
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param State Requested state (EcatStateI, EcatStateP, EcatStateS).
    @param bSendEvent Turn on generating of corresponding events.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription */
ECAT_RESULT requestEcatSlaveStateMS(
    IN UINT8        BoardNumber,
    IN EcatState    State,
    IN ECAT_BOOL    bSendEvent);

/*! @brief Reads data from slave's memory.
    @ingroup ecat_slave_mem
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param StartAddress Start address of data in slave's RAM.
    @param Size Size of data to be read measured in bytes.
        This value must not exceed the size of pData buffer.
    @param pData Buffer that receives data from slave's memory.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT readEcatSlaveMemMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  StartAddress, /* max 64Kbyte */
    IN  UINT16  Size,
    OUT UINT8*  pData);

/*! @brief Writes data to slave's memory.
    @ingroup ecat_slave_mem
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param StartAddress Start address of data in slave's RAM.
    @param Size Size of data to be written measured in bytes.
        This value must not exceed the size of pData buffer.
    @param pData Buffer that contains data that is to be written.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT writeEcatSlaveMemMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  StartAddress,
    IN  UINT16  Size,
    IN  UINT8*  pData);

/*! @brief Reads data from slave's EEPROM.
    @ingroup ecat_slave_mem
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param StartAddress Start address of data in slave's EEPROM (in words).
    @param Size Size of data to be read measured in bytes.
        This value must not exceed the size of pData buffer.
    @param pData Buffer that receives data from slave's EEPROM.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT readEcatSlaveEepromMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  StartAddress,
    IN  UINT16  Size,
    OUT UINT8*  pData);

/*! @brief Writes data to slave's EEPROM
    @ingroup ecat_slave_mem
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param StartAddress Start address of data in slave's EEPROM (in words).
    @param Size Size of data to be written measured in bytes.
        This value must not exceed the size of pData buffer.
    @param pData Buffer that contains data to be written.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT writeEcatSlaveEepromMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  StartAddress,
    IN  UINT16  Size,
    IN  UINT8*  pData);

/*! @brief Updates Checksum in slave's EEPROM.
    @ingroup ecat_slave_mem
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT updateEcatSlaveEepromChecksumMS(IN UINT8 BoardNumber);



#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*!**************************************************************************/
/*! @addtogroup ecat_slave_statistics_diagnostics
    @{
*****************************************************************************/

/*! @brief Retrieves slave's statistic data.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pSlaveStatisticData Pointer to structure that receives slave's
        statistic data.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveStatisticDataMS(
    IN UINT8 BoardNumber,
    OUT struct stEcatSlaveStatisticData* pSlaveStatisticData);

/*! @brief Clears slave's statistic data.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT clearEcatSlaveStatisticDataMS(IN UINT8 BoardNumber);

/*! @brief Retrieves slave's diagnostic data.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pSlaveDiagnosticData Pointer to structure that receives slave's
        diagnostic data.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveDiagnosticDataMS(
    IN UINT8 BoardNumber,
    OUT struct stEcatSlaveDiagnosticData* pSlaveDiagnosticData);

/*! @} (marks end of group "ecat_slave_statistics_diagnostics") */
#endif // #ifndef DONT_SHOW_NOTIMPL_FUNCTIONS

#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*!**************************************************************************/
/*! @addtogroup  ecat_slave_logging
    @{
*****************************************************************************/

/*! @brief Sets size of slave's log buffer.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param LogBufferSize Size of log buffer.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT setEcatSlaveLogBufferSizeMS(
    IN  UINT8   BoardNumber,
    IN  UINT32  LogBufferSize);

/*! @brief Sets mode of logging.
    The mode can be textual or binary, once or ring, there may be a number of
    different log levels.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param LogMode Mode of logging. \n
    The value can be composed of several #tEcatSlaveLogModeConstants constants using bitwise OR. \n
    Example: (LOGMODE_LEVEL_1 | LOGMODE_TYPE_ONCE | LOGMODE_FORMAT_TEXTUAL)
    @return @ref ECAT_RESULT "Result code".
    @see tEcatSlaveLogModeConstants. */
ECAT_RESULT setEcatSlaveLogModeMS(
    IN  UINT8   BoardNumber,
    IN  UINT32  LogMode);

/*! @brief Starts logging into a RAM-based file.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFileName Name of a file the log is written to.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT startEcatSlaveLoggingMS(
    IN  UINT8       BoardNumber,
    IN  ECAT_CHAR*  pFileName);

/*! @brief Stops logging.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT stopEcatSlaveLoggingMS(IN UINT8 BoardNumber);

/*! @} (marks end of group "ecat_slave_logging") */
#endif // #ifndef DONT_SHOW_NOTIMPL_FUNCTIONS

/*!**************************************************************************/
/*! @addtogroup  ecat_slave_dc
    @{
*****************************************************************************/
/*! @brief Enables Ecat chip Distributed Clock features.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code". */
ECAT_RESULT initEcatSlaveDcMS(IN UINT8 BoardNumber);

/*! @brief Function returns status of SYNC0 (SYNC1, etc.) signal.
    Used to acknowledge an interrupt generated by SYNC0 (SYNC1, etc.).

    ESC registers:\n
        - 0x098E: SYNC0\n
        - 0x098F: SYNC1
    @ingroup ecat_slave_irq
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param SyncNumber SYNC signal number (SYNC0: 0, SYNC1: 1, etc.).
    @param pSyncStatus Pointer to variable for storing SYNC0 (SYNC1, etc.) status.
    @return #ECAT_E_BOARD_NOT_FOUND,\n
            #ECAT_E_SLAVE_NOT_INIT,\n
            #ECAT_S_OK.
    @see @ref ECAT_RESULT "Result codes". */
ECAT_RESULT getEcatSlaveSyncStatus (
    IN UINT8    BoardNumber,
    IN UINT8    SyncNumber,
    OUT UINT8*  pSyncStatus);


/*! @brief Gets local copy of system time (32 bits width).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pdwSystemTime Pointer to variable for receiving System time.
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveDcMS */
ECAT_RESULT getEcatSlaveDcSystemTime32MS(
    IN  UINT8   BoardNumber,
    OUT UINT32  *pdwSystemTime);

/*! @brief Gets local copy of system time (64 bits width).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pqwSystemTime Pointer to variable for receiving System time.
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveDcMS */
ECAT_RESULT getEcatSlaveDcSystemTime64MS(
    IN  UINT8   BoardNumber,
    OUT UINT64  *pqwSystemTime);

/*! @brief Initialize slave's DC Latch inputs.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wLatchId Latch signal number (0 or 1).
    @param bIsPositiveEdgeContinuousEvent Continuous mode positive edge flag: \n
        #ECAT_TRUE - Continuous mode, \n
        #ECAT_FALSE - Single event mode
    @param bIsNegativeEdgeContinuousEvent Continuous mode negative edge flag:\n
        #ECAT_TRUE - Continuous mode, \n
        #ECAT_FALSE - Single event mode
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveDcLatchStatusMS */
ECAT_RESULT initEcatSlaveDcLatchMS(
    IN UINT8        BoardNumber,
    IN UINT16       wLatchId,
    IN ECAT_BOOL    bIsPositiveEdgeContinuousEvent,
    IN ECAT_BOOL    bIsNegativeEdgeContinuousEvent);

/*! @brief Gets slave's DC Latch status. Single event mode only, otherwise returns ECAT_FALSE.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wLatchId Latch signal number (0 or 1).
    @param pbPositiveEdgeEvent Pointer to variable indicating if there was
    at least one change on Latch input (positive edge).\n
        #ECAT_FALSE - No change on Latch input,\n
        #ECAT_TRUE  - At least one change on Latch input
    @param pbNegativeEdgeEvent Pointer to variable indicating if there was
    at least one change on Latch input (negative edge).\n
        #ECAT_FALSE - No change on Latch input,\n
        #ECAT_TRUE  - At least one change on Latch input
    @param pbPinState Pointer to variable indicating a pin state.
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveDcLatchMS */
ECAT_RESULT getEcatSlaveDcLatchStatusMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      wLatchId,
    OUT ECAT_BOOL   *pbPositiveEdgeEvent,
    OUT ECAT_BOOL   *pbNegativeEdgeEvent,
    OUT ECAT_BOOL   *pbPinState);

/*! @brief Gets captured system time (32 bits width) of Latch signal.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wLatchId Latch signal number (0 or 1).
    @param pdwTimePositiveEdge Pointer to variable receiving the Positive Edge Time.
    @param pdwTimeNegativeEdge Pointer to variable receiving the Negative Edge Time.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveDcSystemTime32MS */
ECAT_RESULT getEcatSlaveDcLatchTime32MS(
    IN  UINT8   BoardNumber,
    IN  UINT16  wLatchId,
    OUT UINT32  *pdwTimePositiveEdge,
    OUT UINT32  *pdwTimeNegativeEdge);

/*! @brief Gets captured system time (64 bits width) of Latch signal.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wLatchId Latch signal number (0 or 1).
    @param pqwTimePositiveEdge Pointer to variable receiving the Positive Edge Time.
    @param pqwTimeNegativeEdge Pointer to variable receiving the Negative Edge Time.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveDcSystemTime64MS */
ECAT_RESULT getEcatSlaveDcLatchTime64MS(
    IN  UINT8   BoardNumber,
    IN  UINT16  wLatchId,
    OUT UINT64  *pqwTimePositiveEdge,
    OUT UINT64  *pqwTimeNegativeEdge);

/*! @} (marks end of group "ecat_slave_dc") */


/*!**************************************************************************/
/*! @addtogroup ecat_slave_logger
    @{
*****************************************************************************/

/*! @brief Adds a message.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param Type Message type (TraceMsg, WarningMsg, ErrorMsg).
    @param fmt format string.*/
void addEcatLoggerMsgMS(
    IN UINT8                BoardNumber,
    IN EcatLoggerMsgType    Type,
    IN const ECAT_CHAR      *fmt, ...);

/*! @brief Gets ecat logger message. For internal use only.
    Don't call from more then one thread.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pszMsg Message string.
    @param wMaxMsgLen Max size of incoming message string buffer.
    @param pParams Message parameters.
    @return @ref ECAT_RESULT "Result code".
    @see addEcatLoggerMsgMS, getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatLoggerMsgNonRtMS(
    IN UINT8        BoardNumber,
    OUT ECAT_CHAR   *pszMsg,
    IN UINT16       wMaxMsgLen,
    OUT struct EcatSlaveLoggerMsgParams *pParams);
/*! @} (marks end of group "ecat_slave_logger") */

/*!**************************************************************************/
/*! @addtogroup ecat_slave_mailbox_voe
    @{
 *****************************************************************************/

/*! @brief Sets the VoE callback function. Used to notify the user about send/receive events.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param VoEHandler Callback function.
    @return @ref ECAT_RESULT "Result code".
    @see tVoECallBackData, tVoEOperationType, */
ECAT_RESULT setEcatSlaveMailboxVoECallBackHandlerMS(
    IN UINT8        BoardNumber,
    IN tVoECallBack VoEHandler);

/*! @brief Adds VoE write data to output queue (data will be sent to EtherCAT).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param dwVendorId Vendor ID.
    @param wVendorType Vendor Type.
    @param wDstAddress Station address of the destination.
    @param pContextData User context data. Used in callback.
    @param wDataLength Data length.
    @param pbyData Pointer to data. The pointer must be valid until data sending is confirmed by the callback.
    @param dwTimeOut Time while data is valid (milliseconds).
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT addEcatSlaveMailBoxVoEWriteDataMS(
    IN UINT8        BoardNumber,
    IN ECAT_DWORD   dwVendorId,
    IN ECAT_WORD    wVendorType,
    IN ECAT_WORD    wDstAddress,
    IN void*        pContextData,
    IN ECAT_WORD    wDataLength,
    IN ECAT_PBYTE   pbyData,
    IN ECAT_DWORD   dwTimeOut);

/*! @} (marks end of group "ecat_slave_mailbox_voe") */


/*!**************************************************************************/
/*! @addtogroup ecat_slave_mailbox_eoe
    @{
 *****************************************************************************/

/*! @brief Virtual interface calls it when it has new frame to send.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pbyData Pointer to Ethernet frame.
    @param wLen Frame length.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT sendEcatSlaveEoENICFrameMS(
    IN UINT8         BoardNumber,
    IN ECAT_BYTE    *pbyData,
    IN ECAT_WORD     wLen);

/*! @brief Register virtual network interface callbacks.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pContextData Pointer to user context data.
    @param pfnUp Pointer to virtual NIC up callback. (Have to be ECAT_NULL).
    @param pfnDown Pointer to virtual NIC down callback. (Have to be ECAT_NULL).
    @param pfnIPSettingsSet Pointer to callback, which is called when set virtual
                            NIC IP settings request of EoE extention commes.
    @param pfnGetHwAddr Pointer to get virtual NIC MAC address callback. (Have to be ECAT_NULL).
    @param pfnReceive Pointer to virtual NIC receive callback.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT registerEcatSlaveEoENICInterfaceMS(
    IN UINT8                    BoardNumber,
    IN void                    *pContextData,
    IN tEcatEoENICUp            pfnUp,
    IN tEcatEoENICDown          pfnDown,
    IN tEcatEoENICIPSettingsSet pfnIPSettingsSet,
    IN tEcatEoENICGetHwAddr     pfnGetHwAddr,
    IN tEcatEoENICRecive        pfnReceive);

/*! @brief Unregister virtual network interface callbacks.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT unregisterEcatSlaveEoENICInterfaceMS(
    IN UINT8 BoardNumber);
/*! @} (marks end of group "ecat_slave_mailbox_eoe") */


/*!**************************************************************************/
/*! @addtogroup ecat_slave_mailbox_foe
    @{
 *****************************************************************************/

/*! @brief Changes FoE password.

    Default password for FoE access to slave is @c "0xF0EACCECL" (hex, 32-bit value).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nOldPassword Current valid password.
    @param nNewPassword New password.
    @return @ref ECAT_RESULT "Result code".
    @see ecat_slave_foe_osal */
ECAT_RESULT changeEcatSlaveFoePassword (
    IN  UINT8   BoardNumber,
    IN  UINT32  nOldPassword,
    OUT UINT32  nNewPassword);

/*! @} (marks end of group "ecat_slave_mailbox_foe") */


/*!**************************************************************************/
/*! @addtogroup ecat_slave_mailbox_coe
    @{
 *****************************************************************************/

/*! @brief Adds CoE emergency message.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wErrorCode Error code. @see enMailboxEmergencyCodes
    @param byErrorRegister Error register.
    @param pbyData Pointer to data.
    @param wDataSize Data size.
    @param wDstAddress Mailbox destinition address.
    @param dwTimeout Timeout(milliseconds).
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT addEcatMailBoxCoEEmergencyMsgMs(
    IN UINT8        BoardNumber,
    IN ECAT_WORD    wErrorCode,
    IN ECAT_BYTE    byErrorRegister,
    IN ECAT_BYTE   *pbyData,
    IN ECAT_WORD    wDataSize,
    IN ECAT_WORD    wDstAddress,
    IN ECAT_DWORD   dwTimeout);
/*! @} (marks end of group "ecat_slave_mailbox_coe") */

#ifdef __cplusplus
}
#endif


#endif /* ECATSLAVEAPI_H */
