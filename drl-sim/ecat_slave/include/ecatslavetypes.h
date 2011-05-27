/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2007
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatslavetypes.h
    @brief This file contains definitions of various
    EcatSlave-specific structures, enumerations, types. */
/****************************************************************************/

#ifndef ECATSLAVETYPES_H
#define ECATSLAVETYPES_H

#include "ecatbasictypes.h"
#include "ecsresultcodes.h"
#include "ecatslaveconfig.h" /* defines EtherCAT slave controller settings, platform settings flags etc. */


#ifndef AL_EVENT_MAILBOX_START_REQ

/*! @brief Masks and flags of EtherCAT driver events for callback functions.
    @see setEcatSlaveCallbackFunctionMS, setEcatSlaveSdoCallbackFunctionMS. */
typedef enum enEcatSlaveDriverEvent
{

    //setEcatSlaveCallbackFunction in user and kernel mode
    AL_EVENT_MAILBOX_START_REQ          =   0x00000001,  // return ECAT_S_OK if ready
    AL_EVENT_INPUTS_START_REQ           =   0x00000002,  // return ECAT_S_OK if ready
    AL_EVENT_OUTPUTS_START_REQ          =   0x00000004,  // return ECAT_S_OK if ready
    AL_EVENT_MAILBOX_STOP_REQ           =   0x00000008,  // return ECAT_S_OK if ready
    AL_EVENT_INPUTS_STOP_REQ            =   0x00000010,  // return ECAT_S_OK if ready
    AL_EVENT_OUTPUTS_STOP_REQ           =   0x00000020,  // return ECAT_S_OK if ready

    AL_EVENT_MAILBOX_STARTED            =   0x00000040, // PreOperational
    AL_EVENT_INPUTS_STARTED             =   0x00000080, // SO
    AL_EVENT_OUTPUTS_STARTED            =   0x00000100, // OP

    AL_EVENT_MAILBOX_STOPED             =   0x00000200, // Obsolete!!! It is left only for backward
                                                        // compatibility with old user code

    AL_EVENT_MAILBOX_STOPPED            =   0x00000200, // IN

    AL_EVENT_INPUTS_STOPED              =   0x00000400, // Obsolete!!! It is left only for backward
                                                        // compatibility with old user code

    AL_EVENT_INPUTS_STOPPED             =   0x00000400, // PO

    AL_EVENT_OUTPUTS_STOPED             =   0x00000800, // Obsolete!!! It is left only for backward
                                                        // compatibility with old user code

    AL_EVENT_OUTPUTS_STOPPED            =   0x00000800, // SO

    //setEcatSlaveCallbackFunction in kernel mode only
    AL_EVENT_INPUTS_TRANSMITTED         =   0x00001000,  // UINT16 *pEventData - SyncManager ID
    AL_EVENT_OUTPUTS_RECEIVED           =   0x00002000,  // UINT16 *pEventData - SyncManager ID
    AL_EVENT_LATCH_CHANGE               =   0x00004000,

    //setEcatSlaveOnStateChangeCallbackFunction in user mode only.
    /*! EcatSlave state changed. UINT16 *pEventData encodes state transition.
        This constant and setEcatSlaveOnStateChangeCallbackFunction are obsolete. */
    ECATSLAVE_EVENT_SLAVE_STATE_CHANGED =   0x00008000,

    //setEcatSlaveSdoCallbackFunction in user and kernel mode

    ECATSLAVE_EVENT_OD_OBJENTRY_CHANGED =   0x00010000L, /*!< Event handler -
                                                            @link  #tEcatSlaveSdoCallbackFunction
                                                            EcatSlaveSdoCallbackFunction @endlink
                                                            CANopen Object dictionary object entry changed.*/

    ECATSLAVE_EVENT_SDO_UPLOAD_REQUEST  =   0x00020000L, /*!< Event handler -
                                                            @link  #tEcatSlaveSdoCallbackFunction
                                                            EcatSlaveSdoCallbackFunction @endlink
                                                            CANopen Object dictionary object entry upload.*/

    ECATSLAVE_EVENT_SDO_DOWNLOAD_REQUEST=   0x00040000L, /*!< Event handler -
                                                            @link  #tEcatSlaveSdoCallbackFunction
                                                            EcatSlaveSdoCallbackFunction @endlink
                                                            CANopen Object dictionary object entry download.*/

    AL_EVENT_HEARTBEAT                  =   0x00080000L,
    /* to be appended. */

    ECATSLAVE_EVENT_PROCESS_IMAGE_CLEAN =   0x00100000L, /*!< Event handler -
                                                            @link  #tEcatSlaveSdoCallbackFunction
                                                            EcatSlaveSdoCallbackFunction @endlink. */

    ECATSLAVE_EVENT_PROCESS_IMAGE_BUILD =   0x00200000L, /*!< Event handler -
                                                            @link  #tEcatSlaveSdoCallbackFunction
                                                            EcatSlaveSdoCallbackFunction @endlink. */

    ECATSLAVE_EVENT_SAVE_BACKUP_ENTRIES =   0x00400000L, /*!< Request to save backup entries (e.g. to nonvolatile memory). */
    ECATSLAVE_EVENT_LOAD_BACKUP_ENTRIES =   0x00800000L, /*!< Request to restore backup entries with default values. */

} tEcatSlaveDriverEvent;
#endif // #ifndef AL_EVENT_MAILBOX_START_REQ


/*! @brief AL Status codes.
    @see getEcatSlaveALStatusMS */
typedef enum enEcatALStatus
{
    ECAT_AL_STATUS_INIT                 = 1,    /*!< @brief Mode "Initial"          */
    ECAT_AL_STATUS_PRE_OP               = 2,    /*!< @brief Mode "Pre-Operational"  */
    ECAT_AL_STATUS_BOOTSTRAP            = 3,    /*!< @brief Mode "Bootstrap"        */
    ECAT_AL_STATUS_SAFE_OP              = 4,    /*!< @brief Mode "Safe-Operational" */
    ECAT_AL_STATUS_OP                   = 8,    /*!< @brief Mode "Operational"      */
    ECAT_AL_STATUS_WITH_ERROR           = 0x10  /*!< @brief Mask for "Error"-bit    */
} tECAT_AL_STATUS;


/*! @brief SDO Events flags for @ref tEcatSlaveSdoCallbackFunction
    "EcatSLave SDO event callback function". */
typedef enum enEcatSlaveSDOEventsFlags {
    /*! Parameter of @ref tEcatSlaveSdoCallbackFunction "EcatSLave SDO event callback function"
    @endlink The flag idicates an SDO complete access event. */
    SDO_COMPLETE_ACCESS             = 0x00000001,
} tEcatSlaveSDOEventsFlags;


/*! @brief Bit-masks for different modes of logging.
        <pre>
        bytes:   3        2        1        0
        bits: rrrrrrrr rrrrrrrr FFFFTTTT rrrrLLLL
        </pre>

        r - Reserved bits \n
        L - signifies log-level \n
        T - signifies type of logging(ring or once) \n
        F - signifies format of logging(textual or binary) \n
*/
typedef enum enEcatSlaveLogModeConstants
{
    LOGMODE_LEVEL_MASK          = 0x0000000F,
    LOGMODE_LEVEL_0             = 0x00000000,
    LOGMODE_LEVEL_1             = 0x00000001,
    LOGMODE_LEVEL_2             = 0x00000002,
    LOGMODE_LEVEL_3             = 0x00000003,
    LOGMODE_LEVEL_4             = 0x00000004,
    LOGMODE_LEVEL_5             = 0x00000005,

    LOGMODE_TYPE_MASK           = 0x00000F00,
    LOGMODE_TYPE_RING           = 0x00000000,
    LOGMODE_TYPE_ONCE           = 0x00000100,

    LOGMODE_FORMAT_MASK         = 0x0000F000,
    LOGMODE_FORMAT_TEXTUAL      = 0x00000000,
    LOGMODE_FORMAT_BINARY       = 0x00001000,
} tEcatSlaveLogModeConstants;

#ifndef ESC_AL_EVENT_REG_AL_CONTROL
/*! @brief Ecat slave controller Application Level events and masks registers bits. */
typedef enum enEcatAlEventRegBits
{
    ESC_AL_EVENT_REG_AL_CONTROL         = 0x00000001,
    ESC_AL_EVENT_REG_LATCH              = 0x00000002,
    ESC_AL_EVENT_REG_SYNC0              = 0x00000004,
    ESC_AL_EVENT_REG_SYNC1              = 0x00000008,
    ESC_AL_EVENT_REG_ANY_SM_ACTIVATION  = 0x00000010,
    ESC_AL_EVENT_REG_EEPROM_EMUL        = 0x00000020,
    ESC_AL_EVENT_REG_SM0_STATUS         = 0x00000100,
    ESC_AL_EVENT_REG_SM1_STATUS         = 0x00000200,
    ESC_AL_EVENT_REG_SM2_STATUS         = 0x00000400,
    ESC_AL_EVENT_REG_SM3_STATUS         = 0x00000800,
    ESC_AL_EVENT_REG_SM4_STATUS         = 0x00001000,
    ESC_AL_EVENT_REG_SM5_STATUS         = 0x00002000,
    ESC_AL_EVENT_REG_SM6_STATUS         = 0x00004000,
    ESC_AL_EVENT_REG_SM7_STATUS         = 0x00008000,
    ESC_AL_EVENT_REG_SM8_STATUS         = 0x00010000,
    ESC_AL_EVENT_REG_SM9_STATUS         = 0x00020000,
    ESC_AL_EVENT_REG_SM10_STATUS        = 0x00040000,
    ESC_AL_EVENT_REG_SM11_STATUS        = 0x00080000,
    ESC_AL_EVENT_REG_SM12_STATUS        = 0x00100000,
    ESC_AL_EVENT_REG_SM13_STATUS        = 0x00200000,
    ESC_AL_EVENT_REG_SM14_STATUS        = 0x00400000,
    ESC_AL_EVENT_REG_SM15_STATUS        = 0x00800000,
} tEcatAlEventRegBits;
#endif //#ifndef ESC_AL_EVENT_REG_AL_CONTROL

/*! @brief SyncManager channel description (see also specifications
    for ESC registers 0x0800:0x087F).
    @see setEcatSlaveSyncManagerMS, getEcatSlaveSmConfigurationMS */
struct stSyncManagerChannel
{
    UINT16   PhysicalStartAddress;      /*!< @brief SyncManager physical start address.
                                                Specifies first byte that will be handled by SyncManager.*/
    UINT16   Length;                    /*!< @brief Number of bytes assigned to SyncManager.
                                                For 3-buffered mode occupied size is 3 x Length).
                                                The value shall be greater 1, otherwise SyncManager is not activated. */
    UINT8    Control;                   /*!< @brief Control byte of SyncManager. */
    UINT8    Status;                    /*!< @brief Status byte of SyncManager. */
    UINT8    Activate;                  /*!< @brief Register Activate SyncManager. */
    UINT8    PDI_Control;               /*!< @brief Register PDI Control SyncManager. */
};


/*! @brief FMMU channel description (see also specifications
    for ESC registers 0x0600:0x06FF).
    @see getEcatSlaveFmmuConfigurationMS */
struct stFmmuChannel
{
    UINT32      LogicalStartAddress;    /*!< @brief Value of register "Logical Start Address"
                                                    (logical start address withing the EtherCAT address space. */

    UINT16      Length;                 /*!< @brief Value of register "Length"
                                                    (offset from the first logical FMMU byte to the last FMMU byte +1.
                                                    If two bytes are used then "Length" is 2). */

    UINT8       LogicalStartBit;        /*!< @brief Value of register "Logical Start bit"
                                                    (Logical starting bit that shall be mapped. Value range: 0-7).*/

    UINT8       LogicalStopBit;         /*!< @brief Value of register "Logical Stop bit"
                                                    (Last logical bit that shall be mapped. Value range: 0-7). */

    UINT16      PhysicalStartAddress;   /*!< @brief Value of register "Physical Start address" (mapped to logical Start address). */
    UINT8       PhysicalStartBit;       /*!< @brief Value of register "Physical Start bit" (Value range: 0-7). */
    UINT8       Type;                   /*!< @brief Value of register "Type". */
    UINT8       Activate;               /*!< @brief Value of register "Activate". */
    UINT8       Reserved0;              /*!< @brief Reserved. */
    UINT8       Reserved1;              /*!< @brief Reserved. */
    UINT8       Reserved2;              /*!< @brief Reserved. */
};

/*! @brief "ESC DL Status" register representation (see also specifications
    for ESC registers 0x0110:0x0111).
    @see getEcatSlaveDlStatusMS */
struct stEscDlStatus
{
#if defined(_LITTLE_ENDIAN_PLATFORM_)
    UINT16  PdiOperational:1;       /*!< @brief 0: EEPROM not loaded, PDI not operational (no access to Process Data RAM);\n
                                                1: EEPROM loaded correctly, PDI operational (access to Process Data RAM) */
    UINT16  PDI_WD_Status:1;        /*!< @brief 0: Watchdog expired; 1: Watchdog reloaded:1;              */
    UINT16  Enhanced_Link_Detect:1; /*!< @brief 0: Deactivated; 1: Activated                              */
    UINT16  :1;                     /*!< @brief Reserved. */
    UINT16  PhisLinkPort0:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort1:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort2:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort3:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  LoopPort0:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort0:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort1:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort1:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort2:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort2:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort3:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort3:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
#elif defined(_BIG_ENDIAN_PLATFORM_)
    UINT16  CommunicationPort3:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort3:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort2:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort2:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort1:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort1:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  CommunicationPort0:1;   /*!< @brief 0: No stable communication; 1: Communication established  */
    UINT16  LoopPort0:1;            /*!< @brief 0: Open; 1: Closed                                        */
    UINT16  PhisLinkPort3:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort2:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort1:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  PhisLinkPort0:1;        /*!< @brief 0: No link; 1: Link detected                              */
    UINT16  :1;                     /*!< @brief Reserved. */
    UINT16  Enhanced_Link_Detect:1; /*!< @brief 0: Deactivated; 1: Activated                              */
    UINT16  PDI_WD_Status:1;        /*!< @brief 0: Watchdog expired; 1: Watchdog reloaded:1;              */
    UINT16  PdiOperational:1;       /*!< @brief 0: EEPROM not loaded, PDI not operational (no access to Process Data RAM);\n
                                                1: EEPROM loaded correctly, PDI operational (access to Process Data RAM) */
#else
#error Platform endian type is not specified!
#endif
} ;


/*! @brief "ESC DL Control" register representation (see also specifications
    for ESC registers 0x0100:0x0103).
    @see getEcatSlaveDlControlMS */
struct stEscDlControl
{
#ifndef _MICROCONTROLLER_16BIT_
#if defined (_LITTLE_ENDIAN_PLATFORM_)
    UINT32  ForwardingRule:1;   /*!< @brief Forwarding rule. */
    UINT32  TemporaryUse:1;     /*!< @brief Temporary use of settings in Register 0x101
                                            (0: permanent use,
                                             1: use for about 1 sec,
                                                then revert to prev. settings. ). */
    UINT32  :6;                 /*!< @brief Reserved. */
    UINT32  LoopPort0:2;        /*!< @brief "Loop Port 0" settings. */
    UINT32  LoopPort1:2;        /*!< @brief "Loop Port 1" settings. */
    UINT32  LoopPort2:2;        /*!< @brief "Loop Port 2" settings. */
    UINT32  LoopPort3:2;        /*!< @brief "Loop Port 3" settings. */
    UINT32  RxFIFOSize:3;       /*!< @brief RX FIFO Size (ESC delays start of forwarding until FIFO is at least half full). */
    UINT32  LowJitterEBUS:1;    /*!< @brief "EBUS Low Jitter" settings. */
    UINT32  :4;                 /*!< @brief Reserved. */
    UINT32  UseStationAlias:1;  /*!< @brief "Station alias":
                                            0: ignore station alias,
                                            1: alias can be used for all configured address command types. */
    UINT32  :7;                 /*!< @brief Reserved. */
#elif defined(_BIG_ENDIAN_PLATFORM_)
    UINT32  :7;                 /*!< @brief Reserved. */
    UINT32  UseStationAlias:1;  /*!< @brief "Station alias":
                                            0: ignore station alias,
                                            1: alias can be used for all configured address command types. */
    UINT32  :4;                 /*!< @brief Reserved. */
    UINT32  LowJitterEBUS:1;    /*!< @brief "EBUS Low Jitter" settings. */
    UINT32  RxFIFOSize:3;       /*!< @brief RX FIFO Size (ESC delays start of forwarding until FIFO is at least half full). */
    UINT32  LoopPort3:2;        /*!< @brief "Loop Port 3" settings. */
    UINT32  LoopPort2:2;        /*!< @brief "Loop Port 2" settings. */
    UINT32  LoopPort1:2;        /*!< @brief "Loop Port 1" settings. */
    UINT32  LoopPort0:2;        /*!< @brief "Loop Port 0" settings. */
    UINT32  :6;                 /*!< @brief Reserved. */
    UINT32  TemporaryUse:1;     /*!< @brief Temporary use of settings in Register 0x101
                                            (0: permanent use,
                                             1: use for about 1 sec,
                                                then revert to prev. settings. ). */
    UINT32  ForwardingRule:1;   /*!< @brief Forwarding rule. */
#else
#error Platform endian type is not specified!
#endif
#else  //ifndef _MICROCONTROLLER_16BIT_
	UINT32 dword;              /*!< @brief "ESC DL Control" register value. */
#endif //ifndef _MICROCONTROLLER_16BIT_
};

#ifndef DONT_SHOW_NOTIMPL_FUNCTIONS
/*! @brief Slave's statistic data. */
struct stEcatSlaveStatisticData
{
    UINT32 DummyField;
};

/*! @brief Slave's diagnostic data. */
struct stEcatSlaveDiagnosticData
{
    UINT32 DummyField;
};
// / *! @ brief Slave's Object Dictionary information. */
// struct stEcatSlaveOdInformation
// {
//     UINT32 DummyField;
// };
//
// / *! @brief Slave's Object information. */
// struct stEcatSlaveObjectInformation
// {
//     UINT32 DummyField;
// };
#endif /* DONT_SHOW_NOTIMPL_FUNCTIONS */


/*! @defgroup ecat_slave_callback_types Callback functions types
      @ingroup ecat_slave */

/*! @brief @obsolete EcatSlave on-state-change callback function type definition.
           Custom user-defined on-state-change callback function must have
           this type.
    @ingroup ecat_slave_callback_types
    @param OldSlaveState Slave state before the event.\n
           This value is passed by the system to the custom on-state-change callback
           function implemented by the user.
    @param NewSlaveState Slave state after the event.\n
           This value is passed by the system to the custom on-state-change callback
           function implemented by the user.
    @see setEcatSlaveOnStateChangeCallbackFunctionMS */
typedef ECAT_RESULT (*tEcatSlaveOnStateChangeCallbackFunction)(
    IN  UINT8   OldSlaveState,
    IN  UINT8   NewSlaveState);


/*! @brief Slave's callback function type definition.
    @ingroup ecat_slave_callback_types
    @param EventId Id of @ref tEcatSlaveDriverEvent "Event".
    @param pEventData Event data.
    @param pContextData Additional context data.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveCallbackFunctionMS */
typedef ECAT_RESULT (*tEcatSlaveCallbackFunction)(
    IN UINT32   EventId,
    IN void*    pEventData,
    IN void*    pContextData,
    IN ECAT_BOOL bNeedWaitReturn);

/*! @brief SDO callback function type definition.
    @ingroup ecat_slave_callback_types
    @param EventId ID of @ref ECATSLAVE_EVENT_OD_OBJENTRY_CHANGED "event".
    @param ObjIndex Object's index.
    @param ObjSubIndex Object's subindex.
    @param AccessType Event's access type. \n
    Consists of @ref enAccessReadType "read" and @ref enAccessWriteType "write" access types.
    @param pEventData @ref tEcatSlaveSDOEventsFlags "Auxiliary event's flags".
    Currently this pointer points(if not NULL) to a value of type #ECAT_UINT32.
    See tEcatSlaveSDOEventsFlags for interpretation of each bit of that value.
    @param pContextData Pointer to additional context data.
    @return @ref ECAT_RESULT "Result code".
    For #ECATSLAVE_EVENT_SDO_UPLOAD_REQUEST return value not equal to #ECAT_S_OK means
    that the master will get "abort".
    For #ECATSLAVE_EVENT_SDO_DOWNLOAD_REQUEST return value not equal to #ECAT_S_OK means
    that the master will get "abort" and the object entry value will not be changed.
    @see setEcatSlaveSdoCallbackFunctionMS */
typedef ECAT_RESULT (*tEcatSlaveSdoCallbackFunction)(
    IN  UINT32  EventId,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    IN  UINT16  AccessType,
    IN  void*   pEventData,
    IN  void*   pContextData,
    IN UINT32   bNeedWaitReturn);


/*! @brief Can be used as context data for setEcatSlaveSdoCallbackFunctionMS(). */
struct stSdoChangeCallbackContextData {
    UINT8 BoardNumber;      /*!< @brief Board number. */
    void* pData;            /*!< @brief Pointer to context data itself for the board "BoardNumber". */
};


/*! @brief Defines VoE operation types.
    @ingroup ecat_slave_callback_types
    @ingroup ecat_slave_mailbox_voe
    @see tVoECallBack, setEcatSlaveMailboxVoECallBackHandlerMS */
typedef enum stVoEOperationType{
    otVoERead     = 0,      /*!< @brief Read-operation: callback is called with otVoERead flag if data is received from EtherCAT. */
    otVoEWrite    = 1,      /*!< @brief Write-operation: callback is called with otVoEWrite flag if data is to be sent to EtherCAT. */
} tVoEOperationType;

/*! @brief Structure defines the data are passed to the VoE handler.
    @ingroup ecat_slave_callback_types
    @ingroup ecat_slave_mailbox_voe */
typedef struct tagVoECallBackData
{
    ECAT_DWORD          dwVendorId;         /*!< @brief Vendor's Id. */
    ECAT_WORD           wVendorType;        /*!< @brief Vendor's protocol Id. */
    ECAT_WORD           wAddress;           /*!< @brief Station address of the source or destination. */
    void*               pbyContextData;     /*!< @brief Pointer to the context data. Always NULL for read opearations. */
    tVoEOperationType   enOperationType;    /*!< @brief VoE operation types(read/write). See VoEOperationType. */
    ECAT_WORD           wDataLength;        /*!< @brief The length of the data. */
    ECAT_PBYTE          pbyData;            /*!< @brief Pointer to the buffer where the data are placed. */
    ECAT_RESULT         Result;             /*!< @brief Result Operation result. For more information about the
                                                        result see getEcatSlaveResultCodeDescription(). */
} tVoECallBackData;

/*! @brief Type definition of pointer to tVoECallBackData.
    @ingroup ecat_slave_callback_types
    @ingroup ecat_slave_mailbox_voe
    @see tVoEOperationType */
typedef struct tagVoECallBackData* tVoECallBackDataPtr;

/*! @brief Routine that is called when VoE read/write operation is finished.
    @ingroup ecat_slave_callback_types
    @ingroup ecat_slave_mailbox_voe
    @param pVoeCallBackData Pointer to call back data.(see #tVoECallBackData))
    @see setEcatSlaveMailboxVoECallBackHandlerMS */
typedef void (*tVoECallBack)(
    IN tVoECallBackDataPtr pVoeCallBackData);

/*! @brief Type of SyncManager.
    @see setEcatSlaveSyncManagerMS. */
typedef enum enEcatSmType
{
    EcatSmTypeNotUsed   = 0x00,  /*!< @brief Unused SyncManager.           */
    EcatSmTypeMbxOut    = 0x01,  /*!< @brief SyncManager type "MBoxOut" (Mailbox data Master --> Slave).   */
    EcatSmTypeMbxIn     = 0x02,  /*!< @brief SyncManager type "MBoxIn"  (Mailbox data Slave  --> Master).  */
    EcatSmTypeOutputs   = 0x03,  /*!< @brief SyncManager type "Outputs" (Process Data Master --> Slave).   */
    EcatSmTypeInputs    = 0x04,  /*!< @brief SyncManager type "Inputs"  (Process Data Slave  --> Master).  */
} tEcatSmType;

typedef void* pVoid;

/*! @brief Type of diagnostic message object entry data.
    @see addEcatSlaveOdDiagMessage */
typedef struct stOdDiagnosticMessage
{
    UINT32  DiagNumber;     /*!< @brief Diagnostic message number. */
    UINT16  Flags;          /*!< @brief Flags. */
    UINT16  TextID;         /*!< @brief Text string ID. */
    UINT64  TimeStamp;      /*!< @brief Time spamp of a message. */
    UINT32  ParametersSize; /*!< @brief Size of parameters that pParameters point to. */
    pVoid   pParameters;    /*!< @brief Pointer to a list of parameters for a formatted text string. */
} tOdDiagnosticMessage;


/*******************  CANopen related definitions ***************************/

/*! @brief Object Code Definitions.
    @see tCANObjectDescription */
enum enCANObjectType
{
    ot_notset           = 0x0000,
    ot_domain           = 0x0002,
    ot_deftype          = 0x0005,
    ot_defstruct        = 0x0006,
    ot_var              = 0x0007,
    ot_array            = 0x0008,
    ot_record           = 0x0009,
};


/*! @brief Data Type Area
    @see tCANObjectDescription*/
enum enCANObjectDataType
{
    /*========== Basic Data Type Area ==========*/
    dt_notset           = 0x0000,
    dt_boolean          = 0x0001,
    dt_integer8         = 0x0002,
    dt_integer16        = 0x0003,
    dt_integer32        = 0x0004,
    dt_unsigned8        = 0x0005,
    dt_unsigned16       = 0x0006,
    dt_unsigned32       = 0x0007,
    dt_real32           = 0x0008,
    dt_visible_string   = 0x0009,
    dt_octet_string     = 0x000a,
    dt_unicode_string   = 0x000b,
    dt_time_of_day      = 0x000c,
    dt_time_difference  = 0x000d,
    /* RESERVED VALUE     0x000e */
    dt_domain           = 0x000f,
    dt_integer24        = 0x0010,
    dt_real64           = 0x0011,
    dt_integer40        = 0x0012,
    dt_integer48        = 0x0013,
    dt_integer56        = 0x0014,
    dt_integer64        = 0x0015,
    dt_unsigned24       = 0x0016,
    /* RESERVED VALUE     0x0017 */
    dt_unsigned40       = 0x0018,
    dt_unsigned48       = 0x0019,
    dt_unsigned56       = 0x001a,
    dt_unsigned64       = 0x001b,
    /* RESERVED VALUES    0x001c-0x001f */

    /*========== Extended Data Type Area ==========*/
    /* RESERVED VALUE     0x0020 */
    dt_pdo_mapping      = 0x0021,
    /* RESERVED VALUE     0x0022 */
    dt_identity         = 0x0023,
    /* RESERVED VALUE     0x0024 */
    dt_command_par      = 0x0025,
    /* RESERVED VALUES    0x0026-0x0028 */
    dt_sync_par         = 0x0029,
    /* RESERVED VALUES    0x002a-0x002f */
    dt_bit1             = 0x0030,
    dt_bit2             = 0x0031,
    dt_bit3             = 0x0032,
    dt_bit4             = 0x0033,
    dt_bit5             = 0x0034,
    dt_bit6             = 0x0035,
    dt_bit7             = 0x0036,
    dt_bit8             = 0x0037,
    /* RESERVED VALUES    0x0038-0x003f */
    dt_vendor_specific  = 0x0040,/* 0x0040-0x005f: Manufacturer Specific Complex Data Types*/
};

/*! @brief Constants that encode read-access type to OD entry
    for different EtherCAT Slave States.
    It may be OR-ed with read-access type enAccessWriteType in order.
    @see tCANObjectEntryDescription */
enum enAccessReadType
{
    rt_NoAccess = 0x0000, /*!< @brief no read access to element at all */
    rt_ReadP    = 0x0001, /*!< @brief read access to element in PRE-OPERATIONAL mode is allowed */
    rt_ReadS    = 0x0002, /*!< @brief read access to element in SAFE-OPERATIONAL mode is allowed */
    rt_ReadPS   = 0x0003, /*!< @brief read access to element in PRE-OPERATIONAL and SAFE-OPERATIONAL modes is allowed */
    rt_ReadO    = 0x0004, /*!< @brief read access to element in OPERATIONAL mode is allowed */
    rt_ReadPO   = 0x0005, /*!< @brief read access to element in PRE-OPERATIONAL and OPERATIONAL modes is allowed */
    rt_ReadSO   = 0x0006, /*!< @brief read access to element in SAFE-OPERATIONAL and OPERATIONAL modes is allowed */
    rt_ReadPSO  = 0x0007, /*!< @brief read access to element in PRE-OPERATIONAL, SAFE-OPERATIONAL and OPERATIONAL modes is allowed */
};

/*! @brief Constants that encode write-access type to OD entry
    for different EtherCAT Slave States.
    It can be OR-ed with read-access type enAccessReadType.
    @see tCANObjectEntryDescription */
enum enAccessWriteType
{
    wt_NoAccess     = 0x0000, /*!< @brief no write access to element at all */
    wt_WriteP       = 0x0008, /*!< @brief write access to element in PRE-OPERATIONAL mode is allowed */
    wt_WriteS       = 0x0010, /*!< @brief write access to element in SAFE-OPERATIONAL mode is allowed */
    wt_WritePS      = 0x0018, /*!< @brief write access to element in PRE-OPERATIONAL and SAFE-OPERATIONAL modes is allowed */
    wt_WriteO       = 0x0020, /*!< @brief write access to element in OPERATIONAL mode is allowed */
    wt_WritePO      = 0x0028, /*!< @brief write access to element in PRE-OPERATIONAL and OPERATIONAL modes is allowed */
    wt_WriteSO      = 0x0030, /*!< @brief write access to element in SAFE-OPERATIONAL and OPERATIONAL modes is allowed */
    wt_WritePSO     = 0x0038, /*!< @brief write access to element in PRE-OPERATIONAL, SAFE-OPERATIONAL and OPERATIONAL modes is allowed */
};


/*! @brief Object entry: mapping access type.
    It can be OR-ed with read-access type enAccessReadType,
    with write-access type enAccessWriteType.
    @see tCANObjectEntryDescription */
enum enAccessMapType
{
    mt_Empty        = 0x0000,   /*!< @brief Object entry is not mappable. */
    mt_RxPDOmap     = 0x0040,   /*!< @brief Object entry is mappable as Rx PDO. */
    mt_TxPDOmap     = 0x0080,   /*!< @brief Object entry is mappable as Rx PDO. */
};

/*! @brief Category type.
    @see tCANObjectEntryDescription */
enum enAccessCategoryType
{
    ct_Empty        = 0x0000,   /*!< @brief Neither backup- nor settings-object. */
    ct_Backup       = 0x0100,   /*!< @brief Backup-object. */
    ct_Settings     = 0x0200,   /*!< @brief Settings-object. */
};

/*! @brief CANopen object description. */
typedef struct stCANObjectDescription
{
    enum enCANObjectType        ObjectType;     /*!< @brief Object type. */
    enum enCANObjectDataType    DataType;       /*!< @brief Object data type. */
    UINT8                       byMaxSubIndex;  /*!< @brief Maximal subindex of object entries. */
} tCANObjectDescription;

#pragma pack(1)
/*! @brief CANopen object entry description.
    @see enAccessWriteType, enAccessReadType, enAccessMapType */
typedef struct stCANObjectEntryDescription
{
    UINT8                       byValueInfo;    /*!< @brief Object entry value info. */
    enum enCANObjectDataType    DataType;       /*!< @brief Object data type. */
    UINT16                      unBitLength;    /*!< @brief Bit length of object entry value. */
    UINT16                      unAccessType;   /*!< @brief Access type. Must be set to OR-ed value of constants of
                                                     types enAccessWriteType and enAccessReadType. */
    UINT16                      unUnitType;     /*!< @brief Unit type. */
} tCANObjectEntryDescription;
#pragma pack()

#define     CAN_VALUE_INFO_ACCESS_RIGTH_MASK            0x01
#define     CAN_VALUE_INFO_OBJECT_CATEGORY_MASK         0x02
#define     CAN_VALUE_INFO_PDO_MAPPABLE_MASK            0x04
#define     CAN_VALUE_INFO_UNIT_TYPE_MASK               0x08
#define     CAN_VALUE_INFO_DEFAULT_VALUE_MASK           0x10
#define     CAN_VALUE_INFO_MINIMUM_VALUE_MASK           0x20
#define     CAN_VALUE_INFO_MAXIMUM_VALUE_MASK           0x40


/*! @brief Structure that describes items in an array of descriptions of PDOs
    mapped to a particular SyncManager (that array is returned
    by #getEcatSlaveOdPDOMapPtrMS function).

    If there is more than one entry inside PDO, then a PDO map
    returned by #getEcatSlaveOdPDOMapPtrMS function will contain several items
    with the same PDO index (i.e. 0x1Axx, 0x16xx, etc) but with
    different index and/or subindex,
    i.e. if for example PDO index is 0x1A00, and the PDO consists
    of 0x6000:5  and 0x6300:7,
    then PDO map will contain two PDOMapItem items with wPdoIndex==0x1A00: \n
        { 0x1A00, 0x6000, 0x5, vvvvv, xxxx } \n
        { 0x1A00, 0x6300, 0x7, yyyyy, zzzz } \n
*/
struct PDOMapItem
{
    UINT16      wPdoIndex;   /*!< @brief PDO Index, for example 0x1Axx, 0x16xx, etc. */
    UINT16      wIndex;      /*!< @brief Index of object that contains object entry mapped into this PDO. */
    UINT16      wSubIndex;   /*!< @brief SubIndex of object entry mapped into this PDO. */
    UINT16      wBitLength;  /*!< @brief bit-length of object entry mapped into this PDO. */
    UINT16      wAccessType; /*!< @brief Access type of object entry that is mapped into this PDO. */

    void*       pPIDataPtr;  /*!< @brief Pointer to data that represents OD object entry.You can access that data by this pointer. */
};

/*! @brief Types of slave's EcatLogger messages.
    @see addEcatLoggerMsgMS, getEcatLoggerMsgNonRtMS */
typedef enum tagEcatLoggerMsgType
{
    TraceMsg           = 0x0001, /*!< @brief Diagnostic message   */
    WarningMsg         = 0x0002, /*!< @brief Warning              */
    ErrorMsg           = 0x0004  /*!< @brief Error                */
} EcatLoggerMsgType;

/*! @brief Represents characteristics of slave's EcatLogger message.
    @see getEcatLoggerMsgNonRtMS */
struct EcatSlaveLoggerMsgParams {
    UINT16              wLength;    /*!< @brief Message length. */
    EcatLoggerMsgType   Type;       /*!< @brief Message type. */
    UINT64              Time;       /*!< @brief Message timestamp. */
};

#define ECAT_MACADDR_SIZE 6

/*! @brief Structure that contains array of bytes that represent MAC address. */
typedef struct _ECAT_MACADDR
{
    ECAT_BYTE RawData[ECAT_MACADDR_SIZE];  /*!< @brief array of bytes that represent MAC address. */
} ECAT_MACADDR;

/*! @brief Type definition of pointer to ECAT_MACADDR. */
typedef struct _ECAT_MACADDR* ECAT_MACADDR_PTR;


/*! @brief EoE stack calls this function when virtual network interface has to be activated.
    @ingroup ecat_slave_mailbox_eoe
    @param pContextData Pointer to user context data.
    @return @ref ECAT_RESULT "Result code". */
typedef ECAT_RESULT (*tEcatEoENICUp)(IN void *pContextData);

/*! @brief EoE stack calls this function when virtual network interface has to be deactivated.
    @ingroup ecat_slave_mailbox_eoe
    @param pContextData Pointer to user context data.
    @return @ref ECAT_RESULT "Result code".*/
typedef ECAT_RESULT (*tEcatEoENICDown)(IN void *pContextData);

/*! @brief EoE stack calls this function when new recived Ethernet frame is available.
    @ingroup ecat_slave_mailbox_eoe
    @param pContextData Pointer to user context data.
    @param pbyData Pointer to received Ethernet frame.
    @param wLen Frame length.
    @return @ref ECAT_RESULT "Result code".*/
typedef ECAT_RESULT (*tEcatEoENICRecive)(
                        IN void         *pContextData,
                        IN ECAT_BYTE    *pbyData,
                        IN ECAT_WORD     wLen);

/*! @brief EoE stack calls this function to gets virtual network interface MAC address.
    @ingroup ecat_slave_mailbox_eoe
    @param pContextData Pointer to user context data.
    @param pbyAddr Pointer to MAC address.
    @param wMaxLen Max address length.
    @return @ref ECAT_RESULT "Result code".*/
typedef ECAT_RESULT (*tEcatEoENICGetHwAddr)(
                        IN void         *pContextData,
                        OUT ECAT_BYTE   *pbyAddr,
                        IN ECAT_WORD     wMaxLen);

/*! @brief Called by EoE stack when new settings are available.
    @ingroup ecat_slave_mailbox_eoe
    @param pContextData Pointer to user context data.
    @param pMac Pointer to mac addres.
    @param dwFlags Indicates which fields are included.
                    0x01 - MAC included.
                    0x02 - IP address included.
                    0x04 - Subnet Mask included.
                    0x08 - Default Gateway included.
                    0x10 - DNS Server IP Address included.
                    0x20 - DNS Name included.
    @param dwIpAddr IP addres.
    @param dwSubNetMask Subnet mask.
    @param dwDefGateway IP address of default gateway.
    @param dwDnsIpAddr IP address of DNS server.
    @param pszDnsName DNS name.
    @return @ref ECAT_RESULT "Result code".*/
typedef ECAT_RESULT (*tEcatEoENICIPSettingsSet)(
    IN void            *pContextData,
    IN ECAT_DWORD       dwFlags,
    IN ECAT_MACADDR_PTR pMac,
    IN ECAT_DWORD       dwIpAddr,
    IN ECAT_DWORD       dwSubNetMask,
    IN ECAT_DWORD       dwDefGateway,
    IN ECAT_DWORD       dwDnsIpAddr,
    IN ECAT_CHAR       *pszDnsName);

/*! @brief CoE emergancy service error codes. */
enum  enMailboxEmergencyCodes {
    err_ResetOrNoError              = 0x0000, /*!< @brief Error Reset or No Error                 */
    err_GenericError                = 0x1000, /*!< @brief Generic Error                           */
    err_Current                     = 0x2000, /*!< @brief Current                                 */
    err_CurrentDeviceInputSide      = 0x2100, /*!< @brief Current, device input side              */
    err_CurrentInsideDevice         = 0x2200, /*!< @brief Current inside the device               */
    err_CurrentDeviceOutputSide     = 0x2300, /*!< @brief Current, device output side             */
    err_Voltage                     = 0x3000, /*!< @brief Voltage                                 */
    err_MainsVoltage                = 0x3100, /*!< @brief Mains Voltage                           */
    err_VoltageInsideDevice         = 0x3200, /*!< @brief Voltage inside the device               */
    err_OutputVoltage               = 0x3300, /*!< @brief Output Voltage                          */
    err_Temperature                 = 0x4000, /*!< @brief Temperature                             */
    err_AmbientTemperature          = 0x4100, /*!< @brief Ambient Temperature                     */
    err_DeviceTemperature           = 0x4200, /*!< @brief Device Temperature                      */
    err_DeviceHardware              = 0x5000, /*!< @brief Device Hardware                         */
    err_DeviceSoftware              = 0x6000, /*!< @brief Device Software                         */
    err_InternalSoftware            = 0x6100, /*!< @brief Internal Software                       */
    err_UserSoftware                = 0x6200, /*!< @brief User Software                           */
    err_DataSet                     = 0x6300, /*!< @brief Data Set                                */
    err_AdditionalModules           = 0x7000, /*!< @brief Additional Modules                      */
    err_Monitoring                  = 0x8000, /*!< @brief Monitoring                              */
    err_Communication               = 0x8100, /*!< @brief Communication                           */
    err_ProtocolError               = 0x8200, /*!< @brief Protocol Error                          */
    err_PDONotProcessed             = 0x8210, /*!< @brief PDO not processed due to length error   */
    err_PDOLengthExceeded           = 0x8220, /*!< @brief PDO length exceeded                     */
    err_ExternalError               = 0x9000, /*!< @brief External Error                          */
    err_StateMachineError           = 0xA000, /*!< @brief EtherCAT State Machine Transition Error */
    err_AdditionalFunctions         = 0xF000, /*!< @brief Additional Functions                    */
    err_DeviceSpecific              = 0xFF00, /*!< @brief Device specific                         */
};


#endif /* ECATSLAVETYPES_H */
