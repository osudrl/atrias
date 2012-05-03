/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2008
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecsresultcodes.h
    @brief This file contains Result Codes and their descriptions. */
/****************************************************************************/

#ifndef ECATRESULTCODES_H
#define ECATRESULTCODES_H

#include "ecatbasictypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!**************************************************************************/
/*! @defgroup ecat_slave_result_codes Return codes & system messages handling
    @ingroup ecat_slave
    @brief Type definitions, constants, functions used
           for handling system messages and result codes.
    @{
*****************************************************************************/

/*! @brief Constants that represent various @ref ECAT_RESULT "Result codes".
    @anchor ECAT_RESULT
    Constants starting with "ECAT_S_" mean various types of successful results.
    Constants starting with "ECAT_E_" mean various types of failures.
    When masked with 0x8000 success-constants give 0x0000 and
    failure-constants give 0x8000.
    @see getEcatSlaveResultCodeDescription, ECAT_SUCCEEDED, ECAT_FAILED */
enum enEcatSlaveResultCode
{
//#warning THIS TYPE DEFINITION WILL BE IMPROVED SOON!
    ECAT_S_OK                           = 0x0000, /*!< No error. */
    ECAT_S_NOITEMS                      = 0x0002, /*!< No more items (for get next functions). */
    ECAT_S_NOT_ALL                      = 0x0003, /*!< Not complete actions (e.g. not all data have been send). */
    ECAT_S_SYNC_CALL                    = 0x0004, /*!< Call need synchronization (internal). */
    ECAT_S_AGAIN                        = 0x0005, /*!< Operation is in progress. Try again later. */
    ECAT_S_INPUT_EMPTY                  = 0x0006, /*!< No input data for processing (e.g. an empty input data buffer). */

    ECAT_E_FAIL                         = 0x8000, /*!< General error. */
    ECAT_E_NOTIMPL                      = 0x8001, /*!< Function is not implemented. */
    ECAT_E_NOP                          = 0x8002, /*!< Nothing has been done. */
    ECAT_E_INVALIDARG                   = 0x8003, /*!< Function is called with one or more invalid arguments. */
    ECAT_E_INVALID_POINTER              = 0x8004, /*!< Attempt to use invalid pointer. */
    ECAT_E_OUTOFMEMORY                  = 0x8005, /*!< Memory limit (usually can not allocate new memory). */
    ECAT_E_BUF_EMPTY                    = 0x8006, /*!< Ecat slave buffer has not been updated by master since latest read. Operation was not executed.*/

    ECAT_E_INVALID_STATE                = 0x8007, /*!< Invalid state. */
    ECAT_E_OUT_OF_ADR_RANGE             = 0x8008, /*!< Address is out of addresses range. */

    ECAT_E_MB_COE_TRANSITION_ABORTED    = 0x8009, /*!< Mailbox CanOpen transition aborted. */
    ECAT_E_NOTAVAILABLE                 = 0x800A, /*!< Not available or not supported. */

    ECAT_E_ALREADY_STARTED              = 0x800B, /*!< Slave already started. */
    ECAT_E_ALREADY_STOPED               = 0x800C, /*!< Obsolete! Use ECAT_E_ALREADY_STOPPED instead. */
    ECAT_E_ALREADY_STOPPED              = 0x800C, /*!< Slave already stopped. */

    ECAT_E_NOTINITIALIZED               = 0x800D, /*!< Not initialized. */
    ECAT_E_SLAVE_NOT_INIT               = 0x800E, /*!< The Slave has not been initialized. */
    ECAT_E_BOARD_NOT_FOUND              = 0x800F, /*!< The Slave's board has not been found or software/hardware versions mismatch. */
    ECAT_E_SYNC_CALL_TIMEOUT            = 0x8010,   /*!< Call timeout. */
    ECAT_E_SLAVE_EEPROM_NOT_LOADED      = 0x8011, /*!< The ESC EEPROM has not been initialized. */

    ECAT_E_SM_WRONG_NUMBER              = 0x8100, /*!< Wrong SyncManager number. */
    ECAT_E_SM_WRONG_DIRECTION           = 0x8101, /*!< Wrong SyncManager direction. */
    ECAT_E_INSUFFICIENT_BUF_SIZE        = 0x8102, /*!< Buffer size is not enough. */
    ECAT_E_MBX_BUSY                     = 0x8103, /*!< Mailbox is busy (full for write, empty for read). */

    ECAT_E_SM_DISABLED                  = 0x8104, /*!< Access to memory without SM control. */
    ECAT_E_FMMU_WRONG_NUMBER            = 0x8105, /*!< Wrong FMMU number. */
    ECAT_E_SM_WRONG_MODE                = 0x8106, /*!< Mode: mailbox, 3-buffers. */

    ECAT_E_EEPROM_NOT_PDI_ACCESS        = 0x8110, /*!< Slave doesn't have PDI access to EEPROM. */
    ECAT_E_EEPROM_BUSY                  = 0x8111, /*!< Description not available. */

    ECAT_E_OD_OBJECT_NOTFOUND                   = 0x8200, /*!< No object in OD. */
    ECAT_E_OD_OBJECT_ALREADY_EXISTS             = 0x8201, /*!< Object is already in OD. */
    ECAT_E_OD_ENTRY_DESCRIPTION_ALREADY_EXISTS  = 0x8202, /*!< OD entry description already exists. */
    ECAT_E_OD_ENTRY_DESCRIPTION_FAILED          = 0x8203, /*!< Failed to create OD entry description. */
    ECAT_E_OD_INVALID_OBJECT_TYPE               = 0x8204, /*!< OD object type is invalid. */
    ECAT_E_OD_INVALID_ACCESS_TYPE               = 0x8205, /*!< OD access type is invalid.  */
    ECAT_E_OD_INVALID_DATA_TYPE                 = 0x8206, /*!< Invalid OD data type.  */
    ECAT_E_OD_ACCESS_DENIED                     = 0x8207, /*!< Access to OD denied. */
    ECAT_E_OD_NOT_CREATED                       = 0x8208, /*!< OD has not been created. */
    ECAT_E_OD_SDO_CHNG_LIST_NOT_READY           = 0x8209, /*!< SDO changes list is not ready since modification flag has not been cleared by Master. */
    ECAT_E_OD_SDO_CHNG_LIST_NOT_SUPPORTED       = 0x820a, /*!< SDO changes list is not supported. */
    ECAT_E_OD_SDO_CHNG_LIST_FULL                = 0x820b, /*!< SDO changes list is full. */
    ECAT_E_OD_SDO_CHNG_LIST_FAILS               = 0x820c, /*!< Writing to SDO changes list fails. */
    ECAT_E_DMH_OBJ_FAILS                        = 0x820d, /*!< Fail to work with Diagnostic Message History object. */
    ECAT_E_DMH_FULL                             = 0x820e, /*!< Diagnostic Message History is full (can't write new message). */
    ECAT_E_ENTRY_NOT_CREATED                    = 0x820f, /*!< Fails to create OD entry. */
    ECAT_E_DMH_WRONG_PARAMETER                  = 0x8210, /*!< Wrong Diagnostic Message parameter. */
    ECAT_E_OD_SDO_CHNG_LIST_EMPTY               = 0x8211, /*!< SDO changes list is empty. */

    ECAT_E_OD_SDO_SERVICE_NOT_SUPORTED          = 0x8212, /*!< Mailbox error: not supported CoE service in CoE header. */
    ECAT_E_OD_SDO_SIZE_INVALID_HEADER           = 0x8213, /*!< Mailbox error: invalid CoE SDO header. */
    ECAT_E_OD_SDO_SIZE_TOO_SHORT                = 0x8214, /*!< Mailbox error: CoE SDO service with Len < 10. */
    ECAT_E_OD_SDO_INVALID_SIZE                  = 0x8215, /*!< Mailbox error: invalid size. */

    ECAT_E_OD_SUB_OBJ_NOTFOUND                  = 0x8216, /*!< Sub-object not found in object. */
    ECAT_E_OD_MORE_MAXIMUM_VALUE                = 0x8217, /*!< Entry value is more than maximum. */
    ECAT_E_OD_LESS_MINIMUM_VALUE                = 0x8218, /*!< Entry value is less than minimum. */

    ECAT_E_IOCTL_LIB_NOT_INITIALIZED    = 0x8300, /*!< EcatSlave user-mode library is not initialized. */
    ECAT_E_IOCTL_FAIL                   = 0x8301, /*!< Unknown ioctl error. */

    ECAT_E_IOCTL_EFAULT                 = 0x8302, /*! EcatSlave Api function call through IOCTL failed: bad address. */

    ECAT_E_IOCTL_ENOTTY                 = 0x8303, /*!< API call through IOCTL failed: No such ioctl command for device. */
    ECAT_E_IOCTL_NOT_RTEXEC             = 0x8304, /*!< API call was not executed by rtexecuter. Just for debug. */
    ECAT_E_BOARD_VERSION_FAIL           = 0x8305, /*!< Unsupported EtherCAT Slave board version. */

    ECAT_E_FILE_OPEN                    = 0x8310,   /*!< Can't open file. */
    ECAT_E_FILE_NOT_SUP                 = 0x8311,   /*!< The file operation is not supported. */
    ECAT_E_FILE_NAME                    = 0x8312,   /*!< Wrong file name. */
    ECAT_E_FILE_CLOSE                   = 0x8313,   /*!< Can't close file. */
    ECAT_E_FILE_OFFSET                  = 0x8314,   /*!< Wrong file offset. */
    ECAT_E_FILE_ACCESS                  = 0x8315,   /*!< File access denied. E.g. wrong FoE password. */

    /* XML related errors */
    ECAT_E_XML_OPEN                     = 0x8500,   /*!< Can't open XML file. */
    ECAT_E_XML_PARSE                    = 0x8501,   /*!< Error while parsing XML data. */
    /* ... more XML errors here ... */
    /* End of XML related errors */

    /* Start new group from 0x8600 ... */

};

/*! @brief Check the 15th most significant bit of an input value.
    Value "0" in the 15th bit indicates success, and the other bits(0-14)
    describe the type of success.
    @see enEcatSlaveResultCode, ECAT_RESULT */
#define ECAT_SUCCEEDED(Status)  (((ECAT_RESULT)(Status) & 0x8000) == 0)
/*! @brief Constants that represent various @ref ECAT_RESULT "Result codes".
    @see getEcatSlaveResultCodeDescription */
typedef enum enEcatSlaveResultCode ECAT_RESULT;

/*! @brief Check the 15th most significant bit of input value.
    Value "1" in the 15th bit indicates failure, and the other bits(0-14)
    describe the type of failure.
    @see enEcatSlaveResultCode, ECAT_RESULT */
#define ECAT_FAILED(Status)     (((ECAT_RESULT)(Status) & 0x8000) != 0)

/*! @brief Returns string that describes EcatSlave @ref ECAT_RESULT "Result code".
    @param ResultCode EcatSlave @ref ECAT_RESULT "Result code".
    @return String constant that describes EcatSlave @ref ECAT_RESULT "Result code".
    @see enEcatSlaveResultCode, ECAT_RESULT */
extern ECAT_CHAR* getEcatSlaveResultCodeDescription(ECAT_RESULT ResultCode);

/*! @} (marks end of group) */

#ifdef __cplusplus
}
#endif

#endif /* ECATRESULTCODES_H */
