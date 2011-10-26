/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file
    @brief EtherCAT Slave Object Dictionary API declarations. */
/****************************************************************************/

#ifndef APIOBJECTDICTIONARY_H
#define APIOBJECTDICTIONARY_H

#include "ecatslavetypes.h"
#include "ecatcmn.h"
#include "ecatslave_to_MS_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// function name has been changed:
#define addDiagMessage addEcatSlaveOdDiagMessage

/*!**************************************************************************/
/*! @weakgroup ecat_slave_od Object Dictionary API and definitions */

/*! @defgroup ecat_slave_od_cpp \
    Object Dictionary API (C++ implementation, v1.x.x.x)
    @brief  This is a documentation for Object dictionary implementation
            version $(VERSION_KPA_ECATSLAVE_STACK_OD_CPP).
    @ingroup ecat_slave_od */

/*! @defgroup ecat_slave_mapping_and_assignment_cpp \
    Process Image Mapping and Assignment API
    @ingroup ecat_slave_od_cpp */

/*!**************************************************************************/
/*! @addtogroup ecat_slave_od_cpp
    @{
*****************************************************************************/

/* OD object entry values */

/*! @brief Sets the specified value of the specified entry of
        the specified object of the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to variable to be copied to the entry.
    @param DataSize Size of memory that stores the variable (i.e. the size of the buffer pData points to).
    @param bToSysEndian Specifies whether input data must be converted(if ECAT_TRUE)or not(if ECAT_FALSE)to system(cpu)endian format.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryValueByPtrMS, getEcatSlaveOdObjectEntryValueMS  */
ECAT_RESULT setEcatSlaveOdObjectEntryValueMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    IN  UINT8*      pData,
    IN  UINT16      DataSize,
    IN  ECAT_BOOL   bToSysEndian);

/*! @brief Sets the specified value of the specified entry of
        the specified object of the Object Dictionary by using a cached pointer to the object.\n
        The pointer is put to cache on first function call.\n
        Used for speedup entry value access.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to variable to be copied to the entry.
    @param DataSize Size of memory that stores the variable (i.e. the size of the buffer pData points to).
    @param bToSysEndian Specifies whether input data must be converted(if ECAT_TRUE)or not(if ECAT_FALSE)to system(cpu)endian format.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryValueMS, clearObjectsCacheMS */
ECAT_RESULT setEcatSlaveOdObjectEntryValueByPtrMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    IN  UINT8*      pData,
    IN  UINT16      DataSize,
    IN  ECAT_BOOL   bToSysEndian);

/*! @brief Retrieves value of the specified object entry of
        an Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to buffer that receives the specified object
        entry value.
    @param pDataSize Pointer to variable that initially stores the size of the
        specified buffer and after function execution receives the size
        of data that have been written to the buffer.
    @param bFromSysEndian  Specifies whether input data must be converted(if ECAT_TRUE)or not(if ECAT_FALSE)to system(cpu)endian format.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryValueMS, getEcatSlaveOdObjectEntryValueByPtrMS */
ECAT_RESULT getEcatSlaveOdObjectEntryValueMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    OUT UINT8*      pData,
    IN OUT UINT16*  pDataSize,
    IN ECAT_BOOL    bFromSysEndian);

/*! @brief Retrieves the value of the specified object entry of
        the Object Dictionary using a cached pointer to the object.\n
        The pointer is put to cache on first function call.\n
        Used for speedup entry value access.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to buffer that receives the specified object
        entry value.
    @param pDataSize Pointer to variable that initially stores the size of the
        specified buffer and after function execution receives the size
        of data that have been written to the buffer.
    @param bFromSysEndian  Specifies whether input data must be converted(if ECAT_TRUE)or not(if ECAT_FALSE)to system(cpu)endian format.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryValueMS, clearObjectsCacheMS */
ECAT_RESULT getEcatSlaveOdObjectEntryValueByPtrMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    OUT UINT8*      pData,
    IN OUT UINT16  *pDataSize,
    IN ECAT_BOOL    bFromSysEndian);

/*! @brief Retrieves object entry description as it is found in
        the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pObjEntryDescr Pointer to a variable that receives
        object @ref #tCANObjectEntryDescription "entry description".
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryDescriptionMS, getEcatSlaveOdObjectEntryDescriptionByPtrMS */
ECAT_RESULT getEcatSlaveOdObjectEntryDescriptionMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    OUT tCANObjectEntryDescription* pObjEntryDescr);

/*! @brief Retrieves object entry description as it is found in
        the Object Dictionary by using a cached pointer to the object.\n
        The pointer is put to cache on first function call.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pObjEntryDescr Pointer to a variable that receives
        object @ref #tCANObjectEntryDescription "entry description".
    @return @ref ECAT_RESULT "Result code".
    @see   getEcatSlaveOdObjectEntryDescriptionMS, clearObjectsCacheMS  */
ECAT_RESULT getEcatSlaveOdObjectEntryDescriptionByPtrMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    OUT tCANObjectEntryDescription* pObjEntryDescr);

/*! @brief Sets description of the specified entry of the specified
        object(ObjIndex) in the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pObjEntryDescr Poiter to a structure that contains @ref #tCANObjectEntryDescription "entry description".
    @return @ref ECAT_RESULT "Result code".
    @see addEcatSlaveOdObjectEntryMS, getEcatSlaveOdObjectEntryDescriptionMS */
ECAT_RESULT setEcatSlaveOdObjectEntryDescriptionMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    IN  tCANObjectEntryDescription* pObjEntryDescr);

ECAT_RESULT getEcatSlaveOdObjectEntryNameMS(
    IN UINT8            BoardNumber,
    IN  UINT16          ObjIndex,
    IN  UINT8           ObjSubIndex,
    OUT ECAT_STR        pszObjEntryName,
    IN OUT ECAT_WORD*   pwNameLenBytes);

/*! @brief Clears objects pointers' cache. The function is called by the @link #runKpaEcatSlave runKpaEcatSlave @endlink,
    @link #runKpaEcatSlave runKpaEcatSlave @endlink, @link #initEcatSlaveOdMS initEcatSlaveOdMS @endlink,
    @link #removeEcatSlaveOdObjectMS removeEcatSlaveOdObjectMS @endlink functions.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @see getEcatSlaveOdObjectEntryDescriptionByPtrMS, setEcatSlaveOdObjectEntryValueByPtrMS, removeEcatSlaveOdObjectMS */
    ECAT_RESULT clearObjectsCacheMS(IN UINT8 BoardNumber);

/*! @brief Sets SDO callback function.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to @link #tEcatSlaveSdoCallbackFunction SDO callback function @endlink.
    @param pContextData Pointer to data which are passed as argument in callback function.
    @param EventMask Mask for @ref ECATSLAVE_EVENT_OD_OBJENTRY_CHANGED "supported events"(events that will be
        processed by the callback function).
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveSdoCallbackFunctionMS */
ECAT_RESULT setEcatSlaveSdoCallbackFunctionMS(
    IN  UINT8   BoardNumber,
    IN  tEcatSlaveSdoCallbackFunction pFunc,
    IN  void*   pContextData,
    IN  UINT32  EventMask);

/*! @brief Remove SDO callback function.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pFunc Pointer to @link #tEcatSlaveSdoCallbackFunction SDO callback function @endlink.
    @param pContextData Pointer to context data which was previously
        passed to a function that have set the SDO callback function.
    @return @ref ECAT_RESULT "Result code".
    @see  setEcatSlaveSdoCallbackFunctionMS */
ECAT_RESULT removeEcatSlaveSdoCallbackFunctionMS(
    IN UINT8 BoardNumber,
    IN tEcatSlaveSdoCallbackFunction pFunc,
    IN void* pContextData);

/*! @brief Sets OD object entry value and puts Index/Subindex of the object into the SDO Changes List.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to value to be copied to
        the objects entry in the Object Dictionary.
    @param DataSize Size of memory that stores the specified
        entry value(i.e. size of the buffer pData points to).
    @param bToSysEndian Specifies whether input data must be converted(if ECAT_TRUE)or not(if ECAT_FALSE)to system (cpu) endian format.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryValueMS, getNextChangedObject */
ECAT_RESULT setOdObjectEntryValueWithChangeList(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    IN  UINT8       *pData,
    IN  UINT16      DataSize,
    IN  ECAT_BOOL   bToSysEndian);

/*! @brief Extracts Index and Subindex of the oldest object put by the Master to the Changes List.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param Index Pointer to variable that receives object's index.
    @param Subindex Pointer to variable that receives object's subindex.
    @return @ref ECAT_RESULT "Result code".
    @see setOdObjectEntryValueWithChangeList */
ECAT_RESULT getNextChangedObject(
    UINT8   BoardNumber,
    UINT16* Index,
    UINT8*  Subindex);

/*! @brief Adds a diagnostic message to the History object.

    @param BoardNumber Ecat Slave board number (starts from 0).
    @param addMess @ref tOdDiagnosticMessage "OD Diagnostic message" to be added to the History object.
    @param withChangedList Specifies whether a new History object will be added to the Changes List.
    @return Return value #ECAT_S_NOT_ALL means that the Diagnostic message
            shall be sent as Emergency message.
            Return value #ECAT_S_INPUT_EMPTY means that this type of the Diagnostic message
            (info, warning, error) is not sent according to control flags.
            @ref ECAT_RESULT "Result code".
    @see tOdDiagnosticMessage */
ECAT_RESULT addEcatSlaveOdDiagMessage(
    IN UINT8    BoardNumber,
    struct stOdDiagnosticMessage* addMess,
    ECAT_BOOL   withChangedList);

/*! @brief Callback function for storing Backup Parameters to nonvolatile memory.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @note This function must be implemented in user application.
*/
void saveBackupEntries(IN UINT8 BoardNumber);

/*! @brief Callback function for restoring Backup Parameters with default values.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @note This function must be implemented in user application.
*/
void restoreBackupEntries(IN UINT8 BoardNumber);

/*! @brief Calculates CRC and sets it to an Object CRC of backup parameters.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nDataBuf Pointer to data for CRC calculation.
    @param nDataSize Data size.
    @return @ref ECAT_RESULT "Result code."
    @see setBackupParameterChangedFlag
*/
ECAT_RESULT updateBackupCRC(
    IN UINT8    BoardNumber,
    UINT8*      nDataBuf,
    UINT32      nDataSize);

/*! @brief Sets "Backup Parameter Changed" flag and calculates CRC and
    sets it to an Object CRC of backup parameters.
    This function should be called if backup parameters have been
    changed by the slave.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param nDataBuf Pointer to data for CRC calculation.
    @param nDataSize Data size.
    @return @ref ECAT_RESULT "Result code."
    @see updateBackupCRC
    */
ECAT_RESULT setBackupParameterChangedFlag(
    IN UINT8    BoardNumber,
    UINT8*      nDataBuf,
    UINT32      nDataSize);


/*! @brief Creates and initializes slave's Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see loadEcatSlaveOdFromFileMS */
ECAT_RESULT initEcatSlaveOdMS(IN UINT8 BoardNumber);

/*! @brief Releases slave's Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveOdMS */
ECAT_RESULT releaseEcatSlaveOdMS(IN UINT8 BoardNumber);

/*! @brief Starts slave's Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveOdMS, loadEcatSlaveOdFromFileMS, stopEcatSlaveOdMS */
ECAT_RESULT startEcatSlaveOdMS(IN UINT8 BoardNumber);

/*! @brief Stops slave's Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see initEcatSlaveOdMS, loadEcatSlaveOdFromFileMS, startEcatSlaveOdMS */
ECAT_RESULT stopEcatSlaveOdMS(IN UINT8 BoardNumber);

#if 0
// /*! @brief Gets slave's Object Dictionary information.
//     @param pData Pointer to buffer. After successful execution the buffer
//         will contain slave's Object Dictionary information.
//     @param DataSize Size of the buffer measured in bytes.
//     @return @ref ECAT_RESULT "Result code".
//     @see getEcatSlaveResultCodeDescription. */
// ECAT_RESULT getEcatSlaveOdInformation(
//     OUT UINT8*  pData,
//     IN  UINT16  DataSize);
#endif

/*! @brief Gets the total number of objects in the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pObjCount Pointer to variable that receives the total number of
        objects in the Object Dictionary.
    @return @ref ECAT_RESULT "Result code".
    @see  getEcatSlaveOdObjectsListMS */
ECAT_RESULT getEcatSlaveOdObjectsCountMS(
    IN  UINT8   BoardNumber,
    OUT UINT32* pObjCount);

/*! @brief Gets the list of indexes of all objects that are currently in
    the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param pObjIndexList Pointer to array of received indexes.
        Indexes are represented as UINT16 values.
    @param ObjIndexListMaxLen Maximum number of elements of type UINT16 that
        the pObjIndexList array can hold (size of pObjIndexList divided by two).
    @return @ref ECAT_RESULT "Result code".
    @see  getEcatSlaveOdObjectsCountMS */
ECAT_RESULT getEcatSlaveOdObjectsListMS(
    IN  UINT8   BoardNumber,
    OUT UINT16* pObjIndexList,
    IN  UINT32  ObjIndexListMaxLen);

/*--- Working with OD objects */

/*! @brief Adds a new object to Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of a new object.
    @param pName Pointer to a null-terminated string
        containing the object name.
    @param pObjDescr Pointer to @ref #tCANObjectDescription "object description".
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveOdObjectMS, setEcatSlaveOdObjectDescriptionMS */
ECAT_RESULT addEcatSlaveOdObjectMS(
    IN  UINT8                   BoardNumber,
    IN  UINT16                  ObjIndex,
    IN  ECAT_CHAR*              pName,
    IN  tCANObjectDescription*  pObjDescr);

/*! @brief Removes the object with the specified index from the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex  Index of the object.
    @return @ref ECAT_RESULT "Result code".
    @see  addEcatSlaveOdObjectMS, removeAllEcatSlaveOdObjectsMS, clearObjectsCacheMS */
ECAT_RESULT removeEcatSlaveOdObjectMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex);

/*! @brief Removes all objects from the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveOdObjectMS */
ECAT_RESULT removeAllEcatSlaveOdObjectsMS(UINT8 BoardNumber);

/*! @brief Sets description of the specified object in the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param pObjDescr Poiter to a structure that contains the @ref #tCANObjectDescription "object description".
    @return @ref ECAT_RESULT "Result code".
    @see  getEcatSlaveOdObjectDescriptionMS */
ECAT_RESULT setEcatSlaveOdObjectDescriptionMS(
    IN UINT8    BoardNumber,
    IN UINT16   ObjIndex,
    IN tCANObjectDescription*   pObjDescr);

/*! @brief Retrieves object description as it is found in the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param pObjDescr Pointer to variable that receives the @ref #tCANObjectDescription "object description".
    @return @ref ECAT_RESULT "Result code".
    @see  setEcatSlaveOdObjectDescriptionMS */
ECAT_RESULT getEcatSlaveOdObjectDescriptionMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    OUT tCANObjectDescription*  pObjDescr);

/*--- Working with OD object entries */
/*! @brief Adds a new entry into an object of the Object Dictionary.
        The object must already exist in the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pName Pointer to a null-terminated string
        containing the entry name.
    @param pObjEntryDescr Pointer to a variable that contains the object @ref #tCANObjectEntryDescription "entry description".
    @return @ref ECAT_RESULT "Result code".
    @see  addEcatSlaveOdObjectMS, removeEcatSlaveOdObjectEntryMS */
ECAT_RESULT addEcatSlaveOdObjectEntryMS(
    IN  UINT8           BoardNumber,
    IN  UINT16          ObjIndex,
    IN  UINT8           ObjSubIndex,
    IN  ECAT_CHAR*      pName,
    IN  tCANObjectEntryDescription* pObjEntryDescr);

/*! @brief Removes the specified entry from the specified
        object of Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @return @ref ECAT_RESULT "Result code".
    @see addEcatSlaveOdObjectEntryMS */
ECAT_RESULT removeEcatSlaveOdObjectEntryMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex);

/*! @brief  Removes all entries from the specified object of the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveOdObjectEntryMS, removeEcatSlaveOdObjectMS */
ECAT_RESULT removeAllEcatSlaveOdObjectEntriesMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex);

/* default value */

/*! @brief Sets the specified value of the specified entry of
        the specified object of the Object Dictionary as
        a default value of the entry.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to the value to be copied to
        the specified default entry value of the object in
        the Object Dictionary.
    @param DataSize Size of memory that stores the specified
        entry value(i.e. the size of the buffer pData points to).
    @return @ref ECAT_RESULT "Result code".
    @see  getEcatSlaveOdObjectEntryDefaultValueMS */
ECAT_RESULT setEcatSlaveOdObjectEntryDefaultValueMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    IN  UINT8*  pData,
    IN  UINT16  DataSize);

/*! @brief Retrieves a default value of the specified object entry of
        the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to buffer that receives the specified object
        entry value.
    @param pDataSize Pointer to variable that initially stores the size of the
        specified buffer and after function execution receives the size of data
        that have been written to the buffer.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryDefaultValueMS */
ECAT_RESULT getEcatSlaveOdObjectEntryDefaultValueMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    OUT UINT8*      pData,
    IN OUT UINT16*  pDataSize);

/* min value */

/*! @brief Sets the value of the specified entry of
        the object of the Object Dictionary as
        a minimal value of the entry.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to the entry value to be copied to
        the specified minimum entry value of the specified object in
        the Object Dictionary.
    @param DataSize Size of memory that stores the specified
        entry value(i.e.the size of the buffer pData points to).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveOdObjectEntryMinValueMS, CAN_VALUE_INFO_MINIMUM_VALUE_MASK */
ECAT_RESULT setEcatSlaveOdObjectEntryMinValueMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    IN  UINT8*  pData,
    IN  UINT16  DataSize);

/*! @brief Retrieves the minimum value of the specified object entry of
        the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to the buffer that receives the specified object
        entry value.
    @param pDataSize Pointer to the variable that initially stores the size of the
        specified buffer and after function execution receives the size
        of data written to the buffer.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryMinValueMS */
ECAT_RESULT getEcatSlaveOdObjectEntryMinValueMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    OUT UINT8*      pData,
    IN OUT UINT16*  pDataSize);

/* max value */

/*! @brief Sets the specified value of the specified entry of
        the specified object of the Object Dictionary as
        a maximal value of the entry.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to the entry value to be copied to
        the specified maximum entry value of the specified object in
        the Object Dictionary.
    @param DataSize Size of memory that stores the specified
        entry value(i.e. the size of the buffer pData points to).
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveOdObjectEntryMaxValueMS, CAN_VALUE_INFO_MAXIMUM_VALUE_MASK */
ECAT_RESULT setEcatSlaveOdObjectEntryMaxValueMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  ObjIndex,
    IN  UINT8   ObjSubIndex,
    IN  UINT8*  pData,
    IN  UINT16  DataSize);

/*! @brief Retrieves maximal value of the specified object entry of
        the Object Dictionary.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param ObjSubIndex Sub-index of the object.
    @param pData Pointer to the buffer that receives the specified object
        entry value.
    @param pDataSize Pointer to the variable that initially stores the size of the
        specified buffer and after function execution receives the size
        of data written to the buffer.
    @return @ref ECAT_RESULT "Result code".
    @see setEcatSlaveOdObjectEntryMaxValueMS */
ECAT_RESULT getEcatSlaveOdObjectEntryMaxValueMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    IN  UINT8       ObjSubIndex,
    OUT UINT8*      pData,
    IN OUT UINT16*  pDataSize);

#if 0
// /*! @brief Gets Vendor-specific information of slave's object.
//     @param ObjIndex Index of the object.
//     @param ObjSubIndex Sub-index of the object.
//     @param pData Pointer to the buffer. After successful execution the buffer
//         will contain vendor-specific information.
//     @param DataSize Size of the buffer measured in bytes.
//     @return @ref ECAT_RESULT "Result code".
//     @see getEcatSlaveResultCodeDescription. */
// ECAT_RESULT getEcatSlaveObjectVendorSpecificInformation(
//     IN  UINT16  ObjIndex,
//     IN  UINT8   ObjSubIndex,
//     OUT UINT8*  pData,
//     IN  UINT16  DataSize);
#endif

/*! @brief Adds SDO events filter (i.e. disables notification to user
           that SDO event occured) for a range of OD objects.
           If this function is called then SDO events will not be
           generated for OD objects that have indexes from
           (wObjIndex) to (wObjIndex + wObjIndexLen - 1) inclusive.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wFirstObjIndex Start index of OD objects range.
    @param wObjIndexRangeLen Lenght of OD object indexes range.
    @return @ref ECAT_RESULT "Result code".
    @see removeEcatSlaveSdoEventsFilterMS, setEcatSlaveSdoCallbackFunctionMS,
    getEcatSlaveResultCodeDescription. */
ECAT_RESULT addEcatSlaveSdoEventsFilterMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  wFirstObjIndex,
    IN  UINT16  wObjIndexRangeLen);

/*! @brief Removes SDO events filter (i.e. explicitly enables notification
           to user that SDO event occured) for a range of OD objects.
           If this function is called then SDO events will be explicitly
           enabled for OD objects that have indexes from
           (wObjIndex) to (wObjIndex + wObjIndexLen - 1) inclusive.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wFirstObjIndex Start index of OD objects range.
    @param wObjIndexRangeLen Lenght of OD object indexes range.
    @return @ref ECAT_RESULT "Result code".
    @see addEcatSlaveSdoEventsFilterMS, setEcatSlaveSdoCallbackFunctionMS,
    getEcatSlaveResultCodeDescription. */
ECAT_RESULT removeEcatSlaveSdoEventsFilterMS(
    IN  UINT8   BoardNumber,
    IN  UINT16  wFirstObjIndex,
    IN  UINT16  wObjIndexRangeLen);


/*! @brief Gets pointer to process image data of OD Entry.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of an object.
    @param ObjSubIndex Sub-index of an object.
    @param phHandle Pointer to handle.
    @param pBitLength Bit length.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveOdEntryPIDataPtrMS(
    IN  UINT8           BoardNumber,
    IN  UINT16          ObjIndex,
    IN  UINT8           ObjSubIndex,
    OUT ECAT_HANDLE     *phHandle,
    OUT UINT16          *pBitLength);

/*! @brief Gets the number of Object entries (Subindex "0" is included).
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of the object.
    @param pwNum Number of entries.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveOdObjectEntriesListMS */
ECAT_RESULT getEcatSlaveOdObjectEntriesNumberMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    OUT UINT16*     pwNum);

/*! @brief Gets the entries list.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param ObjIndex Index of an object.
    @param SubIndexes Pointer list of entries subindexes.
    @param pwNum In - buffer size (size of buffer the SubIndexes parameter points to). Out - number of entries.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveOdObjectEntriesNumberMS */
ECAT_RESULT getEcatSlaveOdObjectEntriesListMS(
    IN  UINT8       BoardNumber,
    IN  UINT16      ObjIndex,
    OUT UINT8*      SubIndexes,
    IN OUT UINT16*  pwNum);

/*! @} (marks end of group) */

/*!**************************************************************************/
/*! @addtogroup ecat_slave_mapping_and_assignment_cpp
    @{
*****************************************************************************/

/*! @brief Enables PDO Mapping And Assignment.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param bEnable Function switch \n
    #ECAT_TRUE - enabled \n
    #ECAT_FALSE - disabled .
    @return @ref ECAT_RESULT "Result code".
    @see buildEcatSlaveProcessImageMS */
ECAT_RESULT enableEcatSlavePDOMappingAndAssignmentMS(
    IN UINT8        BoardNumber,
    IN ECAT_BOOL    bEnable);

/*! @brief Builds the OD inputs (or outputs) Process Image for the specified SyncManager ID.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param unSMIndex SyncManager ID.
    @return @ref ECAT_RESULT "Result code".
    @see enableEcatSlavePDOMappingAndAssignmentMS */
ECAT_RESULT buildEcatSlaveProcessImageMS(
    IN UINT8    BoardNumber,
    IN UINT16   unSMIndex);

/*! @brief Check if the Process Image of the OD is open (return ECAT_S_OK) or
    locked by setEcatSlaveOdObjectEntryValueMS or getEcatSlaveOdObjectEntryValueMS.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT checkEcatSlavePiIsAvailable (IN UINT8 BoardNumber);


/*! @brief Gets the pointer to the SyncManager Process Image buffer.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wSMIndex SyncManager ID.
    @param ppImage Pointer to the Process Image.
    @param pwSize Process Image Buffer Size.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveSMProcessImagePtrMS(
    IN UINT8    BoardNumber,
    IN UINT16   wSMIndex,
    OUT UINT8** ppImage,
    OUT UINT16* pwSize);

/*! @brief Locks Process Images.
    @param BoardNumber Ecat Slave board number (starts from 0). */
void LockEcatSlavePIMS(IN UINT8 BoardNumber);

/*! @brief Unlocks Process Images.
    @param BoardNumber Ecat Slave board number (starts from 0). */
void ReleaseEcatSlavePIMS(IN UINT8 BoardNumber);


/*! @brief Cleans Process Image.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param unSMIndex SyncManager ID.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT cleanEcatSlaveProcessImageMS(
    IN UINT8    BoardNumber,
    IN UINT16   unSMIndex);

/*! @brief Gets PDO Mapping information for the SyncManager.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wSmId SyncManager ID.
    @param ppMap Pointer to PDO mapping items.
    @param pwNumMapItems Number of mapped items.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveOdPDOMapPtrMS(
    IN UINT8                BoardNumber,
    IN UINT16               wSmId,
    OUT struct PDOMapItem** ppMap,
    OUT UINT16*             pwNumMapItems);

/*! @brief Gets a duplicate of PDO Mapping information for the SyncManager.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param wSmId SyncManager ID.
    @param pMap Pointer to PDO mapping items.
    @param pwNumMapItems IN - max number of items in the buffer, OUT - number of mapped items.
    @return @ref ECAT_RESULT "Result code".
    @see getEcatSlaveResultCodeDescription. */
ECAT_RESULT getEcatSlaveOdPDOMapCopyMS(
    IN UINT8                BoardNumber,
    IN UINT16               wSmId,
    OUT struct PDOMapItem*  pMap,
    IN OUT UINT16*          pwNumMapItems);
/*! @} (marks end of group) */


#ifdef __cplusplus
}
#endif


#endif /* APIOBJECTDICTIONARY_H */
