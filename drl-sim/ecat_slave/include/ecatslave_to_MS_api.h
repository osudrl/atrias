/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatslave_to_MS_api.h
    @brief Single Slave Functions API Redefenition for
           compatibility with Multi-Slave Functions API (suffix MS). */
/****************************************************************************/

#ifndef ECATSLAVE_TO_MS_API_H
#define ECATSLAVE_TO_MS_API_H

#include "ecatslavetypes.h"
#include "ecatcmn.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************/
/* ecat_slave_init
*****************************************************************************/

//#define setEcatSlaveParameters(a,b)                     setEcatSlaveParametersMS(0,a,b)
#define setEcatSlaveMaxIoSize(a,b)                      setEcatSlaveMaxIoSizeMS(0,a,b)
#define setEcatSlaveSyncManager(a,b,c,d,e,f)            setEcatSlaveSyncManagerMS(0,a,b,c,d,e,f)

#define setEcatSlavePollTime(a)                         setEcatSlavePollTimeMS(0,a)
#define setEcatSlaveHeartbeatInterval(a)                setEcatSlaveHeartbeatIntervalMS(0,a)

#define initEcatSlave()                                 initEcatSlaveMS(0)
#define releaseEcatSlave()                              releaseEcatSlaveMS(0)
#define startEcatSlave()                                startEcatSlaveMS(0)
#define stopEcatSlave()                                 stopEcatSlaveMS(0)

#define setEcatSlaveCallbackFunction(a,b,c)             setEcatSlaveCallbackFunctionMS(0,a,b,c)
#define removeEcatSlaveCallbackFunction(a,b)            removeEcatSlaveCallbackFunctionMS(0,a,b)
#define setEcatSlaveOnStateChangeCallbackFunction(a)    setEcatSlaveOnStateChangeCallbackFunctionMS(0,a)
#define removeEcatSlaveOnStateChangeCallbackFunction(a) removeEcatSlaveOnStateChangeCallbackFunctionMS(0,a)

/****************************************************************************/
/* ecat_slave_io_functions
*****************************************************************************/
#define updateEcatSlaveTxInputs(a,b)        updateEcatSlaveTxInputsMS(0,a,b)
#define updateEcatSlaveRxOutputs(a,b)       updateEcatSlaveRxOutputsMS(0,a,b)
#define performEcatSlaveMailboxUpdate       performEcatSlaveMailboxUpdate
#define getEcatSlaveConfiguredIoSize(a,b)   getEcatSlaveConfiguredIoSizeMS(0,a,b)
#define getEcatSlaveSmDebugBuffer(a,b)      getEcatSlaveSmDebugBufferMS(0,a,b)
#define getEcatSlaveAlEventMask(a)          getEcatSlaveAlEventMaskMS(0,a)
#define setEcatSlaveAlEventMask(a)          setEcatSlaveAlEventMaskMS(0,a)

#define readEcatSlaveMem(a,b,c)             readEcatSlaveMemMS(0,a,b,c)
#define writeEcatSlaveMem(a,b,c)            writeEcatSlaveMemMS(0,a,b,c)
#define readEcatSlaveEeprom(a,b,c)          readEcatSlaveEepromMS(0,a,b,c)
#define writeEcatSlaveEeprom(a,b,c)         writeEcatSlaveEepromMS(0,a,b,c)
#define updateEcatSlaveEepromChecksum()     updateEcatSlaveEepromChecksumMS(0)
#define getEcatSlaveSmConfiguration(a,b)    getEcatSlaveSmConfigurationMS(0,a,b)
#define getEcatSlaveFmmuConfiguration(a,b)  getEcatSlaveFmmuConfigurationMS(0,a,b)
#define getEcatSlaveDlStatus(a)             getEcatSlaveDlStatusMS(0,a)
#define getEcatSlaveDlControl(a)            getEcatSlaveDlControlMS(0,a)
#define getEcatSlaveRequestedState(a)       getEcatSlaveRequestedStateMS(0,a)
#define getEcatSlaveALStatus(a)             getEcatSlaveALStatusMS(0,a)
#define setEcatSlaveState(a)                setEcatSlaveStateMS(0,a)
#define setEcatSlaveErrorStatus(a)          setEcatSlaveErrorStatusMS(0,a)
#define requestEcatSlaveState(a,b)          requestEcatSlaveStateMS(0,a,b)

/****************************************************************************/
/* ecat_slave_statistics_diagnostics
*****************************************************************************/
//#define getEcatSlaveStatisticData(a)
//#define clearEcatSlaveStatisticData()
//#define getEcatSlaveDiagnosticData(a)

/***************************************************************************/
/*  ecat_slave_logging
*****************************************************************************/
//#define setEcatSlaveLogBufferSize(a)
//#define setEcatSlaveLogMode(a)
//#define startEcatSlaveLogging(a)
//#define stopEcatSlaveLogging()

/****************************************************************************/
/* ecat_slave_dc
*****************************************************************************/
#define initEcatSlaveDc()                   initEcatSlaveDcMS(0)
#define getEcatSlaveDcSystemTime32(a)       getEcatSlaveDcSystemTime32MS(0,a)
#define getEcatSlaveDcSystemTime64(a)       getEcatSlaveDcSystemTime64MS(0,a)
#define initEcatSlaveDcLatch(a,b,c)         initEcatSlaveDcLatchMS(0,a,b,c)
#define getEcatSlaveDcLatchStatus(a,b,c,d)  getEcatSlaveDcLatchStatusMS(0,a,b,c,d)
#define getEcatSlaveDcLatchTime32(a,b,c)    getEcatSlaveDcLatchTime32MS(0,a,b,c)
#define getEcatSlaveDcLatchTime64(a,b,c)    getEcatSlaveDcLatchTime64MS(0,a,b,c)

/****************************************************************************/
/* ecat_slave_sdo
*****************************************************************************/
#define initEcatSlaveOd()                                   initEcatSlaveOdMS(0)
#define releaseEcatSlaveOd()                                releaseEcatSlaveOdMS(0)
#define startEcatSlaveOd()                                  startEcatSlaveOdMS(0)
#define stopEcatSlaveOd()                                   stopEcatSlaveOdMS(0)
#define getEcatSlaveOdObjectsCount(a)                       getEcatSlaveOdObjectsCountMS(0,a)
#define getEcatSlaveOdObjectsList(a,b)                      getEcatSlaveOdObjectsListMS(0,a,b)
#define addEcatSlaveOdObject(a,b,c)                         addEcatSlaveOdObjectMS(0,a,b,c)
#define removeEcatSlaveOdObject(a)                          removeEcatSlaveOdObjectMS(0,a)
#define removeAllEcatSlaveOdObjects()                       removeAllEcatSlaveOdObjectsMS(0)
#define setEcatSlaveOdObjectDescription(a,b)                setEcatSlaveOdObjectDescriptionMS(0,a,b)
#define getEcatSlaveOdObjectDescription(a,b)                getEcatSlaveOdObjectDescriptionMS(0,a,b)
#define addEcatSlaveOdObjectEntry(a,b,c,d)                  addEcatSlaveOdObjectEntryMS(0,a,b,c,d)
#define removeEcatSlaveOdObjectEntry(a,b)                   removeEcatSlaveOdObjectEntryMS(0,a,b)
#define removeAllEcatSlaveOdObjectEntries(a)                removeAllEcatSlaveOdObjectEntriesMS(0,a)
#define setEcatSlaveOdObjectEntryDescription(a,b,c)         setEcatSlaveOdObjectEntryDescriptionMS(0,a,b,c)
#define getEcatSlaveOdObjectEntryDescription(a,b,c)         getEcatSlaveOdObjectEntryDescriptionMS(0,a,b,c)
//#define clearObjectsCache()                               clearObjectsCacheMS(0)
#define getEcatSlaveOdObjectEntryDescriptionByPtr(a,b,c)    getEcatSlaveOdObjectEntryDescriptionByPtrMS(0,a,b,c)
#define setEcatSlaveOdObjectEntryValue(a,b,c,d,e)           setEcatSlaveOdObjectEntryValueMS(0,a,b,c,d,e)
#define setEcatSlaveOdObjectEntryValueByPtr(a,b,c,d,e)      setEcatSlaveOdObjectEntryValueByPtrMS(0,a,b,c,d,e)
#define getEcatSlaveOdObjectEntryValue(a,b,c,d,e)           getEcatSlaveOdObjectEntryValueMS(0,a,b,c,d,e)
#define getEcatSlaveOdObjectEntryValueByPtr(a,b,c,d,e)      getEcatSlaveOdObjectEntryValueByPtrMS(0,a,b,c,d,e)
#define setEcatSlaveOdObjectEntryDefaultValue(a,b,c,d)      setEcatSlaveOdObjectEntryDefaultValueMS(0,a,b,c,d)
#define getEcatSlaveOdObjectEntryDefaultValue(a,b,c,d)      getEcatSlaveOdObjectEntryDefaultValueMS(0,a,b,c,d)
#define setEcatSlaveOdObjectEntryMinValue(a,b,c,d)          setEcatSlaveOdObjectEntryMinValueMS(0,a,b,c,d)
#define getEcatSlaveOdObjectEntryMinValue(a,b,c,d)          getEcatSlaveOdObjectEntryMinValueMS(0,a,b,c,d)
#define setEcatSlaveOdObjectEntryMaxValue(a,b,c,d)          setEcatSlaveOdObjectEntryMaxValueMS(0,a,b,c,d)
#define getEcatSlaveOdObjectEntryMaxValue(a,b,c,d)          getEcatSlaveOdObjectEntryMaxValueMS(0,a,b,c,d)
#define setEcatSlaveSdoCallbackFunction(a,b,c)              setEcatSlaveSdoCallbackFunctionMS(0,a,b,c)
#define removeEcatSlaveSdoCallbackFunction(a,b)             removeEcatSlaveSdoCallbackFunctionMS(0,a,b)
#define addEcatSlaveSdoEventsFilter(a,b)                    addEcatSlaveSdoEventsFilterMS(0,a,b)
#define removeEcatSlaveSdoEventsFilter(a,b)                 removeEcatSlaveSdoEventsFilterMS(0,a,b)
#define getEcatSlaveOdEntryPIDataPtr(a,b,c,d)               getEcatSlaveOdEntryPIDataPtrMS(0,a,b,c,d)
#define getEcatSlaveOdObjectEntriesNumber(a,b)              getEcatSlaveOdObjectEntriesNumberMS(0,a,b)
#define getEcatSlaveOdObjectEntriesList(a,b,c)              getEcatSlaveOdObjectEntriesListMS(0,a,b,c)

/****************************************************************************/
/* OD API: Mapping and assignment
 ****************************************************************************/
#define enableEcatSlavePDOMappingAndAssignment(a)           enableEcatSlavePDOMappingAndAssignmentMS(0,a)
#define getEcatSlaveSMProcessImagePtr(a,b,c)                getEcatSlaveSMProcessImagePtrMS(0,a,b,c)
#define LockEcatSlavePI()                                   LockEcatSlavePIMS(0)
#define ReleaseEcatSlavePI()                                ReleaseEcatSlavePIMS(0)
#define buildEcatSlaveProcessImage(a)                       buildEcatSlaveProcessImageMS(0,a)
#define cleanEcatSlaveProcessImage(a)                       cleanEcatSlaveProcessImageMS(0,a)
#define getEcatSlaveOdPDOMapPtr(a,b,c)                      getEcatSlaveOdPDOMapPtrMS(0,a,b,c)
#define getEcatSlaveOdPDOMapCopy(a,b,c)                     getEcatSlaveOdPDOMapCopyMS(0,a,b,c)

/****************************************************************************/
/* ecat_slave_logger
*****************************************************************************/
//#define addEcatLoggerMsg(a,b, ...)
//#define getEcatLoggerMsgNonRt(a,b,c)

#define loadEcatSlaveOdFromFile(a)                          loadEcatSlaveOdFromFileMS(0,a)
#define getEcatSlaveEventData(a,b,c)                        getEcatSlaveEventDataMS(0,a,b,c)


/***************************************************************************/
/* Master to master communication
*****************************************************************************/

#define initEcatSlaveM2M(a,b,c,d,e)                         initEcatSlaveM2MMS(0,a,b,c,d,e)
#define addEcatSlaveM2MObjectDictionaryObjects()            addEcatSlaveM2MObjectDictionaryObjectsMS(0)
#define releaseEcatSlaveM2M()                               releaseEcatSlaveM2MMS(0)
#define addEcatSlaveM2MVariable(a,b,c,d,e,f,g,h)            addEcatSlaveM2MVariableMS(0,a,b,c,d,e,f,g,h)
#define doEcatSlaveM2MInputsPDOMapping()                    doEcatSlaveM2MInputsPDOMappingMS(0)
#define doEcatSlaveM2MOutputsPDOMapping()                   doEcatSlaveM2MOutputsPDOMappingMS(0)

/***************************************************************************/
/* Mailbox VoE
*****************************************************************************/
#define setEcatSlaveMailboxVoECallBackHandler(a)            setEcatSlaveMailboxVoECallBackHandlerMS(0,a)
#define addEcatSlaveMailBoxVoEWriteData(a,b,c,d,e,f,g)      addEcatSlaveMailBoxVoEWriteDataMS(0,a,b,c,d,e,f,g)

#ifdef __cplusplus
}
#endif


#endif /* ECATSLAVE_TO_MS_API_H */
