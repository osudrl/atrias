/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2008
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

******************************************************************************
    Declaration of simple API to load Object dictionary structure
    into ecat slave.
*****************************************************************************/

#ifndef ECATSLAVEODLOADER_SIMPLE_H
#define ECATSLAVEODLOADER_SIMPLE_H

#include "ecatslaveapi.h"

#ifdef __cplusplus
extern "C" {
#endif
/*! @addtogroup ecat_slave_sdo
    @{
*/
/*! @brief Builds CANopen object's dictionary structure using the information
           from the OD configuration file (*.xml).
           If there is no object in the file (e.g. the file is empty)
           the function returns ECAT_S_INPUT_EMPTY.
    @param BoardNumber Ecat Slave board number (starts from 0).
    @param  pFilePath Name of the object dictionary configuration file
    @see getEcatSlaveResultCodeDescription
*/
ECAT_RESULT loadEcatSlaveOdFromFileMS(IN UINT8 BoardNumber, IN const ECAT_CHAR* pFilePath);


///* This function is not implemented !!!*/
//ECAT_RESULT loadEcatSlaveOdFromString(
//    IN const ECAT_CHAR* pXmlSubStr,
//    IN ECAT_DWORD       XmlSubStrLen,
//    IN ECAT_BOOL        IsLastSubStr);

/*@}*/


#ifdef __cplusplus
}
#endif


#endif /* ECATSLAVEODLOADER_SIMPLE_H */
