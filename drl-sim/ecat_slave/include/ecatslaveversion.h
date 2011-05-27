/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2007
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/

/*! @file ecatslaveversion.h
    @brief EcatSlave version constants and strings definitions.
*/

/****************************************************************************/

#ifndef ECATSLAVEVERSION_H
#define ECATSLAVEVERSION_H

#include "ecatutils.h"

#define ECATSLAVE_PRODUCTNAME_STRING        "EtherCAT Slave"

#define ECATSLAVE_VERSION_MAJOR             2
#define ECATSLAVE_VERSION_MINOR             3
#define ECATSLAVE_VERSION_SUBLEVEL          6
#define ECATSLAVE_VERSION_PRIVATE_NUMBER    6
#define ECATSLAVE_VERSION_EXTRA

#define ECATSLAVE_VERSION_CODE                                  \
            (((ECATSLAVE_VERSION_MAJOR)   << 24) +              \
            ((ECATSLAVE_VERSION_MINOR)    << 16) +              \
            ((ECATSLAVE_VERSION_SUBLEVEL) << 8 ) +              \
            ECATSLAVE_VERSION_PRIVATE_NUMBER)

#define ECATSLAVE_VERSION_BUILDDATE_STRING  __DATE__
#define ECATSLAVE_VERSION_BUILDTIME_STRING  __TIME__


#define ECATSLAVE_VERSION_STRING                                \
            MAKE_STRING(ECATSLAVE_VERSION_MAJOR) "."            \
            MAKE_STRING(ECATSLAVE_VERSION_MINOR) "."            \
            MAKE_STRING(ECATSLAVE_VERSION_SUBLEVEL) "."         \
            MAKE_STRING(ECATSLAVE_VERSION_PRIVATE_NUMBER)

#define ECATSLAVE_VERSION_FULL_STRING                           \
            ECATSLAVE_VERSION_STRING                            \
            MAKE_STRING(ECATSLAVE_VERSION_EXTRA)", built on "   \
            ECATSLAVE_VERSION_BUILDDATE_STRING", "              \
            ECATSLAVE_VERSION_BUILDTIME_STRING

#endif /* ECATSLAVEVERSION_H */
