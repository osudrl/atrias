/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2007
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatutils.h
    @brief Definitions of general-purpose platform-independent
           common useful macros, constants.  */
/****************************************************************************/

#ifndef ECATUTILS_H
#define ECATUTILS_H

#define MAKE_STRING(s)      ECATUTILS_STR(s)
#define ECATUTILS_STR(s)    #s

/*! @brief Returns the smaller of two values. */
#ifndef ECAT_MIN
#define ECAT_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/*! @brief Returns the larger of two values. */
#ifndef ECAT_MAX
#define ECAT_MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/*! @brief Returns size of a member of a structure measured in bytes.
    This macro is not valid for bit fields.
    @param structType Type of a structure. Example: struct Struct1.
    @param MemberName Name of a field in a structure. */
#define ECAT_SIZEOF_STRUCTFIELD(structType, MemberName) \
        sizeof(((structType*)0)->MemberName)


#endif /* ECATUTILS_H */
