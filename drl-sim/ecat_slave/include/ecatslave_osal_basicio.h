/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file
    @brief This file contains definitions for basic I/O operations.

    A developer needs this header file only if he needs to port
    the EtherCAT Slave Stack sources to a specific platform.
    In that case the developer is responsible for providing implementations
    for all symbols (i.e. functions) described in this file.
    Symbols described here are a part of hardware abstraction layer.

    @anchor EscAddr
    @section EscAddrDescription The EscAddr parameter description
    The parameter <b>EscAddr</b> used in descriptions in this file
    specifies location in ESC memory.
    In a simplest case of implementations of these functions an offset in ESC
    memory may be passed as EscAddr parameter.
    But in many cases implementations of the described functions
    are simpler (and more universal) if EscAddr represents a sum of
    a base address and an offset in ESC memory.

    Let's consider an example when ESC memory is mapped into address space
    of an operating system. In this case in order to access the ESC memory
    it is needed to know a start address of system's memory region
    where ESC memory is mapped to, i.e. <em>base address</em>.
    Therefore in order to access for example register 0x0220:0x0223 of ESC
    the EscAddr must be the following:

        EscAddr = [base address] + 0x0220

    If there are several similar EtherCAT slave boards in a system
    and ESC memory of each board is mapped in its own region of system memory
    (i.e. has its own ioBase address) then it is enough to change only ioBase
    value in order to access memory of other ESC.

    @note For example for PC104 EtherCAT Slave board under Linux the
    ESC memory is simply mapped to application's memory by use
    of @c request_mem_region() and @c ioremap_nocache() functions,
    and then it is possible to read/write memory as usual application memory,
    accessing it by pointer returned by @c ioremap_nocache() functions.

    As a conclusion the @e EscAddr value passed to the functions described
    in this file must be interpreted so:

        EscAddr = ioBase + [ESC RAM offset]

*/
/****************************************************************************/

#ifndef ECATSLAVE_OSAL_BASICIO_H
#define ECATSLAVE_OSAL_BASICIO_H

#include "ecatbasictypes.h"

/*! @addtogroup ecatslave_osal_basicio
    @{ */

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------------*/

/*! @brief This function (or macro, depending on implementation) should read
    a byte (8 bit value) from an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function reads 1 byte from ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @return This function returns an 8-bit value (i.e. value of type #ECAT_BYTE).
    @note Previous name of this function: @anchor ESCREG_READ_BYTE */
extern UINT8  EcatSlave_EscMemReadByte(ECAT_IOMEM EscAddr);


/*! @brief This function (or macro, depending on implementation) should read
    a word (16 bit value) from an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function reads 2 bytes(i.e. a word) from ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @return This function returns a 16-bit value (i.e. value of type #ECAT_WORD).
    @note Previous name of this function: @anchor ESCREG_READ_WORD */
extern UINT16 EcatSlave_EscMemReadWord(ECAT_IOMEM EscAddr);


/*! @brief This function (or macro, depending on implementation) should read
    a double word (32 bit value) from an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function reads 4 bytes(i.e. a double word) from ESC.

    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @return This function returns a 32-bit value (i.e. value of type #ECAT_DWORD).
    @note Previous name of this function: @anchor ESCREG_READ_DWORD */
extern UINT32 EcatSlave_EscMemReadDword(ECAT_IOMEM EscAddr);


/*! @brief This function (or macro, depending on implementation) should read
    a quad word (64 bit value) from an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function reads 8 bytes(i.e. a quad word) from ESC.

    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @return This function returns a 64-bit value (i.e. value of type #ECAT_UINT64).
    @note Previous name of this function: @anchor ESCREG_READ_QWORD */
extern UINT64 EcatSlave_EscMemReadQword(ECAT_IOMEM EscAddr);

/*--------------------------------------------------------------------------*/

/*! @brief This function (or macro, depending on implementation) should write
    a byte (8 bit value) to an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function writes 1 byte to ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param Val Value that is to be written to ESC RAM.
    @note Previous name of this function: @anchor ESCREG_WRITE_BYTE */
extern void EcatSlave_EscMemWriteByte(ECAT_IOMEM EscAddr, UINT8 Val);


/*! @brief This function (or macro, depending on implementation) should write
    a word (16 bit value) to an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function writes 2 bytes(i.e. a word) to ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param Val Value that is to be written to ESC RAM.
    @note Previous name of this function: @anchor ESCREG_WRITE_WORD */
extern void EcatSlave_EscMemWriteWord(ECAT_IOMEM EscAddr, UINT16 Val);


/*! @brief This function (or macro, depending on implementation) should write
    a double word (32 bit value) to an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function writes 4 bytes(i.e. a double word) to ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param Val Value that is to be written to ESC RAM.
    @note Previous name of this function: @anchor ESCREG_WRITE_DWORD */
extern void EcatSlave_EscMemWriteDword(ECAT_IOMEM EscAddr, UINT32 Val);


/*! @brief This function (or macro, depending on implementation) should write
    a quad word (64 bit value) to an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function writes 8 bytes(i.e. a quad word) to ESC.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param Val Value that is to be written to ESC RAM.
    @note Previous name of this function: @anchor ESCREG_WRITE_QWORD */
extern void EcatSlave_EscMemWriteQword(ECAT_IOMEM EscAddr, UINT64 Val);

/*--------------------------------------------------------------------------*/

/*! @brief This function (or macro, depending on implementation) should read
    DataLen bytes from an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function reads a specified amount of data from ESC memory.
    @param DstBufAddr Pointer to destination buffer where data obtained
    from ESC will be put.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param DataLen Number of bytes to read from ESC memory.
    @note Previous name of this function: @anchor ESCREG_MEMCPY_FROM */
extern void EcatSlave_EscMemRead(UINT8* DstBufAddr, ECAT_IOMEM EscAddr, UINT16 DataLen);

/*! @brief This function (or macro, depending on implementation) should write
    DataLen bytes to an EtherCAT slave controller memory.
    Implementation of this function is platform dependent.

    The function represents a function
    that must be implemented by a developer.
    This function writes a specified amount of data to ESC memory.
    @param SrcBufAddr Pointer to source data buffer which data will be
    put to ESC memory.
    @param EscAddr Address that specifies beginning of data in ESC.
    See @ref EscAddrDescription for more information.
    @param DataLen Number of bytes to write to ESC memory.
    @note Previous name of this function: @anchor ESCREG_MEMCPY_TO */
extern void EcatSlave_EscMemWrite(ECAT_IOMEM EscAddr, UINT8* SrcBufAddr, UINT16 DataLen);


#ifdef __cplusplus
}
#endif

/*! @} marks end of group */

#endif /* ECATSLAVE_OSAL_BASICIO_H */
