/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file
    @brief System-dependent functions (abstraction layer between
    Slave Stack's FoE and user system).
    These functions must be implemented by user. */
/****************************************************************************/

#ifndef ECATSLAVE_FOE_PAL
#define ECATSLAVE_FOE_PAL

#include "ecatslaveconfig.h"
#include "ecatslavetypes.h"

/*! @defgroup ecat_slave_foe_osal FoE system-dependent abstraction layer to FS
    @ingroup ecat_slave_mailbox_foe
    @brief System-dependent implementation (abstraction layer between
    Slave Stack's FoE and file-system).
    These functions must be implemented by user.
    @see ecat_slave_mailbox_foe
    @see ecatslave_foe_notes
*/

/*! @addtogroup ecat_slave_foe_osal
    @{
*/

#define ECATFOE_SEEK_SET    0   /*!< @brief Similar to SEEK_SET (see stdio.h). Do not modify, this value is used inside slave stack. */
#define ECATFOE_SEEK_CUR    1   /*!< @brief Similar to SEEK_CUR (see stdio.h). Do not modify, this value is used inside slave stack. */
#define ECATFOE_SEEK_END    2   /*!< @brief Similar to SEEK_END (see stdio.h). Do not modify, this value is used inside slave stack. */

/*! @defgroup ecatslave_foe_notes FoE implementation notes

    Actually a file handle may be a pointer to system-dependent structure
    that contains all information needed to work with a file.

    Simple implementation may look like this:
    @code
    #include <stdio.h>

    void* fopenEcatSlaveFoe(
        const ECAT_CHAR* pFileName,
        const ECAT_CHAR* pszMode)
    {
        if (pFileName && pszMode)
        {
            return (void*)fopen(pFileName, pszMode);
        }
        return NULL;
    }

    <...>
    @endcode

    Memory-based file implementation may look like this:
    @code
    <...>
    struct stMyFile
    {
        char            FileName[512];
        int             FileSize;
        int             FileMaxSize;
        int             FilePos;
        unsigned char   FileContents[1024];
    };
    struct stMyFile File1;

    void* fopenEcatSlaveFoe(
        const ECAT_CHAR* pFileName,
        const ECAT_CHAR* pszMode)
    {
        if (pFileName && pszMode)
        {
            if (strcmp(File1.FileName, pFileName) == 0)
                return (void*)&File1;
        }
        return NULL;
    }
    <...>
    @endcode
*/

/*! @brief Open a file.
    @param pFileName File name.
    @param pszMode Only the following modes are supported: "rb", "w+b".
    @return A file handle on success, NULL otherwise.
    @see ecatslave_foe_notes */
void* fopenEcatSlaveFoe(
    const ECAT_CHAR* pFileName,
    const ECAT_CHAR* pszMode);

/*! @brief Reposition a stream.
    @param pFile File handle.
    @param offset Offset in a file stream.
    @param origin One of #ECATFOE_SEEK_SET, #ECATFOE_SEEK_CUR, #ECATFOE_SEEK_END.
    @return Similar to return value of fseek() (C89).
    @see ecatslave_foe_notes */
INT16 fseekEcatSlaveFoe(
    void*       pFile,
    UINT32      offset,
    INT16       origin);

/*! @brief Reposition a stream.
    @param pFile File handle.
    @return Similar to return value of ftell() (C89).
    @see ecatslave_foe_notes */
UINT32 ftellEcatSlaveFoe(
    void*       pFile);

/*! @brief Read data from a file.
    @param pBuf Pointer to a buffer.
    @param itemSize Item size in bytes.
    @param count Count of items to read from file.
    @param pFile File handle.
    @return Similar to return value of fread() (C89).
    @see ecatslave_foe_notes */
UINT32 freadEcatSlaveFoe(
    void*       pBuf,
    UINT32      itemSize,
    UINT32      count,
    void*       pFile);

/*! @brief Write data to a file.
    @param pBuf Pointer to a buffer.
    @param itemSize Item size in bytes.
    @param count Count of items to write to file.
    @param pFile File handle.
    @return Similar to return value of fwrite() (C89).
    @see ecatslave_foe_notes */
UINT32 fwriteEcatSlaveFoe(
    const void* pBuf,
    UINT32      itemSize,
    UINT32      count,
    void*       pFile);

/*! @brief Close a file.
    @param pFile A handle to a file to be closed.
    @return Similar to return value of fclose() (C89).
    @see ecatslave_foe_notes */
INT16 fcloseEcatSlaveFoe(
    void*       pFile);

/*! @} (marks end of group) */

#endif /* ECATSLAVE_FOE_PAL */
