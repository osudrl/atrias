/********************************************************************
        EtherCAT Master

    Copyright 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*********************************************************************

    EtherCAT Common Declarations

********************************************************************/
/*! @file ecatcmn.h
    @brief EtherCAT Common Declarations
*/
#ifndef _ECATCMN_H_
#define _ECATCMN_H_

#ifdef _DEBUG
#define  _CRTDBG_MAP_ALLOC
#endif /* _DEBUG */

#include "ecatslavetypes.h"

#ifndef countof
#define countof(array) (sizeof(array)/sizeof(array[0]))
#endif

#ifndef ECAT_HIBYTE
#define ECAT_HIBYTE(w) ((ECAT_BYTE)((ECAT_DWORD)(w) >> 8))
#endif

#ifndef ECAT_LOBYTE
#define ECAT_LOBYTE(w) ((ECAT_BYTE)((ECAT_DWORD)(w) & 0xFF))
#endif

#ifndef ECAT_LOWORD
#define ECAT_LOWORD(w) ((ECAT_WORD)((ECAT_DWORD)(w) & 0xFFFF))
#endif

#ifndef ECAT_HIWORD
#define ECAT_HIWORD(w) ((ECAT_WORD)((ECAT_DWORD)(w) >> 16))
#endif

#ifndef ECAT_LODWORD
#define ECAT_LODWORD(w) ((ECAT_DWORD)((ECAT_QWORD)(w) & 0xFFFFFFFF))
#endif

#ifndef ECAT_HIDWORD
#define ECAT_HIDWORD(w) ((ECAT_DWORD)((ECAT_QWORD)(w) >> 32))
#endif


#ifndef ECAT_SWAPWORD
#define ECAT_SWAPWORD(w) (ECAT_HIBYTE((w)) | (ECAT_LOBYTE((w)) << 8))
#endif

#ifndef ECAT_MAKEWORD
#define ECAT_MAKEWORD(hi, lo) (((ECAT_BYTE)(hi) << 8) | ((ECAT_BYTE)(lo)))
#endif

#ifndef ECAT_MAKEDWORD
#define ECAT_MAKEDWORD(hi, lo) (((ECAT_WORD)(hi) << 16) | ((ECAT_WORD)(lo)))
#endif

#ifndef ECAT_MAKEQWORD
#define ECAT_MAKEQWORD(hi, lo) (((ECAT_UINT64)(hi) << 32) | ((ECAT_DWORD)(lo)))
#endif


/** @brief Returns minimal number of octects needed to store
    a sequence of bits which length is BitLength. */
#ifndef ECAT_BITLEN_TO_BYTELEN
#define ECAT_BITLEN_TO_BYTELEN(BitLength)   ( ( (BitLength)+7 ) >> 3 )
#endif

/** @brief Returns number of bits occupied by ByteLength octets. */
#ifndef ECAT_BYTELEN_TO_BITLEN
#define ECAT_BYTELEN_TO_BITLEN(ByteLength)  ( (ByteLength) * 8 )
#endif


#define ECAT_GET_LE_BYTE_BITS(pData, Offs, Cnt)                             \
            ((((ECAT_PBYTE)(pData))[(Offs) >> 3] >> ((Offs) & 7)) & ((1 << (Cnt)) - 1))

#define ECAT_SET_LE_BYTE_BITS(pData, Offs, Cnt, Val) {                        \
            ((ECAT_PBYTE)(pData))[(Offs) >> 3] &= ~(((1 << (Cnt)) - 1) << ((Offs) & 7));\
            ((ECAT_PBYTE)(pData))[(Offs) >> 3] |= ((Val) & ((1 << (Cnt)) - 1)) << ((Offs) & 7); }

#define ECAT_GET_LE_BIT(pData, Bit)      ECAT_GET_LE_BYTE_BITS(pData, Bit, 1)
#define ECAT_SET_LE_BIT(pData, Bit, Val) ECAT_SET_LE_BYTE_BITS(pData, Bit, 1, Val)

#define ECAT_SET_LE_WORD(pData, Val) {                                      \
            ((ECAT_PBYTE)(pData))[0] = ECAT_LOBYTE(Val);                    \
            ((ECAT_PBYTE)(pData))[1] = ECAT_HIBYTE(Val); }
#define ECAT_GET_LE_WORD(pData)                                             \
            ECAT_MAKEWORD(((ECAT_PBYTE)(pData))[1], ((ECAT_PBYTE)(pData))[0])

#define ECAT_SET_LE_DWORD(pData, Val) {                                     \
            ((ECAT_PBYTE)(pData))[0] = ECAT_LOBYTE(ECAT_LOWORD(Val));       \
            ((ECAT_PBYTE)(pData))[1] = ECAT_HIBYTE(ECAT_LOWORD(Val));       \
            ((ECAT_PBYTE)(pData))[2] = ECAT_LOBYTE(ECAT_HIWORD(Val));       \
            ((ECAT_PBYTE)(pData))[3] = ECAT_HIBYTE(ECAT_HIWORD(Val)); }

#define ECAT_GET_LE_DWORD(pData)                                            \
                (((ECAT_DWORD)((ECAT_PBYTE)(pData))[3] << 24) |             \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[2] << 16) |              \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[1] << 8)  |              \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[0]))

#define ECAT_GET_BE_DWORD(pData)                                            \
                (((ECAT_DWORD)((ECAT_PBYTE)(pData))[0] << 24) |             \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[1] << 16) |              \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[2] << 8)  |              \
                ((ECAT_DWORD)((ECAT_PBYTE)(pData))[3]))

#define ECAT_SET_BE_DWORD(pData, Val) {                                     \
                ((ECAT_PBYTE)(pData))[3] = (ECAT_BYTE)(Val);                \
                ((ECAT_PBYTE)(pData))[2] = (ECAT_BYTE)((Val) >> 8);         \
                ((ECAT_PBYTE)(pData))[1] = (ECAT_BYTE)((Val) >> 16);        \
                ((ECAT_PBYTE)(pData))[0] = (ECAT_BYTE)((Val) >> 24); }

#define ECAT_SWAP_WORD(pData)                                               \
                { ECAT_BYTE by;                                             \
                ECAT_PBYTE pby = (ECAT_PBYTE)(pData);                       \
                by = pby[0]; pby[0] = pby[1]; pby[1] = by; }

#define ECAT_SWAP_DWORD(pData)                                              \
                { ECAT_BYTE by;                                             \
                ECAT_PBYTE pby = (ECAT_PBYTE)(pData);                       \
                by = pby[0]; pby[0] = pby[3]; pby[3] = by;                  \
                by = pby[1]; pby[1] = pby[2]; pby[2] = by; }

#define ECAT_SWAP_QWORD(pData)                                              \
                { ECAT_BYTE by;                                             \
                ECAT_PBYTE pby = (ECAT_PBYTE)(pData);                       \
                by = pby[0]; pby[0] = pby[7]; pby[7] = by;                  \
                by = pby[1]; pby[1] = pby[6]; pby[6] = by;                  \
                by = pby[2]; pby[2] = pby[5]; pby[5] = by;                  \
                by = pby[3]; pby[3] = pby[4]; pby[4] = by; }

/*! @brief Copy ECAT_MACADDR_SIZE bytes from location pSrc to location pDst. */
#define ECAT_MACADDR_RAW_COPY(pDst, pSrc)                   \
            ((ECAT_PBYTE)pDst)[0] = ((ECAT_PBYTE)pSrc)[0];  \
            ((ECAT_PBYTE)pDst)[1] = ((ECAT_PBYTE)pSrc)[1];  \
            ((ECAT_PBYTE)pDst)[2] = ((ECAT_PBYTE)pSrc)[2];  \
            ((ECAT_PBYTE)pDst)[3] = ((ECAT_PBYTE)pSrc)[3];  \
            ((ECAT_PBYTE)pDst)[4] = ((ECAT_PBYTE)pSrc)[4];  \
            ((ECAT_PBYTE)pDst)[5] = ((ECAT_PBYTE)pSrc)[5];

/*! @brief Compare MAC addresses located at p1 and p2 using memcpy().
        Result of this expression is a comparison of
        0 and of a value returned by memcpy(). */
#define ECAT_MACADDR_EQUAL(p1, p2)                         \
            (0 == memcmp((ECAT_PBYTE)(p1), (ECAT_PBYTE)(p2), ECAT_MACADDR_SIZE))

static const ECAT_MACADDR ECAT_MAC_BROADCAST = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
static const ECAT_MACADDR ECAT_MAC_NULL = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

#define ECAT_HDR_ETHERNET_SIZE      14  /*!<  Ethernet header size(bytes) */

/*! @brief Structure that contains array of bytes that represent Ethernet header. */
typedef struct _ECAT_HDR_ETHERNET
{
    ECAT_BYTE RawData[ECAT_HDR_ETHERNET_SIZE];
} ECAT_HDR_ETHERNET;

/*! @brief Set destination MAC Address of Ethernet header.
    @param pHdr the pointer to Ethernet header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERNET)
    @param pVal the pointer to MAC address
                (i.e. pointer to a variable of type #ECAT_MACADDR) */
#define ECAT_HDR_ETHERNET_SET_DST_MACADDR(pHdr, pVal)       \
            ECAT_MACADDR_RAW_COPY(pHdr, pVal)

/*! @brief Set source MAC Address of Ethernet header.
    @param pHdr the pointer to Ethernet header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERNET)
    @param pVal the pointer to MAC address
                (i.e. pointer to a variable of type #ECAT_MACADDR) */
#define ECAT_HDR_ETHERNET_SET_SRC_MACADDR(pHdr, pVal)       \
            ECAT_MACADDR_RAW_COPY((((ECAT_PBYTE)pHdr) + 6), pVal)

/*! @brief Set Ethernet protocol type of Ethernet header (big endian for TCP/IP network).
    @param pHdr the pointer to Ethernet header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERNET)
    @param Val the value of Ethernet protocol type. */
#define ECAT_HDR_ETHERNET_SET_ETHERTYPE(pHdr, Val)          \
            ((ECAT_PBYTE)pHdr)[12] = ECAT_HIBYTE(Val);      \
            ((ECAT_PBYTE)pHdr)[13] = ECAT_LOBYTE(Val);

/*! @brief Retrieve Ethernet protocol type of Ethernet header.
    @param pHdr the pointer to Ethernet header to read from
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERNET)
    @return a value of type #ECAT_WORD that represents Ethernet protocol type. */
#define ECAT_HDR_ETHERNET_GET_ETHERTYPE(pHdr)               \
            ECAT_MAKEWORD(((ECAT_PBYTE)pHdr)[12], ((ECAT_PBYTE)pHdr)[13])

/* Some of Ethernet protocol types */
#define ETHER_TYPE_IP       0x0800  /*!<  Internet Protocol packet   */
#define	ETHER_TYPE_ARP1     0x0806
#define	ETHER_TYPE_ARP2     0x0807
#define	ETHER_TYPE_RARP     0x8035  /*!< Reverse Addr Res packet     */
#define	ETHER_TYPE_VLAN     0x8100  /*!< 802.1Q VLAN Extended Header */
#define	ETHER_TYPE_SNMP     0x814C
#define ETHER_TYPE_ECAT     0x88A4  /*!< EtherCAT packet             */

#define ETHERNET_FRAME_HEADER_SIZE  14
#define ETHERNET_MIN_DATA_SIZE      46
#define ETHERNET_MAX_DATA_SIZE      1500
#define ETHERNET_MIN_FRAME_SIZE     (ETHERNET_FRAME_HEADER_SIZE + ETHERNET_MIN_DATA_SIZE)   /* exclude Ethernet CRC */
#define ETHERNET_MAX_FRAME_SIZE     (ETHERNET_FRAME_HEADER_SIZE + ETHERNET_MAX_DATA_SIZE)   /* exclude Ethernet CRC */

#define ETHERCAT_FRAME_HDR_SIZE     2   /*!< EtherCAT frame header size(bytes)    */
#define ETHERCAT_TELEGRAM_HDR_SIZE  10  /*!< EtherCAT telegram header size(bytes) */
#define ETHERCAT_TELEGRAM_WKC_SIZE  2   /*!< EtherCAT telegram work counter size(bytes) */

#define ETHERCAT_MIN_TELEGRAM_SIZE  (ETHERCAT_TELEGRAM_HDR_SIZE + ETHERCAT_TELEGRAM_WKC_SIZE)
#define ETHERCAT_MAX_TELEGRAM_SIZE  (ETHERNET_MAX_DATA_SIZE - ETHERCAT_FRAME_HDR_SIZE)

#define ETHERCAT_MAX_DATA_SIZE      (ETHERCAT_MAX_TELEGRAM_SIZE - ETHERCAT_MIN_TELEGRAM_SIZE)

#define ETHERCAT_MIN_FRAME_SIZE     (ETHERNET_FRAME_HEADER_SIZE + ETHERCAT_FRAME_HDR_SIZE + ETHERCAT_MIN_TELEGRAM_SIZE)
#define ETHERCAT_MAX_FRAME_SIZE     ETHERNET_MAX_FRAME_SIZE


#define ECAT_HDR_IP_SIZE            20  /*!< IP header size (bytes) */

/*! @brief Structure that contains array of bytes that represent IP header. */
typedef struct _ECAT_HDR_IP
{
    ECAT_BYTE RawData[ECAT_HDR_IP_SIZE];
} ECAT_HDR_IP;


#define ECAT_HDR_UDP_SIZE           8  /*!< UDP header size(bytes) */

/*! @brief Structure that contains array of bytes that represent UDP header.

<UL>
<LI> bits  0..15: Source Port
<LI> bits 16..31: Destination Port: 0x88A4 (EtherCAT)
<LI> bits 32..47: Following byte length (excl. CRCs)
<LI> bits 48..63: CRC of the UDP-Header
</UL>
*/
typedef struct _ECAT_HDR_UDP
{
    ECAT_BYTE RawData[ECAT_HDR_UDP_SIZE];
} ECAT_HDR_UDP;


#define ECAT_HDR_ETHERCAT_SIZE      2  /*!< EtherCAT frame header size(bytes) */

#pragma pack(1)

/*! @brief Structure that contains array of bytes that represent EtherCAT frame header.
        Type 0x01 simply indicates that EtherCAT frame contains EtherCAT commands.
        ID of EtherCAT commands is inside EtherCAT telegrams.
    <UL>
    <LI> bits  0..10:  Length of following data (excl. CRC), in bytes
    <LI> bit      11:  Reserved: 0x00
    <LI> bits 12..15:  Protocol type = EtherCAT command (if equal 0x01)
                <UL><LI> Possible types:
                    <UL><LI>  0x00: Reserved
                        <LI>  0x01: EtherCAT command
                        <LI>  0x02: ADS (Automation Device Specification)
                        <LI>  0x03: Raw I/O (Direct I/O Data / BK9000)
                        <LI>  0x04: Network Variables (Publisher / Subscriber)
                        <LI>  0x05-0x0F: Reserved for future use
                    </UL>
                </UL>
    </UL>
*/
typedef struct _ECAT_HDR_ETHERCAT
{
    ECAT_BYTE RawData[ECAT_HDR_ETHERCAT_SIZE];
} ECAT_HDR_ETHERCAT;


/*! @brief Set length of data that follows EtherCAT frame header(excl. CRC).
    @param pHdr the pointer to EtherCAT frame header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERCAT)
    @param Val the length to be set.
            Val is interpreted as a value of type #ECAT_DWORD. */
#define ECAT_HDR_ETHERCAT_SET_LENGTH(pHdr, Val)                             \
            ((ECAT_PBYTE)pHdr)[0] = ECAT_LOBYTE(Val);                       \
            ((ECAT_PBYTE)pHdr)[1] &= ~0x07;                                 \
            ((ECAT_PBYTE)pHdr)[1] |= ECAT_HIBYTE(Val) & 0x07;

/*! @brief Get length of data that follows EtherCAT frame header(excl. CRC).
    @param pHdr the pointer to EtherCAT frame header to read from
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERCAT) */
#define ECAT_HDR_ETHERCAT_GET_LENGTH(pHdr)                                  \
            ECAT_MAKEWORD(((ECAT_PBYTE)pHdr)[1] & 0x07, ((ECAT_PBYTE)pHdr)[0])

/*! @brief Set value of 11-th bit (reserved bit) of EtherCAT frame header.
    @param pHdr the pointer to EtherCAT frame header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERCAT)
    @param Val Bit value to be set (0 or 1). */
#define ECAT_HDR_ETHERCAT_SET_RESERVED(pHdr, Val)                           \
            ECAT_SET_LE_BIT(pHdr, 3, Val)

/*! @brief Set "Protocol type" field  of EtherCAT frame header.
    @param pHdr the pointer to EtherCAT frame header to write to
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERCAT)
    @param Val  type of EtherCAT frame. */
#define ECAT_HDR_ETHERCAT_SET_TYPE(pHdr, Val)                               \
            ECAT_SET_LE_BYTE_BITS(pHdr, 12, 4, Val)

/*! @brief Get "Protocol type" field  of EtherCAT frame header.
    @param pHdr the pointer to EtherCAT frame header to read from
                (i.e. pointer to a variable of type #ECAT_HDR_ETHERCAT)
    @return type of EtherCAT frame. */
#define ECAT_HDR_ETHERCAT_GET_TYPE(pHdr)                                    \
            ECAT_GET_LE_BYTE_BITS(pHdr, 12, 4)

#pragma pack()

/*! @brief Protocol types for EtherCAT frame */
typedef enum _EcatProtocolType
{
    EcatProtocolTypeReserved   = 0x00, /*!< Reserved */
    EcatProtocolTypeCommand    = 0x01, /*!< EtherCAT Command  */
    EcatProtocolTypeAds        = 0x02, /*!< ADS (Automation Device Specification)      */
    EcatProtocolTypeRawIo      = 0x03, /*!< Raw I/O (Direct I/O Data / BK9000...)      */
    EcatProtocolTypeVariables  = 0x04, /*!< Network Variables (Publisher / Subscriber) */
    EcatProtocolTypeCANOPEN    = 0x05, /*!< ETHERCAT_CANOPEN_HEADER follows            */
} EcatProtocolType;


/*! @brief Length of address part of EtherCAT telegram header(in bytes). */
#define ECAT_ADDR_SIZE              4

#pragma pack(1)

/*! @brief Address part of EtherCAT telegram header (consists of two words).

    Interpretations:
    <OL>
    <LI>bits  0..15: ADP: Fixed station address or the auto increment address (position in the ring) <BR>
        bits 16..31: ADO: Physical memory address
    <LI>bits  0..31: ADR: Start address in the logical memory (for the commands LRD, LWR, LRW)
    </OL>*/
typedef struct _ECAT_ADDR
{
    ECAT_BYTE   RawData[ECAT_ADDR_SIZE];
} ECAT_ADDR;

/*! @brief Set fixed station address or the Auto Increment Address (position in the ring).
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @param Val value of the address to assign. */
#define ECAT_ADDR_SET_ADP(pAddr, Val) {                                     \
            ((ECAT_PBYTE)pAddr)[0] = ECAT_LOBYTE(Val);                      \
            ((ECAT_PBYTE)pAddr)[1] = ECAT_HIBYTE(Val); }

/*! @brief Get fixed station address or the Auto Increment Address (position in the ring).
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @return Auto Increment Address (value of type #ECAT_WORD). */
#define ECAT_ADDR_GET_ADP(pAddr)                                            \
            ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[1], ((ECAT_PBYTE)pAddr)[0])

/*! @brief Set Physical Memory Address.
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @param Val value of the Physical Memory Address (#ECAT_WORD type). */
#define ECAT_ADDR_SET_ADO(pAddr, Val)                                       \
            ((ECAT_PBYTE)pAddr)[2] = ECAT_LOBYTE(Val);                      \
            ((ECAT_PBYTE)pAddr)[3] = ECAT_HIBYTE(Val);

/*! @brief Get Physical Memory Address.
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @return Physical Memory Address (value of type #ECAT_WORD). */
#define ECAT_ADDR_GET_ADO(pAddr)                                            \
            ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[3], ((ECAT_PBYTE)pAddr)[2])

/*! @brief Set Logical Address (for the commands LRD, LWR, LRW)
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @param Val Value of the Logical Address (#ECAT_DWORD type). */
#define ECAT_ADDR_SET_ADDRESS(pAddr, Val)                                   \
            ((ECAT_PBYTE)pAddr)[0] = ECAT_LOBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[1] = ECAT_HIBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[2] = ECAT_LOBYTE(ECAT_HIWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[3] = ECAT_HIBYTE(ECAT_HIWORD(Val));

/*! @brief Get Logical Address (for the commands LRD, LWR, LRW)
    @param pAddr Address part of EtherCAT telegram header(consists of two words).
    @return Logical Address (value of type #ECAT_DWORD). */
#define ECAT_ADDR_GET_ADDRESS(pAddr)                                        \
            ECAT_MAKEDWORD(                                                 \
                ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[3],                       \
                            ((ECAT_PBYTE)pAddr)[2]),                        \
                ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[1],                       \
                            ((ECAT_PBYTE)pAddr)[0]))

#pragma pack()

/*! @brief EthetCAT Telegram header size(= EthetCAT command header size), in bytes. */
#define ECAT_HDR_COMMAND_SIZE       10

#pragma pack(1)

/*! @brief Structure that contains array of bytes that
        represent EtherCAT telegram header(=EtherCAT command header).
    <UL>
    <LI>bits  0..7:  CMD (EtherCAT command ID)
    <LI>bits  8..15: IDX (Index)
    <LI>bits 16..47: ADP+ADO / ADR / Reserved+ADO (Address part of EtherCAT command)
    <LI>bits 48..58: LEN (Length of following data of EtherCAT telegram, in bytes)
    <LI>bits 59..62: Reserved: 0x00
    <LI>bit  63      NEXT (Possible values:
                       0x00: Last EtherCAT telegram in Ethernet or UDP frame;
                       0x01: EtherCAT telegram in Ethernet or UDP frame follows.
    <LI>bits 64..79: IRQ: 0x00 (Reserved for future use)
    </UL>*/
typedef struct _ECAT_HDR_COMMAND
{
    ECAT_BYTE RawData[ECAT_HDR_COMMAND_SIZE];
} ECAT_HDR_COMMAND;

/*! @brief Set EtherCAT command ID (CMD). */
#define ECAT_HDR_COMMAND_SET_CMD(pCmdHeader, Val)                           \
            ((ECAT_PBYTE)pCmdHeader)[0] = (ECAT_BYTE)Val;
/*! @brief Get EtherCAT command ID (CMD). */
#define ECAT_HDR_COMMAND_GET_CMD(pCmdHeader)                                \
            ((ECAT_PBYTE)pCmdHeader)[0]

/*! @brief Set EtherCAT command index (IDX). */
#define ECAT_HDR_COMMAND_SET_INDEX(pCmdHeader, Val)                         \
            ((ECAT_PBYTE)pCmdHeader)[1] = (ECAT_BYTE)Val;
/*! @brief Get EtherCAT command index (IDX). */
#define ECAT_HDR_COMMAND_GET_INDEX(pCmdHeader)                              \
            (((ECAT_PBYTE)pCmdHeader)[1])

/*! @brief Get pointer to address part of EtherCAT command header. */
#define ECAT_HDR_COMMAND_ADDRESS(pCmdHeader)                                \
            ((ECAT_ADDR*)(((ECAT_PBYTE)pCmdHeader) + 2))

/*! @brief Set LEN field of EtherCAT command header. */
#define ECAT_HDR_COMMAND_SET_LENGTH(pCmdHeader, Val)                        \
            ((ECAT_PBYTE)pCmdHeader)[6] = ECAT_LOBYTE(Val);                 \
            ((ECAT_PBYTE)pCmdHeader)[7] &= ~0x07;                           \
            ((ECAT_PBYTE)pCmdHeader)[7] |= ECAT_HIBYTE(Val) & 0x07;
/*! @brief Get LEN field of EtherCAT command header. */
#define ECAT_HDR_COMMAND_GET_LENGTH(pCmdHeader)                             \
            ECAT_MAKEWORD(((ECAT_PBYTE)pCmdHeader)[7] & 0x07,               \
                        ((ECAT_PBYTE)pCmdHeader)[6])

/*! @brief Set values of reserved bits 59..62 */
#define ECAT_HDR_COMMAND_SET_RESERVED(pCmdHeader, Val)                      \
            ECAT_SET_LE_BYTE_BITS(pCmdHeader, 59, 4, Val)

/*! @brief Set field NEXT(bit number 63) of EtherCAT command header. */
#define ECAT_HDR_COMMAND_SET_NEXT(pCmdHeader, Val)                          \
            ECAT_SET_LE_BIT(pCmdHeader, 63, Val)
/*! @brief Get field NEXT(bit number 63) of EtherCAT command header. */
#define ECAT_HDR_COMMAND_GET_NEXT(pCmdHeader)                               \
            ECAT_GET_LE_BIT(pCmdHeader, 63)

/*! @brief Set field IRQ(bits 64-79) of EtherCAT command header. */
#define ECAT_HDR_COMMAND_SET_IRQ(pCmdHeader, Val)                           \
            ((ECAT_PBYTE)pCmdHeader)[8] = ECAT_LOBYTE(Val);                 \
            ((ECAT_PBYTE)pCmdHeader)[9] = ECAT_HIBYTE(Val);
/*! @brief Get field IRQ(bits 64-79) of EtherCAT command header. */
#define ECAT_HDR_COMMAND_GET_IRQ(pCmdHeader)                                \
            ECAT_MAKEWORD(((ECAT_PBYTE)pCmdHeader)[9],                      \
                        ((ECAT_PBYTE)pCmdHeader)[8])

#pragma pack()

/*! @brief Size of working conter(WKC) in EtherCAT command measured in bytes. */
#define ECAT_WORKING_COUNTER_SIZE	2


/*! @brief General Mailbox Header size measured in bytes. */
#define ECAT_HDR_MAILBOX_SIZE       6

#pragma pack(1)
/*! @brief Structure that contains array of bytes that
        represent General Mailbox Header.
<UL>
<LI> bits  0..15: Length of the Mailbox Service Data
<LI> bits 16..31: Station Address of the source, if a master is client(requester),
                  Station Address of the destination, if a slave is client(requester)
<LI> bits 32..37: Channel: 0x00 (Reserved for future use)
<LI> bits 38..39: Priority: <UL><LI>0x00: Lowest priority
                                <LI>...
                                <LI>0x03: Highest priority</UL>
<LI> bits 40..43: Type: <UL><LI>0x01: AoE (ADS over EtherCAT)
                            <LI>0x02: EoE (Ethernet over EtherCAT)
                            <LI>0x03: CoE (CANopen over EtherCAT)
                            <LI>0x04: FoE (File Service over EtherCAT)</UL>
<LI> bits 44..47: Ctr: Sequnce number for duplicate detection
</UL>
*/
typedef struct _ECAT_HDR_MAILBOX
{
    ECAT_BYTE RawData[ECAT_HDR_MAILBOX_SIZE];
} ECAT_HDR_MAILBOX;
#pragma pack()

/*! @brief Set Length of the Mailbox Service Data. */
#define ECAT_HDR_MAILBOX_SET_LENGTH(pHdr, Val)                              \
            ((ECAT_PBYTE)pHdr)[0] = ECAT_LOBYTE(Val);                       \
            ((ECAT_PBYTE)pHdr)[1] = ECAT_HIBYTE(Val);
/*! @brief Get Length of the Mailbox Service Data. */
#define ECAT_HDR_MAILBOX_GET_LENGTH(pHdr)                                   \
            ECAT_MAKEWORD(((ECAT_PBYTE)pHdr)[1], ((ECAT_PBYTE)pHdr)[0])

/*! @brief Set "Address" field of General Mailbox Header. */
#define ECAT_HDR_MAILBOX_SET_ADDRESS(pHdr, Val)                             \
            ((ECAT_PBYTE)pHdr)[2] = ECAT_LOBYTE(Val);                       \
            ((ECAT_PBYTE)pHdr)[3] = ECAT_HIBYTE(Val);
/*! @brief Get "Address" field of General Mailbox Header. */
#define ECAT_HDR_MAILBOX_GET_ADDRESS(pHdr)                                  \
            ECAT_MAKEWORD(((ECAT_PBYTE)pHdr)[3], ((ECAT_PBYTE)pHdr)[2])

/*! @brief Set "Channel" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_SET_CHANNEL(pHdr, Val)                             \
            ECAT_SET_LE_BYTE_BITS(pHdr, 32, 6, Val)
/*! @brief Get "Channel" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_GET_CHANNEL(pHdr)                                  \
            ECAT_GET_LE_BYTE_BITS(pHdr, 32, 6)

/*! @brief Set "Priority" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_SET_PRIORITY(pHdr, Val)                            \
            ECAT_SET_LE_BYTE_BITS(pHdr, 38, 2, Val)
/*! @brief Get "Priority" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_GET_PRIORITY(pHdr)                                 \
            ECAT_GET_LE_BYTE_BITS(pHdr, 38, 2)

/*! @brief Set "Type" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_SET_TYPE(pHdr, Val)                                \
            ECAT_SET_LE_BYTE_BITS(pHdr, 40, 4, Val)
/*! @brief Get "Type" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_GET_TYPE(pHdr)                                     \
            ECAT_GET_LE_BYTE_BITS(pHdr, 40, 4)

/*! @brief Set "Reserved" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_SET_CTR(pHdr, Val)                                 \
            ECAT_SET_LE_BYTE_BITS(pHdr, 44, 4, Val)

/*! @brief Get "Ctr" field of General Mailbox Header */
#define ECAT_HDR_MAILBOX_GET_CTR(pHdr)                                     \
            ECAT_GET_LE_BYTE_BITS(pHdr, 44, 4)

/*! @brief Possible types of mailbox. */
typedef enum _EcatMailboxType
{
    EcatMailboxTypeErr       = 0x00, /*!< Error Command */
    EcatMailboxTypeAoE       = 0x01, /*!< AoE (ADS over EtherCAT) */
    EcatMailboxTypeEoE       = 0x02, /*!< EoE (Ethernet over EtherCAT) */
    EcatMailboxTypeCoE       = 0x03, /*!< CoE (CANopen over EtherCAT) */
    EcatMailboxTypeFoE       = 0x04, /*!< FoE (File Service over EtherCAT) */
    EcatMailboxTypeSoE       = 0x05, /*!< SoE (Servo profile over EtherCAT) */
    EcatMailboxTypeVoE       = 0x0f, /*!< VoE (Vendor specific service over EtherCAT) */
    EcatMailboxTypeMax       = 0x10
} EcatMailboxType;


/*! @brief Error Codes for a mailbox error response. */
typedef enum tagMbxErrorCodes {
    ESC_MBXERR_SYNTAX                   = 0x01,
    ESC_MBXERR_UNSUPPORTEDPROTOCOL      = 0x02,
    ESC_MBXERR_INVALIDCHANNEL           = 0x03,
    ESC_MBXERR_SERVICENOTSUPPORTED      = 0x04,
    ESC_MBXERR_INVALIDHEADER            = 0x05,
    ESC_MBXERR_SIZETOOSHORT             = 0x06,
    ESC_MBXERR_NOMOREMEMORY             = 0x07,
    ESC_MBXERR_INVALIDSIZE              = 0x08
} tMbxErrorCodes;

#define ECAT_HDR_MAILBOX_ERR_CMD_LEN        2
#define ECAT_HDR_MAILBOX_ERR_CMD            1
#define ECAT_HDR_MAILBOX_ERR_DATA_LEN       4

/*! @brief Set Mailbox error command. */
#define ECAT_HDR_MAILBOX_ERR_SET_CMD(pHdr, Val)                              \
            ((ECAT_PBYTE)pHdr)[0] = ECAT_LOBYTE(Val);                        \
            ((ECAT_PBYTE)pHdr)[1] = ECAT_HIBYTE(Val);

/*! @brief Set Mailbox error data. */
#define ECAT_HDR_MAILBOX_ERR_SET_DATA(pHdr, Val)                             \
            ((ECAT_PBYTE)pHdr)[2] = ECAT_LOBYTE(Val);                        \
            ((ECAT_PBYTE)pHdr)[3] = ECAT_HIBYTE(Val);

/*! @brief Possible States and additional auxiliary constants.
        See also documentation about EtherCAT State Machine. */
typedef enum _EcatState
{
    EcatStateNotSet    = 0x00, /*!< State is not set                    */
    EcatStateI         = 0x01, /*!< EtherCAT State "Init"               */
    EcatStateP         = 0x02, /*!< EtherCAT State "Pre-Operational"    */
    EcatStateB         = 0x03, /*!< EtherCAT State "Bootstrap"          */
    EcatStateS         = 0x04, /*!< EtherCAT State "Safe-Operational"   */
    EcatStateO         = 0x08, /*!< EtherCAT State "Operational"        */
    /*! Mask for significant bits of a variable that represents EtherCAT State. */
    EcatStateMask = (EcatStateI | EcatStateP | EcatStateB | EcatStateS | EcatStateO),
    EcatStateErrorFlag = 0x10, /*!< Error flag                          */
} EcatState;

/*!< @brief Physical Address that is to be accessed in order to get slave state.
            See also AL Status protocol documentation.*/
#define ECAT_RADDR_SLAVE_STATE      0x0130

/*! @brief State transition constants.
        See also documentation about EtherCAT State Machine. */
typedef enum _EcatTransition
{
#define ECAT_TRANSITION_SHIFT   8
    EcatTransitionIP = ((EcatStateI << ECAT_TRANSITION_SHIFT) | EcatStateP),  /*!< Init to Pre-Operational */
    EcatTransitionIB = ((EcatStateI << ECAT_TRANSITION_SHIFT) | EcatStateB),  /*!< Init to Bootstrap */
    EcatTransitionPI = ((EcatStateP << ECAT_TRANSITION_SHIFT) | EcatStateI),  /*!< Pre-Operational to Init */
    EcatTransitionBI = ((EcatStateB << ECAT_TRANSITION_SHIFT) | EcatStateI),  /*!< Bootstrap to Init */
    EcatTransitionPS = ((EcatStateP << ECAT_TRANSITION_SHIFT) | EcatStateS),  /*!< Pre-Operational to Safe-Operational */
    EcatTransitionSI = ((EcatStateS << ECAT_TRANSITION_SHIFT) | EcatStateI),  /*!< Safe-Operational to Init */
    EcatTransitionSP = ((EcatStateS << ECAT_TRANSITION_SHIFT) | EcatStateP),  /*!< Safe-Operational to Pre-Operational */
    EcatTransitionSO = ((EcatStateS << ECAT_TRANSITION_SHIFT) | EcatStateO),  /*!< Safe-Operational to Operational */
    EcatTransitionOI = ((EcatStateO << ECAT_TRANSITION_SHIFT) | EcatStateI),  /*!< Operational to Init */
    EcatTransitionOP = ((EcatStateO << ECAT_TRANSITION_SHIFT) | EcatStateP),  /*!< Operational to Pre-Operational */
    EcatTransitionOS = ((EcatStateO << ECAT_TRANSITION_SHIFT) | EcatStateS),  /*!< Operational to Safe-Operational */

    /*Invalid requested state change*/
    EcatTransitionPB = ((EcatStateP << ECAT_TRANSITION_SHIFT) | EcatStateB),  /*!< Pre-Operational to Bootstrap */
    EcatTransitionSB = ((EcatStateS << ECAT_TRANSITION_SHIFT) | EcatStateB),  /*!< Safe-Operational to Bootstrap */
    EcatTransitionOB = ((EcatStateO << ECAT_TRANSITION_SHIFT) | EcatStateB),  /*!< Operational to Bootstrap */
    EcatTransitionIS = ((EcatStateI << ECAT_TRANSITION_SHIFT) | EcatStateS),  /*!< Init to Safe-Operational */
    EcatTransitionIO = ((EcatStateI << ECAT_TRANSITION_SHIFT) | EcatStateO),  /*!< Init to Operational */
    EcatTransitionPO = ((EcatStateP << ECAT_TRANSITION_SHIFT) | EcatStateO),  /*!< Pre-Operational to Operational */

    /*no state change*/
    EcatTransitionII = ((EcatStateI << ECAT_TRANSITION_SHIFT) | EcatStateI),  /*!< Init to Init */
    EcatTransitionPP = ((EcatStateP << ECAT_TRANSITION_SHIFT) | EcatStateP),  /*!< Pre-Operational to Pre-Operational */
    EcatTransitionSS = ((EcatStateS << ECAT_TRANSITION_SHIFT) | EcatStateS),  /*!< Safe-Operational to Safe-Operational */
    EcatTransitionOO = ((EcatStateO << ECAT_TRANSITION_SHIFT) | EcatStateO),  /*!< Operational to Operational */
} EcatTransition;


/*! @brief Form a constant of type ECAT_WORD with high-byte containing
        "from" state and low-byte containing "to" state. */
#define ECAT_MAKE_TRANSITION(from, to)  ECAT_MAKEWORD(from, to)


/*! @brief EtherCAT Command IDs.
    For more help see EtherCAT Communication Specification,
    chapter "EtherCAT telegram structure". */
typedef enum _EcatCommands
{
    EcatCommandNOP  = 0x00, /*!< No operation                            */
    EcatCommandAPRD = 0x01, /*!< Auto Increment Physical Read (APRD)     */
    EcatCommandAPWR = 0x02, /*!< Auto Increment Physical Write (APWR)    */
    EcatCommandAPRW = 0x03, /*!< Auto Auto Increment Physical Read/Write */
    EcatCommandFPRD = 0x04, /*!< Fixed Address Physical Read (FPRD)      */
    EcatCommandFPWR = 0x05, /*!< Fixed Address Physical Write (FPWR)     */
    EcatCommandFPRW = 0x06, /*!< Fixed Address Physical Read/Write       */
    EcatCommandBRD  = 0x07, /*!< Broadcast Read (BRD)                    */
    EcatCommandBWR  = 0x08, /*!< Broadcast Write (BWR)                   */
    EcatCommandBRW  = 0x09, /*!< Broadcast Read/Write                    */
    EcatCommandLRD  = 0x0A, /*!< Logical Read (LRD)                      */
    EcatCommandLWR  = 0x0B, /*!< Logical Write (LWR)                     */
    EcatCommandLRW  = 0x0C, /*!< Logical ReadWrite (LRW)                 */
    EcatCommandARMW = 0x0D  /*!< Auto Increment Physical Read Multiple Write (ARMW) */
} EcatCommands;


/*! @brief Size of a structure for one FMMU channel measured in bytes. */
#define ECAT_CHANNEL_FMMU_SIZE      16

#pragma pack(1)
/*! @brief Structure that contains array of bytes that
           represent structure for one channel of
           Fieldbus Memory Management Unit (FMMU).

    Format of the description below: BITS: BYTE OFFSET: FIELD DESCRIPTION
<UL>
<LI>bits    0..31: 0x0000: Logical Start Address
<LI>bits   32..47: 0x0004: Length
<LI>bits   48..50: 0x0006: Logical Start Bit. This parameter
                              contains the bit offset of the
                              logical start address.
<LI>bits   51..55: 0x0006: Reserved : 0x00
<LI>bits   56..58: 0x0007: Logical End Bit. This parameter
                              contains the bit offset of the
                              logical end address.
<LI>bits   59..63: 0x0007: Reserved : 0x00
<LI>bits   64..79: 0x0008: Physical Start Address
<LI>bits   80..82: 0x000A: Physical Start Bit
<LI>bits   83..87: 0x000A: Reserved : 0x00
<LI>bit        88: 0x000B: Read Enable (0x00: channel will
                              be ignored for read service
                              0x01: channel will be used
                              for read service)
<LI>bit        89: 0x000B: Write Enable (0x00: channel will
                              be ignored for write service;
                              0x01: channel will be used
                              for write service)
<LI>bits   90..95: 0x000B: Reserved : 0x00
<LI>bit        96: 0x000C: Channel Enable (0x00: channel
                              not active, 0x01: channel active)
<LI>bits  97..103: 0x000C: Reserved : 0x00
<LI>bits 104..127: 0x000D: Reserved : 0x00, 0x00, 0x00
</UL> */
typedef struct _ECAT_CHANNEL_FMMU
{
    ECAT_BYTE RawData[ECAT_CHANNEL_FMMU_SIZE];
} ECAT_CHANNEL_FMMU;

#pragma pack()

/*! @brief Set Logical Start Address in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_LOGICALSTARTADDRESS(pFmmu, Val)               \
            ((ECAT_PBYTE)pFmmu)[0] = ECAT_LOBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pFmmu)[1] = ECAT_HIBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pFmmu)[2] = ECAT_LOBYTE(ECAT_HIWORD(Val));         \
            ((ECAT_PBYTE)pFmmu)[3] = ECAT_HIBYTE(ECAT_HIWORD(Val));
/*! @brief Get Logical Start Address from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_LOGICALSTARTADDRESS(pFmmu)                    \
            ECAT_MAKEDWORD(                                                 \
                ECAT_MAKEWORD(((ECAT_PBYTE)pFmmu)[3],                       \
                            ((ECAT_PBYTE)pFmmu)[2]),                        \
                ECAT_MAKEWORD(((ECAT_PBYTE)pFmmu)[1],                       \
                            ((ECAT_PBYTE)pFmmu)[0]))

/*! @brief Set Length in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_LENGTH(pFmmu, Val)                            \
            ((ECAT_PBYTE)pFmmu)[4] = ECAT_LOBYTE(Val);                      \
            ((ECAT_PBYTE)pFmmu)[5] = ECAT_HIBYTE(Val);
/*! @brief Get Length from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_LENGTH(pFmmu)                                 \
            ECAT_MAKEWORD(((ECAT_PBYTE)pFmmu)[5], ((ECAT_PBYTE)pHdr)[4])

/*! @brief Set Logical Start Bit in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_LOGICALSTARTBIT(pFmmu, Val)                   \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 48, 3, Val)
/*! @brief Get Logical Start Bit from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_LOGICALSTARTBIT(pFmmu)                        \
            ECAT_GET_LE_BYTE_BITS(pFmmu, 48, 3)

/*! @brief Set bits 51..55(reserved bits) in FMMU channel structure.  */
#define ECAT_CHANNEL_FMMU_SET_RESERVED1(pFmmu, Val)                         \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 51, 5, Val)

/*! @brief Set Logical End Bit in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_LOGICALENDBIT(pFmmu, Val)                     \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 56, 3, Val)
/*! @brief Get Logical End Bit from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_LOGICALENDBIT(pFmmu)                          \
            ECAT_GET_LE_BYTE_BITS(pFmmu, 56, 3)

/*! @brief Set bits 59..63(reserved bits) in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_RESERVED2(pFmmu, Val)                         \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 59, 5, Val)

/*! @brief Set Physical Start Address in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_PHYSICALSTARTADDRESS(pFmmu, Val)              \
            ((ECAT_PBYTE)pFmmu)[8] = ECAT_LOBYTE(Val);                      \
            ((ECAT_PBYTE)pFmmu)[9] = ECAT_HIBYTE(Val);
/*! @brief Get Physical Start Address from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_PHYSICALSTARTADDRESS(pFmmu)                   \
            ECAT_MAKEWORD(((ECAT_PBYTE)pFmmu)[9], ((ECAT_PBYTE)pHdr)[8])

/*! @brief Set Physical Start Bit in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_PHYSICALSTARTBIT(pFmmu, Val)                  \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 80, 3, Val)
/*! @brief Get Physical Start Bit from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_PHYSICALSTARTBIT(pFmmu)                       \
            ECAT_GET_LE_BYTE_BITS(pFmmu, 80, 3)

/*! @brief Set bits 83..87(reserved bits) in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_RESERVED3(pFmmu, Val)                         \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 83, 5, Val)

/*! @brief Set Read Enable field in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_READENABLE(pFmmu, Val)                        \
            ECAT_SET_LE_BIT(pFmmu, 88, Val)
/*! @brief Get Read Enable field from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_READENABLE(pFmmu)                             \
            ECAT_GET_LE_BIT(pFmmu, 88)

/*! @brief Set Write Enable field in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_WRITEENABLE(pFmmu, Val)                       \
            ECAT_SET_LE_BIT(pFmmu, 89, Val)
/*! @brief Get Write Enable field from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_WRITEENABLE(pFmmu)                            \
            ECAT_GET_LE_BIT(pFmmu, 89)

/*! @brief Set bits 90..95(reserved bits) in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_RESERVED4(pFmmu, Val)                         \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 90, 6, Val)

/*! @brief Set Channel Enable field in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_CHANNELENABLE(pFmmu, Val)                     \
            ECAT_SET_LE_BIT(pFmmu, 96, Val)
/*! @brief Get Channel Enable field from FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_GET_CHANNELENABLE(pFmmu)                          \
            ECAT_GET_LE_BIT(pFmmu, 96)

/*! @brief Set bits 97..103(reserved bits) in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_SET_RESERVED5(pFmmu, Val)                         \
            ECAT_SET_LE_BYTE_BITS(pFmmu, 97, 7, Val)

/*! @brief Set bits 104..127(reserved bits) in FMMU channel structure. */
#define ECAT_CHANNEL_FMMU_RESET_RESERVED6(pFmmu, Val)                       \
            ((ECAT_PBYTE)pFmmu)[13] = 0x00;                                 \
            ((ECAT_PBYTE)pFmmu)[14] = 0x00;                                 \
            ((ECAT_PBYTE)pFmmu)[15] = 0x00;

/*! @brief Size of a structure for one Sync Manager channel measured in bytes. */
#define ECAT_CHANNEL_SM_SIZE        8

#pragma pack(1)
/*! @brief Structure that contains array of bytes that
           represent structure for one channel of Sync Manager.

    Format of the description below: BITS: BYTE OFFSET: FIELD DESCRIPTION
    <UL>
    <LI>bits  0..15: 0x0000: Physical Start Address
    <LI>bits 16..31: 0x0002: Length
    <LI>bits 32..33: 0x0004: Buffer Type (0x00: buffered, 0x02: queued)
    <LI>bits 34..35: 0x0004: Direction (0x00: buffer shall be read from the master,
                                        0x01: buffer shall be written from the master)
    <LI>bit      36: 0x0004: Reserved : 0x00
    <LI>bit      37: 0x0004: AL Event Enable (0x00: AL event is notactive,
                                              0x01: AL event is active
                                              (when buffer was accessed
                                              and is no longer locked))
    <LI>bit      38: 0x0004: Watchdog Enable (0x00: Watchdog disabled,
                                              0x01: Watchdog enabled)
    <LI>bit      39: 0x0004: Reserved : 0x00
    <LI>bit      40: 0x0005: Write Event (0x00: no write event,
                                          0x01: write event)
    <LI>bit      41: 0x0005: Read Event (0x00: no read event,
                                         0x01: read event)
    <LI>bit      42: 0x0005: Watchdog Trigger (0x00: watchdog was not triggered,
                                               0x01: watchdog was triggered)
    <LI>bit      43: 0x0005: 1-Buffer State(queued State) (0x00: buffer read,
                                                           0x01: buffer written)
    <LI>bits 44..45: 0x0005: 3-Buffer State(buffered State) (0x00: first buffer,
                                                             0x01: second buffer,
                                                             0x02: third buffer,
                                                             0x03: buffer locked)
    <LI>bits 46..47: 0x0005: Reserved : 0x00
    <LI>bit      48: 0x0006: Channel Enable (0x00: channel disabled,
                                             0x01: channel enabled)
    <LI>bits 49..55: 0x0006: Reserved : 0x00
    <LI>bits 56..63: 0x0007: Reserved : 0x00</UL>*/
typedef struct _ECAT_CHANNEL_SM
{
    ECAT_BYTE RawData[ECAT_CHANNEL_SM_SIZE];
} ECAT_CHANNEL_SM;
#pragma pack()

/*! @brief Set Physical Start Address field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_PHYSICALSTARTADDRESS(pSm, Val)                  \
            ((ECAT_PBYTE)pSm)[0] = ECAT_LOBYTE(Val);                        \
            ((ECAT_PBYTE)pSm)[1] = ECAT_HIBYTE(Val);
/*! @brief Get Physical Start Address field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_PHYSICALSTARTADDRESS(pSm)                       \
            ECAT_MAKEWORD(((ECAT_PBYTE)pSm)[1], ((ECAT_PBYTE)pSm)[0])

/*! @brief Set Length field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_LENGTH(pSm, Val)                                \
            ((ECAT_PBYTE)pSm)[2] = ECAT_LOBYTE(Val);                        \
            ((ECAT_PBYTE)pSm)[3] = ECAT_HIBYTE(Val);
/*! @brief Get Length field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_LENGTH(pSm)                                     \
            ECAT_MAKEWORD(((ECAT_PBYTE)pFmmu)[3], ((ECAT_PBYTE)pHdr)[2])

/*! @brief Set Buffer Type field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_BUFFERTYPE(pSm, Val)                            \
            ECAT_SET_LE_BYTE_BITS(pSm, 32, 2, Val)
/*! @brief Get Buffer Type field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_BUFFERTYPE(pSm)                                 \
            ECAT_GET_LE_BYTE_BITS(pSm, 32, 2)

/*! @brief Set Direction field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_DIRECTION(pSm, Val)                             \
            ECAT_SET_LE_BYTE_BITS(pSm, 34, 2, Val)
/*! @brief Get Direction field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_DIRECTION(pSm)                                  \
            ECAT_GET_LE_BYTE_BITS(pSm, 34, 2)

/*! @brief Set bit 36(reserved bit) of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_RESERVED1(pSm, Val)                             \
            ECAT_SET_LE_BIT(pSm, 36, Val)

/*! @brief Set AL Event Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_ALEVENTENABLE(pSm, Val)                         \
            ECAT_SET_LE_BIT(pSm, 37, Val)
/*! @brief Get AL Event Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_ALEVENTENABLE(pSm)                              \
            ECAT_GET_LE_BIT(pSm, 37)

/*! @brief Set Watchdog Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_WATCHDOGENABLE(pSm, Val)                        \
            ECAT_SET_LE_BIT(pSm, 38, Val)
/*! @brief Get Watchdog Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_WATCHDOGENABLE(pSm)                             \
            ECAT_GET_LE_BIT(pSm, 38)

/*! @brief Set bit 39(reserved bit) of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_RESERVED2(pSm, Val)                             \
            ECAT_SET_LE_BIT(pSm, 39, Val)

/*! @brief Set Write Event field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_WRITEEVENT(pSm, Val)                            \
            ECAT_SET_LE_BIT(pSm, 40, Val)
/*! @brief Get Write Event field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_WRITEEVENT(pSm)                                 \
            ECAT_GET_LE_BIT(pSm, 40)

/*! @brief Set Read Event field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_READEVENT(pSm, Val)                             \
            ECAT_SET_LE_BIT(pSm, 41, Val)
/*! @brief Get Read Event field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_READEVENT(pSm)                                  \
            ECAT_GET_LE_BIT(pSm, 41)

/*! @brief Set Watchdog Trigger field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_WATCHDOGTRIGGER(pSm, Val)                       \
            ECAT_SET_LE_BIT(pSm, 42, Val)
/*! @brief Get Watchdog Trigger field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_WATCHDOGTRIGGER(pSm)                            \
            ECAT_GET_LE_BIT(pSm, 42)

/*! @brief Set 1-Buffer State(queued State) field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_BUFFER1STATE(pSm, Val)                          \
            ECAT_SET_LE_BIT(pSm, 43, Val)
/*! @brief Get 1-Buffer State(queued State) field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_BUFFER1STATE(pSm)                               \
            ECAT_GET_LE_BIT(pSm, 43)

/*! @brief Set 3-Buffer State(buffered State) field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_BUFFER3STATE(pSm, Val)                          \
            ECAT_SET_LE_BYTE_BITS(pSm, 44, 2, Val)
/*! @brief Get 3-Buffer State(buffered State) field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_BUFFER3STATE(pSm)                               \
            ECAT_GET_LE_BYTE_BITS(pSm, 44, 2)

/*! @brief Set bits 46..47(reserved bits) of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_RESERVED3(pSm, Val)                             \
            ECAT_SET_LE_BYTE_BITS(pSm, 46, 2, Val)

/*! @brief Set Channel Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_CHANNELENABLE(pSm, Val)                         \
            ECAT_SET_LE_BIT(pSm, 48, Val)
/*! @brief Get Channel Enable field of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_GET_CHANNELENABLE(pSm)                              \
            ECAT_GET_LE_BIT(pSm, 48)

/*! @brief Set bits 49..55(reserved bits) of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_RESERVED4(pSm, Val)                             \
            ECAT_SET_LE_BYTE_BITS(pSm, 49, 7, Val)

/*! @brief Set bits 56..63(reserved bits) of Sync Manager channel structure. */
#define ECAT_CHANNEL_SM_SET_RESERVED5(pSm, Val)                             \
            ((ECAT_PBYTE)pSm)[6] = (ECAT_BYTE)(Val)

/*! @brief Numeric identifiers of variable's types. */
typedef enum _EcatVariableDataType
{
    EcatDataTypeUnknown = 0, /*!< unspecified type           */
    EcatDataTypeBOOL,        /*!< boolean, 8 bit             */
    EcatDataTypeBYTE,        /*!< byte, 8 bit (0 - 255)      */
    EcatDataTypeWORD,        /*!< word, 16 bit (0 - 65535)   */
    EcatDataTypeDWORD,       /*!< double word, 32 bit (0 - 4294967295)      */
    EcatDataTypeSINT,        /*!< signed short integer, 8 bit (-128 - 127)  */
    EcatDataTypeUSINT,       /*!< unsigned short integer, 8 bit (0 - 255)   */
    EcatDataTypeINT,         /*!< integer, 16 bit (-32768 - 32767)          */
    EcatDataTypeUINT,        /*!< unsigned integer, 16 bit (0 - 65535)      */
    EcatDataTypeDINT,        /*!< double integer, 32 bit (-2147483648 - 2147483647) */
    EcatDataTypeUDINT,       /*!< unsigned double integer, 32 bit (0 - 4294967295)  */
    EcatDataTypeREAL,        /*!< floating-point, 32 bit (~-3.402823*10^38 - ~3.402823*10^38)*/
    EcatDataTypeLREAL,
    EcatDataTypeSTRING,
    EcatDataTypeTIME,
    EcatDataTypeTIME_OF_DAY,
    EcatDataTypeDATE,
    EcatDataTypeDATE_AND_TIME
} EcatVariableDataType;

/* Registers */

/* 1. Data Link Entity */

/* 1.1. DL Information */

#define ECAT_RADDR_DL_TYPE          0x0000  /*!< Physical Address where Type ID of a device is located     */
#define ECAT_RADDR_DL_REVISION      0x0001  /*!< Physical Address where Revision ID of a device is located */
#define ECAT_RADDR_DL_BUILD         0x0002  /*!< Physical Address where Build ID of a device is located    */
#define ECAT_RADDR_DL_CHANNELS_FMMU 0x0004  /*!< Physical Address where Number of supported FMMU channels is located */
#define ECAT_RADDR_DL_CHANNELS_SM   0x0005  /*!< Physical Address where Number of supported Sync Manager channels is located */

#define ECAT_REG_DL_INFO_SIZE       10 /*!< Size of the DL Information structure measured in bytes. */
#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent DL Information structure.

    Format of the description below: BITS: PHYSICAL ADDRESS: FIELD DESCRIPTION
    <UL>
    <LI>bits    0..7: 0x0000: Type
    <LI>bits   8..15: 0x0001: Revision
    <LI>bits  16..31: 0x0002: Build
    <LI>bits  32..39: 0x0004: Number of supported FMMU channels (0x0001-0x0010)
    <LI>bits  40..47: 0x0005: Number of supported Sync Manager channels (0x0001-0x0010)
    <LI>bits  48..55: 0x0006: RAM Size (RAM Size in kByte (5-64))
    <LI>bits  56..63: 0x0007: Reserved
    <LI>bit       64: 0x0008: FMMU Bit Operation Not Supported
                             (0: Bit Operation supported,
                              1: Bit Operation not supported)
    <LI>bits  65..79: 0x0008: Reserved
    </UL>*/
typedef struct _ECAT_REG_DL_INFO
{
    ECAT_BYTE RawData[ECAT_REG_DL_INFO_SIZE];
} ECAT_REG_DL_INFO, **ECAT_REG_DL_INFO_PPTR;
#pragma pack()


/* Type */
#define ECAT_REG_DL_INFO_SET_TYPE(pDl, Val)                                 \
            ((ECAT_PBYTE)pDl)[0] = (Val);
/*! @brief Get Type field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_TYPE(pDl)                                      \
            ((ECAT_PBYTE)pDl)[0]

/* Revision */
#define ECAT_REG_DL_INFO_SET_REVISION(pDl, Val)                             \
            ((ECAT_PBYTE)pDl)[1] = (Val);
/*! @brief Get Revision field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_REVISION(pDl)                                  \
            ((ECAT_PBYTE)pDl)[1]

/* Build */
#define ECAT_REG_DL_INFO_SET_BUILD(pDl, Val)                                \
            ((ECAT_PBYTE)pDl)[2] = ECAT_LOBYTE(Val);                        \
            ((ECAT_PBYTE)pDl)[3] = ECAT_HIBYTE(Val);
/*! @brief Get Build field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_BUILD(pDl)                                     \
            ECAT_MAKEWORD(((ECAT_PBYTE)pDl)[3], ((ECAT_PBYTE)pDl)[2])

/* Number of FMMU */
#define ECAT_REG_DL_INFO_SET_NUMBEROFFMMU(pDl, Val)                         \
            ((ECAT_PBYTE)pDl)[4] = (Val);
/*! @brief Get Number of supported FMMU channels field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_NUMBEROFFMMU(pDl)                              \
            ((ECAT_PBYTE)pDl)[4]

/* Number of SM */
#define ECAT_REG_DL_INFO_SET_NUMBEROFSM(pDl, Val)                           \
            ((ECAT_PBYTE)pDl)[5] = (Val);
/*! @brief Get Number of supported Sync Manager channels field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_NUMBEROFSM(pDl)                                \
            ((ECAT_PBYTE)pDl)[5]

/* RAM Size */
#define ECAT_REG_DL_INFO_SET_RAMSIZE(pDl, Val)                              \
            ((ECAT_PBYTE)pDl)[6] = (Val);
/*! @brief Get RAM Size field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_RAMSIZE(pDl)                                   \
            ((ECAT_PBYTE)pDl)[6]

/* FMMU Bit Operation Not Supported */
#define ECAT_REG_DL_INFO_SET_FMMUBITOP(pDl, Val)                            \
            ECAT_SET_LE_BIT(pDl, 64, Val)
/*! @brief Get FMMU Bit Operation Not Supported field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_FMMUBITOP(pDl)                                 \
            ECAT_GET_LE_BIT(pDl, 64)

/* Reserved */
#define ECAT_REG_DL_INFO_SET_RESERVED(pDl, Val)                             \
            ((ECAT_PBYTE)pDl)[8] &= ~0xFE;                                  \
            ((ECAT_PBYTE)pDl)[8] |= ECAT_LOBYTE((Val) << 1);                \
            ((ECAT_PBYTE)pDl)[9] = ECAT_HIBYTE((Val) << 1);
/*! @brief Get FMMU Bit Operation Not Supported field of DL Information structure. */
#define ECAT_REG_DL_INFO_GET_RESERVED(pDl)                                  \
            (ECAT_MAKEWORD(((ECAT_PBYTE)pDl)[9],                            \
                        ((ECAT_PBYTE)pDl)[8] & 0xFE) >> 1)


/* 1.2. Fixed Station Address( = Configured Station Address) */

/*! @brief Physical Address where value of Fixed Station Address
           (i.e. Configured Station Address) of a device is located.
            That value is of type #ECAT_WORD (16 bit). */
#define ECAT_RADDR_DL_FIXED_ADDRESS	0x0010

/* 1.3. DL Control */

/*! @brief Physical Address of DL Control register. */
#define ECAT_RADDR_DL_CONTROL           0x0100
/*! @brief DL Control register size measured in bytes. */
#define ECAT_REG_DL_CONTROL_SIZE        2

#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent DL Control register.

    <UL>
        <LI>bit       0: Forwarding Rule
        <LI>bits   1..3: Reserved
        <LI>bit       4: Extended Link detection activated on Port 0
        <LI>bit       5: Extended Link detection activated on Port 1
        <LI>bit       6: Extended Link detection activated on Port 2
        <LI>bit       7: Extended Link detection activated on Port 3
        <LI>bits   8..9: Loop Port 0
        <LI>bits 10..11: Loop Port 1
        <LI>bits 12..13: Loop Port 2
        <LI>bits 14..15: Loop Port 3
    </UL>*/
typedef struct _ECAT_REG_DL_CONTROL
{
    ECAT_BYTE RawData[ECAT_REG_DL_CONTROL_SIZE];
} ECAT_REG_DL_CONTROL;
#pragma pack()

/* Forwarding Rule */
#define ECAT_REG_DL_CONTROL_SET_FWD_RULE(pDl, Val)                          \
            ECAT_SET_LE_BIT(pDl, 0, Val)
#define ECAT_REG_DL_CONTROL_GET_FWD_RULE(pDl)                               \
            ECAT_GET_LE_BIT(pDl, 0)

/* Reserved */
#define ECAT_REG_DL_CONTROL_SET_RESERVED(pDl, Val)                          \
            ECAT_SET_LE_BYTE_BITS(pDl, 1, 3, Val)
#define ECAT_REG_DL_CONTROL_GET_RESERVED(pDl)                               \
            ECAT_GET_LE_BYTE_BITS(pDl, 1, 3)

/* Extended Link on Port 0 */
#define ECAT_REG_DL_CONTROL_SET_EXT_LINK_ON_PORT0(pDl, Val)                 \
            ECAT_SET_LE_BIT(pDl, 4, Val)
#define ECAT_REG_DL_CONTROL_GET_EXT_LINK_ON_PORT0(pDl)                      \
            ECAT_GET_LE_BIT(pDl, 4)

/* Extended Link on Port 1 */
#define ECAT_REG_DL_CONTROL_SET_EXT_LINK_ON_PORT1(pDl, Val)                 \
            ECAT_SET_LE_BIT(pDl, 5, Val)
#define ECAT_REG_DL_CONTROL_GET_EXT_LINK_ON_PORT1(pDl)                      \
            ECAT_GET_LE_BIT(pDl, 5)

/* Extended Link on Port 2 */
#define ECAT_REG_DL_CONTROL_SET_EXT_LINK_ON_PORT2(pDl, Val)                 \
            ECAT_SET_LE_BIT(pDl, 6, Val)
#define ECAT_REG_DL_CONTROL_GET_EXT_LINK_ON_PORT2(pDl)                      \
            ECAT_GET_LE_BIT(pDl, 6)

/* Extended Link on Port 3 */
#define ECAT_REG_DL_CONTROL_SET_EXT_LINK_ON_PORT3(pDl, Val)                 \
            ECAT_SET_LE_BIT(pDl, 7, Val)
#define ECAT_REG_DL_CONTROL_GET_EXT_LINK_ON_PORT3(pDl)                      \
            ECAT_GET_LE_BIT(pDl, 7)

/* Loop Port 0 */
#define ECAT_REG_DL_CONTROL_SET_LOOP_PORT0(pDl, Val)                        \
            ECAT_SET_LE_BYTE_BITS(pDl, 8, 2, Val)
#define ECAT_REG_DL_CONTROL_GET_LOOP_PORT0(pDl)                             \
            ECAT_GET_LE_BYTE_BITS(pDl, 8, 2)

/* Loop Port 1 */
#define ECAT_REG_DL_CONTROL_SET_LOOP_PORT1(pDl, Val)                        \
            ECAT_SET_LE_BYTE_BITS(pDl, 10, 2, Val)
#define ECAT_REG_DL_CONTROL_GET_LOOP_PORT1(pDl)                             \
            ECAT_GET_LE_BYTE_BITS(pDl, 10, 2)

/* Loop Port 2 */
#define ECAT_REG_DL_CONTROL_SET_LOOP_PORT2(pDl, Val)                        \
            ECAT_SET_LE_BYTE_BITS(pDl, 12, 2, Val)
#define ECAT_REG_DL_CONTROL_GET_LOOP_PORT2(pDl)                             \
            ECAT_GET_LE_BYTE_BITS(pDl, 12, 2)

/* Loop Port 3 */
#define ECAT_REG_DL_CONTROL_SET_LOOP_PORT3(pDl, Val)                        \
            ECAT_SET_LE_BYTE_BITS(pDl, 14, 2, Val)
#define ECAT_REG_DL_CONTROL_GET_LOOP_PORT3(pDl)                             \
            ECAT_GET_LE_BYTE_BITS(pDl, 14, 2)

#define ECAT_RADDR_DL_CONTROL_LOOP      0x0101  /* DL Control Loop Port */
#define ECAT_REG_DL_CONTROL_LOOP_SIZE   1
/* Loop Port 2 */
#define ECAT_REG_DL_CONTROL_LOOP_SET_LOOP_PORT2(pDl, Val)                   \
            ECAT_SET_LE_BYTE_BITS(pDl, (12 - 8), 2, Val)
#define ECAT_REG_DL_CONTROL_LOOP_GET_LOOP_PORT2(pDl)                        \
            ECAT_GET_LE_BYTE_BITS(pDl, (12 - 8), 2)

/* 1.4. DLL Status */

/*! @brief Physical Address of DL Status register. */
#define ECAT_RADDR_DL_STATUS        0x0110
/*! @brief DL Status register size measured in bytes. */
#define ECAT_REG_DL_STATUS_SIZE     2

#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent DL Status register.

    <UL>
    <LI> bit     0: PDI Operational
    <LI> bit     1: PDI Watchdog Status
    <LI> bits 2..3: Reserved
    <LI> bit     4: Link on Port 0
    <LI> bit     5: Link on Port 1
    <LI> bit     6: Link on Port 2
    <LI> bit     7: Link on Port 3
    <LI> bit     8: Loop Port 0 (Channel A Loop Status)
    <LI> bit     9: Communication on Port 0 (Channel A Signal Detection)
    <LI> bit    10: Loop Port 1 (Channel B Loop Status)
    <LI> bit    11: Communication on Port 1 (Channel B Signal Detection)
    <LI> bit    12: Loop Port 2 (Channel C Loop Status)
    <LI> bit    13: Communication on Port 2 (Channel C Signal Detection)
    <LI> bit    14: Loop Port 3 (Channel D Loop Status)
    <LI> bit    15: Communication on Port 3 (Channel D Signal Detection)
    </UL>*/
typedef struct _ECAT_REG_DL_STATUS
{
    ECAT_BYTE RawData[ECAT_REG_DL_STATUS_SIZE];
} ECAT_REG_DL_STATUS, *ECAT_REG_DL_STATUS_PTR;

#pragma pack()

/* PDI Operational */
#define ECAT_REG_DL_STATUS_SET_PDI_OPERATIONAL(pDl, Val)                    \
            ECAT_SET_LE_BIT(pDl, 0, Val)
#define ECAT_REG_DL_STATUS_GET_PDI_OPERATIONAL(pDl)                         \
            ECAT_GET_LE_BIT(pDl, 0)

/* PDI Watchdog Status */
#define ECAT_REG_DL_STATUS_SET_PDI_WATCHDOG_STATUS(pDl, Val)                \
            ECAT_SET_LE_BIT(pDl, 1, Val)
#define ECAT_REG_DL_STATUS_GET_PDI_WATCHDOG_STATUS(pDl)                     \
            ECAT_GET_LE_BIT(pDl, 1)

/* Reserved */
#define ECAT_REG_DL_STATUS_SET_RESERVED(pDl, Val)                           \
            ECAT_SET_LE_BYTE_BITS(pDl, 2, 2, Val)
#define ECAT_REG_DL_STATUS_GET_RESERVED(pDl)                                \
            ECAT_GET_LE_BYTE_BITS(pDl, 2, 2)

/* Link on Port 0 */
#define ECAT_REG_DL_STATUS_SET_LINK_ON_PORT0(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 4, Val)
#define ECAT_REG_DL_STATUS_GET_LINK_ON_PORT0(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 4)

/* Link on Port 1 */
#define ECAT_REG_DL_STATUS_SET_LINK_ON_PORT1(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 5, Val)
#define ECAT_REG_DL_STATUS_GET_LINK_ON_PORT1(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 5)

/* Link on Port 2 */
#define ECAT_REG_DL_STATUS_SET_LINK_ON_PORT2(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 6, Val)
#define ECAT_REG_DL_STATUS_GET_LINK_ON_PORT2(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 6)

/* Link on Port 3 */
#define ECAT_REG_DL_STATUS_SET_LINK_ON_PORT3(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 7, Val)
#define ECAT_REG_DL_STATUS_GET_LINK_ON_PORT3(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 7)

/* Loop Port 0 */
#define ECAT_REG_DL_STATUS_SET_LOOP_PORT0(pDl, Val)                         \
            ECAT_SET_LE_BIT(pDl, 8, Val)
#define ECAT_REG_DL_STATUS_GET_LOOP_PORT0(pDl)                              \
            ECAT_GET_LE_BIT(pDl, 8)

/* Communication on Port 0 */
#define ECAT_REG_DL_STATUS_SET_COMM_ON_PORT0(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 9, Val)
#define ECAT_REG_DL_STATUS_GET_COMM_ON_PORT0(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 9)

/* Loop Port 1 */
#define ECAT_REG_DL_STATUS_SET_LOOP_PORT1(pDl, Val)                         \
            ECAT_SET_LE_BIT(pDl, 10, Val)
#define ECAT_REG_DL_STATUS_GET_LOOP_PORT1(pDl)                              \
            ECAT_GET_LE_BIT(pDl, 10)

/* Communication on Port 1 */
#define ECAT_REG_DL_STATUS_SET_COMM_ON_PORT1(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 11, Val)
#define ECAT_REG_DL_STATUS_GET_COMM_ON_PORT1(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 11)

/* Loop Port 2 */
#define ECAT_REG_DL_STATUS_SET_LOOP_PORT2(pDl, Val)                         \
            ECAT_SET_LE_BIT(pDl, 12, Val)
#define ECAT_REG_DL_STATUS_GET_LOOP_PORT2(pDl)                              \
            ECAT_GET_LE_BIT(pDl, 12)

/* Communication on Port 2 */
#define ECAT_REG_DL_STATUS_SET_COMM_ON_PORT2(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 13, Val)
#define ECAT_REG_DL_STATUS_GET_COMM_ON_PORT2(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 13)

/* Loop Port 3 */
#define ECAT_REG_DL_STATUS_SET_LOOP_PORT3(pDl, Val)                         \
            ECAT_SET_LE_BIT(pDl, 14, Val)
#define ECAT_REG_DL_STATUS_GET_LOOP_PORT3(pDl)                              \
            ECAT_GET_LE_BIT(pDl, 14)

/* Communication on Port 3 */
#define ECAT_REG_DL_STATUS_SET_COMM_ON_PORT3(pDl, Val)                      \
            ECAT_SET_LE_BIT(pDl, 15, Val)
#define ECAT_REG_DL_STATUS_GET_COMM_ON_PORT3(pDl)                           \
            ECAT_GET_LE_BIT(pDl, 15)

/* 1.5. RX-Error & Lost-Link Counters */

/*! @brief Physical Address of DL Status register. */
#define ECAT_RADDR_COUNTERS                     0x0300

#pragma pack(1)
/*! @brief Size of memory that starts from the beginning of RX-Error counters
           and ends at the end of Lost-Link Counters. */
#define ECAT_REG_COUNTERS_SIZE                  20

/*! @brief Structure that contains array of bytes that represent memory
           that starts from the beginning of RX-Error counters
           and ends at the end of Lost-Link Counters.

    <UL>
    <LI>byte 0x0000: RX-Error Counter Port 0
    <LI>byte 0x0001: Invalid Frame Counter Port 0
    <LI>byte 0x0002: RX-Error Counter Port 1
    <LI>byte 0x0003: Invalid Frame Counter Port 1
    <LI>byte 0x0004: RX-Error Counter Port 2
    <LI>byte 0x0005: Invalid Frame Counter Port 2
    <LI>byte 0x0006: RX-Error Counter Port 3
    <LI>byte 0x0007: Invalid Frame Counter Port 3

    <LI>byte 0x0010: Lost-Link Counter Port 0
    <LI>byte 0x0011: Lost-Link Counter Port 1
    <LI>byte 0x0012: Lost-Link Counter Port 2
    <LI>byte 0x0013: Lost-Link Counter Port 3
    </UL> */
typedef struct _ECAT_REG_COUNTERS
{
    ECAT_BYTE RawData[ECAT_REG_COUNTERS_SIZE];
} ECAT_REG_COUNTERS, *ECAT_ECAT_REG_COUNTERS_PTR;
#pragma pack()

/* RX-Error Counter Port 0 */
#define ECAT_REG_COUNTERS_SET_RX_ERROR_PORT0(pReg, Val)                     \
            ((ECAT_PBYTE)pReg)[1] = (Val);
#define ECAT_REG_COUNTERS_GET_RX_ERROR_PORT0(pReg)                          \
            ((ECAT_PBYTE)pReg)[1]

/* Invalid Frame Counter Port 0 */
#define ECAT_REG_COUNTERS_SET_INVALID_FRAME_PORT0(pReg, Val)                \
            ((ECAT_PBYTE)pReg)[0] = (Val);
#define ECAT_REG_COUNTERS_GET_INVALID_FRAME_PORT0(pReg)                     \
            ((ECAT_PBYTE)pReg)[0]

/* RX-Error Counter Port 1 */
#define ECAT_REG_COUNTERS_SET_RX_ERROR_PORT1(pReg, Val)                     \
            ((ECAT_PBYTE)pReg)[3] = (Val);
#define ECAT_REG_COUNTERS_GET_RX_ERROR_PORT1(pReg)                          \
            ((ECAT_PBYTE)pReg)[3]

/* Invalid Frame Counter Port 1 */
#define ECAT_REG_COUNTERS_SET_INVALID_FRAME_PORT1(pReg, Val)                \
            ((ECAT_PBYTE)pReg)[2] = (Val);
#define ECAT_REG_COUNTERS_GET_INVALID_FRAME_PORT1(pReg)                     \
            ((ECAT_PBYTE)pReg)[2]

/* RX-Error Counter Port 2 */
#define ECAT_REG_COUNTERS_SET_RX_ERROR_PORT2(pReg, Val)                     \
            ((ECAT_PBYTE)pReg)[5] = (Val);
#define ECAT_REG_COUNTERS_GET_RX_ERROR_PORT2(pReg)                          \
            ((ECAT_PBYTE)pReg)[5]

/* Invalid Frame Counter Port 2 */
#define ECAT_REG_COUNTERS_SET_INVALID_FRAME_PORT2(pReg, Val)                \
            ((ECAT_PBYTE)pReg)[4] = (Val);
#define ECAT_REG_COUNTERS_GET_INVALID_FRAME_PORT2(pReg)                     \
            ((ECAT_PBYTE)pReg)[4]

/* RX-Error Counter Port 3 */
#define ECAT_REG_COUNTERS_SET_RX_ERROR_PORT3(pReg, Val)                     \
            ((ECAT_PBYTE)pReg)[7] = (Val);
#define ECAT_REG_COUNTERS_GET_RX_ERROR_PORT3(pReg)                          \
            ((ECAT_PBYTE)pReg)[7]

/* Invalid Frame Counter Port 3 */
#define ECAT_REG_COUNTERS_SET_INVALID_FRAME_PORT3(pReg, Val)                \
            ((ECAT_PBYTE)pReg)[6] = (Val);
#define ECAT_REG_COUNTERS_GET_INVALID_FRAME_PORT3(pReg)                     \
            ((ECAT_PBYTE)pReg)[6]

/* Lost-Link Counter Port 0 */
#define ECAT_REG_COUNTERS_SET_LOST_LINK_PORT0(pReg, Val)                    \
            ((ECAT_PBYTE)pReg)[0x10] = (Val);
#define ECAT_REG_COUNTERS_GET_LOST_LINK_PORT0(pReg)                         \
            ((ECAT_PBYTE)pReg)[0x10]

/* Lost-Link Counter Port 1 */
#define ECAT_REG_COUNTERS_SET_LOST_LINK_PORT1(pReg, Val)                    \
            ((ECAT_PBYTE)pReg)[0x11] = (Val);
#define ECAT_REG_COUNTERS_GET_LOST_LINK_PORT1(pReg)                         \
            ((ECAT_PBYTE)pReg)[0x11]

/* Lost-Link Counter Port 2 */
#define ECAT_REG_COUNTERS_SET_LOST_LINK_PORT2(pReg, Val)                    \
            ((ECAT_PBYTE)pReg)[0x12] = (Val);
#define ECAT_REG_COUNTERS_GET_LOST_LINK_PORT2(pReg)                         \
            ((ECAT_PBYTE)pReg)[0x12]

/* Lost-Link Counter Port 3 */
#define ECAT_REG_COUNTERS_SET_LOST_LINK_PORT3(pReg, Val)                    \
            ((ECAT_PBYTE)pReg)[0x13] = (Val);
#define ECAT_REG_COUNTERS_GET_LOST_LINK_PORT3(pReg)                         \
            ((ECAT_PBYTE)pReg)[0x13]


/* 2. Slave Information Interface */
#pragma pack(1)
#define ECAT_SII_CONTENT_SIZE  128

/*! @brief Structure that contains array of bytes that represent
    Slave Information Interface contents.*/
typedef struct _ECAT_SII_CONTENT
{
    ECAT_BYTE RawData[ECAT_SII_CONTENT_SIZE];
} ECAT_SII_CONTENT, **ECAT_SII_CONTENT_PPTR;

#pragma pack()

/* Vendor ID */
#define ECAT_SII_CONTENT_GET_VENDOR_ID(pSII)                                \
            ECAT_GET_LE_DWORD(((ECAT_PBYTE)pSII) + 0x10)

/* Product Code */
#define ECAT_SII_CONTENT_GET_PRODUCT_CODE(pSII)                             \
            ECAT_GET_LE_DWORD(((ECAT_PBYTE)pSII) + 0x14)

/* Revision Number */
#define ECAT_SII_CONTENT_GET_REVISION_NUMBER(pSII)                          \
            ECAT_GET_LE_DWORD(((ECAT_PBYTE)pSII) + 0x18)


/* 2.1. Serial E2PROM Configuration */

/*! @brief Physical address of the beginning of E2PROM interface register
           and the beginning of E2PROM Interface register configuration. */
#define ECAT_RADDR_E2PROM_CNFG      0x0500  /* Serial E2PROM Configuration */

/* 2.2. Serial E2PROM Control/Status */

/*! @brief Physical address of E2PROM Control/Status structure. */
#define ECAT_RADDR_E2PROM_CONTROL_STATUS	0x0502

/*! @brief Size of E2PROM Control/Status structure. */
#define ECAT_SII_CONTROL_SIZE       2

#pragma pack(1)
/*! @brief Structure that contains array of bytes that
           represent Register E2PROM Control/Status structure.

    <UL>
    <LI>
    <LI>bit  0:      E2PROM Write Access:
                        0: only read access to E2PROM,
                        1: read and write access to E2PROM
    <LI>bits 1..6:   Reserved: 0x00
    <LI>bit  7:      I2C protocol to E2PROM address:
                        0: has one byte,
                        1: has two bytes
    <LI>bit  8:      Read Operation
    <LI>bit  9:      Write Operation
    <LI>bit 10:      Reload Operation
    <LI>bits 11..13: Reserved: 0x00
    <LI>bit 14:      Write Error
    <LI>bit 15:      Busy
    </UL>*/
typedef struct _ECAT_SII_CONTROL
{
    ECAT_BYTE RawData[ECAT_SII_CONTROL_SIZE];
} ECAT_SII_CONTROL;
#pragma pack()

/* Write Access */
#define ECAT_SII_CONTROL_SET_WRITE_ACCESS(pCtrl, Val)                       \
            ECAT_SET_LE_BIT(pCtrl, 0, Val)
#define ECAT_SII_CONTROL_GET_WRITE_ACCESS(pCtrl)                            \
            ECAT_GET_LE_BIT(pCtrl, 0)

/* Reserved */
#define ECAT_SII_CONTROL_SET_RESERVED1(pCtrl, Val)                          \
            ECAT_SET_LE_BYTE_BITS(pCtrl, 1, 6, Val)

/* Protocol Type */
#define ECAT_SII_CONTROL_SET_PROTOCOL_TYPE(pCtrl, Val)                      \
            ECAT_SET_LE_BIT(pCtrl, 7, Val)
#define ECAT_SII_CONTROL_GET_PROTOCOL_TYPE(pCtrl)                           \
            ECAT_GET_LE_BIT(pCtrl, 7)

/* Read Operation */
#define ECAT_SII_CONTROL_SET_READ_OPERATION(pCtrl, Val)                     \
            ECAT_SET_LE_BIT(pCtrl, 8, Val)
#define ECAT_SII_CONTROL_GET_READ_OPERATION(pCtrl)                          \
            ECAT_GET_LE_BIT(pCtrl, 8)

/* Write Operation */
#define ECAT_SII_CONTROL_SET_WRITE_OPERATION(pCtrl, Val)                    \
            ECAT_SET_LE_BIT(pCtrl, 9, Val)
#define ECAT_SII_CONTROL_GET_WRITE_OPERATION(pCtrl)                         \
            ECAT_SET_LE_BIT(pCtrl, 9)

/* Reload Operation */
#define ECAT_SII_CONTROL_SET_RELOAD_OPERATION(pCtrl, Val)                   \
            ECAT_SET_LE_BIT(pCtrl, 10, Val)
#define ECAT_SII_CONTROL_GET_RELOAD_OPERATION(pCtrl)                        \
            ECAT_GET_LE_BIT(pCtrl, 10)

/* Reserved */
#define ECAT_SII_CONTROL_SET_RESERVED2(pCtrl, Val)                          \
            ECAT_SET_LE_BYTE_BITS(pCtrl, 11, 3, Val)

/* Write Error */
#define ECAT_SII_CONTROL_SET_WRITE_ERROR(pCtrl, Val)                        \
            ECAT_SET_LE_BIT(pCtrl, 14, Val)
#define ECAT_SII_CONTROL_GET_WRITE_ERROR(pCtrl)                             \
            ECAT_GET_LE_BIT(pCtrl, 14)

/* Busy */
#define ECAT_SII_CONTROL_SET_BUSY(pCtrl, Val)                               \
            ECAT_SET_LE_BIT(pCtrl, 15, Val)
#define ECAT_SII_CONTROL_GET_BUSY(pCtrl)                                    \
            ECAT_GET_LE_BIT(pCtrl, 15)

/* ECAT SII Address */

/* 2.3. Actual Serial E2PROM Address */
/*! @brief Physical address where a value of Actual Serial E2PROM Address is located. */
#define ECAT_RADDR_E2PROM_ADDRESS	0x0504

/*! @brief Size of Actual Serial E2PROM Address measured in bytes. */
#define ECAT_SII_ADDR_SIZE          4

#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent Actual SII Address.

    bits  0..31: Actual SII Address */
typedef struct _ECAT_SII_ADDR
{
    ECAT_BYTE   RawData[ECAT_SII_ADDR_SIZE];
} ECAT_SII_ADDR;
#pragma pack()

/* Actual SII Address */
#define ECAT_SII_ADDR_SET_ADDRESS(pAddr, Val)                               \
            ((ECAT_PBYTE)pAddr)[0] = ECAT_LOBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[1] = ECAT_HIBYTE(ECAT_LOWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[2] = ECAT_LOBYTE(ECAT_HIWORD(Val));         \
            ((ECAT_PBYTE)pAddr)[3] = ECAT_HIBYTE(ECAT_HIWORD(Val));
#define ECAT_SII_ADDR_GET_ADDRESS(pAddr)                                    \
            ECAT_MAKEDWORD(                                                 \
                ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[3],                       \
                            ((ECAT_PBYTE)pAddr)[2]),                        \
                ECAT_MAKEWORD(((ECAT_PBYTE)pAddr)[1],                       \
                            ((ECAT_PBYTE)pAddr)[0]))


/* 2.4. Actual Serial E2PROM Data */
/*! @brief Physical address where a value of Actual Serial E2PROM Data is located. */
#define ECAT_RADDR_E2PROM_DATA      0x0508

/* 3. AL Management */

/* 3.1. AL Control */

/*! @brief Physical address of AL Control Register. */
#define ECAT_RADDR_AL_CONTROL       0x0120

/* 3.2. AL Status */

/*! @brief Physical address of AL Status Register. */
#define ECAT_RADDR_AL_STATUS        0x0130
/*! @brief Size of AL Status Register measured in bytes. */
#define ECAT_REG_AL_STATUS_SIZE     2


#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent
           contents of AL Status Register.
    <UL>
    <LI>bits 0..3: Actual State of the Device State Machine:
                     1: Init State,
                     3: Request Bootstrap State(optional),
                     2: Pre-Operational State,
                     4: Safe-Operational State,
                     8: Operational State State
    <LI>bit 4:     Error Ind: state change rejected
                     0: Device is in State as requested or Flag cleared by command,
                     1: Device has not entered requested State or changed State as result of a local action
    <LI>bits 5..7: Reserved: 0x00
    <LI>bits 8..15:Application specific
    </UL> */
typedef struct _ECAT_REG_AL_STATUS
{
    ECAT_BYTE RawData[ECAT_REG_AL_STATUS_SIZE];
} ECAT_REG_AL_STATUS;
#pragma pack()

/* State */
#define ECAT_REG_AL_STATUS_SET_STATE(pReg, Val)                             \
            ECAT_SET_LE_BYTE_BITS(pReg, 0, 4, Val)
#define ECAT_REG_AL_STATUS_GET_STATE(pReg)                                  \
            ECAT_GET_LE_BYTE_BITS(pReg, 0, 4)

/* Error Ind */
#define ECAT_REG_AL_STATUS_SET_ERROR_IND(pReg, Val)                         \
            ECAT_SET_LE_BIT(pReg, 4, Val)
#define ECAT_REG_AL_STATUS_GET_ERROR_IND(pReg)                              \
            ECAT_GET_LE_BIT(pReg, 4)


/* 3.3. AL Event */

#define ECAT_RADDR_AL_EVENT         0x0220  /*!< AL Event Physical Address */


/* 4. Watchdogs */

/* 4.1. Watchdog Divider */

#define ECAT_RADDR_WD_DIVIDER       0x0400	/*!< Physical Address of Watchdog Divider */

/* 4.2. PDI Watchdog */

#define ECAT_RADDR_WD_PDI           0x0410	/*!< Physical Address of PDI Watchdog */

/* 4.3. Sync Manager Channel Watchdog */

#define	ECAT_RADDR_WD_SM_CANNEL00	0x0420	/*!< Sync Manager Channel 0 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL01	0x0422	/*!< Sync Manager Channel 1 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL02	0x0424	/*!< Sync Manager Channel 2 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL03	0x0426	/*!< Sync Manager Channel 3 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL04	0x0428	/*!< Sync Manager Channel 4 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL05	0x042A	/*!< Sync Manager Channel 5 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL06	0x042C	/*!< Sync Manager Channel 6 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL07	0x042E	/*!< Sync Manager Channel 7 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL08	0x0430	/*!< Sync Manager Channel 8 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL09	0x0432	/*!< Sync Manager Channel 9 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL10	0x0434	/*!< Sync Manager Channel 10 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL11	0x0436	/*!< Sync Manager Channel 11 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL12	0x0438	/*!< Sync Manager Channel 12 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL13	0x043A	/*!< Sync Manager Channel 13 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL14	0x043C	/*!< Sync Manager Channel 14 Watchdog */
#define ECAT_RADDR_WD_SM_CANNEL15	0x043E	/*!< Sync Manager Channel 15 Watchdog */

/* 4.3. Sync Manager Watchdog Status */

#define ECAT_RADDR_WD_SM_STATUS		0x0450	/*!< 0x0450..0x0451: Watchdog for Sync Manager Status */


/* SDO Information: Get Entry Description Request/Get Object Description Response  */
/*! @brief Bit masks for ValueInfo field of SDO Information Service Data.
           Bits of ValueInfo field say which elements are(or shall be) in the response.
    <UL>
    <LI>Bit 0: access rights
    <LI>Bit 1: object category
    <LI>Bit 2: information, if object is mappable in aPDO
    <LI>Bit 3: unit type
    <LI>Bit 4: default value
    <LI>Bit 5: minimum value
    <LI>Bit 6: maximum value
    </UL>*/
typedef enum tagEcatMailboxCoeSdoOEDValueInfo
{
    EcatMailboxCoeSdoOEDValueInfoAccessRights      = 0x01,
    EcatMailboxCoeSdoOEDValueInfoObjectCategory    = 0x02,
    EcatMailboxCoeSdoOEDValueInfoInformation       = 0x04,
    EcatMailboxCoeSdoOEDValueInfoUnitType          = 0x08,
    EcatMailboxCoeSdoOEDValueInfoDefaultValue      = 0x10,
    EcatMailboxCoeSdoOEDValueInfoMinimumValue      = 0x20,
    EcatMailboxCoeSdoOEDValueInfoMaximumValue      = 0x40
}EcatMailboxCoeSdoOEDValueInfo;



#define ECAT_DC_TIME_SIZE	4  /*!< Size of a variable that contains time value */
/*  0..31 bits: Time */

#pragma pack(1)
/*! @brief Structure that contains array of bytes that represent a time value(in ns). */
typedef struct _ECAT_DC_TIME
{
    ECAT_BYTE RawData[ECAT_DC_TIME_SIZE];
} ECAT_DC_TIME, **ECAT_DC_TIME_PPTR;
#pragma pack()

/* Time (in ns) */
#define ECAT_DC_TIME_SET_TIME(pDC, Val)                                 \
    ((ECAT_PBYTE)pDC)[0] = ECAT_LOBYTE(ECAT_LOWORD(Val));               \
    ((ECAT_PBYTE)pDC)[1] = ECAT_HIBYTE(ECAT_LOWORD(Val));               \
    ((ECAT_PBYTE)pDC)[2] = ECAT_LOBYTE(ECAT_HIWORD(Val));               \
    ((ECAT_PBYTE)pDC)[3] = ECAT_HIBYTE(ECAT_HIWORD(Val));

#define ECAT_DC_TIME_GET_TIME(pDC)                                      \
            ECAT_MAKEDWORD(                                             \
                ECAT_MAKEWORD(((ECAT_PBYTE)pDC)[3],                     \
                            ((ECAT_PBYTE)pDC)[2]),                      \
                ECAT_MAKEWORD(((ECAT_PBYTE)pDC)[1],                     \
                            ((ECAT_PBYTE)pDC)[0]))


#define ECAT_INCADDR_2_INDEXPOS(Addr)               (0 - (Addr))
#define ECAT_INDEXPOS_2_INCADDR(Ipos)               (0 - (Ipos))

#define ECAT_CLEAR_SLAVE_STATE(byState)             ((byState) & 0x0F)


/* Distributed Clocks */
/*! @brief Address of Receive Time Channel A register.
    Write access to that register will enable the time stamping of
    the beginning of frame and enables the latches for time stamping at
    the other ports. */
#define ECAT_RADDR_DC_RECV_TIME_A       0x0900
/*! @brief Address of Receive Time Channel B register.
    If Latch enabled, the end of receive frame at Port 1will be time stamped here*/
#define ECAT_RADDR_DC_RECV_TIME_B       0x0904
#define ECAT_RADDR_DC_RECV_TIME_C       0x0908	/* Receive Time Channel C */

/*! @brief Address of System Time register.
    System time at Begin of Frame (start first bit of preamble - measured
    at MII or directly at LVDS Ports)
    Write access to system time will compare value with transmitted time.
    The result is input to the control loop.
    Read access returns the local time.*/
#define ECAT_RADDR_DC_SYS_TIME          0x0910
/*! @brief Address of System time Offset register.
    Offset between local Time and global EtherCAT Time. */
#define ECAT_RADDR_DC_SYS_TIME_OFFS     0x0920
/*! @brief Address of System time Delay register.
    Delay between Slave with Master time and this EtherCAT node. */
#define ECAT_RADDR_DC_SYS_TIME_DELAY    0x0928

/*! @brief Address in Slave Information Interface Area where
           the values we are interested in are located.
    <UL>
    <LI>byte address 0x0010:0x0013 - Vendor Id
    <LI>byte address 0x0014:0x0017 - Product Code
    <LI>byte address 0x0018:0x001B - Revision Number
    <LI>byte address 0x001C:0x001F - Serial Number
    </UL>*/
#define ECAT_EEPROM_SII_DATA_ADDR       0x0010

/*! @brief Size of the SII area we are interested in. */
#define ECAT_EEPROM_SII_DATA_SIZE       (4 * 4)

/*! @brief Address in Slave Information Interface Area where
    Configured Station Alias parameter is located.
    byte address 0x08:0x09 - Alias Address */
#define ECAT_EEPROM_ALIAS_ADDR          0x0008
/*! @brief Length of the Configured Station Alias parameter measured in bytes. */
#define ECAT_EEPROM_ALIAS_SIZE          0x0002

/*! @brief Address in Slave Information Interface Area where
    Checksum parameter is located.
    byte address 0x0E:0x0F - Checksum */
#define ECAT_EEPROM_CHECKSUM_ADDR       0x000E
/*! @brief Length of the Checksum parameter measured in bytes. */
#define ECAT_EEPROM_CHECKSUM_SIZE       0x0002

/*! @brief Address(in Slave Information Interface Area) of the beginning of
           the area to be checksumed.
    byte address 0x00:0x0D - Checksum area */
#define ECAT_EEPROM_CHECKSUM_AREA_ADDR  0x0000
/*! @brief Size of the area to be checksumed. */
#define ECAT_EEPROM_CHECKSUM_AREA_SIZE  0x000E

#pragma pack(1)
/*! @brief Part of Slave Information Interface Area (machine-dependent format).
 */
typedef struct _ECAT_SII_CONST_CONTENT_SHORT
{
    ECAT_DWORD dwVendorId;
    ECAT_DWORD dwProductCode;
    ECAT_DWORD dwRevisionNo;
    ECAT_DWORD dwSerialNo;
} ECAT_SII_CONST_CONTENT_SHORT;
#pragma pack()

#endif /* _ECATCMN_H_ */
