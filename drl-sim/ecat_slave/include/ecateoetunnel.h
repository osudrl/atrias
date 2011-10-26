/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2008
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecateoetunnel.h
    @brief This file contains some declarations and definitions of
           EoE tunnel structures and functions.*/
/****************************************************************************/


#ifndef ECAT_EOE_TUNNEL_H
#define ECAT_EOE_TUNNEL_H

#include "ecatslaveconfig.h"
#include "ecatslavetypes.h"
#include "ecatcmn.h"

#if (ECATSLAVE_CONFIG_ENABLE_EOE != 0)
#if (ECATSLAVE_CONFIG_ENABLE_EOE_API != 0)

/*! @brief Ethernet over ethercat tunnel.*/
typedef struct tagEcatEoETunnel
{
    void                       *pContextData;
    tEcatEoENICUp               pfnUp;
    tEcatEoENICDown             pfnDown;
    tEcatEoENICIPSettingsSet    pfnIPSettingsSet;
    tEcatEoENICGetHwAddr        pfnGetHwAddr;
    tEcatEoENICRecive           pfnReceive;

    void*           pParent;
    ECAT_WORD       wUsersCount;
    ECAT_MACADDR    HwAddr;
} EcatEoETunnel;

/*! @brief Virtual interface calls it when it has new frame to send.
    @param pEoETunnel Pointer to EoE tunnel.
    @param pbyData Pointer to Ethernet frame.
    @param wLen Frame length.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT EcatEoENICSend(
    IN EcatEoETunnel   *pEoETunnel,
    IN ECAT_BYTE       *pbyData,
    IN ECAT_WORD        wLen);

/*! @brief Register virtual network interface callbacks.
    @param pEoETunnel Pointer to EoE tunnel.
    @param pContextData Pointer to user context data.
    @param pfnUp Pointer to virtual NIC up callback.
    @param pfnDown Pointer to virtual NIC down callback.
    @param pfnIPSettingsSet Pointer to set virtual NIC IP settings callback.
    @param pfnGetHwAddr Pointer to get virtual NIC MAC address callback.
    @param pfnReceive Pointer to virtual NIC receive callback.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT EcatEoENICRegisterInterface(
    IN EcatEoETunnel           *pEoETunnel,
    IN void                    *pContextData,
    IN tEcatEoENICUp            pfnUp,
    IN tEcatEoENICDown          pfnDown,
    IN tEcatEoENICIPSettingsSet pfnIPSettingsSet,
    IN tEcatEoENICGetHwAddr     pfnGetHwAddr,
    IN tEcatEoENICRecive        pfnReceive);

/*! @brief Unregister virtual network interface callbacks.
    @param pEoETunnel Pointer to EoE tunnel.
    @return @ref ECAT_RESULT "Result code".*/
ECAT_RESULT EcatEoENICUnRegisterInterface(
    IN EcatEoETunnel           *pEoETunnel);



ECAT_RESULT EcatEoETunnelInit(EcatEoETunnel *pEoETunnel, void  *pParent);
ECAT_RESULT EcatEoETunnelUnInit(EcatEoETunnel *pEoETunnel);
ECAT_RESULT EcatEoETunnelReceive(EcatEoETunnel *pEoETunnel, ECAT_WORD Lengs, ECAT_BYTE *pData);
ECAT_RESULT EcatEoETunnelSettingsChange(
    IN EcatEoETunnel   *pEoETunnel,
    IN ECAT_BYTE        byPort,
    IN ECAT_DWORD       dwFlags,
    IN ECAT_MACADDR_PTR pMac,
    IN ECAT_DWORD       dwIpAddr,
    IN ECAT_DWORD       dwSubNetMask,
    IN ECAT_DWORD       dwDefGateway,
    IN ECAT_DWORD       dwDnsIpAddr,
    IN ECAT_CHAR       *pszDnsName);
ECAT_RESULT EcatEoETunnelSend(EcatEoETunnel* pEoETunnel, ECAT_WORD Lengs, ECAT_BYTE* pData);
ECAT_RESULT EcatEoETunnelDevUp(EcatEoETunnel *pEoETunnel);
ECAT_RESULT EcatEoETunnelDevDown(EcatEoETunnel *pEoETunnel);
ECAT_BOOL EcatEoETunnelIsReady(EcatEoETunnel *pEoETunnel);

#else

#include "ecatethdev.h"

/*! @brief Ethernet over ethercat tunnel.*/
typedef struct tagEcatEoETunnel
{
    EcatEthernetDevice  EthDev;
    ECAT_WORD           wUsersCount;
    void*               pParent;
    ECAT_MACADDR        HwAddr;
} EcatEoETunnel;

#ifdef __cplusplus
extern "C" {
#endif
ECAT_RESULT EcatEoETunnelInit(EcatEoETunnel *pEoETunnel, void  *pParent);
ECAT_RESULT EcatEoETunnelUnInit(EcatEoETunnel *pEoETunnel);
ECAT_RESULT EcatEoETunnelReceive(EcatEoETunnel *pEoETunnel, ECAT_WORD Lengs, ECAT_BYTE *pData);
ECAT_RESULT EcatEoETunnelSend(EcatEoETunnel* pEoETunnel, ECAT_WORD Lengs, ECAT_BYTE* pData);
ECAT_RESULT EcatEoETunnelDevUp(EcatEoETunnel *pEoETunnel);
ECAT_RESULT EcatEoETunnelDevDown(EcatEoETunnel *pEoETunnel);
ECAT_WORD EcatEoETunnelIsReady(EcatEoETunnel *pEoETunnel);
#ifdef __cplusplus
}
#endif

#endif /*ECATSLAVE_CONFIG_ENABLE_EOE_API*/
/*--------------------------------------------------------------------------*/
#else /* here (ECATSLAVE_CONFIG_ENABLE_EOE == 0) */

typedef struct tagEcatEoETunnel
{
    /* Attention!!! This is a meaningless dummy field just for compilers that
       do not understand empty structures!!! */
    int DummyField;
} EcatEoETunnel;

#ifdef __cplusplus
extern "C" {
#endif
ECAT_INLINE ECAT_RESULT EcatEoETunnelInit(EcatEoETunnel *pEoETunnel, void  *pParent)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_RESULT EcatEoETunnelUnInit(EcatEoETunnel *pEoETunnel)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_RESULT EcatEoETunnelReceive(
    EcatEoETunnel*  pEoETunnel,
    ECAT_WORD       Lengs,
    ECAT_BYTE*      pData)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_RESULT EcatEoETunnelSend(
    EcatEoETunnel*  pEoETunnel,
    ECAT_WORD       Lengs,
    ECAT_BYTE*      pData)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_RESULT EcatEoETunnelDevUp(EcatEoETunnel *pEoETunnel)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_RESULT EcatEoETunnelDevDown(EcatEoETunnel *pEoETunnel)
{
    return ECAT_E_FAIL;
}

ECAT_INLINE ECAT_WORD EcatEoETunnelIsReady(EcatEoETunnel *pEoETunnel)
{
    return 0;
}
#ifdef __cplusplus
}
#endif

#endif /* #if (ECATSLAVE_CONFIG_ENABLE_EOE != 0) */

#endif /* ECAT_EOE_TUNNEL_H */
