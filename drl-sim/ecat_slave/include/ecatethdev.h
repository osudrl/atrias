/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2008
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

*****************************************************************************/
/*! @file ecatethdev.h
    @brief This file contains declarations of Ethernet virtual device structures and functions.*/
/****************************************************************************/

#ifndef TAPDEV_IMPL_H
#define TAPDEV_IMPL_H

#include "ecatslavetypes.h"

typedef struct tagEcatEoETunnel* ECAT_EOE_TUNNEL_PTR;

/*! @brief Send callback type.
    @param dev Pointer to EoE tunnel.
    @param len Frame length.
    @param data Pointer to Ethernet frame.
    @return @ref ECAT_RESULT "Result code".*/
typedef ECAT_RESULT (*EthDevSendCallBack)(ECAT_EOE_TUNNEL_PTR pTun, ECAT_WORD len, ECAT_BYTE* data);

/*! @brief Ethernet virtual network interface type definition.*/
typedef struct tagEcatEthernetDevice
{
    //System specific data

    EthDevSendCallBack  send_callbak;
    ECAT_EOE_TUNNEL_PTR owner;
} EcatEthernetDevice;

/*! @brief EoE tunnel calls this function when virtual network interface has to be activated.
    @param dev Pointer to device.
    @return Less then 0 if error occurred.*/
int EcatEthernetDeviceUp(EcatEthernetDevice *dev);

/*! @brief EoE tunnel calls this function when virtual network interface has to be deactivated.
    @param dev Pointer to device.
    @return Less then 0 if error occurred.*/
int EcatEthernetDeviceDown(EcatEthernetDevice *dev);

/*! @brief EoE tunnel calls this function when new recived Ethernet frame is available.
    @param dev Pointer to device.
    @param len Frame length.
    @param data Pointer to received Ethernet frame.
    @return Less then 0 if error occurred.*/
int EcatEthernetDeviceRecive(EcatEthernetDevice *dev, int len, void *data);

/*! @brief Sets pointer to EoE tunnel which owns this virtual device.
    This pointer has to be passed as send-callback function argument (@see EthDevSendCallBack).
    @param dev Pointer to device.
    @param owner Pointer to EoE tunnel.*/
void EcatEthernetDeviceSetOwner(EcatEthernetDevice *dev, ECAT_EOE_TUNNEL_PTR owner);

/*! @brief Gets virtual network interface MAC address.
    @param dev Pointer to device.
    @param addr Pointer to MAC address.
    @param len Max address length.
    @return Less then 0 if error occured.*/
int EcatEthernetDeviceGetHwAddr(EcatEthernetDevice *dev, void *addr, int len);

/*! @brief Sets callback function which virtual interface calls when it has new frame to send.
    @param dev Pointer to device.
    @param call Callback function.*/
void EcatEthernetDeviceSetSendCallBack(EcatEthernetDevice *dev, EthDevSendCallBack call);

#endif
