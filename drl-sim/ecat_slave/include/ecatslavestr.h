/*****************************************************************************
            EtherCAT Slave

    Copyright (c) 2004-2009
    Koenig Prozessautomatisierungs GmbH.
    Visutech System Ltd.
    All rights reserved.

******************************************************************************
    String-related definitions, etc.
*****************************************************************************/

#ifndef ECATSLAVESTR_H
#define ECATSLAVESTR_H

#ifdef __KERNEL__

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#   include <linux/autoconf.h>
#else
#   include <linux/config.h>
#endif
#include <linux/string.h>

#else

#include <string.h>

#endif

#endif /* ECATSLAVESTR_H */
