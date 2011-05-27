/** @file
    @brief KPA EtherCAT Slave PCI board constants. */
#ifndef ECATSLAVE_KPAPCIBOARD_DEFS_H
#define ECATSLAVE_KPAPCIBOARD_DEFS_H

/* NOTE: See description of KPA EtherCAT Slave PCI board for
   more details about its address map. */

#define PCI_VENDOR_ID_KPA                           0x10b5
#define PCI_DEVICE_ID_KPA_PCISLAVE                  0x5201
#define PCI_BASE_ADDRESS_REGISTER                   2

/*! @brief For KPA EtherCAT PCI board: ISA offset. */
#define KPA_PCISLV_ESC_ISA_OFFSET                   0xE0000

/*! @brief Offet from the beginning of Board's mappable memory. */
#define KPA_PCISLV_FPGA_BASE_ADDR                   0x08000


#define KPA_PCISLV_IRQ_MASK_REG_FPGAOFFSET          0x02
#define KPA_PCISLV_IRQ_POLATIRY_REG_FPGAOFFSET      0x04
#define KPA_PCISLV_ROTARY_SWITCHES_REG_FPGAOFFSET   0x0C
#define KPA_PCISLV_IRQ_STATUS_REG_FPGAOFFSET        0x0E


#define KPA_PCISLV_IRQ_MASK_REG_BASEOFFSET          (KPA_PCISLV_FPGA_BASE_ADDR + KPA_PCISLV_IRQ_MASK_REG_FPGAOFFSET)
#define KPA_PCISLV_IRQ_POLARITY_REG_BASEOFFSET      (KPA_PCISLV_FPGA_BASE_ADDR + KPA_PCISLV_IRQ_POLATIRY_REG_FPGAOFFSET)
#define KPA_PCISLV_ROTARY_SWITCHES_REG_BASEOFFSET   (KPA_PCISLV_FPGA_BASE_ADDR + KPA_PCISLV_ROTARY_SWITCHES_REG_FPGAOFFSET)
#define KPA_PCISLV_IRQ_STATUS_REG_BASEOFFSET        (KPA_PCISLV_FPGA_BASE_ADDR + KPA_PCISLV_IRQ_STATUS_REG_FPGAOFFSET)


#define KPA_PCISLV_FPGAOFFSET_MAX                   0xFF

/* Size of memory needed for slave */
#define KPA_PCISLV_IOMEM_SIZE                       (KPA_PCISLV_FPGA_BASE_ADDR + KPA_PCISLV_FPGAOFFSET_MAX)


/*--------------------------------------------------------------------------*/
/*! @brief Mask a bit for enabling/disabling KPA slave board IRQ controller.
    If that bit is set to "0" then ALL IRQ sources are
    disabled regardless of other bits.

    @see #ECATSLAVE_IRQMASK_FROM_ESC,  \n
         #ECATSLAVE_IRQMASK_FROM_SYNC0,\n
         #ECATSLAVE_IRQMASK_FROM_SYNC1 */
#define ECATSLAVE_IRQMASK_ENABLE_ALL                0x01

/*! @brief Mask for bit "IRQ from ESC"
    @see @ref ECATSLAVE_IRQMASK "other IRQ masks" */
#define ECATSLAVE_IRQMASK_FROM_ESC                  0x02

/*! @brief Mask for bit "IRQ from SYNC0 signal"
    @see @ref ECATSLAVE_IRQMASK "other IRQ masks" */
#define ECATSLAVE_IRQMASK_FROM_SYNC0                0x04

/*! @brief Mask for bit "IRQ from SYNC1 signal"
    @see @ref ECATSLAVE_IRQMASK "other IRQ masks" */
#define ECATSLAVE_IRQMASK_FROM_SYNC1                0x08


#endif /* ECATSLAVE_KPAPCIBOARD_DEFS_H */
