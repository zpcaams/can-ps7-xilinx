/*
 * xcanps.h -  CAN PS7 Zynq network device driver
 *
 */

#ifndef CAN_XILINX_PS7_H	/* prevent circular inclusions */
#define CAN_XILINX_PS7_H	/* by using protection macros */


/***************************** Include Files *********************************/

#include <linux/irqreturn.h>
#include <linux/can/dev.h>
#include "xil_types.h"
#include "xstatus.h"
//#include <linux/can/platform/xcanps.h>

/************************** Constant Definitions *****************************/

#define SJA1000_ECHO_SKB_MAX	1 /* the SJA1000 has one TX buffer object */

#define SJA1000_MAX_IRQ 20	/* max. number of interrupts handled in ISR */

#define XCANPS_MAX_FRAME_SIZE_IN_WORDS (XCANPS_MAX_FRAME_SIZE / sizeof(u32))

#define FRAME_DATA_LENGTH	8 /* Frame Data field length */

/** @name Register offsets for the CAN. Each register is 32 bits.
 *  @{
 */
#define XCANPS_SRR_OFFSET	  	0x00 /**< Software Reset Register */
#define XCANPS_MSR_OFFSET	  	0x04 /**< Mode Select Register */
#define XCANPS_BRPR_OFFSET	  	0x08 /**< Baud Rate Prescaler */
#define XCANPS_BTR_OFFSET	  	0x0C /**< Bit Timing Register */
#define XCANPS_ECR_OFFSET	  	0x10 /**< Error Counter Register */
#define XCANPS_ESR_OFFSET	  	0x14 /**< Error Status Register */
#define XCANPS_SR_OFFSET	  	0x18 /**< Status Register */

#define XCANPS_ISR_OFFSET	  	0x1C /**< Interrupt Status Register */
#define XCANPS_IER_OFFSET	  	0x20 /**< Interrupt Enable Register */
#define XCANPS_ICR_OFFSET	  	0x24 /**< Interrupt Clear Register */
#define XCANPS_TCR_OFFSET	  	0x28 /**< Timestamp Control Register */
#define XCANPS_WIR_OFFSET	  	0x2C /**< Watermark Interrupt Reg */

#define XCANPS_TXFIFO_ID_OFFSET  	0x30 /**< TX FIFO ID */
#define XCANPS_TXFIFO_DLC_OFFSET 	0x34 /**< TX FIFO DLC */
#define XCANPS_TXFIFO_DW1_OFFSET	0x38 /**< TX FIFO Data Word 1 */
#define XCANPS_TXFIFO_DW2_OFFSET 	0x3C /**< TX FIFO Data Word 2 */

#define XCANPS_TXHPB_ID_OFFSET   	0x40 /**< TX High Priority Buffer ID */
#define XCANPS_TXHPB_DLC_OFFSET  	0x44 /**< TX High Priority Buffer DLC */
#define XCANPS_TXHPB_DW1_OFFSET  	0x48 /**< TX High Priority Buf Data 1 */
#define XCANPS_TXHPB_DW2_OFFSET  	0x4C /**< TX High Priority Buf Data Word 2 */

#define XCANPS_RXFIFO_ID_OFFSET  	0x50 /**< RX FIFO ID */
#define XCANPS_RXFIFO_DLC_OFFSET 	0x54 /**< RX FIFO DLC */
#define XCANPS_RXFIFO_DW1_OFFSET 	0x58 /**< RX FIFO Data Word 1 */
#define XCANPS_RXFIFO_DW2_OFFSET 	0x5C /**< RX FIFO Data Word 2 */

#define XCANPS_AFR_OFFSET	  	0x60 /**< Acceptance Filter Register */
#define XCANPS_AFMR1_OFFSET	  	0x64 /**< Acceptance Filter Mask 1 */
#define XCANPS_AFIR1_OFFSET	  	0x68 /**< Acceptance Filter ID  1 */
#define XCANPS_AFMR2_OFFSET	  	0x6C /**< Acceptance Filter Mask  2 */
#define XCANPS_AFIR2_OFFSET	  	0x70 /**< Acceptance Filter ID 2 */
#define XCANPS_AFMR3_OFFSET	  	0x74 /**< Acceptance Filter Mask 3 */
#define XCANPS_AFIR3_OFFSET	  	0x78 /**< Acceptance Filter ID 3 */
#define XCANPS_AFMR4_OFFSET	  	0x7C /**< Acceptance Filter Mask  4 */
#define XCANPS_AFIR4_OFFSET	  	0x80 /**< Acceptance Filter ID 4 */
/* @} */

/** @name Software Reset Register (SRR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_SRR_CEN_MASK	0x00000002  /**< Can Enable */
#define XCANPS_SRR_SRST_MASK	0x00000001  /**< Reset */
/* @} */

/** @name Mode Select Register (MSR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_MSR_SNOOP_MASK	0x00000004 /**< Snoop Mode Select */
#define XCANPS_MSR_LBACK_MASK	0x00000002 /**< Loop Back Mode Select */
#define XCANPS_MSR_SLEEP_MASK	0x00000001 /**< Sleep Mode Select */
/* @} */

/** @name Baud Rate Prescaler register (BRPR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_BRPR_BRP_MASK	0x000000FF /**< Baud Rate Prescaler */
/* @} */

/** @name Bit Timing Register (BTR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_BTR_SJW_MASK	0x00000180 /**< Synchronization Jump Width */
#define XCANPS_BTR_SJW_SHIFT	7
#define XCANPS_BTR_TS2_MASK	0x00000070 /**< Time Segment 2 */
#define XCANPS_BTR_TS2_SHIFT	4
#define XCANPS_BTR_TS1_MASK	0x0000000F /**< Time Segment 1 */
/* @} */

/** @name Error Counter Register (ECR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_ECR_REC_MASK	0x0000FF00 /**< Receive Error Counter */
#define XCANPS_ECR_REC_SHIFT	8
#define XCANPS_ECR_TEC_MASK	0x000000FF /**< Transmit Error Counter */
/* @} */

/** @name Error Status Register (ESR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_ESR_ACKER_MASK	0x00000010 /**< ACK Error */
#define XCANPS_ESR_BERR_MASK	0x00000008 /**< Bit Error */
#define XCANPS_ESR_STER_MASK	0x00000004 /**< Stuff Error */
#define XCANPS_ESR_FMER_MASK	0x00000002 /**< Form Error */
#define XCANPS_ESR_CRCER_MASK	0x00000001 /**< CRC Error */
/* @} */

/** @name Status Register (SR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_SR_SNOOP_MASK	0x00001000 /**< Snoop Mask */
#define XCANPS_SR_ACFBSY_MASK	0x00000800 /**< Acceptance Filter busy */
#define XCANPS_SR_TXFLL_MASK	0x00000400 /**< TX FIFO is full */
#define XCANPS_SR_TXBFLL_MASK	0x00000200 /**< TX High Priority Buffer full */
#define XCANPS_SR_ESTAT_MASK	0x00000180 /**< Error Status */
#define XCANPS_SR_ESTAT_SHIFT	7
#define XCANPS_SR_ERRWRN_MASK	0x00000040 /**< Error Warning */
#define XCANPS_SR_BBSY_MASK	0x00000020 /**< Bus Busy */
#define XCANPS_SR_BIDLE_MASK	0x00000010 /**< Bus Idle */
#define XCANPS_SR_NORMAL_MASK	0x00000008 /**< Normal Mode */
#define XCANPS_SR_SLEEP_MASK	0x00000004 /**< Sleep Mode */
#define XCANPS_SR_LBACK_MASK	0x00000002 /**< Loop Back Mode */
#define XCANPS_SR_CONFIG_MASK	0x00000001 /**< Configuration Mode */
/* @} */

/** @name Interrupt Status/Enable/Clear Register Bit Definitions and Masks
 *  @{
 */
#define XCANPS_IXR_TXFEMP_MASK   0x00004000 /**< Tx Fifo Empty Interrupt */
#define XCANPS_IXR_TXFWMEMP_MASK 0x00002000 /**< Tx Fifo Watermark Empty */
#define XCANPS_IXR_RXFWMFLL_MASK 0x00001000 /**< Rx FIFO Watermark Full */
#define XCANPS_IXR_WKUP_MASK	0x00000800 /**< Wake up Interrupt */
#define XCANPS_IXR_SLP_MASK	0x00000400 /**< Sleep Interrupt */
#define XCANPS_IXR_BSOFF_MASK	0x00000200 /**< Bus Off Interrupt */
#define XCANPS_IXR_ERROR_MASK	0x00000100 /**< Error Interrupt */
#define XCANPS_IXR_RXNEMP_MASK	0x00000080 /**< RX FIFO Not Empty Interrupt */
#define XCANPS_IXR_RXOFLW_MASK	0x00000040 /**< RX FIFO Overflow Interrupt */
#define XCANPS_IXR_RXUFLW_MASK	0x00000020 /**< RX FIFO Underflow Interrupt */
#define XCANPS_IXR_RXOK_MASK	0x00000010 /**< New Message Received Intr */
#define XCANPS_IXR_TXBFLL_MASK	0x00000008 /**< TX High Priority Buf Full */
#define XCANPS_IXR_TXFLL_MASK	0x00000004 /**< TX FIFO Full Interrupt */
#define XCANPS_IXR_TXOK_MASK	0x00000002 /**< TX Successful Interrupt */
#define XCANPS_IXR_ARBLST_MASK	0x00000001 /**< Arbitration Lost Interrupt */
#define XCANPS_IXR_ALL		(XCANPS_IXR_RXFWMFLL_MASK | \
				XCANPS_IXR_WKUP_MASK   | \
				XCANPS_IXR_SLP_MASK	| \
				XCANPS_IXR_BSOFF_MASK  | \
				XCANPS_IXR_ERROR_MASK  | \
				XCANPS_IXR_RXNEMP_MASK | \
 				XCANPS_IXR_RXOFLW_MASK | \
				XCANPS_IXR_RXUFLW_MASK | \
	 			XCANPS_IXR_RXOK_MASK   | \
				XCANPS_IXR_TXBFLL_MASK | \
				XCANPS_IXR_TXFLL_MASK  | \
				XCANPS_IXR_TXOK_MASK   | \
				XCANPS_IXR_ARBLST_MASK)
/* @} */

/** @name CAN Timestamp Control Register (TCR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_TCR_CTS_MASK	0x00000001 /**< Clear Timestamp counter mask */
/* @} */

/** @name CAN Watermark Register (WIR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_WIR_FW_MASK   	0x0000003F /**< Rx Full Threshold mask */
#define XCANPS_WIR_EW_MASK 	0x00003F00 /**< Tx Empty Threshold mask */
#define XCANPS_WIR_EW_SHIFT 	0x00000008 /**< Tx Empty Threshold shift */

/* @} */

/** @name CAN Frame Identifier (TX High Priority Buffer/TX/RX/Acceptance Filter
				Mask/Acceptance Filter ID)
 *  @{
 */
#define XCANPS_IDR_ID1_MASK	0xFFE00000 /**< Standard Messg Identifier */
#define XCANPS_IDR_ID1_SHIFT	21
#define XCANPS_IDR_SRR_MASK	0x00100000 /**< Substitute Remote TX Req */
#define XCANPS_IDR_SRR_SHIFT	20
#define XCANPS_IDR_IDE_MASK	0x00080000 /**< Identifier Extension */
#define XCANPS_IDR_IDE_SHIFT	19
#define XCANPS_IDR_ID2_MASK	0x0007FFFE /**< Extended Message Ident */
#define XCANPS_IDR_ID2_SHIFT	1
#define XCANPS_IDR_RTR_MASK	0x00000001 /**< Remote TX Request */
/* @} */

/** @name CAN Frame Data Length Code (TX High Priority Buffer/TX/RX)
 *  @{
 */
#define XCANPS_DLCR_DLC_MASK	 0xF0000000	/**< Data Length Code */
#define XCANPS_DLCR_DLC_SHIFT	 28
#define XCANPS_DLCR_TIMESTAMP_MASK 0x0000FFFF	/**< Timestamp Mask (Rx only) */

/* @} */

/** @name CAN Frame Data Word 1 (TX High Priority Buffer/TX/RX)
 *  @{
 */
#define XCANPS_DW1R_DB0_MASK	0xFF000000 /**< Data Byte 0 */
#define XCANPS_DW1R_DB0_SHIFT	24
#define XCANPS_DW1R_DB1_MASK	0x00FF0000 /**< Data Byte 1 */
#define XCANPS_DW1R_DB1_SHIFT	16
#define XCANPS_DW1R_DB2_MASK	0x0000FF00 /**< Data Byte 2 */
#define XCANPS_DW1R_DB2_SHIFT	8
#define XCANPS_DW1R_DB3_MASK	0x000000FF /**< Data Byte 3 */
/* @} */

/** @name CAN Frame Data Word 2 (TX High Priority Buffer/TX/RX)
 *  @{
 */
#define XCANPS_DW2R_DB4_MASK	0xFF000000 /**< Data Byte 4 */
#define XCANPS_DW2R_DB4_SHIFT	24
#define XCANPS_DW2R_DB5_MASK	0x00FF0000 /**< Data Byte 5 */
#define XCANPS_DW2R_DB5_SHIFT	16
#define XCANPS_DW2R_DB6_MASK	0x0000FF00 /**< Data Byte 6 */
#define XCANPS_DW2R_DB6_SHIFT	8
#define XCANPS_DW2R_DB7_MASK	0x000000FF /**< Data Byte 7 */
/* @} */

/** @name Acceptance Filter Register (AFR) Bit Definitions and Masks
 *  @{
 */
#define XCANPS_AFR_UAF4_MASK	0x00000008 /**< Use Acceptance Filter No.4 */
#define XCANPS_AFR_UAF3_MASK	0x00000004 /**< Use Acceptance Filter No.3 */
#define XCANPS_AFR_UAF2_MASK	0x00000002 /**< Use Acceptance Filter No.2 */
#define XCANPS_AFR_UAF1_MASK	0x00000001 /**< Use Acceptance Filter No.1 */
#define XCANPS_AFR_UAF_ALL_MASK	(XCANPS_AFR_UAF4_MASK | \
					XCANPS_AFR_UAF3_MASK | \
					XCANPS_AFR_UAF2_MASK | \
					XCANPS_AFR_UAF1_MASK)
/* @} */

/** @name CAN frame length constants
 *  @{
 */
#define XCANPS_MAX_FRAME_SIZE 16 /**< Maximum CAN frame length in bytes */
/* @} */

/* For backwards compatibilty */
#define XCANPS_TXBUF_ID_OFFSET   XCANPS_TXHPB_ID_OFFSET
#define XCANPS_TXBUF_DLC_OFFSET  XCANPS_TXHPB_DLC_OFFSET
#define XCANPS_TXBUF_DW1_OFFSET  XCANPS_TXHPB_DW1_OFFSET
#define XCANPS_TXBUF_DW2_OFFSET  XCANPS_TXHPB_DW2_OFFSET

#define XCANPS_RXFWIR_RXFLL_MASK XCANPS_WIR_FW_MASK
#define XCANPS_RXWIR_OFFSET 	 XCANPS_WIR_OFFSET
#define XCANPS_IXR_RXFLL_MASK 	 XCANPS_IXR_RXFWMFLL_MASK

/** @name CAN operation modes
 *  @{
 */
#define XCANPS_MODE_CONFIG	0x00000001 /**< Configuration mode */
#define XCANPS_MODE_NORMAL	0x00000002 /**< Normal mode */
#define XCANPS_MODE_LOOPBACK	0x00000004 /**< Loop Back mode */
#define XCANPS_MODE_SLEEP	0x00000008 /**< Sleep mode */
#define XCANPS_MODE_SNOOP	0x00000010 /**< Snoop mode */
/* @} */

/** @name Callback identifiers used as parameters to XCanPs_SetHandler()
 *  @{
 */
#define XCANPS_HANDLER_SEND 1 /**< Handler type for frame sending interrupt */
#define XCANPS_HANDLER_RECV 2 /**< Handler type for frame reception interrupt*/
#define XCANPS_HANDLER_ERROR  3 /**< Handler type for error interrupt */
#define XCANPS_HANDLER_EVENT  4 /**< Handler type for all other interrupts */
/* @} */

/****************************************************************************/
/**
*
* This macro reads the given register.
*
* @param	BaseAddr is the base address of the device.
* @param	RegOffset is the register offset to be read.
*
* @return	The 32-bit value of the register
*
* @note		None.
*
*****************************************************************************/
#define XCanPs_ReadReg(BaseAddr, RegOffset) \
		ioread32((BaseAddr) + (RegOffset))

/****************************************************************************/
/**
*
* This macro writes the given register.
*
* @param	BaseAddr is the base address of the device.
* @param	RegOffset is the register offset to be written.
* @param	Data is the 32-bit value to write to the register.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
#define XCanPs_WriteReg(BaseAddr, RegOffset, Data) \
		iowrite32((Data), (BaseAddr) + (RegOffset))


/**************************** Type Definitions *******************************/
/**
 * This typedef contains configuration information for a device.
 */
typedef struct {
	u16 DeviceId;		/**< Unique ID of device */
	u32 BaseAddr;		/**< Register base address */
} XCanPs_Config;

/******************************************************************************/
/**
 * Callback type for frame sending and reception interrupts.
 *
 * @param 	CallBackRef is a callback reference passed in by the upper layer
 *		when setting the callback functions, and passed back to the
 *		upper layer when the callback is invoked.
*******************************************************************************/
typedef void (*XCanPs_SendRecvHandler) (void *CallBackRef);

/******************************************************************************/
/**
 * Callback type for error interrupt.
 *
 * @param	CallBackRef is a callback reference passed in by the upper layer
 *		when setting the callback functions, and passed back to the
 *		upper layer when the callback is invoked.
 * @param	ErrorMask is a bit mask indicating the cause of the error. Its
 *		value equals 'OR'ing one or more XCANPS_ESR_* values defined in
 *		xcanps_hw.h
*******************************************************************************/
typedef void (*XCanPs_ErrorHandler) (void *CallBackRef, u32 ErrorMask);

/******************************************************************************/
/**
 * Callback type for all kinds of interrupts except sending frame interrupt,
 * receiving frame interrupt, and error interrupt.
 *
 * @param	CallBackRef is a callback reference passed in by the upper layer
 *		when setting the callback functions, and passed back to the
 *		upper layer when the callback is invoked.
 * @param	Mask is a bit mask indicating the pending interrupts. Its value
 *		equals 'OR'ing one or more XCANPS_IXR_* defined in xcanps_hw.h
*******************************************************************************/
typedef void (*XCanPs_EventHandler) (void *CallBackRef, u32 Mask);

/*
 * Flags for xcanpspriv.flags
 */
#define SJA1000_CUSTOM_IRQ_HANDLER 0x1

/*
 * SJA1000 private data structure
 */
struct xcanps_priv {
	struct can_priv can;	/* must be the first member */
	int open_time;
	struct sk_buff *echo_skb;

	/* the lower-layer is responsible for appropriate locking */
	u32 (*read_reg) (const struct xcanps_priv *priv, int reg);
	void (*write_reg) (const struct xcanps_priv *priv, int reg, u32 val);
	void (*pre_irq) (const struct xcanps_priv *priv);
	void (*post_irq) (const struct xcanps_priv *priv);

	void *priv;		/* for board-specific data */
	struct net_device *dev;

	void __iomem *reg_base;	 /* ioremap'ed address to registers */
	unsigned long irq_flags; /* for request_irq() */
	spinlock_t cmdreg_lock;  /* lock for concurrent cmd register writes */

	u16 flags;		/* custom mode flags */
	XCanPs_Config CanConfig; 	/**< Device configuration */

	/**
	 * Callback and callback reference for TXOK interrupt.
	 */
	XCanPs_SendRecvHandler SendHandler;
	void *SendRef;

	/**
	 * Callback and callback reference for RXOK/RXNEMP/RXFLL interrupts.
	 */
	XCanPs_SendRecvHandler RecvHandler;
	void *RecvRef;

	/**
	 * Callback and callback reference for ERROR interrupt.
	 */
	XCanPs_ErrorHandler ErrorHandler;
	void *ErrorRef;

	/**
	 * Callback  and callback reference for RXOFLW/RXUFLW/TXBFLL/TXFLL/
	 * Wakeup/Sleep/Bus off/ARBLST interrupts.
	 */
	XCanPs_EventHandler EventHandler;
	void *EventRef;
};

/****************************************************************************/
/**
*
* This macro checks if the transmission is complete.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- TRUE if the transmission is done.
*		- FALSE if the transmission is not done.
*
* @note		C-Style signature:
*		int XCanPs_IsTxDone(XCanPs *InstancePtr);
*
*******************************************************************************/
#define XCanPs_IsTxDone(InstancePtr) \
	((XCanPs_ReadReg(((InstancePtr)->CanConfig.BaseAddr),		\
		XCANPS_ISR_OFFSET) & XCANPS_IXR_TXOK_MASK) ? TRUE : FALSE)


/****************************************************************************/
/**
*
* This macro checks if the transmission FIFO is full.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- TRUE if TX FIFO is full.
*		- FALSE if the TX FIFO is NOT full.
*
* @note		C-Style signature:
*		int XCanPs_IsTxFifoFull(XCanPs *InstancePtr);
*
*****************************************************************************/
#define XCanPs_IsTxFifoFull(InstancePtr) \
	((XCanPs_ReadReg(((InstancePtr)->CanConfig.BaseAddr), 	\
		XCANPS_SR_OFFSET) & XCANPS_SR_TXFLL_MASK) ? TRUE : FALSE)


/****************************************************************************/
/**
*
* This macro checks if the Transmission High Priority Buffer is full.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- TRUE if the TX High Priority Buffer is full.
*		- FALSE if the TX High Priority Buffer is NOT full.
*
* @note		C-Style signature:
*		int XCanPs_IsHighPriorityBufFull(XCanPs *InstancePtr);
*
*****************************************************************************/
#define XCanPs_IsHighPriorityBufFull(InstancePtr) \
	((XCanPs_ReadReg(((InstancePtr)->CanConfig.BaseAddr), 	\
		XCANPS_SR_OFFSET) & XCANPS_SR_TXBFLL_MASK) ? TRUE : FALSE)


/****************************************************************************/
/**
*
* This macro checks if the receive FIFO is empty.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- TRUE if RX FIFO is empty.
*		- FALSE if the RX FIFO is NOT empty.
*
* @note		C-Style signature:
*		int XCanPs_IsRxEmpty(XCanPs *InstancePtr);
*
*****************************************************************************/
#define XCanPs_IsRxEmpty(InstancePtr) \
	((XCanPs_ReadReg(((InstancePtr)->CanConfig.BaseAddr), 	\
		XCANPS_ISR_OFFSET) & XCANPS_IXR_RXNEMP_MASK) ? FALSE : TRUE)


/****************************************************************************/
/**
*
* This macro checks if the CAN device is ready for the driver to change
* Acceptance Filter Identifier Registers (AFIR) and Acceptance Filter Mask
* Registers (AFMR).
*
* AFIR and AFMR for a filter are changeable only after the filter is disabled
* and this routine returns FALSE. The filter can be disabled using the
* XCanPs_AcceptFilterDisable function.
*
* Use the XCanPs_Accept_* functions for configuring the acceptance filters.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- TRUE if the device is busy and NOT ready to accept writes to
*		AFIR and AFMR.
*		- FALSE if the device is ready to accept writes to AFIR and
*		AFMR.
*
* @note		C-Style signature:
*		int XCanPs_IsAcceptFilterBusy(XCanPs *InstancePtr);
*
*****************************************************************************/
#define XCanPs_IsAcceptFilterBusy(InstancePtr) 		\
	((XCanPs_ReadReg(((InstancePtr)->CanConfig.BaseAddr), 	\
		XCANPS_SR_OFFSET) & XCANPS_SR_ACFBSY_MASK) ? TRUE : FALSE)


/****************************************************************************/
/**
*
* This macro calculates CAN message identifier value given identifier field
* values.
*
* @param	StandardId contains Standard Message ID value.
* @param	SubRemoteTransReq contains Substitute Remote Transmission
*		Request value.
* @param	IdExtension contains Identifier Extension value.
* @param	ExtendedId contains Extended Message ID value.
* @param	RemoteTransReq contains Remote Transmission Request value.
*
* @return	Message Identifier value.
*
* @note		C-Style signature:
*		u32 XCanPs_CreateIdValue(u32 StandardId,
*					u32 SubRemoteTransReq,
*					u32 IdExtension, u32 ExtendedId,
*					u32 RemoteTransReq);
*
*		Read the CAN specification for meaning of each parameter.
*
*****************************************************************************/
#define XCanPs_CreateIdValue(StandardId, SubRemoteTransReq, IdExtension, \
		ExtendedId, RemoteTransReq) 				\
 ((((StandardId) << XCANPS_IDR_ID1_SHIFT) & XCANPS_IDR_ID1_MASK) |	\
 (((SubRemoteTransReq) << XCANPS_IDR_SRR_SHIFT) & XCANPS_IDR_SRR_MASK)|\
 (((IdExtension) << XCANPS_IDR_IDE_SHIFT) & XCANPS_IDR_IDE_MASK) |	\
 (((ExtendedId) << XCANPS_IDR_ID2_SHIFT) & XCANPS_IDR_ID2_MASK) |	\
 ((RemoteTransReq) & XCANPS_IDR_RTR_MASK))


/****************************************************************************/
/**
*
* This macro calculates value for Data Length Code register given Data
* Length Code value.
*
* @param	DataLengCode indicates Data Length Code value.
*
* @return	Value that can be assigned to Data Length Code register.
*
* @note		C-Style signature:
*		u32 XCanPs_CreateDlcValue(u32 DataLengCode);
*
*		Read the CAN specification for meaning of Data Length Code.
*
*****************************************************************************/
#define XCanPs_CreateDlcValue(DataLengCode) \
	(((DataLengCode) << XCANPS_DLCR_DLC_SHIFT) & XCANPS_DLCR_DLC_MASK)


/****************************************************************************/
/**
*
* This macro clears the timestamp in the Timestamp Control Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	None.
*
* @note		C-Style signature:
*		void XCanPs_ClearTimestamp(XCanPs *InstancePtr);
*
*****************************************************************************/
#define XCanPs_ClearTimestamp(InstancePtr) 			\
	XCanPs_WriteReg((InstancePtr)->CanConfig.BaseAddr, 		\
				XCANPS_TCR_OFFSET, XCANPS_TCR_CTS_MASK)


/************************** Function Prototypes ******************************/

/*
 * Functions in xcanps.c
 */
int XCanPs_CfgInitialize(struct xcanps_priv *InstancePtr, XCanPs_Config *ConfigPtr,
				u32 EffectiveAddr);

void XCanPs_Reset(struct xcanps_priv *InstancePtr);
u8 XCanPs_GetMode(struct xcanps_priv *InstancePtr);
void XCanPs_EnterMode(struct xcanps_priv *InstancePtr, u8 OperationMode);
u32 XCanPs_GetStatus(struct xcanps_priv *InstancePtr);
void XCanPs_GetBusErrorCounter(struct xcanps_priv *InstancePtr, u8 *RxErrorCount,
				 u8 *TxErrorCount);
u32 XCanPs_GetBusErrorStatus(struct xcanps_priv *InstancePtr);
void XCanPs_ClearBusErrorStatus(struct xcanps_priv *InstancePtr, u32 Mask);
int XCanPs_Send(struct xcanps_priv *InstancePtr, u32 *FramePtr);
int XCanPs_Recv(struct xcanps_priv *InstancePtr, u32 *FramePtr);
int XCanPs_SendHighPriority(struct xcanps_priv *InstancePtr, u32 *FramePtr);
void XCanPs_AcceptFilterEnable(struct xcanps_priv *InstancePtr, u32 FilterIndexes);
void XCanPs_AcceptFilterDisable(struct xcanps_priv *InstancePtr, u32 FilterIndexes);
u32 XCanPs_AcceptFilterGetEnabled(struct xcanps_priv *InstancePtr);
int XCanPs_AcceptFilterSet(struct xcanps_priv *InstancePtr, u32 FilterIndex,
			 u32 MaskValue, u32 IdValue);
void XCanPs_AcceptFilterGet(struct xcanps_priv *InstancePtr, u32 FilterIndex,
			  u32 *MaskValue, u32 *IdValue);

int XCanPs_SetBaudRatePrescaler(struct xcanps_priv *InstancePtr, u8 Prescaler);
u8 XCanPs_GetBaudRatePrescaler(struct xcanps_priv *InstancePtr);
int XCanPs_SetBitTiming(struct xcanps_priv *InstancePtr, u8 SyncJumpWidth,
			  u8 TimeSegment2, u8 TimeSegment1);
void XCanPs_GetBitTiming(struct xcanps_priv *InstancePtr, u8 *SyncJumpWidth,
			   u8 *TimeSegment2, u8 *TimeSegment1);

int XCanPs_SetRxIntrWatermark(struct xcanps_priv *InstancePtr, u8 Threshold);
u8 XCanPs_GetRxIntrWatermark(struct xcanps_priv *InstancePtr);

/*
 * Diagnostic functions in xcanps_selftest.c
 */
int XCanPs_SelfTest(struct xcanps_priv *InstancePtr);

/*
 * Functions in xcanps_intr.c
 */
void XCanPs_IntrEnable(struct xcanps_priv *InstancePtr, u32 Mask);
void XCanPs_IntrDisable(struct xcanps_priv *InstancePtr, u32 Mask);
u32 XCanPs_IntrGetEnabled(struct xcanps_priv *InstancePtr);
u32 XCanPs_IntrGetStatus(struct xcanps_priv *InstancePtr);
void XCanPs_IntrClear(struct xcanps_priv *InstancePtr, u32 Mask);
void XCanPs_IntrHandler(void *InstancePtr);
int XCanPs_SetHandler(struct xcanps_priv *InstancePtr, u32 HandlerType,
			void *CallBackFunc, void *CallBackRef);

struct net_device *alloc_xcanpsdev(int sizeof_priv);
void free_xcanpsdev(struct net_device *dev);
int register_xcanpsdev(struct net_device *dev);
void unregister_xcanpsdev(struct net_device *dev);

irqreturn_t xcanps_interrupt(int irq, void *dev_id);


#endif /* CAN_XILINX_PS7_H */
