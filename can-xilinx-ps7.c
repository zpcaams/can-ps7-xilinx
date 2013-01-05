/*
 * can-xilinx-ps7.c
 *
 *  Created on: Dec 19, 2012
 *      Author: root
 */

#include "can-xilinx-ps7.h"

static struct can_bittiming_const xcanps_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static int xcanps_set_bittiming(struct net_device *dev);
static void set_reset_mode(struct net_device *dev);

static void SendHandler(void *CallBackRef);
static void RecvHandler(void *CallBackRef);
static void ErrorHandler(void *CallBackRef, u32 ErrorMask);
static void EventHandler(void *CallBackRef, u32 Mask);
/****************************************************************************/
/**
*
* This routine enables interrupt(s). Use the XCANPS_IXR_* constants defined in
* xcanps_hw.h to create the bit-mask to enable interrupts.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Mask is the mask to enable. Bit positions of 1 will be enabled.
*		Bit positions of 0 will keep the previous setting. This mask is
*		formed by OR'ing XCANPS_IXR_* bits defined in xcanps_hw.h.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
void XCanPs_IntrEnable(struct xcanps_priv *InstancePtr, u32 Mask)
{
	u32 IntrValue;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Write to the IER to enable the specified interrupts.
	 */
	IntrValue = XCanPs_IntrGetEnabled(InstancePtr);
	IntrValue |= Mask & XCANPS_IXR_ALL;
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_IER_OFFSET, IntrValue);
}

/****************************************************************************/
/**
*
* This routine disables interrupt(s). Use the XCANPS_IXR_* constants defined in
* xcanps_hw.h to create the bit-mask to disable interrupt(s).
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Mask is the mask to disable. Bit positions of 1 will be
*		disabled. Bit positions of 0 will keep the previous setting.
*		This mask is formed by OR'ing XCANPS_IXR_* bits defined in
*		xcanps_hw.h.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
void XCanPs_IntrDisable(struct xcanps_priv *InstancePtr, u32 Mask)
{
	u32 IntrValue;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Write to the IER to disable the specified interrupts.
	 */
	IntrValue = XCanPs_IntrGetEnabled(InstancePtr);
	IntrValue &= ~Mask;
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_IER_OFFSET, IntrValue);
}

/****************************************************************************/
/**
*
* This routine returns enabled interrupt(s). Use the XCANPS_IXR_* constants
* defined in xcanps_hw.h to interpret the returned value.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	Enabled interrupt(s) in a 32-bit format.
*
* @note		None.
*
*****************************************************************************/
u32 XCanPs_IntrGetEnabled(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_IER_OFFSET);
}


/****************************************************************************/
/**
*
* This routine returns interrupt status read from Interrupt Status Register.
* Use the XCANPS_IXR_* constants defined in xcanps_hw.h to interpret the
* returned value.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The value stored in Interrupt Status Register.
*
* @note		None.
*
*****************************************************************************/
u32 XCanPs_IntrGetStatus(struct xcanps_priv *InstancePtr)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_ISR_OFFSET);
}

/****************************************************************************/
/**
*
* This function clears interrupt(s). Every bit set in Interrupt Status
* Register indicates that a specific type of interrupt is occurring, and this
* function clears one or more interrupts by writing a bit mask to Interrupt
* Clear Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Mask is the mask to clear. Bit positions of 1 will be cleared.
*		Bit positions of 0 will not change the previous interrupt
*		status. This mask is formed by OR'ing XCANPS_IXR_* bits defined
* 		in xcanps_hw.h.
*
* @note		None.
*
*****************************************************************************/
void XCanPs_IntrClear(struct xcanps_priv *InstancePtr, u32 Mask)
{
	u32 IntrValue;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Clear the currently pending interrupts.
	 */
	IntrValue = XCanPs_IntrGetStatus(InstancePtr);
	IntrValue &= Mask;
	XCanPs_WriteReg(InstancePtr->BaseAddr, XCANPS_ICR_OFFSET,
				IntrValue);
}

/*****************************************************************************/
/**
*
* This routine is the interrupt handler for the CAN driver.
*
* This handler reads the interrupt status from the ISR, determines the source of
* the interrupts, calls according callbacks, and finally clears the interrupts.
*
* Application beyond this driver is responsible for providing callbacks to
* handle interrupts and installing the callbacks using XCanPs_SetHandler()
* during initialization phase. An example delivered with this driver
* demonstrates how this could be done.
*
* @param	InstancePtr is a pointer to the XCanPs instance that just
*		interrupted.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void XCanPs_IntrHandler(void *InstancePtr)
{
	u32 PendingIntr;
	u32 EventIntr;
	u32 ErrorStatus;
	struct net_device *dev = (struct net_device *)InstancePtr;
	struct xcanps_priv *CanPtr = netdev_priv(dev);

	PendingIntr = XCanPs_IntrGetStatus(CanPtr);
	printk("PendingIntr = 0x%x\n", PendingIntr);

	PendingIntr &= XCanPs_IntrGetEnabled(CanPtr);
	printk("Enabled PendingIntr = 0x%x\n", PendingIntr);

	/*
	 * Clear all pending interrupts.
	 * Rising Edge interrupt
	 */
	XCanPs_IntrClear(CanPtr, PendingIntr);

	/*
	 * An error interrupt is occurring.
	 */
	if ((PendingIntr & XCANPS_IXR_ERROR_MASK)) {
		ErrorStatus = XCanPs_GetBusErrorStatus(CanPtr);
		CanPtr->ErrorHandler(CanPtr->ErrorRef, ErrorStatus);

		/*
		 * Clear Error Status Register.
		 */
		XCanPs_ClearBusErrorStatus(CanPtr, ErrorStatus);
	}

	/*
	 * Check if any following event interrupt is pending:
	 *	  - RX FIFO Overflow
	 *	  - RX FIFO Underflow
	 *	  - TX High Priority Buffer full
	 *	  - TX FIFO Full
	 *	  - Wake up from sleep mode
	 *	  - Enter sleep mode
	 *	  - Enter Bus off status
	 *	  - Arbitration is lost
	 *
	 * If so, call event callback provided by upper level.
	 */
	EventIntr = PendingIntr & (XCANPS_IXR_RXOFLW_MASK |
				XCANPS_IXR_RXUFLW_MASK |
				XCANPS_IXR_TXBFLL_MASK |
				XCANPS_IXR_TXFLL_MASK |
				XCANPS_IXR_WKUP_MASK |
				XCANPS_IXR_SLP_MASK |
				XCANPS_IXR_BSOFF_MASK |
				XCANPS_IXR_ARBLST_MASK);
	if (EventIntr) {
		CanPtr->EventHandler(CanPtr->EventRef, EventIntr);

		if ((EventIntr & XCANPS_IXR_BSOFF_MASK)) {
			/*
			 * Event callback should reset whole device if "Enter
			 * Bus Off Status" interrupt occurred. All pending
			 * interrupts are cleared and no further checking and
			 * handling of other interrupts is needed any more.
			 */
			return;
		}
	}


	if ((PendingIntr & (XCANPS_IXR_RXFWMFLL_MASK |
			XCANPS_IXR_RXNEMP_MASK))) {

		/*
		 * This case happens when
		 * A number of frames depending on the Rx FIFO Watermark
		 * threshold are received.
		 * And  also when frame was received and is sitting in RX FIFO.
		 *
		 * XCANPS_IXR_RXOK_MASK is not used because the bit is set
		 * just once even if there are multiple frames sitting
		 * in the RX FIFO.
		 *
		 * XCANPS_IXR_RXNEMP_MASK is used because the bit can be
		 * set again and again automatically as long as there is
		 * at least one frame in RX FIFO.
		 */
		CanPtr->RecvHandler(CanPtr->RecvRef);
	}

	/*
	 * A frame was transmitted successfully.
	 */
	if ((PendingIntr & XCANPS_IXR_TXOK_MASK)) {
		CanPtr->SendHandler(CanPtr->SendRef);
	}
}

/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle confirmation of
* transmit events when in interrupt mode.
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the driver instance.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
static void SendHandler(void *CallBackRef)
{
	struct net_device *dev = (struct net_device *)CallBackRef;
	struct net_device_stats *stats = &dev->stats;

	/*
	 * The frame was sent successfully. Notify the task context.
	 */
	stats->tx_packets++;
	can_get_echo_skb(dev, 0);
	netif_wake_queue(dev);
}


/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle frames received in
* interrupt mode.  This function is called once all the frames are received.
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the device instance.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
static void RecvHandler(void *CallBackRef)
{
	struct net_device *dev = (struct net_device *)CallBackRef;
	struct xcanps_priv *InstancePtr = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	canid_t id;
	int i;
	int Status;
	u32 RxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];
	u8 *FramePtr;

	/* create zero'ed CAN frame buffer */
	skb = alloc_can_skb(dev, &cf);
	if (skb == NULL)
		return;

	Status = XCanPs_Recv(InstancePtr, RxFrame);
	if (Status != XST_SUCCESS){
		stats->rx_errors++;
		return ;
	}

	/*
	 * Substitute Remote Transmission Request ?
	 * Get id
	 */
	if(((RxFrame[0] | XCANPS_IDR_SRR_MASK) >> XCANPS_IDR_SRR_SHIFT)){
		id = ((RxFrame[0] | XCANPS_IDR_ID1_MASK) >> 3) +
			((RxFrame[0] | XCANPS_IDR_ID2_MASK) >> XCANPS_IDR_ID2_SHIFT);
		id |= CAN_EFF_FLAG;
	} else {
		id = ((RxFrame[0] | XCANPS_IDR_ID1_MASK) >> XCANPS_IDR_ID1_SHIFT);
	}

	/* Get dlc */
	cf->can_dlc = get_can_dlc(RxFrame[0] | XCANPS_DLCR_DLC_MASK >>
			XCANPS_DLCR_DLC_SHIFT);

	/*
	 * Remote Transmission Request ?
	 */
	if(RxFrame[0] | XCANPS_IDR_RTR_MASK){
		id |= CAN_RTR_FLAG;
	} else {
		FramePtr = (u8 *)(&RxFrame[2]);
		for(i=0; i<cf->can_dlc; i++) {
			cf->data[i] = *FramePtr++;
		}
	}

	cf->can_id = id;

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	printk("new packet receive!");
}


/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle error interrupt.
* Error code read from Error Status register is passed into this function
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the driver instance.
* @param	ErrorMask is a bit mask indicating the cause of the error.
*		Its value equals 'OR'ing one or more XCANPS_ESR_* defined in
*		xcanps_hw.h
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
static void ErrorHandler(void *CallBackRef, u32 ErrorMask)
{
	struct net_device *dev = (struct net_device *)CallBackRef;

	if(ErrorMask & XCANPS_ESR_ACKER_MASK) {
		/*
		 * ACK Error handling code should be put here.
		 */

		dev_err(dev->dev.parent, "ACK Error: Acknowledgment error!\n");
	}

	if(ErrorMask & XCANPS_ESR_BERR_MASK) {
		/*
		 * Bit Error handling code should be put here.
		 */
		dev_err(dev->dev.parent, "Bit Error: the received bit is \n"
				"not the same as the transmitted bit during bus communication!\n");
	}

	if(ErrorMask & XCANPS_ESR_STER_MASK) {
		/*
		 * Stuff Error handling code should be put here.
		 */
		dev_err(dev->dev.parent, "Stuff Error: there is a stuffing violation!\n");
	}

	if(ErrorMask & XCANPS_ESR_FMER_MASK) {
		/*
		 * Form Error handling code should be put here.
		 */
		dev_err(dev->dev.parent, "Form Error: an error in one of the fixed form fields \n"
				" in the message frame!\n");
	}

	if(ErrorMask & XCANPS_ESR_CRCER_MASK) {
		/*
		 * CRC Error handling code should be put here.
		 */
		dev_err(dev->dev.parent, "CRC Error: a CRC error has occurred!\n");
	}

	/*
	 * Stop Can if any error happened
	 */
	dev_err(dev->dev.parent, "Error happened, CAN is stopped, check the cable, the board for reasons!\n");
	set_reset_mode(dev);

}


/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle the following
* interrupts:
*   - XCANPS_IXR_BSOFF_MASK:  Bus Off Interrupt
*   - XCANPS_IXR_RXOFLW_MASK: RX FIFO Overflow Interrupt
*   - XCANPS_IXR_RXUFLW_MASK: RX FIFO Underflow Interrupt
*   - XCANPS_IXR_TXBFLL_MASK: TX High Priority Buffer Full Interrupt
*   - XCANPS_IXR_TXFLL_MASK:  TX FIFO Full Interrupt
*   - XCANPS_IXR_WKUP_MASK:   Wake up Interrupt
*   - XCANPS_IXR_SLP_MASK:    Sleep Interrupt
*   - XCANPS_IXR_ARBLST_MASK: Arbitration Lost Interrupt
*
*
* @param	CallBackRef is the callback reference passed from the
*		interrupt Handler, which in our case is a pointer to the
*		driver instance.
* @param	IntrMask is a bit mask indicating pending interrupts.
*		Its value equals 'OR'ing one or more of the XCANPS_IXR_*_MASK
*		value(s) mentioned above.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*		This function needs to be changed to meet specific application
*		needs.
*
******************************************************************************/
static void EventHandler(void *CallBackRef, u32 IntrMask)
{
	struct net_device *dev = (struct net_device *)CallBackRef;
	struct xcanps_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	if (IntrMask & XCANPS_IXR_BSOFF_MASK) {

		set_reset_mode(dev);

		dev_err(dev->dev.parent, " Bus off status Event!\n");
	}

	if(IntrMask & XCANPS_IXR_RXOFLW_MASK) {
		/*
		 * Code to handle RX FIFO Overflow
		 * Interrupt should be put here.
		 */
		stats->rx_over_errors++;
		stats->rx_errors++;
		dev_err(dev->dev.parent, "RX FIFO Overflow Event!\n");
	}

	if(IntrMask & XCANPS_IXR_RXUFLW_MASK) {
		/*
		 * Code to handle RX FIFO Underflow
		 * Interrupt should be put here.
		 */
		dev_err(dev->dev.parent, "RX FIFO Underflow Event!\n");
	}

	if(IntrMask & XCANPS_IXR_TXBFLL_MASK) {
		/*
		 * Code to handle TX High Priority Buffer Full
		 * Interrupt should be put here.
		 */
		dev_err(dev->dev.parent, "TX High Priority Buffer Full Event!\n");
	}

	if(IntrMask & XCANPS_IXR_TXFLL_MASK) {
		/*
		 * Code to handle TX FIFO Full
		 * Interrupt should be put here.
		 */
		dev_err(dev->dev.parent, "TX FIFO Full Event!\n");
	}

	if (IntrMask & XCANPS_IXR_WKUP_MASK) {
		/*
		 * Code to handle Wake up from sleep mode
		 * Interrupt should be put here.
		 */

		dev_err(dev->dev.parent, "Wake up from sleep mode Event!\n");
	}

	if (IntrMask & XCANPS_IXR_SLP_MASK) {
		/*
		 * Code to handle Enter sleep mode
		 * Interrupt should be put here.
		 */
		dev_err(dev->dev.parent, "Enter sleep mode Event!\n");
	}

	if (IntrMask & XCANPS_IXR_ARBLST_MASK) {
		/*
		 * Code to handle Lost bus arbitration
		 * Interrupt should be put here.
		 */
		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		dev_err(dev->dev.parent, "Lost bus arbitration Event!\n");
	}
}

/*****************************************************************************/
/**
*
* This routine installs an asynchronous callback function for the given
* HandlerType:
*
* <pre>
* HandlerType			Callback Function Type
* -----------------------	------------------------
* XCANPS_HANDLER_SEND		XCanPs_SendRecvHandler
* XCANPS_HANDLER_RECV		XCanPs_SendRecvHandler
* XCANPS_HANDLER_ERROR		XCanPs_ErrorHandler
* XCANPS_HANDLER_EVENT		XCanPs_EventHandler
*
* HandlerType			Invoked by this driver when:
* -------------------------------------------------------------------------
* XCANPS_HANDLER_SEND		A frame transmitted by a call to
*				XCanPs_Send() has been sent successfully.
*
* XCANPS_HANDLER_RECV		A frame(s) has been received and is sitting in
*				the RX FIFO.
*
* XCANPS_HANDLER_ERROR		An error interrupt is occurring.
*
* XCANPS_HANDLER_EVENT		Any other kind of interrupt is occurring.
* </pre>
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	HandlerType specifies which handler is to be attached.
* @param	CallBackFunc is the address of the callback function.
* @param	CallBackRef is a user data item that will be passed to the
*		callback function when it is invoked.
*
* @return
*		- XST_SUCCESS when handler is installed.
*		- XST_INVALID_PARAM when HandlerType is invalid.
*
* @note
* Invoking this function for a handler that already has been installed replaces
* it with the new handler.
*
******************************************************************************/
int XCanPs_SetHandler(struct xcanps_priv *InstancePtr, u32 HandlerType,
			void *CallBackFunc, void *CallBackRef)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	switch (HandlerType) {
	case XCANPS_HANDLER_SEND:
		InstancePtr->SendHandler =
			(XCanPs_SendRecvHandler) CallBackFunc;
		InstancePtr->SendRef = CallBackRef;
		break;

	case XCANPS_HANDLER_RECV:
		InstancePtr->RecvHandler =
			(XCanPs_SendRecvHandler) CallBackFunc;
		InstancePtr->RecvRef = CallBackRef;
		break;

	case XCANPS_HANDLER_ERROR:
		InstancePtr->ErrorHandler = (XCanPs_ErrorHandler) CallBackFunc;
		InstancePtr->ErrorRef = CallBackRef;
		break;

	case XCANPS_HANDLER_EVENT:
		InstancePtr->EventHandler = (XCanPs_EventHandler) CallBackFunc;
		InstancePtr->EventRef = CallBackRef;
		break;

	default:
		return (XST_INVALID_PARAM);

	}
	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
*
* This function resets the CAN device. Calling this function resets the device
* immediately, and any pending transmission or reception is terminated at once.
* Both Object Layer and Transfer Layer are reset. This function does not reset
* the Physical Layer. All registers are reset to the default values, and no
* previous status will be restored. TX FIFO, RX FIFO and TX High Priority
* Buffer are also reset.
*
* When a reset is required due to an internal error, the driver notifies the
* upper layer software of this need through the error status code or interrupts.
* The upper layer software is responsible for calling this Reset function and
* then re-configuring the device.
*
* The CAN device will be in Configuration Mode immediately after this function
* returns.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void XCanPs_Reset(struct xcanps_priv *InstancePtr)
{
//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	XCanPs_WriteReg(InstancePtr->BaseAddr, XCANPS_SRR_OFFSET, \
			   XCANPS_SRR_SRST_MASK);
}

/****************************************************************************/
/**
*
* This routine returns the current operation mode of the CAN device.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
* 		- XCANPS_MODE_CONFIG if the device is in Configuration Mode.
* 		- XCANPS_MODE_SLEEP if the device is in Sleep Mode.
* 		- XCANPS_MODE_NORMAL if the device is in Normal Mode.
* 		- XCANPS_MODE_LOOPBACK if the device is in Loop Back Mode.
* 		- XCANPS_MODE_SNOOP if the device is in Snoop Mode.
*
* @note		None.
*
*****************************************************************************/
u8 XCanPs_GetMode(struct xcanps_priv *InstancePtr)
{
	u32 StatusReg;

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	StatusReg = XCanPs_GetStatus(InstancePtr);

	if (StatusReg & XCANPS_SR_CONFIG_MASK) {
		return XCANPS_MODE_CONFIG;

	} else if (StatusReg & XCANPS_SR_SLEEP_MASK) {
		return XCANPS_MODE_SLEEP;

	} else if (StatusReg & XCANPS_SR_NORMAL_MASK) {
		if (StatusReg & XCANPS_SR_SNOOP_MASK) {
			return XCANPS_MODE_SNOOP;
		} else {
			return XCANPS_MODE_NORMAL;
		}
	} else {
		/*
		 * If this line is reached, the device is in Loop Back Mode.
		 */
		return XCANPS_MODE_LOOPBACK;
	}
}

/*****************************************************************************/
/**
*
* This function allows the CAN device to enter one of the following operation
* modes:
*	- Configuration Mode: Pass in parameter XCANPS_MODE_CONFIG
*	- Sleep Mode: Pass in parameter XCANPS_MODE_SLEEP
*	- Normal Mode: Pass in parameter XCANPS_MODE_NORMAL
*	- Loop Back Mode: Pass in parameter XCANPS_MODE_LOOPBACK.
*	- Snoop Mode: Pass in parameter XCANPS_MODE_SNOOP.
*
* Read the xcanps.h file and device specification for detailed description of
* each operation mode.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	OperationMode specify which operation mode to enter. Valid value
*		is any of XCANPS_MODE_* defined in xcanps.h. Multiple modes
*		can not be entered at the same time.
*
* @return	None.
*
* @note
*
* This function does NOT ensure CAN device enters the specified operation mode
* before it returns the control to the caller. The caller is responsible for
* checking current operation mode using XCanPs_GetMode().
*
******************************************************************************/
void XCanPs_EnterMode(struct xcanps_priv *InstancePtr, u8 OperationMode)
{
	u8 CurrentMode;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertVoid((OperationMode == XCANPS_MODE_CONFIG) ||
//			(OperationMode == XCANPS_MODE_SLEEP) ||
//			(OperationMode == XCANPS_MODE_NORMAL) ||
//			(OperationMode == XCANPS_MODE_LOOPBACK) ||
//			(OperationMode == XCANPS_MODE_SNOOP));

	CurrentMode = XCanPs_GetMode(InstancePtr);

	/*
	 * If current mode is Normal Mode and the mode to enter is Sleep Mode,
	 * or if current mode is Sleep Mode and the mode to enter is Normal
	 * Mode, no transition through Configuration Mode is needed.
	 */
	if ((CurrentMode == XCANPS_MODE_NORMAL) &&
		(OperationMode == XCANPS_MODE_SLEEP)) {
		/*
		 * Normal Mode ---> Sleep Mode
		 */
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_MSR_OFFSET, XCANPS_MSR_SLEEP_MASK);
		return;

	} else if ((CurrentMode == XCANPS_MODE_SLEEP) &&
		 (OperationMode == XCANPS_MODE_NORMAL)) {
		/*
		 * Sleep Mode ---> Normal Mode
		 */
		XCanPs_WriteReg(InstancePtr->BaseAddr,
					XCANPS_MSR_OFFSET, 0);
		return;
	}


	/*
	 * If the mode transition is not any of the two cases above, CAN must
	 * enter Configuration Mode before switching into the target operation
	 * mode.
	 */
	XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_SRR_OFFSET, 0);

	/*
	 * Check if the device has entered Configuration Mode, if not, return to
	 * the caller.
	 */
	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG) {
		return;
	}

	switch (OperationMode) {
	case XCANPS_MODE_CONFIG:
		/*
		 * As CAN is in Configuration Mode already.
		 * Nothing is needed to be done here.
		 */
		break;

	case XCANPS_MODE_SLEEP:
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_MSR_OFFSET, XCANPS_MSR_SLEEP_MASK);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_SRR_OFFSET, XCANPS_SRR_CEN_MASK);
		break;

	case XCANPS_MODE_NORMAL:
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_MSR_OFFSET, 0);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_SRR_OFFSET, XCANPS_SRR_CEN_MASK);
		break;

	case XCANPS_MODE_LOOPBACK:
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_MSR_OFFSET, XCANPS_MSR_LBACK_MASK);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_SRR_OFFSET, XCANPS_SRR_CEN_MASK);
		break;

	case XCANPS_MODE_SNOOP:
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_MSR_OFFSET, XCANPS_MSR_SNOOP_MASK);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_SRR_OFFSET, XCANPS_SRR_CEN_MASK);
		break;

	}
}

/*****************************************************************************/
/**
*
* This function returns Status value from Status Register (SR). Use the
* XCANPS_SR_* constants defined in xcanps_hw.h to interpret the returned
* value.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The 32-bit value read from Status Register.
*
* @note		None.
*
******************************************************************************/
u32 XCanPs_GetStatus(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_SR_OFFSET);
}

/*****************************************************************************/
/**
*
* This function reads Error Status value from Error Status Register (ESR). Use
* the XCANPS_ESR_* constants defined in xcanps_hw.h to interpret the
* returned value.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The 32-bit value read from Error Status Register.
*
* @note		None.
*
******************************************************************************/
u32 XCanPs_GetBusErrorStatus(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_ESR_OFFSET);
}

/*****************************************************************************/
/**
*
* This function clears Error Status bit(s) previously set in Error
* Status Register (ESR). Use the XCANPS_ESR_* constants defined in xcanps_hw.h
* to create the value to pass in. If a bit was cleared in Error Status Register
* before this function is called, it will not be modified.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @param	Mask is he 32-bit mask used to clear bits in Error Status
*		Register. Multiple XCANPS_ESR_* values can be 'OR'ed to clear
*		multiple bits.
*
* @note		None.
*
******************************************************************************/
void XCanPs_ClearBusErrorStatus(struct xcanps_priv *InstancePtr, u32 Mask)
{
//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_ESR_OFFSET, Mask);
}

/*****************************************************************************/
/**
*
* This function sends a CAN Frame. If the TX FIFO is not full then the given
* frame is written into the the TX FIFO otherwise, it returns an error code
* immediately.
* This function does not wait for the given frame being sent to CAN bus.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FramePtr is a pointer to a 32-bit aligned buffer containing the
*		CAN frame to be sent.
*
* @return
*		- XST_SUCCESS if TX FIFO was not full and the given frame was
*		written into the FIFO.
*		- XST_FIFO_NO_ROOM if there is no room in the TX FIFO for the
*		given frame.
*
* @note		None.
*
******************************************************************************/
int XCanPs_Send(struct xcanps_priv *InstancePtr, u32 *FramePtr)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(FramePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	if (XCanPs_IsTxFifoFull(InstancePtr) == TRUE) {
		return XST_FIFO_NO_ROOM;
	}

	/*
	 * Write IDR, DLC, Data Word 1 and Data Word 2 to the CAN device.
	 */
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXFIFO_ID_OFFSET, FramePtr[0]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXFIFO_DLC_OFFSET, FramePtr[1]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXFIFO_DW1_OFFSET, FramePtr[2]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXFIFO_DW2_OFFSET, FramePtr[3]);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function receives a CAN Frame. This function first checks if RX FIFO is
* empty, if not, it then reads a frame from the RX FIFO into the given buffer.
* This function returns error code immediately if there is no frame in the RX
* FIFO.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FramePtr is a pointer to a 32-bit aligned buffer where the CAN
*		frame to be written.
*
* @return
*		- XST_SUCCESS if RX FIFO was not empty and a frame was read from
*		RX FIFO successfully and written into the given buffer.
*		- XST_NO_DATA if there is no frame to be received from the FIFO.
*
* @note		None.
*
******************************************************************************/
int XCanPs_Recv(struct xcanps_priv *InstancePtr, u32 *FramePtr)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(FramePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	if (XCanPs_IsRxEmpty(InstancePtr) == TRUE) {
		return XST_NO_DATA;
	}

	/*
	 * Read IDR, DLC, Data Word 1 and Data Word 2 from the CAN device.
	 */
	FramePtr[0] = XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_RXFIFO_ID_OFFSET);
	FramePtr[1] = XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_RXFIFO_DLC_OFFSET);
	FramePtr[2] = XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_RXFIFO_DW1_OFFSET);
	FramePtr[3] = XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_RXFIFO_DW2_OFFSET);

	/*
	 * Clear RXNEMP bit in ISR. This allows future XCanPs_IsRxEmpty() call
	 * returns correct RX FIFO occupancy/empty condition.
	 */
	XCanPs_IntrClear(InstancePtr, XCANPS_IXR_RXNEMP_MASK);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This routine sends a CAN High Priority frame. This function first checks if
* TX High Priority Buffer is empty. If yes, it then writes the given frame into
* the Buffer. If not, this function returns immediately. This function does not
* wait for the given frame being sent to CAN bus.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FramePtr is a pointer to a 32-bit aligned buffer containing the
*		CAN High Priority frame to be sent.
*
* @return
*		- XST_SUCCESS if TX High Priority Buffer was not full and the
*		given frame was written into the buffer;
*		- XST_FIFO_NO_ROOM if there is no room in the TX High Priority
*		Buffer for this frame.
*
* @note
*
* If the frame needs to be sent immediately and not delayed by processor's
* interrupt handling, the caller should disable interrupt at processor
* level before invoking this function.
*
******************************************************************************/
int XCanPs_SendHighPriority(struct xcanps_priv *InstancePtr, u32 *FramePtr)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(FramePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	if (XCanPs_IsHighPriorityBufFull(InstancePtr) == TRUE) {
		return XST_FIFO_NO_ROOM;
	}

	/*
	 * Write IDR, DLC, Data Word 1 and Data Word 2 to the CAN device.
	 */
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXHPB_ID_OFFSET, FramePtr[0]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXHPB_DLC_OFFSET, FramePtr[1]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXHPB_DW1_OFFSET, FramePtr[2]);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_TXHPB_DW2_OFFSET, FramePtr[3]);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This routine enables individual acceptance filters. Up to 4 filters could
* be enabled.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FilterIndexes specifies which filter(s) to enable. Use
*		any XCANPS_AFR_UAF*_MASK to enable one filter, and "Or"
*		multiple XCANPS_AFR_UAF*_MASK values if multiple filters need
*		to be enabled. Any filter not specified in this parameter will
*		keep its previous enable/disable setting.
*
* @return	None.
*
* @note		None.
*
*
******************************************************************************/
void XCanPs_AcceptFilterEnable(struct xcanps_priv *InstancePtr, u32 FilterIndexes)
{
	u32 EnabledFilters;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 *  Calculate the new value and write to AFR.
	 */
	EnabledFilters =  XCanPs_ReadReg(InstancePtr->BaseAddr,
						XCANPS_AFR_OFFSET);
	EnabledFilters |= FilterIndexes;
	EnabledFilters &= XCANPS_AFR_UAF_ALL_MASK;
	XCanPs_WriteReg(InstancePtr->BaseAddr, XCANPS_AFR_OFFSET,
			EnabledFilters);
}

/*****************************************************************************/
/**
*
* This routine disables individual acceptance filters. Up to 4 filters could
* be disabled. If all acceptance filters are disabled then all the received
* frames are stored in the RX FIFO.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FilterIndexes specifies which filter(s) to disable. Use
*		any XCANPS_AFR_UAF*_MASK to disable one filter, and "Or"
*		multiple XCANPS_AFR_UAF*_MASK values if multiple filters need
* 		to be disabled. Any filter not specified in this parameter will
*		keep its previous enable/disable setting. If all acceptance
*		filters are disabled then all received frames are stored in the
*		RX FIFO.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void XCanPs_AcceptFilterDisable(struct xcanps_priv *InstancePtr, u32 FilterIndexes)
{
	u32 EnabledFilters;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 *  Calculate the new value and write to AFR.
	 */
	EnabledFilters = XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_AFR_OFFSET);
	EnabledFilters &= XCANPS_AFR_UAF_ALL_MASK & (~FilterIndexes);
	XCanPs_WriteReg(InstancePtr->BaseAddr, XCANPS_AFR_OFFSET,
			   EnabledFilters);
}

/*****************************************************************************/
/**
*
* This function returns enabled acceptance filters. Use XCANPS_AFR_UAF*_MASK
* defined in xcanps_hw.h to interpret the returned value. If no acceptance
* filters are enabled then all received frames are stored in the RX FIFO.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The value stored in Acceptance Filter Register.
*
* @note		None.
*
*
******************************************************************************/
u32 XCanPs_AcceptFilterGetEnabled(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_AFR_OFFSET);

}

/*****************************************************************************/
/**
*
* This function sets values to the Acceptance Filter Mask Register (AFMR) and
* Acceptance Filter ID Register (AFIR) for the specified Acceptance Filter.
* Use XCANPS_IDR_* defined in xcanps_hw.h to create the values to set the
* filter. Read the xcanps.h file and device specification for details.
*
* This function should be called only after:
*   - The given filter is disabled by calling XCanPs_AcceptFilterDisable();
*   - And the CAN device is ready to accept writes to AFMR and AFIR, i.e.,
*	 XCanPs_IsAcceptFilterBusy() returns FALSE.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FilterIndex defines which Acceptance Filter Mask and ID Register
*		to set. Use any single XCANPS_AFR_UAF*_MASK value.
* @param	MaskValue is the value to write to the chosen Acceptance Filter
*		Mask Register.
* @param	IdValue is the value to write to the chosen Acceptance Filter
*		ID Register.
*
* @return
*		- XST_SUCCESS if the values were set successfully.
*		- XST_FAILURE if the given filter was not disabled, or the CAN
*		device was not ready to accept writes to AFMR and AFIR.
*
* @note		None.
*
******************************************************************************/
int XCanPs_AcceptFilterSet(struct xcanps_priv *InstancePtr, u32 FilterIndex,
			 u32 MaskValue, u32 IdValue)
{
	u32 EnabledFilters;

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertNonvoid((FilterIndex == XCANPS_AFR_UAF4_MASK) ||
//			(FilterIndex == XCANPS_AFR_UAF3_MASK) ||
//			(FilterIndex == XCANPS_AFR_UAF2_MASK) ||
//			(FilterIndex == XCANPS_AFR_UAF1_MASK));

	/*
	 * Return an error if the given filter is currently enabled.
	 */
	EnabledFilters = XCanPs_AcceptFilterGetEnabled(InstancePtr);
	if ((EnabledFilters & FilterIndex) == FilterIndex) {
		return XST_FAILURE;
	}

	/*
	 * If the CAN device is not ready to accept writes to AFMR and AFIR,
	 * return error code.
	 */
	if (XCanPs_IsAcceptFilterBusy(InstancePtr) == TRUE) {
		return XST_FAILURE;
	}

	/*
	 * Write to the AFMR and AFIR of the specified filter.
	 */
	switch (FilterIndex) {
	case XCANPS_AFR_UAF1_MASK:	/* Acceptance Filter No. 1 */

		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFMR1_OFFSET, MaskValue);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFIR1_OFFSET, IdValue);
		break;

	case XCANPS_AFR_UAF2_MASK:	/* Acceptance Filter No. 2 */
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFMR2_OFFSET, MaskValue);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFIR2_OFFSET, IdValue);
		break;

	case XCANPS_AFR_UAF3_MASK:	/* Acceptance Filter No. 3 */
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFMR3_OFFSET, MaskValue);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFIR3_OFFSET, IdValue);
		break;

	case XCANPS_AFR_UAF4_MASK:	/* Acceptance Filter No. 4 */
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFMR4_OFFSET, MaskValue);
		XCanPs_WriteReg(InstancePtr->BaseAddr,
				XCANPS_AFIR4_OFFSET, IdValue);
		break;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function reads the values of the Acceptance Filter Mask and ID Register
* for the specified Acceptance Filter. Use XCANPS_IDR_* defined in xcanps_hw.h
* to interpret the values. Read the xcanps.h file and device specification for
* details.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	FilterIndex defines which Acceptance Filter Mask Register to get
*		Mask and ID from. Use any single XCANPS_FILTER_* value.
* @param	MaskValue is a pointer to the data in which the Mask value read
*		from the chosen Acceptance Filter Mask Register is returned.
* @param	IdValue is a pointer to the data in which the ID value read
*		from the chosen Acceptance Filter ID Register is returned.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void XCanPs_AcceptFilterGet(struct xcanps_priv *InstancePtr, u32 FilterIndex,
			  u32 *MaskValue, u32 *IdValue)
{
//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertVoid((FilterIndex == XCANPS_AFR_UAF4_MASK) ||
//			 (FilterIndex == XCANPS_AFR_UAF3_MASK) ||
//			 (FilterIndex == XCANPS_AFR_UAF2_MASK) ||
//			 (FilterIndex == XCANPS_AFR_UAF1_MASK));
//	Xil_AssertVoid(MaskValue != NULL);
//	Xil_AssertVoid(IdValue != NULL);

	/*
	 * Read from the AFMR and AFIR of the specified filter.
	 */
	switch (FilterIndex) {
	case XCANPS_AFR_UAF1_MASK:	/* Acceptance Filter No. 1 */
		*MaskValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFMR1_OFFSET);
		*IdValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFIR1_OFFSET);
		break;

	case XCANPS_AFR_UAF2_MASK:	/* Acceptance Filter No. 2 */
		*MaskValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFMR2_OFFSET);
		*IdValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFIR2_OFFSET);
		break;

	case XCANPS_AFR_UAF3_MASK:	/* Acceptance Filter No. 3 */
		*MaskValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFMR3_OFFSET);
		*IdValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFIR3_OFFSET);
		break;

	case XCANPS_AFR_UAF4_MASK:	/* Acceptance Filter No. 4 */
		*MaskValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFMR4_OFFSET);
		*IdValue = XCanPs_ReadReg(InstancePtr->BaseAddr,
					  XCANPS_AFIR4_OFFSET);
		break;
	}
}

/*****************************************************************************/
/**
*
* This routine sets Baud Rate Prescaler value. The system clock for the CAN
* controller is divided by (Prescaler + 1) to generate the quantum clock
* needed for sampling and synchronization. Read the device specification
* for details.
*
* Baud Rate Prescaler can be set only if the CAN device is in Configuration
* Mode. Call XCanPs_EnterMode() to enter Configuration Mode before using this
* function.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Prescaler is the value to set. Valid values are from 0 to 255.
*
* @return
*		- XST_SUCCESS if the Baud Rate Prescaler value is set
*		successfully.
*		- XST_FAILURE if CAN device is not in Configuration Mode.
*
* @note		None.
*
******************************************************************************/
int XCanPs_SetBaudRatePrescaler(struct xcanps_priv *InstancePtr, u8 Prescaler)
{
	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG) {
		return XST_FAILURE;
	}

	XCanPs_WriteReg(InstancePtr->BaseAddr, XCANPS_BRPR_OFFSET,
				(u32)Prescaler);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This routine gets Baud Rate Prescaler value. The system clock for the CAN
* controller is divided by (Prescaler + 1) to generate the quantum clock
* needed for sampling and synchronization. Read the device specification for
* details.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	Current used Baud Rate Prescaler value. The value's range is
*		from 0 to 255.
*
* @note		None.
*
******************************************************************************/
u8 XCanPs_GetBaudRatePrescaler(struct xcanps_priv *InstancePtr)
{
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	return (u8) XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_BRPR_OFFSET);

}

/*****************************************************************************/
/**
*
* This routine sets Bit time. Time segment 1, Time segment 2 and
* Synchronization Jump Width are set in this function. Device specification
* requires the values passed into this function be one less than the actual
* values of these fields. Read the device specification for details.
*
* Bit time can be set only if the CAN device is in Configuration Mode.
* Call XCanPs_EnterMode() to enter Configuration Mode before using this
* function.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	SyncJumpWidth is the Synchronization Jump Width value to set.
*		Valid values are from 0 to 3.
* @param	TimeSegment2 is the Time Segment 2 value to set. Valid values
*		are from 0 to 7.
* @param	TimeSegment1 is the Time Segment 1 value to set. Valid values
*		are from 0 to 15.
*
* @return
*		- XST_SUCCESS if the Bit time is set successfully.
*		- XST_FAILURE if CAN device is not in Configuration Mode.
*
* @note		None.
*
******************************************************************************/
int XCanPs_SetBitTiming(struct xcanps_priv *InstancePtr, u8 SyncJumpWidth,
			  u8 TimeSegment2, u8 TimeSegment1)
{
	u32 Value;

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertNonvoid(SyncJumpWidth <= 3);
//	Xil_AssertNonvoid(TimeSegment2 <= 7);
//	Xil_AssertNonvoid(TimeSegment1 <= 15 );

	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG) {
		return XST_FAILURE;
	}

	Value = ((u32) TimeSegment1) & XCANPS_BTR_TS1_MASK;
	Value |= (((u32) TimeSegment2) << XCANPS_BTR_TS2_SHIFT) &
		XCANPS_BTR_TS2_MASK;
	Value |= (((u32) SyncJumpWidth) << XCANPS_BTR_SJW_SHIFT) &
		XCANPS_BTR_SJW_MASK;

	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_BTR_OFFSET, Value);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This routine gets Bit time. Time segment 1, Time segment 2 and
* Synchronization Jump Width values are read in this function. According to
* device specification, the actual value of each of these fields is one
* more than the value read. Read the device specification for details.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	SyncJumpWidth will store the Synchronization Jump Width value
*		after this function returns. Its value ranges from 0 to 3.
* @param	TimeSegment2 will store the Time Segment 2 value after this
*		function returns. Its value ranges from 0 to 7.
* @param	TimeSegment1 will store the Time Segment 1 value after this
*		function returns. Its value ranges from 0 to 15.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void XCanPs_GetBitTiming(struct xcanps_priv *InstancePtr, u8 *SyncJumpWidth,
			   u8 *TimeSegment2, u8 *TimeSegment1)
{
	u32 Value;

//	Xil_AssertVoid(InstancePtr != NULL);
//	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertVoid(SyncJumpWidth != NULL);
//	Xil_AssertVoid(TimeSegment2 != NULL);
//	Xil_AssertVoid(TimeSegment1 != NULL);

	Value = XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_BTR_OFFSET);

	*TimeSegment1 = (u8) (Value & XCANPS_BTR_TS1_MASK);
	*TimeSegment2 =
		(u8) ((Value & XCANPS_BTR_TS2_MASK) >> XCANPS_BTR_TS2_SHIFT);
	*SyncJumpWidth =
		(u8) ((Value & XCANPS_BTR_SJW_MASK) >> XCANPS_BTR_SJW_SHIFT);
}


/****************************************************************************/
/**
*
* This routine sets the Rx Full threshold in the Watermark Interrupt Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Threshold is the threshold to be set. The valid values are
*		from 1 to 63.
*
* @return
*		- XST_FAILURE - If the CAN device is not in Configuration Mode.
*		- XST_SUCCESS - If the Rx Full threshold is set in Watermark
*		Interrupt Register.
*
* @note		The threshold can only be set when the CAN device is in the
*		configuration mode.
*
*****************************************************************************/
int XCanPs_SetRxIntrWatermark(struct xcanps_priv *InstancePtr, u8 Threshold)
{

	u32 ThrReg;
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertNonvoid(Threshold <= 63);

	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG)
		return XST_FAILURE;

	ThrReg = XCanPs_ReadReg(InstancePtr->BaseAddr,
			XCANPS_WIR_OFFSET);

	ThrReg &= XCANPS_WIR_EW_MASK;
	ThrReg |= ((u32)Threshold & XCANPS_WIR_FW_MASK);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_WIR_OFFSET, ThrReg);

	return XST_SUCCESS;
}

/****************************************************************************/
/**
*
* This routine gets the Rx Full threshold from the Watermark Interrupt Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The Rx FIFO full watermark threshold value. The valid values
*		are 1 to 63.
*
* @note		None.
*
*****************************************************************************/
u8 XCanPs_GetRxIntrWatermark(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);


	return (u8) (XCanPs_ReadReg(InstancePtr->BaseAddr,
					XCANPS_WIR_OFFSET) &
					XCANPS_WIR_FW_MASK);
}


/****************************************************************************/
/**
*
* This routine sets the Tx Empty Threshold in the Watermark Interrupt Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
* @param	Threshold is the threshold to be set. The valid values are
*		from 1 to 63.
*
* @return
*		- XST_FAILURE - If the CAN device is not in Configuration Mode.
*		- XST_SUCCESS - If the threshold is set in Watermark
*		Interrupt Register.
*
* @note		The threshold can only be set when the CAN device is in the
*		configuration mode.
*
*****************************************************************************/
int XCanPs_SetTxIntrWatermark(struct xcanps_priv *InstancePtr, u8 Threshold)
{
	u32 ThrReg;
//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
//	Xil_AssertNonvoid(Threshold <= 63);

	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG)
		return XST_FAILURE;

	ThrReg = XCanPs_ReadReg(InstancePtr->BaseAddr,
			XCANPS_WIR_OFFSET);

	ThrReg &= XCANPS_WIR_FW_MASK;
	ThrReg |= ((u32)(Threshold << XCANPS_WIR_EW_SHIFT)
			& XCANPS_WIR_EW_MASK);
	XCanPs_WriteReg(InstancePtr->BaseAddr,
			XCANPS_WIR_OFFSET, ThrReg);

	return XST_SUCCESS;
}

/****************************************************************************/
/**
*
* This routine gets the Tx Empty threshold from Watermark Interrupt Register.
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return	The Tx Empty FIFO threshold value. The valid values are 1 to 63.
*
* @note		None.
*
*****************************************************************************/
u8 XCanPs_GetTxIntrWatermark(struct xcanps_priv *InstancePtr)
{

//	Xil_AssertNonvoid(InstancePtr != NULL);
//	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);


	return (u8) ((XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_WIR_OFFSET) & XCANPS_WIR_EW_MASK) >>
					XCANPS_WIR_EW_SHIFT);
}

/*****************************************************************************/
/**
*
* This function runs a self-test on the CAN driver/device. The test resets
* the device, sets up the Loop Back mode, sends a standard frame, receives the
* frame, verifies the contents, and resets the device again.
*
* Note that this is a destructive test in that resets of the device are
* performed. Refer the device specification for the device status after
* the reset operation.
*
*
* @param	InstancePtr is a pointer to the XCanPs instance.
*
* @return
*		- XST_SUCCESS if the self-test passed. i.e., the frame
*		  received via the internal loop back has the same contents as
*		  the frame sent.
* 		- XST_FAILURE   Otherwise.
*
* @note
*
* If the CAN device does not work properly, this function may enter an
* infinite loop and will never return to the caller.
* <br><br>
* If XST_FAILURE is returned, the device is not reset so that the caller could
* have a chance to check reason(s) causing the failure.
*
******************************************************************************/
int XCanPs_SelfTest(struct xcanps_priv *InstancePtr)
{
	u8 *FramePtr;
	u32 Status;
	u32 Index;
	u32 TxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];
	u32 RxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];

	XCanPs_Reset(InstancePtr);

	/*
	 * The device should enter Configuration Mode immediately after
	 * reset above is finished. Now check the mode and return error code if
	 * it is not Configuration Mode.
	 */
	if (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG) {
		return XST_FAILURE;
	}

	/*
	 * Setup Baud Rate Prescaler Register (BRPR) and Bit Timing Register
	 * (BTR) such that CAN baud rate equals 40Kbps, given the CAN clock
	 * equal to 24MHz. For more information see the CAN 2.0A, CAN 2.0B,
	 * ISO 11898-1 specifications.
	 */
	XCanPs_SetBaudRatePrescaler(InstancePtr, 1);
	XCanPs_SetBitTiming(InstancePtr, 1, 3, 8);

	/*
	 * Enter the loop back mode.
	 */
	XCanPs_EnterMode(InstancePtr, XCANPS_MODE_LOOPBACK);
	while (XCanPs_GetMode(InstancePtr) != XCANPS_MODE_LOOPBACK);

	/*
	 * Create a frame to send with known values so we can verify them
	 * on receive.
	 */
	TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)2000, 0, 0, 0, 0);
	TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)8);

	FramePtr = (u8 *) (&TxFrame[2]);
	for (Index = 0; Index < 8; Index++) {
		*FramePtr++ = (u8) Index;
	}

	/*
	 * Send the frame.
	 */
	Status = XCanPs_Send(InstancePtr, TxFrame);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Wait until the frame arrives RX FIFO via internal loop back.
	 */
	while (XCanPs_IsRxEmpty(InstancePtr) == TRUE);

	/*
	 * Receive the frame.
	 */
	Status = XCanPs_Recv(InstancePtr, RxFrame);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Verify Identifier and Data Length Code.
	 */
	if (RxFrame[0] !=
		(u32)XCanPs_CreateIdValue((u32)2000, 0, 0, 0, 0)) {
		return XST_FAILURE;
	}

	if ((RxFrame[1] & ~XCANPS_DLCR_TIMESTAMP_MASK) != TxFrame[1]) {
		return XST_FAILURE;
	}


	for (Index = 2; Index < XCANPS_MAX_FRAME_SIZE_IN_WORDS; Index++) {
		if (RxFrame[Index] != TxFrame[Index]) {
			return XST_FAILURE;
		}
	}

	/*
	 * Reset device again before returning to the caller.
	 */
	XCanPs_Reset(InstancePtr);

	return XST_SUCCESS;
}

static void set_reset_mode(struct net_device *dev)
{
	struct xcanps_priv *InstancePtr = netdev_priv(dev);

	XCanPs_IntrDisable(InstancePtr, XCANPS_IXR_ALL);
	XCanPs_Reset(InstancePtr);
	InstancePtr->can.state = CAN_STATE_STOPPED;
	dev_info(dev->dev.parent, "set_reset_mode\n");
}

static int set_normal_mode(struct net_device *dev)
{
	struct xcanps_priv *InstancePtr = netdev_priv(dev);

	/*
	 * Config CAN bittiming
	 */
	xcanps_set_bittiming(dev);

	/*
	 * Set the interrupt handlers.
	 */
	XCanPs_SetHandler(InstancePtr, XCANPS_HANDLER_SEND,
			(void *)SendHandler, (void *)dev);
	XCanPs_SetHandler(InstancePtr, XCANPS_HANDLER_RECV,
			(void *)RecvHandler, (void *)dev);
	XCanPs_SetHandler(InstancePtr, XCANPS_HANDLER_ERROR,
			(void *)ErrorHandler, (void *)dev);
	XCanPs_SetHandler(InstancePtr, XCANPS_HANDLER_EVENT,
			(void *)EventHandler, (void *)dev);

	/*
	 * Enable interrupts in CAN device.
	 */
	XCanPs_IntrEnable(InstancePtr, XCANPS_IXR_WKUP_MASK
			| XCANPS_IXR_SLP_MASK
			| XCANPS_IXR_BSOFF_MASK
			| XCANPS_IXR_ERROR_MASK
			| XCANPS_IXR_RXOK_MASK
			| XCANPS_IXR_RXNEMP_MASK
			| XCANPS_IXR_TXOK_MASK
			| XCANPS_IXR_ARBLST_MASK);

	/*
	 * Enter Normal Mode.
	 */
	XCanPs_EnterMode(InstancePtr, XCANPS_MODE_NORMAL);
	while(XCanPs_GetMode(InstancePtr) != XCANPS_MODE_NORMAL);


	InstancePtr->can.state = CAN_STATE_ERROR_ACTIVE;
	dev_info(dev->dev.parent, "set_normal_mode\n");
	return XST_SUCCESS;
}

static int xcanps_set_bittiming(struct net_device *dev)
{
	struct xcanps_priv *InstancePtr = netdev_priv(dev);
	struct can_bittiming *bt = &InstancePtr->can.bittiming;
	int Status;
	u8 Prescaler, SyncJumpWidth, TimeSegment2, TimeSegment1;

	/*
	 * Enter Configuration Mode if the device is not currently in
	 * Configuration Mode.
	 */
	XCanPs_EnterMode(InstancePtr, XCANPS_MODE_CONFIG);
	while(XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG);


	/*
	 * The Baud rate Prescalar value in the Baud Rate Prescaler Register
	 * needs to be set based on the input clock  frequency to the CAN core and
	 * the desired CAN baud rate.
	 * This value is for a 500 kbps baudrate assuming the CAN input clock frequency
	 * is 100 MHz.
	 *
	 * tq = tosc*(BRP+1) = 10ns*(24+1) = 250ns(4MHz)
	 */
	Prescaler = 24;//bt->brp -1;	//25
	Status = XCanPs_SetBaudRatePrescaler(InstancePtr, Prescaler);

	/*
	 * Timing parameters to be set in the Bit Timing Register (BTR).
	 * These values are for a 500 kbps baudrate assuming the CAN input clock
	 * frequency is 100 MHz.
	 *
	 * tTSEG1 = tq*(TSEG1+1) = tq*(pro_s+pha_s1) = tq*(3+3)
	 * tTSEG2 = tq*(TSEG2+1) = tq*pha_s2 = tq*1
	 * tSJW = tq*(SJW+1) = tq*((1-1)+1
	 * t(Normal Bit Time) = tq*(sync+ts1+ts2) = tq*(6+1+1) = tq*8 = 2us(500kHz)
	 */
	SyncJumpWidth = bt->sjw -1;	//1
	TimeSegment1 = bt->phase_seg1 + bt-> prop_seg - 1;	//6
	TimeSegment2 = bt->phase_seg2 -1;	//1
	Status = XCanPs_SetBitTiming(InstancePtr, SyncJumpWidth,
			TimeSegment2, TimeSegment1);

	dev_info(dev->dev.parent, "xcanps_set_bittiming, "
			"Prescaler:%d, SyncJumpWidth:%d, TimeSegment1:%d, TimeSegment2:%d\n",
			(Prescaler+1), (SyncJumpWidth+1), (TimeSegment1+1), (TimeSegment2+1));
	return 0;
}

static int xcanps_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	struct xcanps_priv *InstancePtr = netdev_priv(dev);
	u32 ErrorCount;

	/*
	 * Read Error Counter Register and parse it.
	 */
	ErrorCount = XCanPs_ReadReg(InstancePtr->BaseAddr,
				XCANPS_ECR_OFFSET);
	bec->rxerr = (ErrorCount & XCANPS_ECR_REC_MASK) >>
				XCANPS_ECR_REC_SHIFT;
	bec->txerr = ErrorCount & XCANPS_ECR_TEC_MASK;

//	dev_info(dev->dev.parent,
//			"get_berr_counter rxerr=%d txerr=%d\n",
//			bec->rxerr, bec->txerr);

	return 0;
}

/*
 * transmit a CAN message
 * message layout in the sk_buff should be like this:
 * xx xx xx xx	 ff	 ll   00 11 22 33 44 55 66 77
 * [  can-id ] [flags] [len] [can data (up to 8 bytes]
 */
static netdev_tx_t xcanps_start_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	struct xcanps_priv *InstancePtr = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf = (struct can_frame *)skb->data;
	int i;
	int Status;
	u32 TxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];
	u8 *FramePtr;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	/*
	 * Create correct values for Identifier and Data Length Code Register.
	 */
	TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)cf->can_id, 0, 0, 0, 0);
	TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)cf->can_dlc);

	/*
	 * Now fill in the data field.
	 */
	FramePtr = (u8 *)(&TxFrame[2]);
	for (i = 0; i < cf->can_dlc; i++) {
		*FramePtr++ = cf->data[i];
	}

	/*
	 * Now send the frame.
	 */
	Status = XCanPs_Send(InstancePtr, TxFrame);

	/* tx_packets is incremented in flexcan_irq */
	stats->tx_bytes +=  cf->can_dlc;

	can_put_echo_skb(skb, dev, 0);

	dev_info(dev->dev.parent, "xcanps_start_xmit\n");
	return NETDEV_TX_OK;
}

irqreturn_t xcanps_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct xcanps_priv *CanPtr = netdev_priv(dev);
	u32 PendingIntr;
	u32 EventIntr;
	u32 ErrorStatus;

	/* Shared interrupts and IRQ off? */
	if(XCanPs_IntrGetEnabled(CanPtr) == 0)
		return IRQ_NONE;

	PendingIntr = XCanPs_IntrGetStatus(CanPtr);
	//printk("PendingIntr = 0x%x\n", PendingIntr);

	PendingIntr &= XCanPs_IntrGetEnabled(CanPtr);
	//printk("Enabled PendingIntr = 0x%x\n", PendingIntr);

	/*
	 * Clear all enabled pending interrupts.
	 * Rising Edge interrupt
	 */
	XCanPs_IntrClear(CanPtr, PendingIntr);

	/*
	 * An error interrupt is occurring.
	 */
	if ((PendingIntr & XCANPS_IXR_ERROR_MASK)) {
		dev_info(dev->dev.parent, "error\n");
		ErrorStatus = XCanPs_GetBusErrorStatus(CanPtr);
		CanPtr->ErrorHandler(CanPtr->ErrorRef, ErrorStatus);

		/*
		 * Clear Error Status Register.
		 */
		XCanPs_ClearBusErrorStatus(CanPtr, ErrorStatus);
	}

	/*
	 * Check if any following event interrupt is pending:
	 *	  - RX FIFO Overflow
	 *	  - RX FIFO Underflow
	 *	  - TX High Priority Buffer full
	 *	  - TX FIFO Full
	 *	  - Wake up from sleep mode
	 *	  - Enter sleep mode
	 *	  - Enter Bus off status
	 *	  - Arbitration is lost
	 *
	 * If so, call event callback provided by upper level.
	 */
	EventIntr = PendingIntr & (XCANPS_IXR_RXOFLW_MASK |
				XCANPS_IXR_RXUFLW_MASK |
				XCANPS_IXR_TXBFLL_MASK |
				XCANPS_IXR_TXFLL_MASK |
				XCANPS_IXR_WKUP_MASK |
				XCANPS_IXR_SLP_MASK |
				XCANPS_IXR_BSOFF_MASK |
				XCANPS_IXR_ARBLST_MASK);
	if (EventIntr) {
		dev_info(dev->dev.parent, "event\n");
		CanPtr->EventHandler(CanPtr->EventRef, EventIntr);

		if ((EventIntr & XCANPS_IXR_BSOFF_MASK)) {
			/*
			 * Event callback should reset whole device if "Enter
			 * Bus Off Status" interrupt occurred. All pending
			 * interrupts are cleared and no further checking and
			 * handling of other interrupts is needed any more.
			 */
			//return IRQ_HANDLED;
		}
	}


	if ((PendingIntr & (XCANPS_IXR_RXFWMFLL_MASK |
			XCANPS_IXR_RXNEMP_MASK))) {

		/*
		 * This case happens when
		 * A number of frames depending on the Rx FIFO Watermark
		 * threshold are received.
		 * And  also when frame was received and is sitting in RX FIFO.
		 *
		 * XCANPS_IXR_RXOK_MASK is not used because the bit is set
		 * just once even if there are multiple frames sitting
		 * in the RX FIFO.
		 *
		 * XCANPS_IXR_RXNEMP_MASK is used because the bit can be
		 * set again and again automatically as long as there is
		 * at least one frame in RX FIFO.
		 */
		dev_info(dev->dev.parent, "receive\n");
		CanPtr->RecvHandler(CanPtr->RecvRef);
	}

	/*
	 * A frame was transmitted successfully.
	 */
	if ((PendingIntr & XCANPS_IXR_TXOK_MASK)) {
		dev_info(dev->dev.parent, "transmit\n");
		CanPtr->SendHandler(CanPtr->SendRef);
	}

	dev_info(dev->dev.parent, "xcanps_interrupt\n");
	return IRQ_HANDLED;
}
//EXPORT_SYMBOL_GPL(xcanps_interrupt);

static int xcanps_open(struct net_device *dev)
{
	struct xcanps_priv *priv = netdev_priv(dev);
	int err;

	/* common open */
	err = open_candev(dev);
	if (err)
		return err;

	/* register interrupt handler, if not done by the device driver */
	err = request_irq(dev->irq, xcanps_interrupt, priv->irq_flags,
			  dev->name, (void *)dev);
	if (err) {
		close_candev(dev);
		return -EAGAIN;
	}

	priv->open_time = jiffies;

	err = set_normal_mode(dev);
	if (err != XST_SUCCESS) {

	}
	netif_start_queue(dev);
	dev_info(dev->dev.parent, "xcanps_open\n");
	return 0;
}

static int xcanps_close(struct net_device *dev)
{
	struct xcanps_priv *priv = netdev_priv(dev);

	priv->open_time = 0;
	netif_stop_queue(dev);
	set_reset_mode(dev);
	free_irq(dev->irq, (void *)dev);
	close_candev(dev);

	dev_info(dev->dev.parent, "xcanps_close\n");
	return 0;
}

static const struct net_device_ops xcanps_netdev_ops = {
       .ndo_open               = xcanps_open,
       .ndo_stop               = xcanps_close,
       .ndo_start_xmit         = xcanps_start_xmit,
};

static int __devinit xcanps_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct xcanps_priv *priv;
	struct resource res;
	void __iomem *base;
	const u32 *prop;
	int err, irq, res_size;
	u32 clk;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		dev_err(&pdev->dev, "invalid address\n");
		return err;
	}

	res_size = resource_size(&res);

	if (!request_mem_region(res.start, res_size, pdev->name)) {
		dev_err(&pdev->dev, "couldn't request %pR\n", &res);
		return -EBUSY;
	}

	base = ioremap(res.start, res_size);
	if (!base) {
		dev_err(&pdev->dev, "couldn't ioremap %pR\n", &res);
		err = -ENOMEM;
		goto exit_release_mem;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq == NO_IRQ) {
		dev_err(&pdev->dev, "no irq found\n");
		err = -ENODEV;
		goto exit_unmap_mem;
	}

	prop = of_get_property(np, "xlnx,can-clk-freq-hz", NULL);
	if (prop)
		clk = be32_to_cpup(prop);
	else {
		dev_err(&pdev->dev, "couldn't determine input-clk\n");
		goto exit_dispose_irq ;
	}

	dev = alloc_candev(sizeof(struct xcanps_priv), XCANPS_ECHO_SKB_MAX);
	if (!dev) {
		err = -ENOMEM;
		goto exit_dispose_irq;
	}

	dev->netdev_ops = &xcanps_netdev_ops;
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->irq = irq;

	priv = netdev_priv(dev);
	priv->irq_flags = IRQF_SHARED;
	priv->BaseAddr = base;
	priv->dev = dev;
	priv->can.clock.freq = clk;
	priv->can.bittiming_const = &xcanps_bittiming_const;
	//	priv->can.do_set_bittiming = xcanps_set_bittiming;
	//	priv->can.do_set_mode = xcanps_set_mode;
	//	priv->can.do_get_state = xcanps_get_state;
	priv->can.do_get_berr_counter = xcanps_get_berr_counter;
	//	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK
	//			| CAN_CTRLMODE_LISTENONLY
	//			| CAN_CTRLMODE_3_SAMPLES
	//			| CAN_CTRLMODE_BERR_REPORTING;
	//	spin_lock_init(&priv->cmdreg_lock);

	dev_set_drvdata(&pdev->dev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_candev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
				DRV_NAME, err);
		goto exit_free_xcanps;
	}

	//hardware setup, only run self test here
	err = XCanPs_SelfTest(priv);
	if (err != XST_SUCCESS) {
		dev_err(dev->dev.parent, "XCanPs_SelfTest failed!\n");
		err = -ENODEV;
		goto exit_free_xcanps;
	}

	dev_info(&pdev->dev,
		 "reg_base=0x%p irq=%d clock=%d\n",
		 priv->BaseAddr, dev->irq, priv->can.clock.freq);

	return 0;

exit_free_xcanps:
	free_candev(dev);
exit_dispose_irq:
	irq_dispose_mapping(irq);
exit_unmap_mem:
	iounmap(base);
exit_release_mem:
	release_mem_region(res.start, res_size);
	return err;
}

static int __devexit xcanps_remove(struct platform_device *pdev)
{
	struct net_device *dev = dev_get_drvdata(&pdev->dev);
	struct xcanps_priv *priv = netdev_priv(dev);
	struct device_node *np = pdev->dev.of_node;
	struct resource res;

	set_reset_mode(dev);
	unregister_candev(dev);
	dev_set_drvdata(&pdev->dev, NULL);
	irq_dispose_mapping(dev->irq);
	iounmap(priv->BaseAddr);
	of_address_to_resource(np, 0, &res);
	release_mem_region(res.start, resource_size(&res));

	return 0;
}

static struct of_device_id xcanps_of_match[] __devinitdata = {
	{ .compatible = "xlnx,ps7-can-1.00.a", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xcanps_of_match);

static struct platform_driver xcanps_drv = {
	.driver = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xcanps_of_match,
	},
	.probe  = xcanps_probe,
	.remove = __devexit_p(xcanps_remove),
};

/**
 * xcanps_init - Initial driver registration function
 * Returns zero on success, otherwise negative error.
 */
static int __init xcanps_init(void)
{
	printk(KERN_INFO "%s netdevice driver\n", DRV_NAME);
	return platform_driver_register(&xcanps_drv);
}
module_init(xcanps_init);

/**
 * xcanps_exit - Driver Un-registration function
 */
static void __exit xcanps_exit(void)
{
	printk(KERN_INFO "%s: driver removed\n", DRV_NAME);
	platform_driver_unregister(&xcanps_drv);
}
module_exit(xcanps_exit);


MODULE_AUTHOR("ZhouPeng<zp_caams@163.com>");
MODULE_DESCRIPTION("Xilinx PS CAN bus driver");
MODULE_LICENSE("GPL");
