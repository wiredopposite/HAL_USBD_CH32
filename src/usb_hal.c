#include <string.h>
#include <stdio.h>
#include "ch32v20x.h"
#include "usbd_ll.h"
#include "usbd_hal.h"

#define PMA_DATA_START 0x40U
#define PMA_SIZE       512U

#ifndef BIT
#define BIT(x) (1U << (x))
#endif

#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM  | CNTR_SOFM \
                | CNTR_ESOFM | CNTR_RESETM )

#define USBD_EPS_MAX                    8U
#define USBD_EPNUM_Msk                  (USBD_EPS_MAX - 1)
#define USBD_EP_NUM(epaddr)             ((uint8_t)((epaddr) & USBD_EPNUM_Msk))

#define USB_REQ_DIR_Msk                 (1U << 7)   /* Request direction mask.*/
#define USB_REQ_DIR_HOSTTODEV           (0U << 7)   /* Request direction is HOST to DEVICE.*/
#define USB_REQ_DIR_DEVTOHOST           (1U << 7)   /* Request direction is DEVICE to HOST.*/

#define USB_REQ_TYPE_Msk                (3U << 5)   /* Request type mask.*/
#define USB_REQ_TYPE_STANDARD           (0U << 5)   /* Standard request.*/
#define USB_REQ_TYPE_CLASS              (1U << 5)   /* Class specified request.*/
#define USB_REQ_TYPE_VENDOR             (2U << 5)   /* Vendor specified request.*/

#define USB_REQ_RECIP_Msk               (3U << 0)   /* Request recipient mask.*/
#define USB_REQ_RECIP_DEVICE            (0U << 0)   /* Request to device.*/
#define USB_REQ_RECIP_INTERFACE         (1U << 0)   /* Request to interface.*/
#define USB_REQ_RECIP_ENDPOINT          (2U << 0)   /* Request to endpoint.*/
#define USB_REQ_RECIP_OTHER             (3U << 0)   /* Other request.*/

/* USB Standard requests */
#define USB_REQ_STD_GET_STATUS          0x00    /* Returns status for the specified recipient.*/
#define USB_REQ_STD_CLEAR_FEATURE       0x01    /* Used to clear or disable a specific feature.*/
#define USB_REQ_STD_SET_FEATURE         0x03    /* Used to set or enable a specific feature.*/
#define USB_REQ_STD_SET_ADDRESS         0x05    /* Sets the device address for all future device accesses.*/
#define USB_REQ_STD_GET_DESCRIPTOR      0x06    /* Returns the specified descriptor if the descriptor exists.*/
#define USB_REQ_STD_SET_DESCRIPTOR      0x07    /* This request is optional and may be used to update existing descriptors or new descriptors may be added.*/
#define USB_REQ_STD_GET_CONFIG          0x08    /* Returns the current device configuration value.*/
#define USB_REQ_STD_SET_CONFIG          0x09    /* Sets the device configuration.*/
#define USB_REQ_STD_GET_INTERFACE       0x0A    /* Returns the selected alternate setting for the specified interface.*/
#define USB_REQ_STD_SET_INTERFACE       0x0B    /* Allows the host to select an alternate setting for the specified interface.*/
#define USB_REQ_STD_SYNCH_FRAME         0x0C    /* Used to set and then report an endpoint's synchronization frame.*/

#define USB_EP_TYPE_CONTROL             0x00    /* Control endpoint.*/
#define USB_EP_TYPE_ISOCHRONUS          0x01    /* Isochronous endpoint.*/
#define USB_EP_TYPE_BULK                0x02    /* Bbulk endpoint.*/
#define USB_EP_TYPE_INTERRUPT           0x03    /* Interrupt endpoint.*/
#define USB_EP_TYPE_DBLBUF              0x04    /* Doublebuffered endpoint.*/

#define USB_EP_ATTR_NO_SYNC             0x00    /* No synchronization.*/
#define USB_EP_ATTR_ASYNC               0x04    /* Asynchronous endpoint.*/
#define USB_EP_ATTR_ADAPTIVE            0x08    /* Adaptive endpoint.*/
#define USB_EP_ATTR_SYNC                0x0C    /* Synchronous endpoint.*/

/* USB Standard descriptor types */
#define USB_DTYPE_DEVICE                0x01    /* Device descriptor.*/
#define USB_DTYPE_CONFIGURATION         0x02    /* Configuration descriptor.*/
#define USB_DTYPE_STRING                0x03    /* String descriptor.*/
#define USB_DTYPE_INTERFACE             0x04    /* Interface descriptor.*/
#define USB_DTYPE_ENDPOINT              0x05    /* Endpoint  descriptor.*/
#define USB_DTYPE_QUALIFIER             0x06    /* Qualifier descriptor.*/
#define USB_DTYPE_OTHER                 0x07    /* Descriptor is of other type. */
#define USB_DTYPE_INTERFACEPOWER        0x08    /* Interface power descriptor. */
#define USB_DTYPE_OTG                   0x09    /* OTG descriptor.*/
#define USB_DTYPE_DEBUG                 0x0A    /* Debug descriptor.*/
#define USB_DTYPE_IAD                   0x0B    /* Interface association descriptor.*/
#define USB_DTYPE_CS_INTERFACE          0x24    /* Class specific interface descriptor.*/
#define USB_DTYPE_CS_ENDPOINT           0x25    /* Class specific endpoint descriptor.*/
#define USB_DTYPE_HUB                   0x29    /* Hub descriptor.*/

typedef struct __attribute__((packed)) _USB_DescDevice {
    uint8_t  bLength;           
    uint8_t  bDescriptorType;   
    uint16_t bcdUSB;            
    uint8_t  bDeviceClass;      
    uint8_t  bDeviceSubClass;   
    uint8_t  bDeviceProtocol;   
    uint8_t  bMaxPacketSize0;   
    uint16_t idVendor;          
    uint16_t idProduct;         
    uint16_t bcdDevice;         
    uint8_t  iManufacturer;     
    uint8_t  iProduct;          
    uint8_t  iSerialNumber;     
    uint8_t  bNumConfigurations;
} USB_DescDevice;

typedef struct __attribute__((packed)) _USB_DescConfig {
    uint8_t  bLength;             
    uint8_t  bDescriptorType;     
    uint16_t wTotalLength;        
    uint8_t  bNumInterfaces;      
    uint8_t  bConfigurationValue; 
    uint8_t  iConfiguration;      
    uint8_t  bmAttributes;        
    uint8_t  bMaxPower;           
} USB_DescConfig;

typedef struct __attribute__((packed)) _USB_DescInterface {
    uint8_t bLength;           
    uint8_t bDescriptorType;   
    uint8_t bInterfaceNumber;  
    uint8_t bAlternateSetting; 
    uint8_t bNumEndpoints;     
    uint8_t bInterfaceClass;   
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;        
} USB_DescInterface;

typedef struct __attribute__((packed)) _USB_DescEndpoint {
    uint8_t  bLength;          
    uint8_t  bDescriptorType;  
    uint8_t  bEndpointAddress; 
    uint8_t  bmAttributes;     
    uint16_t wMaxPacketSize;   
    uint8_t  bInterval;        
} USB_DescEndpoint;

static void HAL_USBD_SIL_Init(HAL_USBD_Ctx* ctx);
static void HAL_USBD_PowerOn(HAL_USBD_Ctx* ctx);
static void HAL_USBD_PowerOff(HAL_USBD_Ctx* ctx);
static void HAL_USBD_Enter_LowPowerMode(HAL_USBD_Ctx* ctx);
static void HAL_USBD_Leave_LowPowerMode(HAL_USBD_Ctx* ctx);
static void HAL_USBD_Suspend(HAL_USBD_Ctx* ctx);
static void HAL_USBD_Resume_Init(HAL_USBD_Ctx* ctx);
static void HAL_USBD_Resume(HAL_USBD_Ctx* ctx, USBD_ResumeState resumeState);

static void HAL_USBD_CTR_LP(HAL_USBD_Ctx* ctx);
static void HAL_USBD_CTR_HP(HAL_USBD_Ctx* ctx);

static void HAL_USBD_HandleReset(HAL_USBD_Ctx* ctx);
static bool HAL_USBD_HandleCtrlReq(HAL_USBD_Ctx* ctx);
static void HAL_USBD_HandleCtrlOut(HAL_USBD_Ctx* ctx);
static void HAL_USBD_HandleCtrlIn(HAL_USBD_Ctx* ctx);
static void HAL_USBD_HandleCtrlStall(HAL_USBD_Ctx* ctx);
static bool HAL_USBD_HandleSetConfigReq(HAL_USBD_Ctx* ctx);
static bool HAL_USBD_HandleStdReq(HAL_USBD_Ctx* ctx);
static void HAL_USBD_SetAddressComplete_Cb(HAL_USBD_Ctx* ctx, const USB_SetupReq* req, const uint8_t* data, uint16_t len);
static void HAL_USBD_SetConfigComplete_Cb(HAL_USBD_Ctx* ctx, const USB_SetupReq* req, const uint8_t* data, uint16_t len);
static void HAL_USBD_EPOpen(HAL_USBD_Ctx* ctx, uint8_t epAddr, uint16_t epMps, uint8_t epType);
static void HAL_USBD_EPClose(HAL_USBD_Ctx* ctx, uint8_t epAddr);
static void HAL_USBD_EPSetStall(HAL_USBD_Ctx* ctx, uint8_t epAddr, bool stall);
static bool HAL_USBD_EPStalled(HAL_USBD_Ctx* ctx, uint8_t epAddr);

/* Low level methods */

static void HAL_USBD_SIL_Init(HAL_USBD_Ctx* ctx) {
    USBD_LL_SetISTR(0);
    ctx->interruptMask = IMR_MSK;
    USBD_LL_SetCNTR(ctx->interruptMask);
}

static void HAL_USBD_PowerOn(HAL_USBD_Ctx* ctx) {
    uint16_t wRegVal;
    wRegVal = CNTR_FRES;
    USBD_LL_SetCNTR(wRegVal);
    ctx->interruptMask = 0;
    USBD_LL_SetCNTR(ctx->interruptMask);
    USBD_LL_SetISTR(0);
    ctx->interruptMask = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;
    USBD_LL_SetCNTR(ctx->interruptMask);
}

static void HAL_USBD_PowerOff(HAL_USBD_Ctx* ctx) {
    USBD_LL_SetCNTR(CNTR_FRES); 
    USBD_LL_SetISTR(0); 
    USBD_LL_SetCNTR(CNTR_FRES + CNTR_PDWN);
}   

static void HAL_USBD_Enter_LowPowerMode(HAL_USBD_Ctx* ctx) {
    printf("usb enter low power mode\r\n"); 
    ctx->devState = USBD_SUSPENDED;
    // if (HAL_USBD_Suspend_Cb) {
    //     HAL_USBD_Suspend_Cb(ctx);
    // }
}

static void HAL_USBD_Leave_LowPowerMode(HAL_USBD_Ctx* ctx) {
	printf("usb leave low power mode\r\n"); 
	if (ctx->currentCfg != 0) {
        ctx->devState = USBD_CONFIGURED; 
    } else {
        ctx->devState = USBD_ATTACHED; 
    }
} 

static void HAL_USBD_Suspend(HAL_USBD_Ctx* ctx) {
	uint32_t i =0;
	uint16_t wCNTR; 
    uint16_t EP[8];

	wCNTR = USBD_LL_GetCNTR();  
    for (i = 0; i < USBD_EPS_MAX; i++) {
        EP[i] = USBD_LL_GetENDPOINT(i);
    }
	
	wCNTR|=CNTR_RESETM;
	USBD_LL_SetCNTR(wCNTR);
	
	wCNTR|=CNTR_FRES;
	USBD_LL_SetCNTR(wCNTR);
	
	wCNTR&=~CNTR_FRES;
	USBD_LL_SetCNTR(wCNTR);

	while((USBD_LL_GetISTR() & ISTR_RESET) == 0);
	
	USBD_LL_SetISTR((uint16_t)CLR_RESET);
	
	for (i = 0; i < USBD_EPS_MAX; i++) {
        USBD_LL_SetENDPOINT(i, EP[i]);
    }
	
	wCNTR |= CNTR_FSUSP;
	USBD_LL_SetCNTR(wCNTR);
	
	wCNTR = USBD_LL_GetCNTR();
	wCNTR |= CNTR_LPMODE;
	USBD_LL_SetCNTR(wCNTR);

	HAL_USBD_Enter_LowPowerMode(ctx);
}

static void HAL_USBD_Resume_Init(HAL_USBD_Ctx* ctx) {
    uint16_t wCNTR;
    wCNTR = USBD_LL_GetCNTR();
    wCNTR &= (~CNTR_LPMODE);
    USBD_LL_SetCNTR(wCNTR);
    HAL_USBD_Leave_LowPowerMode(ctx);
    USBD_LL_SetCNTR(IMR_MSK);
}

static void HAL_USBD_Resume(HAL_USBD_Ctx* ctx, USBD_ResumeState resumeState) {
    uint16_t wCNTR;
    
    if (resumeState != USBD_RESUME_ESOF) {
        ctx->hw.resumeState = resumeState;
    }
        
    switch (ctx->hw.resumeState) {
    case USBD_RESUME_EXTERNAL:
        if (ctx->hw.remoteWakeupOn == 0) {
            HAL_USBD_Resume_Init(ctx);
            ctx->hw.resumeState = USBD_RESUME_OFF;
        } else {
            ctx->hw.resumeState = USBD_RESUME_ON;
        }
        break;
            
    case USBD_RESUME_INTERNAL:
        HAL_USBD_Resume_Init(ctx);
        ctx->hw.resumeState = USBD_RESUME_START;
        ctx->hw.remoteWakeupOn = 1;
        break;
        
    case USBD_RESUME_LATER:
        ctx->hw.resumeSofCount = 2;
        ctx->hw.resumeState = USBD_RESUME_WAIT;
        break;
        
    case USBD_RESUME_WAIT:
        ctx->hw.resumeSofCount--;
        if (ctx->hw.resumeSofCount == 0) {
            ctx->hw.resumeState = USBD_RESUME_START;
        }
        break;
            
    case USBD_RESUME_START:
        wCNTR = USBD_LL_GetCNTR();
        wCNTR |= CNTR_RESUME;
        USBD_LL_SetCNTR(wCNTR);
        ctx->hw.resumeState = USBD_RESUME_ON;
        ctx->hw.resumeSofCount = 10;
        break;
        
    case USBD_RESUME_ON:    
        ctx->hw.resumeSofCount--;
        if (ctx->hw.resumeSofCount == 0) {
            wCNTR = USBD_LL_GetCNTR();
            wCNTR &= (~CNTR_RESUME);
            USBD_LL_SetCNTR(wCNTR);
            ctx->hw.resumeState = USBD_RESUME_OFF;
            ctx->hw.remoteWakeupOn = 0;
        }
        break;
            
    case USBD_RESUME_OFF:
            
    case USBD_RESUME_ESOF:
            
    default:
        ctx->hw.resumeState = USBD_RESUME_OFF;
        break;
    }
}

void HAL_USBD_RxIrqHandler(HAL_USBD_Ctx* ctx) {
    uint32_t i = 0;
    __IO uint16_t wCNTR;
    __IO uint16_t wIstr;
    __IO uint32_t EP[8];

    if ((*USBD_LL_GetEPRxCountReg(0) & 0xFC00 )!= ctx->hw.ctrlRxBlocks) {
        *USBD_LL_GetEPRxCountReg(0) |= (ctx->hw.ctrlRxBlocks & 0xFC00);
    }
    wIstr = USBD_LL_GetISTR();
    if (wIstr & ISTR_SOF & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_SOF);
        ctx->hw.bIntPackSOF++;
        // SOF_Callback();
    }

    if (wIstr & ISTR_CTR & ctx->interruptMask) {
        HAL_USBD_CTR_LP(ctx);
        // CTR_Callback();
    }
 
    if (wIstr & ISTR_RESET & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_RESET);
        HAL_USBD_HandleReset(ctx);
        // RESET_Callback();
    }

    if (wIstr & ISTR_DOVR & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_DOVR);
        // DOVR_Callback();
    }

    if (wIstr & ISTR_ERR & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_ERR);
        // ERR_Callback();
    }

    if (wIstr & ISTR_WKUP & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_WKUP);
        HAL_USBD_Resume(ctx, USBD_RESUME_EXTERNAL);
    }

    if (wIstr & ISTR_SUSP & ctx->interruptMask) {
        if (HAL_USBD_SUSPEND_ENABLED) {
            HAL_USBD_Suspend(ctx);
        } else {
            HAL_USBD_Resume(ctx, USBD_RESUME_LATER);
        }
        USBD_LL_SetISTR((uint16_t)CLR_SUSP);
    }

    if (wIstr & ISTR_ESOF & ctx->interruptMask) {
        USBD_LL_SetISTR((uint16_t)CLR_ESOF);
        
        if ((USBD_LL_GetFNR() & FNR_RXDP) != 0)  {
            ctx->hw.sofCount++;
            
            if ((ctx->hw.sofCount > 3) && ((USBD_LL_GetCNTR() & CNTR_FSUSP) == 0)) {           
                wCNTR = USBD_LL_GetCNTR(); 
            
                for (i = 0; i < USBD_EPS_MAX; i++) {
                    EP[i] = USBD_LL_GetENDPOINT(i);
                }
            
                wCNTR |= CNTR_FRES;
                USBD_LL_SetCNTR(wCNTR);
        
                wCNTR &= ~CNTR_FRES;
                USBD_LL_SetCNTR(wCNTR);
            
                while ((USBD_LL_GetISTR() & ISTR_RESET) == 0);
        
                USBD_LL_SetISTR((uint16_t)CLR_RESET);
        
                for (i = 0; i < USBD_EPS_MAX; i++) {
                    USBD_LL_SetENDPOINT(i, EP[i]);
                }
            
                ctx->hw.sofCount = 0;
            }
        } else {
            ctx->hw.sofCount = 0;
        }

        HAL_USBD_Resume(ctx, USBD_RESUME_ESOF);
    }
}

void HAL_USBD_WakeupIrqHandler(HAL_USBD_Ctx* ctx, uint32_t extiLine) {
    EXTI_ClearITPendingBit(EXTI_Line18);
}

static void HAL_USBD_CTR_LP(HAL_USBD_Ctx* ctx) {
    __IO uint16_t wEPVal = 0;
    __IO uint8_t EPindex = 0;
    __IO uint16_t wIstr = 0;
        
    while (((wIstr = USBD_LL_GetISTR()) & ISTR_CTR) != 0) {
        EPindex = (uint8_t)(wIstr & ISTR_EP_ID);
            
        if (EPindex == 0) {
            ctx->ctrlRxStatus = USBD_LL_GetENDPOINT(ENDP0);
            ctx->ctrlTxStatus = ctx->ctrlRxStatus & EPTX_STAT;
            ctx->ctrlRxStatus &= EPRX_STAT;	

            USBD_LL_SetEPRxTxStatus(ENDP0, EP_RX_NAK, EP_TX_NAK);
                
            if ((wIstr & ISTR_DIR) == 0) {
                USBD_LL_ClearEP_CTR_TX(ENDP0);
                HAL_USBD_HandleCtrlIn(ctx);
                USBD_LL_SetEPRxTxStatus(ENDP0, ctx->ctrlRxStatus, ctx->ctrlTxStatus);  
                return;
            } else {
                wEPVal = USBD_LL_GetENDPOINT(ENDP0);
                if ((wEPVal & EP_SETUP) != 0) {
                    USBD_LL_ClearEP_CTR_RX(ENDP0);
                    ctx->ctrlState = USBD_CTRL_IDLE;
                    HAL_USBD_HandleCtrlOut(ctx);
                    USBD_LL_SetEPRxTxStatus(ENDP0, ctx->ctrlRxStatus, ctx->ctrlTxStatus);
                    return;
                } else if ((wEPVal & EP_CTR_RX) != 0) {
                    USBD_LL_ClearEP_CTR_RX(ENDP0);
                    HAL_USBD_HandleCtrlOut(ctx);
                    USBD_LL_SetEPRxTxStatus(ENDP0, ctx->ctrlRxStatus, ctx->ctrlTxStatus);
                    return;
                }
            }
        } else {
            wEPVal = USBD_LL_GetENDPOINT(EPindex);
            if ((wEPVal & EP_CTR_RX) != 0)  {
                USBD_LL_ClearEP_CTR_RX(EPindex);
                ctx->userDriver.epComplete(ctx, USBD_EP_NUM(EPindex));
            } 

            if ((wEPVal & EP_CTR_TX) != 0) {
                USBD_LL_ClearEP_CTR_TX(EPindex);  
                ctx->epInBusy &= ~BIT(USBD_EP_NUM(EPindex));   
                ctx->userDriver.epComplete(ctx, USBD_EP_NUM(EPindex) | 0x80);
            }
        }
    }
}

static void HAL_USBD_CTR_HP(HAL_USBD_Ctx* ctx) {
    __IO uint32_t wEPVal = 0;
    __IO uint16_t wIstr = 0;
    __IO uint8_t EPindex = 0;

    while (((wIstr = USBD_LL_GetISTR()) & ISTR_CTR) != 0) {
        USBD_LL_SetISTR((uint16_t)CLR_CTR); 
        EPindex = (uint8_t)(wIstr & ISTR_EP_ID);   
        wEPVal = USBD_LL_GetENDPOINT(EPindex);
            
        if ((wEPVal & EP_CTR_RX) != 0) {
            USBD_LL_ClearEP_CTR_RX(EPindex); 
            ctx->userDriver.epComplete(ctx, EPindex);
        } else if ((wEPVal & EP_CTR_TX) != 0) {
            USBD_LL_ClearEP_CTR_TX(EPindex);   
            ctx->userDriver.epComplete(ctx, EPindex | 0x80);
        } 
    }
}

/* USBD Stack Methods */

static void HAL_USBD_EPClose(HAL_USBD_Ctx* ctx, uint8_t epAddr) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    if (epNum >= USBD_EPS_MAX) {
        return;
    }
    if (epAddr & 0x80) {
        if (!(ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum))) {
            return;
        }
        USBD_LL_SetEPTxStatus(epNum, EP_TX_DIS);
        USBD_LL_SetENDPOINT(epNum, USBD_LL_GetENDPOINT(epNum) & 0x7F7F & EPREG_MASK);
        USBD_LL_SetISTR((uint16_t)0x00FF);
        ctx->epSize[USBD_EPDIR_IN][epNum] = 0;
        ctx->epType[USBD_EPDIR_IN][epNum] = 0;
        ctx->epOpen[USBD_EPDIR_IN] &= ~BIT(epNum);
        ctx->epInBusy &= ~BIT(epNum);
    } else {
        if (!(ctx->epOpen[USBD_EPDIR_OUT] & BIT(epNum))) {
            return;
        }
        USBD_LL_SetEPRxStatus(epNum, EP_RX_DIS);
        USBD_LL_SetENDPOINT(epNum, USBD_LL_GetENDPOINT(epNum) & 0x7F7F & EPREG_MASK);
        USBD_LL_SetISTR((uint16_t)0x00FF);
        ctx->epSize[USBD_EPDIR_OUT][epNum] = 0;
        ctx->epType[USBD_EPDIR_OUT][epNum] = 0;
        ctx->epOpen[USBD_EPDIR_OUT] &= ~BIT(epNum);
    }
}

static void HAL_USBD_EPOpen(HAL_USBD_Ctx* ctx, uint8_t epAddr, uint16_t epMps, uint8_t epAttr) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    epMps = ((epMps & 0x7FFU) + 1U) & ~1U; /* Align 2 bytes */
    epAttr &= 0x03U;

    if (epNum >= USBD_EPS_MAX) {
        return;
    }

    if (epNum == 0) {
        ctx->pmaOffset = PMA_DATA_START;
        USBD_LL_SetEPAttr(0, USB_EP_TYPE_CONTROL);
        USBD_LL_SetEPTxStatus(0, EP_TX_STALL);
        USBD_LL_SetEPRxAddr(0, ctx->pmaOffset);
        ctx->pmaOffset += epMps;
        USBD_LL_SetEPTxAddr(0, ctx->pmaOffset);
        ctx->pmaOffset += epMps;
        USBD_LL_ClearStatusOut(0);
        USBD_LL_SetEPRxCount(0, epMps);
        ctx->epSize[USBD_EPDIR_IN][0] = epMps;
        ctx->epSize[USBD_EPDIR_OUT][0] = epMps;
        ctx->epType[USBD_EPDIR_IN][0] = USB_EP_TYPE_CONTROL;
        ctx->epType[USBD_EPDIR_OUT][0] = USB_EP_TYPE_CONTROL;
        ctx->epOpen[USBD_EPDIR_IN] |= BIT(0);
        ctx->epOpen[USBD_EPDIR_OUT] |= BIT(0);
        USBD_LL_SetEPRxStatus(0, EP_RX_VALID);
        USBD_LL_ClearDTOG_RX(0);
        USBD_LL_ClearDTOG_TX(0);
        return;
    }

    if ((ctx->pmaOffset + epMps) > PMA_SIZE) {
        printf("Error: No more room in PMA for EP%d\r\n", epNum);
        return;
    }

    if (epAddr & 0x80) {
        USBD_LL_SetEPAttr(epNum, epAttr);
        USBD_LL_SetEPTxAddr(epNum, ctx->pmaOffset);
        ctx->pmaOffset += epMps;
        USBD_LL_SetEPTxStatus(epNum, EP_TX_NAK);
        USBD_LL_ClearDTOG_TX(epNum);
        ctx->epInBusy &= ~BIT(epNum);
        ctx->epSize[USBD_EPDIR_IN][epNum] = epMps;
        ctx->epType[USBD_EPDIR_IN][epNum] = epAttr;
        ctx->epOpen[USBD_EPDIR_IN] |= BIT(epNum);
    } else {
        USBD_LL_SetEPAttr(epNum, epAttr);
        USBD_LL_SetEPRxAddr(epNum, ctx->pmaOffset);
        ctx->pmaOffset += epMps;
        USBD_LL_SetEPRxCount(epNum, epMps);
        USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
        USBD_LL_ClearDTOG_RX(epNum);
        ctx->epSize[USBD_EPDIR_OUT][epNum] = epMps;
        ctx->epType[USBD_EPDIR_OUT][epNum] = epAttr;
        ctx->epOpen[USBD_EPDIR_OUT] |= BIT(epNum);
    }
}

static void HAL_USBD_EPSetStall(HAL_USBD_Ctx* ctx, uint8_t epAddr, bool stall) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    if (epNum >= USBD_EPS_MAX) {
        return;
    }
    if (epNum == 0) {
        HAL_USBD_HandleCtrlStall(ctx);
        return;
    }
    if (ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum)) {
        if (stall) {
            USBD_LL_SetEPTxStatus(epNum, EP_TX_STALL);
        } else {
            ctx->epInBusy &= ~BIT(epNum);
            USBD_LL_ClearDTOG_TX(epNum);
            USBD_LL_SetEPTxStatus(epNum, EP_TX_NAK);
        }
    }
    if (ctx->epOpen[USBD_EPDIR_OUT] & BIT(epNum)) {
        if (stall) {
            USBD_LL_SetEPRxStatus(epNum, EP_RX_STALL);
        } else {
            USBD_LL_ClearDTOG_RX(epNum);
            USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
        }
    }
}

static bool HAL_USBD_EPStalled(HAL_USBD_Ctx* ctx, uint8_t epAddr) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    uint16_t epReg = 0;
    if (epNum >= USBD_EPS_MAX) {
        return false;
    }
    if ((ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum))) {
        epReg = USBD_LL_GetENDPOINT(epNum);
        if ((epReg & EPTX_STAT) == EP_TX_STALL) {
            return true;
        }
    }
    if ((ctx->epOpen[USBD_EPDIR_OUT] & BIT(epNum))) {
        epReg = USBD_LL_GetENDPOINT(epNum);
        if ((epReg & EPRX_STAT) == EP_RX_STALL) {
            return true;
        }
    }
    return false;
}

static void HAL_USBD_HandleReset(HAL_USBD_Ctx* ctx) {
    printf("USB RESET\r\n");
    ctx->devState = USBD_ATTACHED;
    ctx->ctrlState = USBD_CTRL_IDLE;
    ctx->currentCfg = 0;
    ctx->userDriver.deinit(ctx);
    for (uint8_t i = 0; i < USBD_EPS_MAX; i++) {
        HAL_USBD_EPClose(ctx, i);
        HAL_USBD_EPClose(ctx, i | 0x80);
    }
    USBD_LL_SetDeviceAddress(0);
    HAL_USBD_EPOpen(ctx, 0x00, ctx->maxPacketSize, USB_EP_TYPE_CONTROL);
    ctx->userDriver.init(ctx);
}

static void HAL_USBD_HandleCtrlStall(HAL_USBD_Ctx* ctx) {
    ctx->ctrlRxStatus = EP_RX_STALL;
    ctx->ctrlTxStatus = EP_TX_STALL;
    ctx->ctrlState = USBD_CTRL_IDLE;
    ctx->ctrlCompleteCb = NULL;
    printf("USB CTRL STALL\r\n");
}

static void HAL_USBD_SetAddressComplete_Cb(HAL_USBD_Ctx* ctx, const USB_SetupReq* req, const uint8_t* data, uint16_t len) {
    uint8_t devAddr = (uint8_t)(req->wValue & 0x7F);
    if (devAddr != 0) {
        ctx->devState = USBD_ADDRESSED;
    } else {
        ctx->devState = USBD_ATTACHED;
    }
    USBD_LL_SetDeviceAddress(devAddr);
}

static void HAL_USBD_SetConfigComplete_Cb(HAL_USBD_Ctx* ctx, const USB_SetupReq* req, const uint8_t* data, uint16_t len) {
    if (ctx->currentCfg != 0) {
        ctx->devState = USBD_CONFIGURED;
        ctx->userDriver.setConfig(ctx, ctx->currentCfg);
    } else {
        ctx->devState = USBD_ADDRESSED;
    }
}

static bool HAL_USBD_ConfigureDevice(HAL_USBD_Ctx* ctx, uint8_t cfgNum) {
    const uint8_t* pDesc = NULL;
    const uint8_t* pEnd = NULL;
    uint16_t len = 0;
    bool itfOkay = false;
    USB_DescEndpoint* epDesc = NULL;

    if (!ctx->userDriver.getDescriptor(ctx, USB_DTYPE_CONFIGURATION, cfgNum, &pDesc, &len) ||
        (pDesc == NULL) || (len < 9)) {
        return false;
    }

    pEnd = pDesc + len;

    while (pDesc < pEnd) {
        if (pDesc[0] == 0) {
            break;
        }
        if (pDesc[1] == USB_DTYPE_INTERFACE) {
            if (((USB_DescInterface*)pDesc)->bAlternateSetting == 0) {
                itfOkay = true;
            } else {
                itfOkay = false;
            }
        }
        if ((pDesc[1] == USB_DTYPE_ENDPOINT) && itfOkay) {
            epDesc = (USB_DescEndpoint*)pDesc;
            HAL_USBD_EPOpen(
                ctx, 
                epDesc->bEndpointAddress, 
                epDesc->wMaxPacketSize, 
                epDesc->bmAttributes);
        }
        pDesc += pDesc[0];
    }
    return true;
}

static bool HAL_USBD_HandleSetConfigReq(HAL_USBD_Ctx* ctx) {
    USB_SetupReq* req = (USB_SetupReq*)&ctx->rxBuffer;
    uint8_t cfg = (uint8_t)(req->wValue & 0xFF);

    if (cfg == 0) {
        ctx->devState = USBD_ADDRESSED;
        ctx->currentCfg = 0;
        ctx->userDriver.deinit(ctx);
        for (uint8_t i = 1; i < USBD_EPS_MAX; i++) {
            HAL_USBD_EPClose(ctx, i);
            HAL_USBD_EPClose(ctx, i | 0x80);
        }
        ctx->userDriver.init(ctx);
        return true;
    }
    if (ctx->devState == USBD_CONFIGURED) {
        /* Host sending a SET_CONFIGURATION request 
        while already configured shouldn't happen */
        if (cfg == ctx->currentCfg) {
            return true;
        }
        return false;
    } else {
        HAL_USBD_ConfigureDevice(ctx, cfg);
    }
    ctx->currentCfg = cfg;
    ctx->ctrlCompleteCb = HAL_USBD_SetConfigComplete_Cb;
    return true;
}

static bool HAL_USBD_HandleStdReq(HAL_USBD_Ctx* ctx) {
    USB_SetupReq* req = (USB_SetupReq*)&ctx->rxBuffer;
    static uint16_t status = 0;

    switch (req->bmRequestType & USB_REQ_RECIP_Msk) {
    case USB_REQ_RECIP_DEVICE:
        switch (req->bRequest) {
        case USB_REQ_STD_SET_ADDRESS:
            /* This gets set after the ZLP response is sent */
            ctx->ctrlCompleteCb = HAL_USBD_SetAddressComplete_Cb;
            return true;
        case USB_REQ_STD_SET_CONFIG:
            return HAL_USBD_HandleSetConfigReq(ctx);
        case USB_REQ_STD_GET_CONFIG:
            return HAL_USBD_CtrlSend(ctx, &ctx->currentCfg, sizeof(ctx->currentCfg), NULL);
        case USB_REQ_STD_SET_FEATURE:
        case USB_REQ_STD_CLEAR_FEATURE:
            /* Not supported, ignore and ZLP anyway */
            return true;
        default:
            break;
        }
        break;
    case USB_REQ_RECIP_INTERFACE:
        switch (req->bRequest) {
        case USB_REQ_STD_GET_STATUS:
            {
            /* Default itf status is zero */
            return HAL_USBD_CtrlSend(ctx, (const uint8_t*)&status, sizeof(uint16_t), NULL);
            }
        default:
            break;
        }
        break;
    case USB_REQ_RECIP_ENDPOINT:
        if (USBD_EP_NUM(req->wIndex) >= USBD_EPS_MAX) {
            break;
        }
        switch (req->bRequest) {
        case USB_REQ_STD_SET_FEATURE:
        case USB_REQ_STD_CLEAR_FEATURE:
            /* EP feature req sets or clears STALL */
            printf("Feature Req EP: %d, %s\n", req->wIndex, 
                (req->bRequest == USB_REQ_STD_SET_FEATURE) ? "SET" : "CLEAR");
            HAL_USBD_EPSetStall(
                ctx,
                req->wIndex,
                (req->bRequest == USB_REQ_STD_SET_FEATURE)
            );
            return true;
        case USB_REQ_STD_GET_STATUS: 
            {
            /* EP status is just stall state */
            static uint8_t status[2] = {0};
            status[0] = HAL_USBD_EPStalled(ctx, req->wIndex) ? 1U : 0U;
            status[1] = 0;
            return HAL_USBD_CtrlSend(ctx, status, sizeof(status), NULL);
            }
        default:
            break;
        }
        break;
    default:
        break;
    }
    return false;
}

static bool HAL_USBD_HandleCtrlReq(HAL_USBD_Ctx* ctx) {
    USB_SetupReq* req = (USB_SetupReq*)&ctx->rxBuffer;
    uint8_t dType = req->wValue >> 8;
    uint8_t dIndex = req->wValue & 0xFF;

    if (req->bRequest == USB_REQ_STD_GET_DESCRIPTOR) {
        return ctx->userDriver.getDescriptor(
            ctx, dType, dIndex, &ctx->txBuffer, &ctx->txLen);
    }

    switch (req->bmRequestType & USB_REQ_TYPE_Msk) {
    case USB_REQ_TYPE_STANDARD:
        if (HAL_USBD_HandleStdReq(ctx)) {
            return true;
        }
        /* fall through */
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
        return ctx->userDriver.ctrl(ctx, req);
    default:
        break;
    }
    return false;
}

static void HAL_USBD_HandleCtrlIn(HAL_USBD_Ctx* ctx) {
    uint16_t len = 0;
    USB_SetupReq* req = (USB_SetupReq*)&ctx->rxBuffer;

    switch (ctx->ctrlState) {
    case USBD_CTRL_DATA_IN:
        len = ctx->txLen - ctx->txIdx;
        if (len > ctx->maxPacketSize) {
            len = ctx->maxPacketSize;
        }
        USBD_LL_EPWrite(0, &ctx->txBuffer[ctx->txIdx], len);
        ctx->txIdx += len;
        ctx->ctrlTxStatus = EP_TX_VALID;

        if (ctx->txIdx < ctx->txLen) {
            break;
        }
        if (len != ctx->maxPacketSize || ctx->txLen == req->wLength) {
            ctx->ctrlState = USBD_CTRL_STATUS_OUT;
        }
        break;
    case USBD_CTRL_STATUS_IN:
        ctx->ctrlState = USBD_CTRL_IDLE;
        if (ctx->ctrlCompleteCb) {
            ctx->ctrlCompleteCb(ctx, req, ctx->txBuffer, ctx->txLen);
        }
        ctx->ctrlCompleteCb = NULL;
        break;
    default:
        break;
    }
}

static void HAL_USBD_HandleCtrlOut(HAL_USBD_Ctx* ctx) {
    uint16_t len = 0;
    USB_SetupReq* req = (USB_SetupReq*)&ctx->rxBuffer;
    
    switch (ctx->ctrlState) {
    case USBD_CTRL_IDLE:
        ctx->txCompleteCb = NULL;
        ctx->txBuffer = NULL;
        ctx->txLen = 0;
        ctx->txIdx = 0;

        len = USBD_LL_EPRead(0, ctx->rxBuffer, sizeof(ctx->rxBuffer));
        if (len == 8) {
            ctx->ctrlRxStatus = EP_RX_VALID;
        } else {
            printf("Error: Invalid setup packet length %ld\r\n", len);
            HAL_USBD_HandleCtrlStall(ctx);
            return;
        }
        ctx->rxIdx = len;
        ctx->rxLen = len + req->wLength;

        if ((req->bmRequestType & USB_REQ_DIR_DEVTOHOST) ||
            (req->wLength == 0)) {
            break;
        }
        if (ctx->rxLen > sizeof(ctx->rxBuffer)) {
            printf("Error: Ctrl RX buffer overflow %d > %d\r\n", ctx->rxLen, sizeof(ctx->rxBuffer));
            HAL_USBD_HandleCtrlStall(ctx);
            return;
        }

        ctx->ctrlState = USBD_CTRL_DATA_OUT;
        return;

    case USBD_CTRL_DATA_OUT:
        len = USBD_LL_EPRead(0, &ctx->rxBuffer[ctx->rxIdx], ctx->rxLen - ctx->rxIdx);
        if (!len) {
            printf("no data received\r\n");
            HAL_USBD_HandleCtrlStall(ctx);
            return;
        }
        ctx->ctrlRxStatus = EP_RX_VALID;
        ctx->rxIdx += len;
        if (ctx->rxIdx < ctx->rxLen) {
            return;
        }
        break;

    case USBD_CTRL_STATUS_OUT:
        // must read a ZLP here
        if (USBD_LL_GetEPRxCount(0) != 0) {
            printf("status out with data\r\n");
            HAL_USBD_HandleCtrlStall(ctx);
            return;
        }
        ctx->ctrlRxStatus = EP_RX_VALID;
        ctx->ctrlState = USBD_CTRL_IDLE;
        if (ctx->txCompleteCb) {
            ctx->txCompleteCb(ctx, req, ctx->txBuffer, ctx->txLen);
        }
        return;

    default:
        len = USBD_LL_GetEPRxCount(0);
        printf("unexpected packet, len=%ld\r\n", len);
        if (len == 0) {
            /* Random ZLP, reset the request */
            ctx->ctrlState = USBD_CTRL_IDLE;
            ctx->txCompleteCb = NULL;
            ctx->ctrlRxStatus = EP_RX_VALID;
        } else {
            /* Unexpected packet */
            printf("Error: Unexpected packet in state %d\r\n", ctx->ctrlState); 
            HAL_USBD_HandleCtrlStall(ctx);
        }
        return;
    }

    /* Full request received */
    if ((req->bmRequestType & USB_REQ_DIR_Msk) == USB_REQ_DIR_DEVTOHOST) {
        ctx->ctrlState = USBD_CTRL_DATA_IN;
        ctx->txIdx = 0;
        ctx->txLen = 0;
    }

    if (HAL_USBD_HandleCtrlReq(ctx)) {
        if (ctx->ctrlState == USBD_CTRL_DATA_IN) {
            if (ctx->txLen == 0 && req->wLength != 0) {
                /* No data queued by the application */
                printf("no data queued for ctrl in\r\n");
                HAL_USBD_HandleCtrlStall(ctx);
                return;
            }
            if (ctx->txLen > req->wLength) {
                ctx->txLen = req->wLength;
            }
            HAL_USBD_HandleCtrlIn(ctx);
        } else {
            ctx->ctrlState = USBD_CTRL_STATUS_IN;
            USBD_LL_SetEPTxCount(0, 0);
            ctx->ctrlTxStatus = EP_TX_VALID;
        }
    } else {
        /* Unhandled request */
        printf("Error: Unhandled request 0x%02X\r\n", req->bRequest);
        HAL_USBD_HandleCtrlStall(ctx);
    }
}

/* Public Methods */

bool HAL_USBD_Init(HAL_USBD_Ctx* ctx, const HAL_USBD_UserDriver* driver) {
    uint8_t	i;
    USB_DescDevice* pDevDesc = NULL;

    if (ctx == NULL || driver == NULL) {
        return false;
    }
    if (!driver->init || !driver->deinit || 
        !driver->getDescriptor ||
        !driver->setConfig || !driver->ctrl ||
        !driver->epComplete) {
        return false;
    }

    memset(ctx, 0, sizeof(HAL_USBD_Ctx));
    ctx->instance = USBD_INSTANCE_USB1D;
    ctx->devState = USBD_UNCONNECTED;
    ctx->userDriver = *driver;
    ctx->pmaOffset = PMA_DATA_START;

    ctx->userDriver.init(ctx);
    if (!ctx->userDriver.getDescriptor(ctx, USB_DTYPE_DEVICE, 0, (const uint8_t**)&pDevDesc, NULL) ||
        (pDevDesc == NULL) || (pDevDesc->bLength < 18) ||
        (pDevDesc->bDescriptorType != USB_DTYPE_DEVICE) ||
        (pDevDesc->bMaxPacketSize0 < 8) || (pDevDesc->bMaxPacketSize0 > 64) ||
        ((pDevDesc->bMaxPacketSize0 & (pDevDesc->bMaxPacketSize0 - 1)) != 0)) {
        return false;
    }

    ctx->maxPacketSize = pDevDesc->bMaxPacketSize0;
    ctx->numConfigs = pDevDesc->bNumConfigurations;

    HAL_USBD_PowerOn(ctx);

    for (i = 0; i < USBD_EPS_MAX; i++) {
        USBD_LL_SetENDPOINT(i, USBD_LL_GetENDPOINT(i) & 0x7F7F & EPREG_MASK);
    }

    USBD_LL_SetISTR((uint16_t)0x00FF);
    HAL_USBD_SIL_Init(ctx);

    USBD_LL_SetCNTR(USBD_LL_GetCNTR()|(1<<1));
    HAL_USBD_MspSetPort_Cb(ctx, false);
    Delay_Ms(20);
    USBD_LL_SetCNTR(USBD_LL_GetCNTR()&(~(1<<1)));
    HAL_USBD_MspSetPort_Cb(ctx, true);

    HAL_USBD_EPOpen(ctx, 0x00, ctx->maxPacketSize, USB_EP_TYPE_CONTROL);

    HAL_USBD_MspInit_Cb(ctx);

    return true;
}

void HAL_USBD_DeInit(HAL_USBD_Ctx* ctx) {
    if (ctx == NULL) {
        return;
    }
    USBD_LL_SetCNTR(USBD_LL_GetCNTR()|(1<<1));
    HAL_USBD_MspSetPort_Cb(ctx, false);
    Delay_Ms(20);
    ctx->userDriver.deinit(ctx);
    HAL_USBD_PowerOff(ctx);
    HAL_USBD_MspDeInit_Cb(ctx);
}

void HAL_USBD_Task(HAL_USBD_Ctx* ctx) {
    if ((ctx == NULL) || (ctx->devState != USBD_CONFIGURED)) {
        return;
    }
    if (ctx->userDriver.task) {
        ctx->userDriver.task(ctx);
    }
}

bool HAL_USBD_CtrlSend(HAL_USBD_Ctx* ctx, const uint8_t* data, uint16_t len, HAL_USBD_CtrlComplete_Cb completeCb) {
    if ((ctx == NULL) || (ctx->ctrlState != USBD_CTRL_DATA_IN)) {
        return false;
    }
    ctx->txBuffer = data;
    ctx->txLen = len;
    ctx->txIdx = 0;
    ctx->txCompleteCb = completeCb;
    return true;
}

void HAL_USBD_EPFlush(HAL_USBD_Ctx* ctx, uint8_t epAddr) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    if ((epNum >= USBD_EPS_MAX) || (ctx == NULL)) {
        return;
    }
    if (epAddr & 0x80) {
        if (!(ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum))) {
            return;
        }
        USBD_LL_SetEPTxStatus(epNum, EP_TX_NAK);
        USBD_LL_SetEPTxCount(epNum, 0);
        USBD_LL_ClearDTOG_TX(epNum);
        ctx->epInBusy &= ~BIT(epNum);
    } else {
        if (!(ctx->epOpen[USBD_EPDIR_OUT] & BIT(epNum))) {
            return;
        }
        USBD_LL_SetEPRxStatus(epNum, EP_RX_NAK);
        USBD_LL_ClearDTOG_RX(epNum);
        USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
    }
}

int32_t HAL_USBD_EPWrite(HAL_USBD_Ctx* ctx, uint8_t epAddr, const uint8_t* data, uint16_t len) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    uint16_t pmaAddr = 0;
    if ((ctx == NULL) || ((data == NULL) && (len != 0)) || 
        (epNum >= USBD_EPS_MAX) || 
        !(ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum))) {
        return -1;
    }
    if ((ctx->epInBusy & BIT(epNum)) && 
        (ctx->epType[USBD_EPDIR_IN][epNum] != USB_EP_TYPE_ISOCHRONUS)) {
        return -1;
    }
    if (len > ctx->epSize[USBD_EPDIR_IN][epNum]) {
        len = ctx->epSize[USBD_EPDIR_IN][epNum];
    }
    ctx->epInBusy |= BIT(epNum);
    if (data != NULL) {
        pmaAddr = USBD_LL_GetEPTxAddr(epNum);
        USBD_LL_WritePmaBuffer(data, pmaAddr, len);
    }
    USBD_LL_SetEPTxCount(epNum, len);
    USBD_LL_SetEPTxStatus(epNum, EP_TX_VALID);
    return (int32_t)len;
}

int32_t HAL_USBD_EPRead(HAL_USBD_Ctx* ctx, uint8_t epAddr, uint8_t* data, uint16_t len) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    uint16_t pmaAddr = 0;
    uint16_t rxLen = 0;

    if ((ctx == NULL) || ((data == NULL) && (len != 0)) || 
        (epNum >= USBD_EPS_MAX) || 
        !(ctx->epOpen[USBD_EPDIR_OUT] & BIT(epNum))) {
        return -1;
    }
    rxLen = USBD_LL_GetEPRxCount(epNum);
    if (rxLen > len) {
        rxLen = len;
    }
    if (rxLen == 0) {
        USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
        return 0;
    }
    pmaAddr = USBD_LL_GetEPRxAddr(epNum);
    USBD_LL_ReadPmaBuffer(data, pmaAddr, rxLen);
    USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
    return (int32_t)rxLen;
}

bool HAL_USBD_EPInReady(HAL_USBD_Ctx* ctx, uint8_t epAddr) {
    uint8_t epNum = USBD_EP_NUM(epAddr);
    if ((ctx == NULL) || (epNum >= USBD_EPS_MAX) || 
        !(ctx->epOpen[USBD_EPDIR_IN] & BIT(epNum))) {
        return false;
    }
    return ((ctx->epInBusy & BIT(epNum)) == 0);
}