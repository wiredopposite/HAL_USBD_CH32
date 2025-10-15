#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __IO
#define __IO volatile
#endif

#define USBRegBase      (0x40005C00L)  
#define USBPMAAddr      (0x40006000L) 
#define USBD_EPS_MAX    8U
#define USBD_EPNUM_Msk  (USBD_EPS_MAX - 1)

/******************************************************************************/
/*                         General registers                                  */
/******************************************************************************/

/* Control register */
#define CNTLR    ((__IO unsigned *)(USBRegBase + 0x40))
/* Interrupt status register */
#define ISTR    ((__IO unsigned *)(USBRegBase + 0x44))
/* Frame number register */
#define FNR     ((__IO unsigned *)(USBRegBase + 0x48))
/* Device address register */
#define DADDR   ((__IO unsigned *)(USBRegBase + 0x4C))
/* Buffer Table address register */
#define BTABLE  ((__IO unsigned *)(USBRegBase + 0x50))
/******************************************************************************/
/*                         Endpoint registers                                 */
/******************************************************************************/
#define EP0REG  ((__IO unsigned *)(USBRegBase)) /* endpoint 0 register address */

/* Endpoint Addresses (w/direction) */
#define EP0_OUT     ((uint8_t)0x00)  
#define EP0_IN      ((uint8_t)0x80) 
#define EP1_OUT     ((uint8_t)0x01)  
#define EP1_IN      ((uint8_t)0x81)  
#define EP2_OUT     ((uint8_t)0x02)  
#define EP2_IN      ((uint8_t)0x82)  
#define EP3_OUT     ((uint8_t)0x03)  
#define EP3_IN      ((uint8_t)0x83) 
#define EP4_OUT     ((uint8_t)0x04)  
#define EP4_IN      ((uint8_t)0x84)
#define EP5_OUT     ((uint8_t)0x05)  
#define EP5_IN      ((uint8_t)0x85)
#define EP6_OUT     ((uint8_t)0x06)  
#define EP6_IN      ((uint8_t)0x86)
#define EP7_OUT     ((uint8_t)0x07)  
#define EP7_IN      ((uint8_t)0x87)

/* endpoints enumeration */
#define ENDP0       ((uint8_t)0)
#define ENDP1       ((uint8_t)1)
#define ENDP2       ((uint8_t)2)
#define ENDP3       ((uint8_t)3)
#define ENDP4       ((uint8_t)4)
#define ENDP5       ((uint8_t)5)
#define ENDP6       ((uint8_t)6)
#define ENDP7       ((uint8_t)7)

/******************************************************************************/
/*                       ISTR interrupt events                                */
/******************************************************************************/
#define ISTR_CTR    (0x8000) /* Correct TRansfer (clear-only bit) */
#define ISTR_DOVR   (0x4000) /* DMA OVeR/underrun (clear-only bit) */
#define ISTR_ERR    (0x2000) /* ERRor (clear-only bit) */
#define ISTR_WKUP   (0x1000) /* WaKe UP (clear-only bit) */
#define ISTR_SUSP   (0x0800) /* SUSPend (clear-only bit) */
#define ISTR_RESET  (0x0400) /* RESET (clear-only bit) */
#define ISTR_SOF    (0x0200) /* Start Of Frame (clear-only bit) */
#define ISTR_ESOF   (0x0100) /* Expected Start Of Frame (clear-only bit) */


#define ISTR_DIR    (0x0010)  /* DIRection of transaction (read-only bit)  */
#define ISTR_EP_ID  (0x000F)  /* EndPoint IDentifier (read-only bit)  */

#define CLR_CTR    (~ISTR_CTR)   /* clear Correct TRansfer bit */
#define CLR_DOVR   (~ISTR_DOVR)  /* clear DMA OVeR/underrun bit*/
#define CLR_ERR    (~ISTR_ERR)   /* clear ERRor bit */
#define CLR_WKUP   (~ISTR_WKUP)  /* clear WaKe UP bit     */
#define CLR_SUSP   (~ISTR_SUSP)  /* clear SUSPend bit     */
#define CLR_RESET  (~ISTR_RESET) /* clear RESET bit      */
#define CLR_SOF    (~ISTR_SOF)   /* clear Start Of Frame bit   */
#define CLR_ESOF   (~ISTR_ESOF)  /* clear Expected Start Of Frame bit */

/******************************************************************************/
/*             CNTR control register bits definitions                         */
/******************************************************************************/
#define CNTR_CTRM   (0x8000) /* Correct TRansfer Mask */
#define CNTR_DOVRM  (0x4000) /* DMA OVeR/underrun Mask */
#define CNTR_ERRM   (0x2000) /* ERRor Mask */
#define CNTR_WKUPM  (0x1000) /* WaKe UP Mask */
#define CNTR_SUSPM  (0x0800) /* SUSPend Mask */
#define CNTR_RESETM (0x0400) /* RESET Mask   */
#define CNTR_SOFM   (0x0200) /* Start Of Frame Mask */
#define CNTR_ESOFM  (0x0100) /* Expected Start Of Frame Mask */


#define CNTR_RESUME (0x0010) /* RESUME request */
#define CNTR_FSUSP  (0x0008) /* Force SUSPend */
#define CNTR_LPMODE (0x0004) /* Low-power MODE */
#define CNTR_PDWN   (0x0002) /* Power DoWN */
#define CNTR_FRES   (0x0001) /* Force USB RESet */

/******************************************************************************/
/*                FNR Frame Number Register bit definitions                   */
/******************************************************************************/
#define FNR_RXDP    (0x8000) /* status of D+ data line */
#define FNR_RXDM    (0x4000) /* status of D- data line */
#define FNR_LCK     (0x2000) /* LoCKed */
#define FNR_LSOF    (0x1800) /* Lost SOF */
#define FNR_FN      (0x07FF) /* Frame Number */
/******************************************************************************/
/*               DADDR Device ADDRess bit definitions                         */
/******************************************************************************/
#define DADDR_EF    (0x80)
#define DADDR_ADD   (0x7F)
/******************************************************************************/
/*                            Endpoint register                               */
/******************************************************************************/
/* bit positions */
#define EP_CTR_RX      (0x8000) /* EndPoint Correct TRansfer RX */
#define EP_DTOG_RX     (0x4000) /* EndPoint Data TOGGLE RX */
#define EPRX_STAT      (0x3000) /* EndPoint RX STATus bit field */
#define EP_SETUP       (0x0800) /* EndPoint SETUP */
#define EP_T_FIELD     (0x0600) /* EndPoint TYPE */
#define EP_KIND        (0x0100) /* EndPoint KIND */
#define EP_CTR_TX      (0x0080) /* EndPoint Correct TRansfer TX */
#define EP_DTOG_TX     (0x0040) /* EndPoint Data TOGGLE TX */
#define EPTX_STAT      (0x0030) /* EndPoint TX STATus bit field */
#define EPADDR_FIELD   (0x000F) /* EndPoint ADDRess FIELD */

/* EndPoint REGister MASK (no toggle fields) */
#define EPREG_MASK     (EP_CTR_RX|EP_SETUP|EP_T_FIELD|EP_KIND|EP_CTR_TX|EPADDR_FIELD)

/* EP_TYPE[1:0] EndPoint TYPE */
#define EP_TYPE_MASK   (0x0600) /* EndPoint TYPE Mask */
#define EP_BULK        (0x0000) /* EndPoint BULK */
#define EP_CONTROL     (0x0200) /* EndPoint CONTROL */
#define EP_ISOCHRONOUS (0x0400) /* EndPoint ISOCHRONOUS */
#define EP_INTERRUPT   (0x0600) /* EndPoint INTERRUPT */
#define EP_T_MASK      (~EP_T_FIELD & EPREG_MASK)


/* EP_KIND EndPoint KIND */
#define EPKIND_MASK    (~EP_KIND & EPREG_MASK)

/* STAT_TX[1:0] STATus for TX transfer */
#define EP_TX_DIS      (0x0000) /* EndPoint TX DISabled */
#define EP_TX_STALL    (0x0010) /* EndPoint TX STALLed */
#define EP_TX_NAK      (0x0020) /* EndPoint TX NAKed */
#define EP_TX_VALID    (0x0030) /* EndPoint TX VALID */
#define EPTX_DTOG1     (0x0010) /* EndPoint TX Data TOGgle bit1 */
#define EPTX_DTOG2     (0x0020) /* EndPoint TX Data TOGgle bit2 */
#define EPTX_DTOGMASK  (EPTX_STAT|EPREG_MASK)

/* STAT_RX[1:0] STATus for RX transfer */
#define EP_RX_DIS      (0x0000) /* EndPoint RX DISabled */
#define EP_RX_STALL    (0x1000) /* EndPoint RX STALLed */
#define EP_RX_NAK      (0x2000) /* EndPoint RX NAKed */
#define EP_RX_VALID    (0x3000) /* EndPoint RX VALID */
#define EPRX_DTOG1     (0x1000) /* EndPoint RX Data TOGgle bit1 */
#define EPRX_DTOG2     (0x2000) /* EndPoint RX Data TOGgle bit1 */
#define EPRX_DTOGMASK  (EPRX_STAT|EPREG_MASK)

typedef enum _ResumeState {
    RESUME_EXTERNAL = 0,
    RESUME_INTERNAL,
    RESUME_LATER,
    RESUME_WAIT,
    RESUME_START,
    RESUME_ON,
    RESUME_OFF,
    RESUME_ESOF
} ResumeState;

typedef enum _EpDir {
    EPDIR_IN = 0,
    EPDIR_OUT,
    EPDIR_TOTAL
} EpDir;

typedef enum _EpDblBufDir {
    EP_DBUF_ERR = 0,
    EP_DBUF_OUT,
    EP_DBUF_IN
} EpDblBufDir;

/* endpoint buffer number */
enum EP_BUF_NUM
{
    EP_NOBUF,
    EP_BUF0,
    EP_BUF1
};

static inline void USBD_LL_SetCNTR(uint16_t regValue) {
    (*CNTLR = (uint16_t)regValue);
}

static inline void USBD_LL_SetISTR(uint16_t regValue) {
    (*ISTR = (uint16_t)regValue);
}

static inline void USBD_LL_SetDADDR(uint16_t regValue) {
    (*DADDR = (uint16_t)regValue);
}

static inline void USBD_LL_SetBTABLE(uint16_t regValue) {
    (*BTABLE = (uint16_t)(regValue & 0xFFF8));
}

static inline uint16_t USBD_LL_GetCNTR(void) {
    return ((uint16_t)*CNTLR);
}

static inline uint16_t USBD_LL_GetISTR(void) {
    return ((uint16_t)*ISTR);
}

static inline uint16_t USBD_LL_GetFNR(void) {
    return ((uint16_t)*FNR);
}

static inline uint16_t USBD_LL_GetDADDR(void) {
    return ((uint16_t)*DADDR);
}

static inline uint16_t USBD_LL_GetBTABLE(void) {
    return ((uint16_t)*BTABLE);
}

static inline void USBD_LL_SetENDPOINT(uint8_t epNum, uint16_t regValue) {
    (*(EP0REG + (epNum & 0x0f)) = (uint16_t)regValue);
}

static inline uint16_t USBD_LL_GetENDPOINT(uint8_t epNum) {
    return ((uint16_t)(*(EP0REG + (epNum & 0x0f))));
}

static inline uint16_t USBD_LL_EpAttrToType(uint8_t epAttr) {
    switch (epAttr & 0x03) {
    case 0:
        return EP_CONTROL;
    case 1:
        return EP_ISOCHRONOUS;
    case 2:
        return EP_BULK;
    case 3:
        return EP_INTERRUPT;
    default:
        return EP_BULK;
    }
}

static inline uint8_t USBD_LL_EpTypeToAttr(uint16_t epType) {
    switch (epType & EP_T_FIELD) {
    case EP_CONTROL:
        return 0;
    case EP_ISOCHRONOUS:
        return 1;
    case EP_BULK:
        return 2;
    case EP_INTERRUPT:
        return 3;
    default:
        return 2;
    }
}

static inline void USBD_LL_SetEPType(uint8_t epNum, uint16_t epType) {
    USBD_LL_SetENDPOINT(
        epNum,
        (USBD_LL_GetENDPOINT(epNum) & EP_T_MASK) | epType);
}

static inline void USBD_LL_SetEPAttr(uint8_t epNum, uint16_t epAttr) {
    USBD_LL_SetENDPOINT(
        epNum,
        (USBD_LL_GetENDPOINT(epNum) & EP_T_MASK) | USBD_LL_EpAttrToType(epAttr));
}

static inline uint16_t USBD_LL_GetEPType(uint8_t epNum) {
    return (USBD_LL_GetENDPOINT(epNum) & EP_T_FIELD);
}

static inline uint8_t USBD_LL_GetEPAttr(uint8_t epNum) {
    return USBD_LL_EpTypeToAttr(USBD_LL_GetENDPOINT(epNum));
}

static inline void USBD_LL_SetEPTxStatus(uint8_t epNum, uint16_t status) {
    __IO uint16_t regVal;
    regVal = USBD_LL_GetENDPOINT(epNum) & EPTX_DTOGMASK;
    if ((EPTX_DTOG1 & status) != 0) {
        regVal ^= EPTX_DTOG1;
    }
    if ((EPTX_DTOG2 & status) != 0) {
        regVal ^= EPTX_DTOG2;
    }
    USBD_LL_SetENDPOINT(epNum, (regVal | EP_CTR_RX | EP_CTR_TX));
    while ((USBD_LL_GetENDPOINT(epNum) & EPTX_STAT) != status) {
        USBD_LL_SetENDPOINT(epNum, (regVal | EP_CTR_RX | EP_CTR_TX));
    };
}

static inline void USBD_LL_SetEPRxStatus(uint8_t epNum, uint16_t status) {
    __IO uint16_t regVal;
    regVal = USBD_LL_GetENDPOINT(epNum) & EPRX_DTOGMASK;
    if ((EPRX_DTOG1 & status) != 0) {
        regVal ^= EPRX_DTOG1;
    }
    if ((EPRX_DTOG2 & status) != 0) {
        regVal ^= EPRX_DTOG2;
    }
    USBD_LL_SetENDPOINT(epNum, (regVal | EP_CTR_RX | EP_CTR_TX));
    while ((USBD_LL_GetENDPOINT(epNum) & EPRX_STAT) != status) {
        USBD_LL_SetENDPOINT(epNum, (regVal | EP_CTR_RX | EP_CTR_TX));
    }
}

static inline void USBD_LL_SetEPRxTxStatus(uint8_t epNum, uint16_t statusRx, uint16_t statusTx) {
    __IO uint16_t regValTx, regValRx;
    regValTx = USBD_LL_GetENDPOINT(epNum) & EPTX_DTOGMASK;
    regValRx = USBD_LL_GetENDPOINT(epNum) & EPRX_DTOGMASK;
    if ((EPTX_DTOG1 & statusTx) != 0) {
        regValTx ^= EPTX_DTOG1;
    }
    if ((EPTX_DTOG2 & statusTx) != 0) {
        regValTx ^= EPTX_DTOG2;
    }
    if ((EPRX_DTOG1 & statusRx) != 0) {
        regValRx ^= EPRX_DTOG1;
    }
    if ((EPRX_DTOG2 & statusRx) != 0) {
        regValRx ^= EPRX_DTOG2;
    }
    USBD_LL_SetENDPOINT(epNum, (regValTx | regValRx | EP_CTR_RX | EP_CTR_TX));
    while ((USBD_LL_GetENDPOINT(epNum) & EPTX_STAT) != statusTx) {
        USBD_LL_SetENDPOINT(epNum, (regValTx | EP_CTR_RX | EP_CTR_TX));
    }
    while ((USBD_LL_GetENDPOINT(epNum) & EPRX_STAT) != statusRx) {
        USBD_LL_SetENDPOINT(epNum, (regValRx | EP_CTR_RX | EP_CTR_TX));
    }
}

static inline uint16_t USBD_LL_GetEPTxStatus(uint8_t epNum) {
    return (USBD_LL_GetENDPOINT(epNum) & EPTX_STAT);
}

static inline uint16_t USBD_LL_GetEPRxStatus(uint8_t epNum) {
    return (USBD_LL_GetENDPOINT(epNum) & EPRX_STAT);
}

static inline void USBD_LL_SetDblBuffEPStall(uint8_t epNum, uint8_t dir) {
    uint16_t status = USBD_LL_GetENDPOINT(epNum);
    if (dir == EP_DBUF_OUT) {
        USBD_LL_SetENDPOINT(epNum, status & ~EPRX_DTOG1);
    } else if (dir == EP_DBUF_IN) {
        USBD_LL_SetENDPOINT(epNum, status & ~EPTX_DTOG1);
    }
}

static inline void USBD_LL_SetEPTxValid(uint8_t epNum) {
    USBD_LL_SetEPTxStatus(epNum, EP_TX_VALID);
}

static inline void USBD_LL_SetEPRxValid(uint8_t epNum) {
    USBD_LL_SetEPRxStatus(epNum, EP_RX_VALID);
}

static inline void USBD_LL_SetEP_KIND(uint8_t epNum) {
    USBD_LL_SetENDPOINT(
        epNum, 
        (EP_CTR_RX|EP_CTR_TX|((USBD_LL_GetENDPOINT(epNum) | EP_KIND) & EPREG_MASK)));
}

static inline void USBD_LL_ClearEP_KIND(uint8_t epNum) {
    USBD_LL_SetENDPOINT(
        epNum, 
        (EP_CTR_RX|EP_CTR_TX|(USBD_LL_GetENDPOINT(epNum) & EPKIND_MASK)));
}

static inline uint8_t USBD_LL_GetEPTxStallStatus(uint8_t epNum) {
    return (USBD_LL_GetEPTxStatus(epNum) == EP_TX_STALL);
}

static inline uint8_t USBD_LL_GetEPRxStallStatus(uint8_t epNum) {
    return (USBD_LL_GetEPRxStatus(epNum) == EP_RX_STALL);
}

static inline void USBD_LL_SetStatusOut(uint8_t epNum) {
    USBD_LL_SetEP_KIND(epNum);
}

static inline void USBD_LL_ClearStatusOut(uint8_t epNum) {
    USBD_LL_ClearEP_KIND(epNum);
}

static inline void USBD_LL_SetEPDoubleBuff(uint8_t epNum) {
    USBD_LL_SetEP_KIND(epNum);
}

static inline void USBD_LL_ClearEPDoubleBuff(uint8_t epNum) {
    USBD_LL_ClearEP_KIND(epNum);
}

static inline void USBD_LL_ClearEP_CTR_RX(uint8_t epNum) {
    USBD_LL_SetENDPOINT(epNum, USBD_LL_GetENDPOINT(epNum) & 0x7FFF & EPREG_MASK);
}

static inline void USBD_LL_ClearEP_CTR_TX(uint8_t epNum) {
    USBD_LL_SetENDPOINT(epNum, USBD_LL_GetENDPOINT(epNum) & 0xFF7F & EPREG_MASK);
}

static inline void USBD_LL_ToggleDTOG_RX(uint8_t epNum) {
    USBD_LL_SetENDPOINT(epNum, EP_CTR_RX|EP_CTR_TX|EP_DTOG_RX | (USBD_LL_GetENDPOINT(epNum) & EPREG_MASK));
}

static inline void USBD_LL_ToggleDTOG_TX(uint8_t epNum) {
    USBD_LL_SetENDPOINT(epNum, EP_CTR_RX|EP_CTR_TX|EP_DTOG_TX | (USBD_LL_GetENDPOINT(epNum) & EPREG_MASK));
}

static inline void USBD_LL_ClearDTOG_RX(uint8_t epNum) {
    if ((USBD_LL_GetENDPOINT(epNum) & EP_DTOG_RX) != 0) {
        USBD_LL_ToggleDTOG_RX(epNum);
    }
}

static inline void USBD_LL_ClearDTOG_TX(uint8_t epNum) {
    if ((USBD_LL_GetENDPOINT(epNum) & EP_DTOG_TX) != 0) {
        USBD_LL_ToggleDTOG_TX(epNum);
    }
}

static inline void USBD_LL_SetEPAddress(uint8_t epNum, uint8_t epAddr) {
    USBD_LL_SetENDPOINT(epNum, EP_CTR_RX|EP_CTR_TX|(USBD_LL_GetENDPOINT(epNum) & EPREG_MASK) | epAddr);
}

static inline uint8_t USBD_LL_GetEPAddress(uint8_t epNum) {
    return (uint8_t)(USBD_LL_GetENDPOINT(epNum) & EPADDR_FIELD);
}

static inline uint32_t* USBD_LL_GetEPTxAddrReg(uint8_t epNum) {
    return (uint32_t *)((USBD_LL_GetBTABLE() + (epNum & USBD_EPNUM_Msk) * 8) * 2 + USBPMAAddr);
}

static inline uint32_t* USBD_LL_GetEPRxAddrReg(uint8_t epNum) {
    return (uint32_t *)((USBD_LL_GetBTABLE() + (epNum & USBD_EPNUM_Msk) * 8 + 4) * 2 + USBPMAAddr);
}

static inline uint32_t* USBD_LL_GetEPTxCountReg(uint8_t epNum) {
    return (uint32_t *)((USBD_LL_GetBTABLE() + (epNum & USBD_EPNUM_Msk) * 8 + 2) * 2 + USBPMAAddr);
}

static inline uint32_t* USBD_LL_GetEPRxCountReg(uint8_t epNum) {
    return (uint32_t *)((USBD_LL_GetBTABLE() + (epNum & USBD_EPNUM_Msk) * 8 + 6) * 2 + USBPMAAddr);
}

static inline void USBD_LL_SetEPTxAddr(uint8_t epNum, uint16_t addr) {
    *USBD_LL_GetEPTxAddrReg(epNum) = ((addr >> 1) << 1);
}

static inline void USBD_LL_SetEPRxAddr(uint8_t epNum, uint16_t addr) {
    *USBD_LL_GetEPRxAddrReg(epNum) = ((addr >> 1) << 1);
}

static inline uint16_t USBD_LL_GetEPTxAddr(uint8_t epNum) {
    return (uint16_t)*USBD_LL_GetEPTxAddrReg(epNum);
}

static inline uint16_t USBD_LL_GetEPRxAddr(uint8_t epNum) {
    return (uint16_t)*USBD_LL_GetEPRxAddrReg(epNum);
}

static inline void  USBD_LL_SetEPTxCount(uint8_t epNum, uint16_t count) {
    *USBD_LL_GetEPTxCountReg(epNum) = count;
}

static inline uint16_t USBD_LL_SetEPCountRxReg(uint32_t* reg, uint16_t count) {
    uint16_t numBlocks;
    if (count > 62) {
        numBlocks = count >> 5;
        if((count & 0x1f) == 0) {
            numBlocks--;
        }
        *reg = (uint32_t)((numBlocks << 10) | 0x8000);
    } else {
        numBlocks = count >> 1;
        if((count & 0x1) != 0) {
            numBlocks++;
        }
        *reg = (uint32_t)(numBlocks << 10);
    }
    return numBlocks;
}

static inline uint16_t USBD_LL_GetNumBlock(uint16_t count) {
    uint16_t wNBlocks;
    uint16_t ret;
    if (count > 62) {
        wNBlocks = count >> 5;
        if((count & 0x1f) == 0) {
            wNBlocks--;
        }
        ret = (uint32_t)((wNBlocks << 10) | 0x8000);\
    }
    else {
        wNBlocks = count >> 1;
        if((count & 0x1) != 0) {
            wNBlocks++;
        }
        ret = (uint32_t)(wNBlocks << 10);\
    }
    return ret;
}

/* Returns the number of required blocks for EP0 */
static inline uint16_t USBD_LL_SetEPRxCount(uint8_t epNum, uint16_t count) {
    uint32_t* reg = USBD_LL_GetEPRxCountReg(epNum);
    uint16_t numBlocks = USBD_LL_SetEPCountRxReg(reg, count);
    if ((epNum & USBD_EPNUM_Msk) == 0) {
        numBlocks = USBD_LL_GetNumBlock(count);
    }
    return numBlocks;
}

static inline uint16_t USBD_LL_GetEPTxCount(uint8_t epNum) {
    return (uint16_t)(*USBD_LL_GetEPTxCountReg(epNum) & 0x3FF);
}

static inline uint16_t USBD_LL_GetEPRxCount(uint8_t epNum) {
    return (uint16_t)(*USBD_LL_GetEPRxCountReg(epNum) & 0x3FF);
}

static inline void USBD_LL_SetEPDblBuf0Addr(uint8_t epNum, uint16_t addr) {
    USBD_LL_SetEPTxAddr(epNum, addr);
}

static inline void USBD_LL_SetEPDblBuf1Addr(uint8_t epNum, uint16_t addr) {
    USBD_LL_SetEPRxAddr(epNum, addr);
}

static inline void USBD_LL_SetEPDblBufAddr(uint8_t epNum, uint16_t addr0, uint16_t addr1) {
    USBD_LL_SetEPTxAddr(epNum, addr0);
    USBD_LL_SetEPRxAddr(epNum, addr1);
}

static inline void USBD_LL_SetEPDblBuf0Count(uint8_t epNum, EpDblBufDir dblBufDir, uint16_t count) {
    if (dblBufDir == EP_DBUF_OUT) {
        uint32_t* reg = USBD_LL_GetEPTxCountReg(epNum);
        USBD_LL_SetEPCountRxReg(reg, count);
    } else if (dblBufDir == EP_DBUF_IN) {
        USBD_LL_SetEPTxCount(epNum, count);
    }
}

static inline void USBD_LL_SetEPDblBuf1Count(uint8_t epNum, EpDblBufDir dblBufDir, uint16_t count) {
    if (dblBufDir == EP_DBUF_OUT) {
        USBD_LL_SetEPRxCount(epNum, count);
    } else if (dblBufDir == EP_DBUF_IN) {
        *USBD_LL_GetEPRxCountReg(epNum) = (uint32_t)count;
    }
}

static inline void USBD_LL_SetEPDblBufCount(uint8_t epNum, EpDblBufDir dblBufDir, uint16_t count) {
    USBD_LL_SetEPDblBuf0Count(epNum, dblBufDir, count);
    USBD_LL_SetEPDblBuf1Count(epNum, dblBufDir, count);
}

static inline uint16_t USBD_LL_GetEPDblBuf0Count(uint8_t epNum) {
    return USBD_LL_GetEPTxCount(epNum);
}

static inline uint16_t USBD_LL_GetEPDblBuf1Count(uint8_t epNum) {
    return USBD_LL_GetEPRxCount(epNum);
}

static inline EpDblBufDir USBD_LL_GetEPDblBufDir(uint8_t epNum) {
    if ((USBD_LL_GetEPRxCount(epNum) & 0xFC00) != 0) {
        return EP_DBUF_OUT;
    } else if ((USBD_LL_GetEPTxCount(epNum) & 0x03FF) != 0) {
        return EP_DBUF_IN;
    } else {
        return EP_DBUF_ERR;
    }
}

static inline uint16_t USBD_LL_ToWord(uint8_t high, uint8_t low) {
    return ((uint16_t)(high << 8) | low);
}

static inline uint16_t USBD_LL_ByteSwap16(uint16_t value) {
    return (uint16_t)(((value & 0x00FF )<< 8) | ((value & 0xFF00) >> 8));
}

static inline void USBD_LL_WritePmaBuffer(const uint8_t* src, uint16_t pmaBufAddr, uint16_t len) {
    uint32_t n = (len + 1) >> 1;
    uint32_t i, temp1, temp2;
    uint16_t *pdwVal;
    pdwVal = (uint16_t *)(pmaBufAddr * 2 + USBPMAAddr);
        
    for (i = n; i != 0; i--) {
        temp1 = (uint16_t) * src;
        src++;
        temp2 = temp1 | (uint16_t) * src << 8;
        *pdwVal++ = temp2;
        pdwVal++;
        src++;
    }
}

static inline void USBD_LL_ReadPmaBuffer(uint8_t* dest, uint16_t pmaBufAddr, uint16_t len) {
    uint32_t n = (len + 1) >> 1;
    uint32_t i;
    uint32_t *pdwVal = (uint32_t *)(pmaBufAddr * 2 + USBPMAAddr);
        
    for (i = n; i != 0; i--) {
        *(uint16_t*)dest++ = *pdwVal++;
        dest++;
    } 
}

static inline uint16_t USBD_LL_EPRead(uint8_t epNum, uint8_t* dst, uint16_t len) {
    uint16_t pmaAddr = USBD_LL_GetEPRxAddr(epNum);
    uint16_t xferCount = USBD_LL_GetEPRxCount(epNum);
    if (len > xferCount) {
        len = xferCount;
    }
    USBD_LL_ReadPmaBuffer(dst, pmaAddr, len);
    return len;
}

static inline void USBD_LL_EPWrite(uint8_t epNum, const uint8_t* src, uint16_t len) {
    uint16_t pmaAddr = USBD_LL_GetEPTxAddr(epNum);
    USBD_LL_WritePmaBuffer(src, pmaAddr, len);
    USBD_LL_SetEPTxCount(epNum, len);
}

static inline void USBD_LL_SetDeviceAddress(uint8_t address) {
    for (uint8_t i = 0; i < USBD_EPS_MAX; i++) {
        USBD_LL_SetEPAddress(i, i);
    }
    *DADDR = (uint16_t)(DADDR_EF | (address & DADDR_ADD));
}

#ifdef __cplusplus
}
#endif