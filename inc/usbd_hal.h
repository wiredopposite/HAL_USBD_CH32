#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAL_USBD_ENUMERATION_BUF_SIZE
#define HAL_USBD_ENUMERATION_BUF_SIZE  128U /* Size of buffer for receiving control request + payload */
#endif

#ifndef HAL_USBD_MAX_INTERFACES
#define HAL_USBD_MAX_INTERFACES 6U /* Maximum number of interfaces supported */
#endif

#ifndef HAL_USBD_SUSPEND_ENABLED
#define HAL_USBD_SUSPEND_ENABLED 1U /* Enable USB suspend handling */
#endif

typedef enum {
    USBD_INSTANCE_USB1D = 0,
    // USBD_INSTANCE_USB2HD = 1,
} USBD_Instance;

typedef enum _USBD_CtrlState {
    USBD_CTRL_IDLE = 0,
    USBD_CTRL_DATA_OUT,
    USBD_CTRL_DATA_IN,
    USBD_CTRL_STATUS_IN,
    USBD_CTRL_STATUS_OUT,
    USBD_CTRL_STALL,
    USBD_CTRL_PAUSE
} USBD_CtrlState;

typedef enum _USBD_DeviceState {
    USBD_UNCONNECTED = 0,
    USBD_ATTACHED,
    USBD_POWERED,
    USBD_SUSPENDED,
    USBD_ADDRESSED,
    USBD_CONFIGURED
} USBD_DeviceState;

typedef enum _USBD_EpDir {
    USBD_EPDIR_IN = 0,
    USBD_EPDIR_OUT = 1,
    USBD_EPDIR_TOTAL
} USBD_EpDir;

typedef enum _USBD_CtrlResult {
    USBD_CTRL_OKAY = 0,
    USBD_CTRL_FAIL,
    USBD_CTRL_BUSY,
} USBD_CtrlResult;

typedef enum _USBD_EPResult {
    USBD_EP_OKAY = 0,
    USBD_EP_BUSY,
    USBD_EP_STALL,
} USBD_EPResult;

typedef enum _USBD_ResumeState {
    USBD_RESUME_EXTERNAL,
    USBD_RESUME_INTERNAL,
    USBD_RESUME_LATER,
    USBD_RESUME_WAIT,
    USBD_RESUME_START,
    USBD_RESUME_ON,
    USBD_RESUME_OFF,
    USBD_RESUME_ESOF
} USBD_ResumeState;

typedef struct __attribute__((__packed__)) _USB_SetupReq {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_SetupReq;

typedef struct _HAL_USBD_Ctx HAL_USBD_Ctx;

typedef void (*USBD_Init_Cb)(HAL_USBD_Ctx* ctx);
typedef void (*USBD_Deinit_Cb)(HAL_USBD_Ctx* ctx);
typedef bool (*USBD_GetDescriptor_Cb)(HAL_USBD_Ctx* ctx, uint8_t dtype, uint8_t dindex, const uint8_t** data, uint16_t* len);
typedef bool (*USBD_Ctrl_Cb)(HAL_USBD_Ctx* ctx, const USB_SetupReq* req);
typedef void (*USBD_SetConfig_Cb)(HAL_USBD_Ctx* ctx, uint8_t config);
typedef void (*USBD_EpComplete_Cb)(HAL_USBD_Ctx* ctx, uint8_t epaddr);
typedef void (*USBD_Task_Cb)(HAL_USBD_Ctx* ctx);

typedef struct {
    USBD_Init_Cb            init;
    USBD_Deinit_Cb          deinit;
    USBD_GetDescriptor_Cb   getDescriptor;
    USBD_SetConfig_Cb       setConfig;
    USBD_Ctrl_Cb            ctrl;
    USBD_EpComplete_Cb      epComplete;
    USBD_Task_Cb            task;
} HAL_USBD_UserDriver;

typedef void (*HAL_USBD_CtrlComplete_Cb)(HAL_USBD_Ctx* ctx, const USB_SetupReq* req, const uint8_t* data, uint16_t len);

typedef struct _HAL_USBD_Ctx {
    USBD_Instance instance;
    USBD_DeviceState devState;
    USBD_CtrlState ctrlState;

    uint16_t epOpen[USBD_EPDIR_TOTAL];
    volatile uint16_t epInBusy;
    uint16_t epSize[USBD_EPDIR_TOTAL][16];
    uint8_t epType[USBD_EPDIR_TOTAL][16];

    uint8_t currentFeat;
    uint8_t currentCfg;
    uint8_t currentAltSetting[HAL_USBD_MAX_INTERFACES];
    
    uint8_t numConfigs;
    uint8_t maxPacketSize;

    const uint8_t* txBuffer;
    uint16_t txLen;
    uint16_t txIdx;
    HAL_USBD_CtrlComplete_Cb txCompleteCb;

    uint8_t rxBuffer[HAL_USBD_ENUMERATION_BUF_SIZE] __attribute__((aligned(4)));
    uint16_t rxLen;
    uint16_t rxIdx;

    HAL_USBD_CtrlComplete_Cb ctrlCompleteCb;

    HAL_USBD_UserDriver userDriver;

    uint16_t pmaOffset;
    volatile uint16_t ctrlRxStatus;
    volatile uint16_t ctrlTxStatus;
    volatile uint16_t interruptMask;

    struct {
        volatile USBD_ResumeState resumeState;
        volatile uint8_t resumeSofCount;
        volatile uint16_t ctrlRxBlocks;

        volatile uint32_t sofCount;
        volatile uint8_t bIntPackSOF;
        // volatile uint8_t suspendEnabled;
        volatile uint8_t remoteWakeupOn;
    } hw;
} HAL_USBD_Ctx;

void HAL_USBD_MspInit_Cb(HAL_USBD_Ctx* ctx);
void HAL_USBD_MspDeInit_Cb(HAL_USBD_Ctx* ctx);
void HAL_USBD_MspSetPort_Cb(HAL_USBD_Ctx* ctx, bool connect);

void HAL_USBD_Suspend_Cb(HAL_USBD_Ctx* ctx) __attribute__((weak));
void HAL_USBD_Resume_Cb(HAL_USBD_Ctx* ctx) __attribute__((weak));

void HAL_USBD_RxIrqHandler(HAL_USBD_Ctx* ctx);
void HAL_USBD_WakeupIrqHandler(HAL_USBD_Ctx* ctx, uint32_t extiLine);

bool HAL_USBD_Init(HAL_USBD_Ctx* ctx, const HAL_USBD_UserDriver* userDriver);
void HAL_USBD_DeInit(HAL_USBD_Ctx* ctx);
void HAL_USBD_Task(HAL_USBD_Ctx* ctx);

bool HAL_USBD_CtrlSend(HAL_USBD_Ctx* ctx, const uint8_t* data, uint16_t len, HAL_USBD_CtrlComplete_Cb completeCb);
int32_t HAL_USBD_EPWrite(HAL_USBD_Ctx* ctx, uint8_t epAddr, const uint8_t* data, uint16_t len);
int32_t HAL_USBD_EPRead(HAL_USBD_Ctx* ctx, uint8_t epAddr, uint8_t* data, uint16_t len);
void HAL_USBD_EPFlush(HAL_USBD_Ctx* ctx, uint8_t epAddr);
bool HAL_USBD_EPInReady(HAL_USBD_Ctx* ctx, uint8_t epAddr);

#ifdef __cplusplus
}
#endif