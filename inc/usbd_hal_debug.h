#pragma once

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_USBD_DEBUG
#define USBD_PRINTF(...) printf(__VA_ARGS__)
#else
#define USBD_PRINTF(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif