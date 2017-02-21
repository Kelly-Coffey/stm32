
#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#include "usbd_ioreq.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_cdc.h"

#define USB_COMPOSITE_CONFIG_DESC_SIZ   107

typedef struct _USBD_COMPOSITE_Itf
{
  USBD_CUSTOM_HID_ItfTypeDef *hid_fops;
  USBD_CDC_ItfTypeDef *cdc_fops;
} USBD_COMPOSITE_ItfTypeDef;

extern USBD_ClassTypeDef  USBD_COMPOSITE;
extern USBD_COMPOSITE_ItfTypeDef fops;

uint8_t  USBD_COMPOSITE_RegisterInterface  (USBD_HandleTypeDef *pdev,
                                             USBD_CUSTOM_HID_ItfTypeDef *hid_fops,
                                             USBD_CDC_ItfTypeDef *cdc_fops);

#endif  /* __USB_COMPOSITE_H */
  