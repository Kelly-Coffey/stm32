#include "usbd_composite.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

static uint8_t  USBD_COMPOSITE_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_COMPOSITE_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_COMPOSITE_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  USBD_COMPOSITE_EP0_RxReady (USBD_HandleTypeDef  *pdev);
static uint8_t  USBD_COMPOSITE_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_COMPOSITE_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  *USBD_COMPOSITE_GetCfgDesc (uint16_t *length);

USBD_COMPOSITE_ItfTypeDef fops;

extern USBD_ClassTypeDef  USBD_CUSTOM_HID;

USBD_ClassTypeDef  USBD_COMPOSITE = 
{
  USBD_COMPOSITE_Init,
  USBD_COMPOSITE_DeInit,
  USBD_COMPOSITE_Setup,
  NULL, /*EP0_TxSent*/  
  USBD_COMPOSITE_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  USBD_COMPOSITE_DataIn, /*DataIn*/
  USBD_COMPOSITE_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,      
  NULL,
  USBD_COMPOSITE_GetCfgDesc, 
  NULL,
  NULL,
};

static uint8_t USBD_COMPOSITE_CfgDesc[USB_COMPOSITE_CONFIG_DESC_SIZ] =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_COMPOSITE_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x02,         /*bNumInterfaces: HID:1 CDC:2 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x04,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  
#if 1
  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0x05,         /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_DAP_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  
  CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  0x01,          /*bInterval: Polling Interval (1 ms)*/
  /* 34 */
  
  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  0x01, /* bInterval: Polling Interval (1 ms) */
  /* 41 */
#endif

#if 0
  /* 09 */
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  /* 18 */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /* 23 */
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  /* 28 */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /* 32 */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  /* 37 */
  
  /*Endpoint Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /* 44 */

  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  /* 53 */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* 60 */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
  /* 67 */
#endif
} ;

/**
  * @brief  USBD_COMPOSITE_Init
  *         Initialize the COMPOSITE interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  printf("USBD Init\n");
  return USBD_CUSTOM_HID.Init(pdev, cfgidx);
//  return USBD_CDC.Init(pdev, cfgidx);
}

/**
  * @brief  USBD_COMPOSITE_DeInit
  *         DeInitialize the COMPOSITE layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  printf("USBD DeInit\n");
  return USBD_CUSTOM_HID.DeInit(pdev, cfgidx);
//  return USBD_CDC.DeInit(pdev, cfgidx);
}

/**
  * @brief  USBD_COMPOSITE_Setup
  *         Handle the COMPOSITE specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  static uint8_t ifalt = 0;
  
  printf("USBD Setup \n");

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
    {
      case USB_REQ_RECIPIENT_INTERFACE:
        printf("USBD Setup Interface:%d\n", req->wIndex);
        if (req->wIndex == 0) {
          return USBD_CUSTOM_HID.Setup(pdev, req);
        }
        else
        if (req->wIndex < 2) {
          return USBD_CDC.Setup(pdev, req);
        }
        break;

      default:
        break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      printf("USBD Setup GetDesc Idx:%d\n", req->wIndex);
      return USBD_CUSTOM_HID.Setup(pdev, req);
      break;

    case USB_REQ_GET_INTERFACE:
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  printf("USBD DataIn ep:%d\n", epnum);
  switch (epnum) {
    case 1:

    case 2:
    case 3:
      return USBD_CUSTOM_HID.DataIn(pdev, epnum);

//    case 2:
//    case 3:
//      return USBD_CDC.DataIn(pdev, epnum);

    default:
      break;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_COMPOSITE_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  printf("USBD DataOut ep:%d\n", epnum);
  switch (epnum) {
    case 1:
    case 2:
    case 3:
      return USBD_CUSTOM_HID.DataOut(pdev, epnum);

//    case 2:
//    case 3:
//      return USBD_CDC.DataOut(pdev, epnum);

    default:
      break;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_COMPOSITE_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  printf("USBD RxRdy Req:%02x Idx:%d\n", pdev->request.bmRequest, pdev->request.wIndex);
  switch (pdev->request.bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (pdev->request.bmRequest & USB_REQ_RECIPIENT_MASK)
      {
        case USB_REQ_RECIPIENT_INTERFACE:
          printf("USBD RxRdy Idx:%d\n", pdev->request.wIndex);
          if (pdev->request.wIndex == 0) {
            return USBD_CUSTOM_HID.EP0_RxReady(pdev);
          }
          else
          if (pdev->request.wIndex < 2) {
            return USBD_CDC.EP0_RxReady(pdev);
          }
          break;

        default:
          break;
      }
  }
  return 0;
}

/**
  * @brief  USBD_COMPOSITE_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_COMPOSITE_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMPOSITE_CfgDesc);
  printf("GetCfgDesc %d\n", *length);
  return USBD_COMPOSITE_CfgDesc;
}

/**
* @brief  USBD_COMPOSITE_RegisterInterface
  * @param  pdev: COMPOSITE device instance
  * @param  hid_fops: CUSTOMHID Interface callback
  * @param  cdc_fops: CDC Interface callback
  * @retval status
  */
uint8_t  USBD_COMPOSITE_RegisterInterface  (USBD_HandleTypeDef *pdev,
                                             USBD_CUSTOM_HID_ItfTypeDef *hid_fops,
                                             USBD_CDC_ItfTypeDef *cdc_fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(hid_fops != NULL && cdc_fops != NULL)
  {
    fops.hid_fops = hid_fops;
    fops.cdc_fops = cdc_fops;

    pdev->pUserData= &fops;

    ret = USBD_OK;    
  }
  
  return ret;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
