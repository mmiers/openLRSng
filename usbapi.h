#ifndef __USBAPI_H_
#define __USBAPI_H_

//================================================================================
//================================================================================
//    USB

typedef struct USBDevice
{
    uint8_t dummy;
}
USBDevice;

extern USBDevice usbDevice;

bool USBDevice_Configured();
void USBDevice_Attach();
void USBDevice_Detach();    // Serial port goes down too...
void USBDevice_Poll();

//================================================================================
//================================================================================
//    Serial over CDC (Serial1 is the physical port)

void USBSerial_Accept(void);
size_t USBSerial_Write(uint8_t c);

//================================================================================
//================================================================================
//    Low level API

typedef struct
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint16_t wIndex;
    uint16_t wLength;
} Setup;

//================================================================================
//================================================================================
//    CSC 'Driver'

int     CDC_GetInterface(uint8_t* interfaceNum);
int     CDC_GetDescriptor(int i);
bool    CDC_Setup(Setup* setup);

//================================================================================
//================================================================================

#define TRANSFER_PGM        0x80
#define TRANSFER_RELEASE    0x40
#define TRANSFER_ZERO        0x20

int USB_SendControl(uint8_t flags, const void* d, int len);
int USB_RecvControl(void* d, int len);

uint8_t USB_Available(uint8_t ep);
int     USB_Send(uint8_t ep, const void* data, int len); // blocking
int     USB_Recv(uint8_t ep, void* data, int len);       // non-blocking
int     USB_Get(uint8_t ep);                             // non-blocking
void    USB_Flush(uint8_t ep);

#endif /* if defined(USBCON) */
