/*****************************************************************************
 * Steve Moskovchenko  (aka evil wombat)                                     *
 * University of Maryland                                                    *
 * stevenm86 at gmail dot com                                                *
 * http://wam.umd.edu/~stevenm/carnetix.html                                 *
 *                                                                           *
 * Linux and Mac OS API for the Carnetix P2140 power supply.                 *
 * The program uses libusb, which works on Mac OS X, Linux, and Windows.     *
 *                                                                           *
 * Released under the BSD License. See License.txt for info.                 *
 *                                                                           *
 * Version 0.2. Enjoy!                                                       *
 *                                                                           *
 ****************************************************************************/


/* For API Documentation, see ctxapi.h */

#include <stdio.h>
#include <stdlib.h>
#include <usb.h>
#include <stdarg.h>
#include <string.h>
#include "ctxapi.h"

#define VENDOR              0x04D8
#define READ_ENDPOINT           0x81
#define WRITE_ENDPOINT          0x01
#define DEFAULT_CONFIGURATION       1
#define DEFAULT_INTERFACE           0
#define DEFAULT_ALT_INTERFACE       0
#define SHORT_TIMEOUT           3000

/* Names of the states for the state field. These also came from the C# API */
static const char * ctxStateNames[] =
{
    "Idle",
    "Power PSU",
    "Power PC",
    "Bootup Lockout",
    "Run PC",
    "Shutdown Delay",
    "ACPI Pulse",
    "Shutdown Lockout",
    "Standby/Sleep",
    "Forced Shutdown",
    "Initialization"
};


static unsigned short m_product_id;
static unsigned short m_vendor_id;



void debug(char *format, ...)
{
    va_list ap;
    char logmessage[2048];
    memset(logmessage,0,sizeof(logmessage));
    va_start(ap,format);
    (void)vsnprintf(logmessage,sizeof(logmessage),format,ap);
    va_end(ap);
    fprintf(stderr,"%s\n",logmessage);
}



/* This function is based on pv2fetch, the program to download pictures from a disposable CVS camera */
usb_dev_handle * ctxInit()
{
    struct usb_bus *p_bus;
    struct usb_device* pdev=NULL;

    usb_init();
    if (usb_find_busses() < 0)
    {
        debug("Error: Could not find USB bus.");
        return NULL;
    }

    if (usb_find_devices() < 0)
    {
        debug("Error: Could not find any USB devices.");
        return NULL;
    }

    p_bus = usb_get_busses();

    while (p_bus)
    {
        struct usb_device *p_device = p_bus->devices;
        while (p_device)
        {
            if (p_device->descriptor.idVendor == VENDOR)
            {
                usb_dev_handle* udev = usb_open(p_device);

                if (udev)
                {
                    pdev = p_device;
                    m_vendor_id = p_device->descriptor.idVendor;
                    m_product_id = p_device->descriptor.idProduct;
                    usb_close(udev);
                    debug("Found power supply: VID:%.4X PID:%.4X",m_vendor_id,m_product_id);
                    break;
                }
                else
                {
                    debug("Error: Found the power supply, but could not open it.");
                    break;
                }

            }
            p_device = p_device->next;
        }

        if (pdev)
            break;

        p_bus = p_bus->next;
    }

    if (pdev == NULL)
    {
        debug("Error: Could not find the power supply.");
        return NULL;
    }

    usb_dev_handle * hDev = usb_open(pdev);

    if(!hDev)
    {
    	printf("\nusb_open returns null\n");
        return NULL;
    }

    if (usb_set_configuration(hDev, DEFAULT_CONFIGURATION) >= 0)
    {
        if (usb_claim_interface(hDev, DEFAULT_INTERFACE) >= 0)
        {
            if (usb_set_altinterface(hDev, DEFAULT_ALT_INTERFACE) < 0)
            {
            
                usb_close(hDev);
                return NULL;
            }
        }
        else
        {
            usb_close(hDev);
            return NULL;
        }
    }
    else
    {
        usb_close(hDev);
        return NULL;
    }

    return hDev;
}

int ctxRead(usb_dev_handle * hDev, unsigned char* p_buffer, unsigned int length, int timeout)
{
    int ret = usb_interrupt_read(hDev, READ_ENDPOINT, (char*)p_buffer, length, timeout);
    return ret;
}

int ctxWrite(usb_dev_handle * hDev, unsigned char* p_buffer, unsigned int length, int timeout)
{
    int ret = usb_interrupt_write(hDev, WRITE_ENDPOINT, (char*)p_buffer, length, timeout);
    return ret;
}


int ctxReadValues(usb_dev_handle * hDev, struct ctxValues * val)
{
    unsigned char getValCmd[]={0x40, 0x17};
    unsigned char buf[23];

    if(!hDev)
        return -1;

    if(!val)
        return -1;

    if(ctxWrite(hDev, getValCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 23, SHORT_TIMEOUT) != 23)
        return -1;

    if(buf[0] != 0x40 || buf[1] != 0x17)
        return -1;

    val->battVoltage = ((buf[3]<<8) | buf[2]) * 0.0321;
    val->battCurrent = ((buf[5]<<8) | buf[4]) * 0.016;

    val->priVoltage = ((buf[7]<<8) | buf[6]) * 0.0321;
    val->priCurrent = ((buf[9]<<8) | buf[8]) * 0.016;

    val->secVoltage = ((buf[11]<<8) | buf[10]) * 0.0134;
    val->secCurrent = ((buf[13]<<8) | buf[12]) * 0.004;

    val->temperature = (((buf[15]<<8) | buf[14])-125) * 0.403;
    val->ledValue = ((buf[17]<<8) | buf[16]);
    val->state = buf[18];
    return 0;
}


int ctxReadParams(usb_dev_handle * hDev, struct ctxParams * prm)
{
    unsigned char getPrmCmd[]={0x44, 0x15};
    unsigned char buf[21];

    if(!hDev)
        return -1;

    if(!prm)
        return -1;

    if(ctxWrite(hDev, getPrmCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 21, SHORT_TIMEOUT) != 21)
        return -1;

    if(buf[0] != 0x44 || buf[1] != 0x15)
        return -1;

    prm->sd_dly = ((buf[3]<<8) | buf[2]);
    prm->dmt    = ((buf[5]<<8) | buf[4]) * 0.1;
    prm->dlyon  = ((buf[7]<<8) | buf[6]) * 0.1;
    prm->bu_lo  = ((buf[9]<<8) | buf[8]) * 0.1;
    prm->sd_lo  = ((buf[11]<<8) | buf[10]) * 0.1;
    prm->lobatt = ((buf[13]<<8) | buf[12]) * 0.0321;
    prm->softJumpers = buf[14];

    prm->acpiDelay = ((buf[16]<<8) | buf[15]) * 0.1;
    prm->acpiDuration = ((buf[18]<<8) | buf[17]) * 0.1;
    prm->lowTemp = (((buf[20]<<8) | buf[19])-124) * 0.403;
    return 0;
}


int ctxWriteParams(usb_dev_handle * hDev, struct ctxParams * prm)
{
    unsigned char buf[23];
    unsigned char reply[3];

    if(!hDev)
        return -1;

    if(!prm)
        return -1;

    buf[0] = 0x43;
    buf[1] = 0x03;

    buf[2] =  ((int)(prm->sd_dly)) & 0xFF;
    buf[3] = (((int)(prm->sd_dly)) >> 8 ) & 0xFF;

    buf[4] =  ((int)(prm->dmt * 10)) & 0xFF;
    buf[5] = (((int)(prm->dmt * 10)) >> 8 ) & 0xFF;

    buf[6] =  ((int)(prm->dlyon * 10)) & 0xFF;
    buf[7] = (((int)(prm->dlyon * 10)) >> 8 ) & 0xFF;

    buf[8] =  ((int)(prm->bu_lo * 10)) & 0xFF;
    buf[9] = (((int)(prm->bu_lo * 10)) >> 8 ) & 0xFF;

    buf[10] =  ((int)(prm->sd_lo * 10)) & 0xFF;
    buf[11] = (((int)(prm->sd_lo * 10)) >> 8 ) & 0xFF;

    buf[12] =  ((int)(prm->lobatt / 0.0321)) & 0xFF;
    buf[13] = (((int)(prm->lobatt / 0.0321)) >> 8 ) & 0xFF;

    buf[14] = prm->softJumpers & 0xF7;  /* Always keep fan on */

    buf[15] =  ((int)(prm->acpiDelay * 10)) & 0xFF;
    buf[16] = (((int)(prm->acpiDelay * 10)) >> 8 ) & 0xFF;

    buf[17] =  ((int)(prm->acpiDuration * 10)) & 0xFF;
    buf[18] = (((int)(prm->acpiDuration * 10)) >> 8 ) & 0xFF;

    buf[19] =  ((int)(prm->lowTemp / 0.403)+124) & 0xFF;
    buf[20] = (((int)(prm->lowTemp / 0.403)+124) >> 8 ) & 0xFF;

    buf[21] = 0;
    buf[22] = 0;

    if(ctxWrite(hDev, buf, 23, SHORT_TIMEOUT) != 23)
        return -1;

    if(ctxRead(hDev, reply, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(reply[0] != 0x43 || reply[1] != 0x03 || reply[2] != 0xff)
        return -1;

    return 0;
}


int ctxPriOn(usb_dev_handle * hDev)
{
    unsigned char ctxPriOnCmd[]={0x41, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOnCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x41 || buf[1] != 0x03 || buf[2] != 0xFF)
        return -1;

    return 0;
}

int ctxPriOff(usb_dev_handle * hDev)
{
    unsigned char ctxPriOffCmd[]={0x42, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOffCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x42 || buf[1] != 0x03 || buf[2] != 0x00)
        return -1;

    return 0;
}


int ctxSecOff(usb_dev_handle * hDev)
{
    unsigned char ctxPriOffCmd[]={0x46, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOffCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x46 || buf[1] != 0x03 || buf[2] != 0x00)
        return -1;

    return 0;
}

int ctxSecOn(usb_dev_handle * hDev)
{
    unsigned char ctxPriOffCmd[]={0x45, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOffCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x45 || buf[1] != 0x03 || buf[2] != 0xFF)
        return -1;

    return 0;
}


int ctxP5VOff(usb_dev_handle * hDev)
{
    unsigned char ctxPriOffCmd[]={0x48, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOffCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x48 || buf[1] != 0x03 || buf[2] != 0x00)
        return -1;

    return 0;
}

int ctxP5VOn(usb_dev_handle * hDev)
{
    unsigned char ctxPriOffCmd[]={0x47, 0x03};
    unsigned char buf[3];

    if(!hDev)
        return -1;

    if(ctxWrite(hDev, ctxPriOffCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    if(ctxRead(hDev, buf, 3, SHORT_TIMEOUT) != 3)
        return -1;

    if(buf[0] != 0x47 || buf[1] != 0x03 || buf[2] != 0xFF)
        return -1;

    return 0;
}

int ctxGetFWVersion(usb_dev_handle * hDev, char * vbuf, int len)
{
    unsigned char ctxVerCmd[]={0x00, 0x03};
    unsigned char buf[15];

    if(!hDev || len < 12 || !vbuf)
        return -1;

    if(ctxWrite(hDev, ctxVerCmd, 2, SHORT_TIMEOUT) != 2)
        return -1;

    int rlen = ctxRead(hDev, buf, 5, SHORT_TIMEOUT);

    if(len < 4)
        return -1;

    if(buf[0] != 0x00 || buf[1] != 0x03)
        return -1;

    sprintf(vbuf, "??");

    if(rlen == 4)    /* Old FW, reports XX.xx */
        sprintf(vbuf, "%d.%d", buf[3], buf[2]);

    if(rlen == 5)
        sprintf(vbuf, "%d.%d.%d", buf[3], buf[2], buf[4]);
    return 0;
}


void ctxClose(usb_dev_handle * hDev)
{
    if(hDev)
        usb_close(hDev);
}

const char* ctxStateNameToText(int state)
{
    return ctxStateNames[state];
}
