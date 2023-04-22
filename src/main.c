/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Peter Lawrence
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "FreeRTOS.h"
#include "task.h"

#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"

#include "picoprobe_config.h"
#include "probe.h"
#include "cdc_uart.h"
#include "get_serial.h"
#include "led.h"
#include "DAP.h"

#include "pico/stdio_uart.h"

#include "swd_dmi.h"

#define DM_DATA0        0x04
#define DM_DMCONTROL    0x10
#define DM_DMSTATUS     0x11
#define DM_HARTINFO     0x12
#define DM_HALTSUM1     0x13
#define DM_HALTSUM0     0x40
#define DM_HAWINDOWSEL  0x14
#define DM_HAWINDOW     0x15
#define DM_ABSTRACTCS   0x16
#define DM_COMMAND      0x17
#define DM_ABSTRACTAUTO 0x18
#define DM_CONFSTRPTR0  0x19
#define DM_CONFSTRPTR1  0x1a
#define DM_CONFSTRPTR2  0x1b
#define DM_CONFSTRPTR3  0x1c
#define DM_NEXTDM       0x1d
#define DM_PROGBUF0     0x20
#define DM_PROGBUF1     0x21
#define DM_SBCS         0x38
#define DM_SBADDRESS0   0x39
#define DM_SBDATA0      0x3c



// UART0 for Picoprobe debug
// UART1 for picoprobe to target device

static uint8_t TxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];
static uint8_t RxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];

#define THREADED 1

#define UART_TASK_PRIO (tskIDLE_PRIORITY + 3)
#define TUD_TASK_PRIO  (tskIDLE_PRIORITY + 2)
#define DAP_TASK_PRIO  (tskIDLE_PRIORITY + 1)

static TaskHandle_t dap_taskhandle, tud_taskhandle;

void usb_thread(void *ptr)
{
    do {
        tud_task();
        // Trivial delay to save power
        vTaskDelay(1);
    } while (1);
}

// Workaround API change in 0.13
#if (TUSB_VERSION_MAJOR == 0) && (TUSB_VERSION_MINOR <= 12)
#define tud_vendor_flush(x) ((void)0)
#endif

void dap_thread(void *ptr)
{
    uint32_t resp_len;
    do {
        if (tud_vendor_available()) {
            tud_vendor_read(RxDataBuffer, sizeof(RxDataBuffer));
            resp_len = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
            tud_vendor_write(TxDataBuffer, resp_len);
            tud_vendor_flush();
        } else {
            // Trivial delay to save power
            vTaskDelay(2);
        }
    } while (1);
}

static int test_swd_dmi(void) {
    probe_init();

    swd_dmi_t *dmi = swd_dmi_create(0, 0);
    printf("\n\nIssuing connect sequence...\n");
    int rc = swd_dmi_connect(dmi);
    if (rc) {
        printf("Failed to connect\n");
        return rc;
    }
    printf("Connected successfully\n");

    uint32_t data;
    swd_dmi_read(dmi, DM_DMSTATUS, &data);
    printf("dstatus   = %08lx\n", data);
    if ((data & 0xf) == 0x2) {
        printf("RISC-V debug version: 0.13\n");
    } else {
        printf("Unknown RISC-V debug version\n");
        return -1;
    }

    swd_dmi_write(dmi, DM_DMCONTROL, 0);
    swd_dmi_write(dmi, DM_DMCONTROL, 1);
    swd_dmi_read(dmi, DM_DMCONTROL, &data);
    if (data != 1) {
        printf("Failed to set dmcontrol.dmactive=1\n");
        return -1;
    }

    // Detect number of harts on this DM (should be 1 for us)
    int hart;
    for (hart = 0; hart < 32; ++hart) {
        uint32_t dmcontrol_wdata = 1 | (hart << 16);
        swd_dmi_write(dmi, DM_DMCONTROL, dmcontrol_wdata);
        // Running out of hasel index bits means no more harts:
        swd_dmi_read(dmi, DM_DMCONTROL, &data);
        if (data != dmcontrol_wdata)
            break;
        // anyunavail=1 also means no more harts:
        swd_dmi_read(dmi, DM_DMSTATUS, &data);
        if (data & (1u << 12))
            break;
    }
    printf("Discovered %d harts\n", hart);

    return 0;
}

int main(void) {
    uint32_t resp_len;

    stdio_uart_init();
    (void)test_swd_dmi();


    board_init();
    usb_serial_init();
    cdc_uart_init();
    tusb_init();
#if (PICOPROBE_DEBUG_PROTOCOL == PROTO_OPENOCD_CUSTOM)
    probe_gpio_init();
    probe_init();
#else
    DAP_Setup();
#endif
    led_init();

    picoprobe_info("Welcome to Picoprobe!\n");

    if (THREADED) {
        /* UART needs to preempt USB as if we don't, characters get lost */
        xTaskCreate(cdc_thread, "UART", configMINIMAL_STACK_SIZE, NULL, UART_TASK_PRIO, &uart_taskhandle);
        xTaskCreate(usb_thread, "TUD", configMINIMAL_STACK_SIZE, NULL, TUD_TASK_PRIO, &tud_taskhandle);
        /* Lowest priority thread is debug - need to shuffle buffers before we can toggle swd... */
        xTaskCreate(dap_thread, "DAP", configMINIMAL_STACK_SIZE, NULL, DAP_TASK_PRIO, &dap_taskhandle);
        vTaskStartScheduler();
    }

    while (!THREADED) {
        tud_task();
        cdc_task();
#if (PICOPROBE_DEBUG_PROTOCOL == PROTO_OPENOCD_CUSTOM)
        probe_task();
        led_task();
#elif (PICOPROBE_DEBUG_PROTOCOL == PROTO_DAP_V2)
        if (tud_vendor_available()) {
            tud_vendor_read(RxDataBuffer, sizeof(RxDataBuffer));
            resp_len = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);
            tud_vendor_write(TxDataBuffer, resp_len);
        }
#endif
    }

    return 0;
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* RxDataBuffer, uint16_t bufsize)
{
  uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, bufsize);

  // This doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;

  DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);

  tud_hid_report(0, TxDataBuffer, response_size);
}

#if (PICOPROBE_DEBUG_PROTOCOL == PROTO_DAP_V2)
extern uint8_t const desc_ms_os_20[];

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bmRequestType_bit.type)
  {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest)
      {
        case 1:
          if ( request->wIndex == 7 )
          {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20+8, 2);

            return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
          }else
          {
            return false;
          }

        default: break;
      }
    break;
    default: break;
  }

  // stall unknown request
  return false;
}
#endif

void vApplicationTickHook (void)
{
};

void vApplicationStackOverflowHook(TaskHandle_t Task, char *pcTaskName)
{
  panic("stack overflow (not the helpful kind) for %s\n", *pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
  panic("Malloc Failed\n");
};
