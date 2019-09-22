/* Mouse
 * https://github.com/R3M0H/
 */

/*
             LUFA Library
     Copyright (C) Dean Camera, 2018.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2018  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Mouse. This file contains the main tasks of
 *  the mouse and is responsible for the initial application hardware configuration.
 */

#include "Mouse.h"
#include "PMW3360.h"
#include "Buttons.h"
#include "Rotary.h"

/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
//static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_MouseReport_Data_t)];
static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_GamingMouseReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Mouse_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Mouse,
				.ReportINEndpoint             =
					{
						.Address              = MOUSE_EPADDR,
						.Size                 = MOUSE_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevMouseHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevMouseHIDReportBuffer),
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */

static volatile uint8_t _poll;
static volatile uint8_t _host_status;
static volatile uint8_t buttons;
static volatile int8_t wheel;
int main(void)
{
	SetupHardware();
	GlobalInterruptEnable();

	for (;;)
	{
		HID_Device_USBTask(&Mouse_HID_Interface);
		USB_USBTask();

		uint8_t status = PMW3360status() + (_host_status<<1);
		if(status == 0x00)
			continue;
		else if(status == 0x01){
			PMW3360shutdown();
			PORTD &= ~(1 << 6);		//LED OFF
		}
		else if(status == 0x02){
			PMW3360powerUp();
			PORTD |= (1 << 6);		//LED ON
		}

		if(_poll){
			_poll = 0;
			PMW3360motionBurst();
			buttons = buttonsCheck();
			wheel = rotaryProcess();
		}
	}
}

/** Configures the board hardware and chip peripherals for the mouse's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	USB_Init();
	ioInit();
	spiInit();
	buttonsInit();
	rotaryInit();
}

/** Configures the board IO pins. */
void ioInit(void){
	// IO
	DDRD |= (1 << 6);	// PD6 (LED) output
}

/** Event handler for the library USB Suspend event. */
void EVENT_USB_Device_Suspend()
{
	_host_status = 0;
}

/** Event handler for the library USB Wake Up event. */
void EVENT_USB_Device_WakeUp() 
{
	_host_status = 1;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;
	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);
	USB_Device_EnableSOFEvents();

}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Mouse_HID_Interface);

}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	memset((void*)ReportData, 0, sizeof(USB_GamingMouseReport_Data_t));
	*ReportSize = 0;
	static uint8_t last_btn = 0;

	if(last_btn != buttons){
		*ReportSize = sizeof(USB_GamingMouseReport_Data_t);
		*((uint8_t*)ReportData) = buttons;
		last_btn = buttons;
	}
	if(motion_read[0] & 0x80){
		*ReportSize = sizeof(USB_GamingMouseReport_Data_t);
		*((uint8_t*)ReportData) = last_btn;
		memcpy(ReportData+1, motion_read+2, 4);
	}
	if(wheel){
		*ReportSize = sizeof(USB_GamingMouseReport_Data_t);
		*((uint8_t*)ReportData) = last_btn;
		*((uint8_t*)ReportData + 5) = wheel;
		wheel = 0;
	}

	_poll = 1;
	return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

