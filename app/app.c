/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>
#include "app/action.h"
#if defined(ENABLE_AIRCOPY)
#include "app/aircopy.h"
#endif
#include "app/app.h"
#include "app/dtmf.h"
#if defined(ENABLE_FMRADIO)
#include "app/fm.h"
#endif
#include "app/generic.h"
#include "app/main.h"
#include "app/menu.h"
#include "app/scanner.h"
#if defined(ENABLE_UART)
#include "app/uart.h"
#endif
#include "ARMCM0.h"
#include "audio.h"
#include "board.h"
#include "bsp/dp32g030/gpio.h"
#include "driver/backlight.h"
#if defined(ENABLE_FMRADIO)
#include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "am_fix.h"
#include "dtmf.h"
#include "external/printf/printf.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#if defined(ENABLE_OVERLAY)
#include "sram-overlay.h"
#endif
#include "ui/battery.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/menu.h"
#include "ui/rssi.h"
#include "ui/status.h"
#include "ui/ui.h"

static void APP_ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);

static void updateRSSI(const int vfo)
{
	int16_t rssi = BK4819_GetRSSI();

	#ifdef ENABLE_AM_FIX
		// add RF gain adjust compensation
		if (gEeprom.VfoInfo[vfo].IsAM && gSetting_AM_fix)
			rssi -= rssi_gain_diff[vfo];
	#endif
	
	if (gCurrentRSSI[vfo] == rssi)
		return;     // no change
	
	gCurrentRSSI[vfo] = rssi;

	UI_UpdateRSSI(rssi, vfo);
}

static void APP_CheckForIncoming(void)
{
	if (!g_SquelchLost) {
		return;
	}
	if (gScanState == SCAN_OFF) {
		if (gCssScanMode != CSS_SCAN_MODE_OFF && gRxReceptionMode == RX_MODE_NONE) {
			ScanPauseDelayIn10msec = 100;
			gScheduleScanListen = false;
			gRxReceptionMode = RX_MODE_DETECTED;
		}
		if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF) {
#if defined(ENABLE_NOAA)
			if (gIsNoaaMode) {
				gNOAA_Countdown = 20;
				gScheduleNOAA = false;
			}
#endif
			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}
		if (gRxReceptionMode != RX_MODE_NONE) {
			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}
		gDualWatchCountdown = 100;
		gScheduleDualWatch = false;

		// let the user see DW is not active
		gDualWatchActive = false;
		gUpdateStatus    = true;
	} else {
		if (gRxReceptionMode != RX_MODE_NONE) {
			FUNCTION_Select(FUNCTION_INCOMING);
			return;
		}
		ScanPauseDelayIn10msec = 20;
		gScheduleScanListen = false;
	}
	gRxReceptionMode = RX_MODE_DETECTED;
	FUNCTION_Select(FUNCTION_INCOMING);
}

static void APP_HandleIncoming(void)
{
	bool bFlag;

	if (!g_SquelchLost) {
		FUNCTION_Select(FUNCTION_FOREGROUND);
		gUpdateDisplay = true;
		return;
	}

	bFlag = (gScanState == SCAN_OFF && gCurrentCodeType == CODE_TYPE_OFF);
#if defined(ENABLE_NOAA)
	if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE) && gSystickCountdown2) {
		bFlag = true;
		gSystickCountdown2 = 0;
	}
#endif
	if (g_CTCSS_Lost && gCurrentCodeType == CODE_TYPE_CONTINUOUS_TONE) {
		bFlag = true;
		gFoundCTCSS = false;
	}
	if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE && (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL)) {
		gFoundCDCSS = false;
	} else if (!bFlag) {
		return;
	}

	DTMF_HandleRequest();

	if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF) {
		if (gRxVfo->DTMF_DECODING_ENABLE || gSetting_KILLED) {
			if (gDTMF_CallState == DTMF_CALL_STATE_NONE) {
				if (gRxReceptionMode == RX_MODE_DETECTED) {
					gDualWatchCountdown = 500;
					gScheduleDualWatch = false;
					gRxReceptionMode = RX_MODE_LISTENING;

					// let the user see DW is not active
					gDualWatchActive = false;
					gUpdateStatus    = true;

					return;
				}
			}
		}
	}

	APP_StartListening(FUNCTION_RECEIVE);
}

static void APP_HandleReceive(void)
{
	uint8_t Mode;

#define END_OF_RX_MODE_SKIP 0
#define END_OF_RX_MODE_END  1
#define END_OF_RX_MODE_TTE  2

	Mode = END_OF_RX_MODE_SKIP;

	if (gFlagTteComplete) {
		Mode = END_OF_RX_MODE_END;
		goto Skip;
	} else if (gScanState != SCAN_OFF && IS_FREQ_CHANNEL(gNextMrChannel)) {
		if (g_SquelchLost) {
			return;
		}
		Mode = END_OF_RX_MODE_END;
		goto Skip;
	}
	switch (gCurrentCodeType) {
	case CODE_TYPE_CONTINUOUS_TONE:
		if (gFoundCTCSS && gFoundCTCSSCountdown == 0) {
			gFoundCTCSS = false;
			gFoundCDCSS = false;
			Mode = END_OF_RX_MODE_END;
			goto Skip;
		}
		break;
	case CODE_TYPE_DIGITAL:
	case CODE_TYPE_REVERSE_DIGITAL:
		if (gFoundCDCSS && gFoundCDCSSCountdown == 0) {
			gFoundCTCSS = false;
			gFoundCDCSS = false;
			Mode = END_OF_RX_MODE_END;
			goto Skip;
		}
		break;
	default:
		break;
	}

	if (g_SquelchLost) {
#ifdef ENABLE_NOAA
		if (!gEndOfRxDetectedMaybe && IS_NOT_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE)) {
#else
		if (!gEndOfRxDetectedMaybe) {
#endif
			switch (gCurrentCodeType) {
			case CODE_TYPE_OFF:
				if (gEeprom.SQUELCH_LEVEL) {
					if (g_CxCSS_TAIL_Found) {
						Mode = END_OF_RX_MODE_TTE;
						g_CxCSS_TAIL_Found = false;
					}
				}
				break;

			case CODE_TYPE_CONTINUOUS_TONE:
				if (g_CTCSS_Lost) {
					gFoundCTCSS = false;
				} else if (!gFoundCTCSS) {
					gFoundCTCSS = true;
					gFoundCTCSSCountdown = 100;
				}
				if (g_CxCSS_TAIL_Found) {
					Mode = END_OF_RX_MODE_TTE;
					g_CxCSS_TAIL_Found = false;
				}
				break;

			case CODE_TYPE_DIGITAL:
			case CODE_TYPE_REVERSE_DIGITAL:
				if (g_CDCSS_Lost && gCDCSSCodeType == CDCSS_POSITIVE_CODE) {
					gFoundCDCSS = false;
				} else if (!gFoundCDCSS) {
					gFoundCDCSS = true;
					gFoundCDCSSCountdown = 100;
				}
				if (g_CxCSS_TAIL_Found) {
					if (BK4819_GetCTCType() == 1) {
						Mode = END_OF_RX_MODE_TTE;
					}
					g_CxCSS_TAIL_Found = false;
				}
				break;

			default:
				break;
			}
		}
	} else {
		Mode = END_OF_RX_MODE_END;
	}

	if (!gEndOfRxDetectedMaybe && Mode == END_OF_RX_MODE_SKIP && gNextTimeslice40ms && gEeprom.TAIL_NOTE_ELIMINATION && (gCurrentCodeType == CODE_TYPE_DIGITAL || gCurrentCodeType == CODE_TYPE_REVERSE_DIGITAL) && BK4819_GetCTCType() == 1) {
		Mode = END_OF_RX_MODE_TTE;
	} else {
		gNextTimeslice40ms = false;
	}

Skip:
	switch (Mode) {
	case END_OF_RX_MODE_END:
		RADIO_SetupRegisters(true);
#if defined(ENABLE_NOAA)
		if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE)) {
			gSystickCountdown2 = 300;
		}
#endif
		gUpdateDisplay = true;
		if (gScanState != SCAN_OFF) {
			switch (gEeprom.SCAN_RESUME_MODE) {
			case SCAN_RESUME_CO:
				ScanPauseDelayIn10msec = 360;
				gScheduleScanListen = false;
				break;
			case SCAN_RESUME_SE:
				SCANNER_Stop();
				break;
			}
		}
		break;
	case END_OF_RX_MODE_TTE:
		if (gEeprom.TAIL_NOTE_ELIMINATION) {
			GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
			gTailNoteEliminationCountdown = 20;
			gFlagTteComplete = false;
			gEnableSpeaker = false;
			gEndOfRxDetectedMaybe = true;
		}
		break;
	}
}

static void APP_HandleFunction(void)
{
	switch (gCurrentFunction) {
	case FUNCTION_FOREGROUND:
		APP_CheckForIncoming();
		break;
	case FUNCTION_POWER_SAVE:
		if (!gRxIdleMode) {
			APP_CheckForIncoming();
		}
		break;
	case FUNCTION_INCOMING:
		APP_HandleIncoming();
		break;
	case FUNCTION_RECEIVE:
		APP_HandleReceive();
		break;
	default:
		break;
	}
}

void APP_StartListening(FUNCTION_Type_t Function, const bool reset_am_fix)
{
	if (!gSetting_KILLED) {
#if defined(ENABLE_FMRADIO)
		if (gFmRadioMode) {
			BK1080_Init(0, false);
		}
#endif

	#ifdef ENABLE_AM_FIX
		if (gEeprom.VfoInfo[gEeprom.RX_CHANNEL].IsAM && reset_am_fix)
			AM_fix_reset(gEeprom.RX_CHANNEL);      // TODO: only reset it when moving channel/frequency
	#endif

		gVFO_RSSI_Level[gEeprom.RX_CHANNEL == 0] = 0;
		GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
		gEnableSpeaker = true;
		BACKLIGHT_TurnOn();
		if (gScanState != SCAN_OFF) {
			switch (gEeprom.SCAN_RESUME_MODE) {
			case SCAN_RESUME_TO:
				if (!gScanPauseMode) {
					ScanPauseDelayIn10msec = 500;
					gScheduleScanListen = false;
					gScanPauseMode = true;
				}
				break;
			case SCAN_RESUME_CO:
			case SCAN_RESUME_SE:
				ScanPauseDelayIn10msec = 0;
				gScheduleScanListen = false;
				break;
			}
			bScanKeepFrequency = true;
		}
#if defined(ENABLE_NOAA)
		if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE) && gIsNoaaMode) {
			gRxVfo->CHANNEL_SAVE = gNoaaChannel + NOAA_CHANNEL_FIRST;
			gRxVfo->pRX->Frequency = NoaaFrequencyTable[gNoaaChannel];
			gRxVfo->pTX->Frequency = NoaaFrequencyTable[gNoaaChannel];
			gEeprom.ScreenChannel[gEeprom.RX_CHANNEL] = gRxVfo->CHANNEL_SAVE;
			gNOAA_Countdown = 500;
			gScheduleNOAA = false;
		}
#endif
		if (gCssScanMode != CSS_SCAN_MODE_OFF) {
			gCssScanMode = CSS_SCAN_MODE_FOUND;
		}
		if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) {
			gRxVfoIsActive = true;
			gDualWatchCountdown = 360;
			gScheduleDualWatch = false;

		// let the user see DW is not active
		gDualWatchActive = false;
		gUpdateStatus    = true;
		}

	// ******************************************
	
	// original setting
	uint8_t lna_short = orig_lna_short;
	uint8_t lna       = orig_lna;
	uint8_t mixer     = orig_mixer;
	uint8_t pga       = orig_pga;

	if (gRxVfo->IsAM)
	{	// AM
/*
		#ifndef ENABLE_AM_FIX
			const uint32_t rx_frequency = gRxVfo->pRX->Frequency;

			// the RX gain abrutly reduces above this frequency
			// I guess this is (one of) the freq the hardware switches the front ends over ?
			if (rx_frequency <= 22640000)
			{	// decrease front end gain - AM demodulator saturates at a slightly higher signal level
				lna_short = 3;   // 3 original
				lna       = 2;   // 2 original
				mixer     = 3;   // 3 original
				pga       = 3;   // 6 original, 3 reduced
			}
			else
			{	// increase the front end to compensate the reduced gain, but more gain decreases dynamic range :(
				lna_short = 3;   // 3 original
				lna       = 4;   // 2 original, 4 increased
				mixer     = 3;   // 3 original
				pga       = 7;   // 6 original, 7 increased
			}
		#endif
*/
		// what do these 4 other gain settings do ???
		//BK4819_WriteRegister(BK4819_REG_12, 0x037B);  // 000000 11 011 11 011
		//BK4819_WriteRegister(BK4819_REG_11, 0x027B);  // 000000 10 011 11 011
		//BK4819_WriteRegister(BK4819_REG_10, 0x007A);  // 000000 00 011 11 010
		//BK4819_WriteRegister(BK4819_REG_14, 0x0019);  // 000000 00 000 11 001

		gNeverUsed = 0;
	}

	// apply the front end gain settings
	BK4819_WriteRegister(BK4819_REG_13, ((uint16_t)lna_short << 8) | ((uint16_t)lna << 5) | ((uint16_t)mixer << 3) | ((uint16_t)pga << 0));

	// ******************************************

	// AF gain - original
	BK4819_WriteRegister(BK4819_REG_48,
		(11u << 12)                |     // ??? .. 0 to 15, doesn't seem to make any difference
		( 0u << 10)                |     // AF Rx Gain-1
		(gEeprom.VOLUME_GAIN << 4) |     // AF Rx Gain-2
		(gEeprom.DAC_GAIN    << 0));     // AF DAC Gain (after Gain-1 and Gain-2)

	#ifdef ENABLE_VOICE
		if (gVoiceWriteIndex == 0)
	#endif
			BK4819_SetAF(gRxVfo->IsAM ? BK4819_AF_AM : BK4819_AF_OPEN);

	FUNCTION_Select(Function);

	#ifdef ENABLE_FMRADIO
		if (Function == FUNCTION_MONITOR || gFmRadioMode)
	#else
		if (Function == FUNCTION_MONITOR)
	#endif
	{	// squelch is disabled
		GUI_SelectNextDisplay(DISPLAY_MAIN);
		return;
	}

	gUpdateDisplay = true;
}

uint32_t APP_SetFrequencyByStep(VFO_Info_t *pInfo, int8_t Step)
{
	uint32_t Frequency;

	Frequency = pInfo->ConfigRX.Frequency + (Step * pInfo->StepFrequency);

	if (pInfo->StepFrequency == 833) {
		const uint32_t Lower = LowerLimitFrequencyBandTable[pInfo->Band];
		const uint32_t Delta = Frequency - Lower;
		uint32_t Base = (Delta / 2500) * 2500;
		const uint32_t Index = ((Delta - Base) % 2500) / 833;

		if (Index == 2) {
			Base++;
		}

		Frequency = Lower + Base + (Index * 833);
	}

	if (Frequency > UpperLimitFrequencyBandTable[pInfo->Band]) {
		Frequency = LowerLimitFrequencyBandTable[pInfo->Band];
	} else if (Frequency < LowerLimitFrequencyBandTable[pInfo->Band]) {
		Frequency = FREQUENCY_FloorToStep(UpperLimitFrequencyBandTable[pInfo->Band], pInfo->StepFrequency, LowerLimitFrequencyBandTable[pInfo->Band]);
	} 

	return Frequency;
}

static void FREQ_NextChannel(void)
{
	APP_SetFrequencyByStep(gRxVfo, gScanState);
	RADIO_ApplyOffset(gRxVfo);
	RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
	RADIO_SetupRegisters(true);
	gUpdateDisplay = true;
	ScanPauseDelayIn10msec = 10;
	bScanKeepFrequency = false;
}

static void MR_NextChannel(void)
{
	uint8_t Ch1 = gEeprom.SCANLIST_PRIORITY_CH1[gEeprom.SCAN_LIST_DEFAULT];
	uint8_t Ch2 = gEeprom.SCANLIST_PRIORITY_CH2[gEeprom.SCAN_LIST_DEFAULT];
	uint8_t PreviousCh, Ch;
	bool bEnabled;

	PreviousCh = gNextMrChannel;
	bEnabled = gEeprom.SCAN_LIST_ENABLED[gEeprom.SCAN_LIST_DEFAULT];
	if (bEnabled) {
		if (gCurrentScanList == 0) {
			gPreviousMrChannel = gNextMrChannel;
			if (RADIO_CheckValidChannel(Ch1, false, 0)) {
				gNextMrChannel = Ch1;
			} else {
				gCurrentScanList = 1;
			}
		}
		if (gCurrentScanList == 1) {
			if (RADIO_CheckValidChannel(Ch2, false, 0)) {
				gNextMrChannel = Ch2;
			} else {
				gCurrentScanList = 2;
			}
		}
		if (gCurrentScanList == 2) {
			gNextMrChannel = gPreviousMrChannel;
		} else {
			goto Skip;
		}
	}

	Ch = RADIO_FindNextChannel(gNextMrChannel + gScanState, gScanState, true, gEeprom.SCAN_LIST_DEFAULT);
	if (Ch == 0xFF) {
		return;
	}

	gNextMrChannel = Ch;

Skip:
	if (PreviousCh != gNextMrChannel) {
		gEeprom.MrChannel[gEeprom.RX_CHANNEL] = gNextMrChannel;
		gEeprom.ScreenChannel[gEeprom.RX_CHANNEL] = gNextMrChannel;
		RADIO_ConfigureChannel(gEeprom.RX_CHANNEL, 2);
		RADIO_SetupRegisters(true);
		gUpdateDisplay = true;
	}
	ScanPauseDelayIn10msec = 20;
	bScanKeepFrequency = false;
	if (bEnabled) {
		gCurrentScanList++;
		if (gCurrentScanList >= 2) {
			gCurrentScanList = 0;
		}
	}
}

#if defined(ENABLE_NOAA)
static void NOAA_NextChannel(void)
{
	gNoaaChannel++;
	if (gNoaaChannel > 9) {
		gNoaaChannel = 0;
	}
}
#endif

static void DUALWATCH_Alternate(void)
{
#if defined(ENABLE_NOAA)
	if (gIsNoaaMode) {
		if (IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[0]) || IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[1])) {
			gEeprom.RX_CHANNEL = gEeprom.RX_CHANNEL == 0;
		} else {
			gEeprom.RX_CHANNEL = 0;
		}
		gRxVfo = &gEeprom.VfoInfo[gEeprom.RX_CHANNEL];
		if (gEeprom.VfoInfo[0].CHANNEL_SAVE >= NOAA_CHANNEL_FIRST) {
			NOAA_NextChannel();
		}
	} else {
#endif
		gEeprom.RX_CHANNEL = gEeprom.RX_CHANNEL == 0;
		gRxVfo = &gEeprom.VfoInfo[gEeprom.RX_CHANNEL];

		if (!gDualWatchActive)
		{	// let the user see DW is active
			gDualWatchActive = true;
			gUpdateStatus    = true;
		}
#if defined(ENABLE_NOAA)
	}
#endif
	RADIO_SetupRegisters(false);
#if defined(ENABLE_NOAA)
	if (gIsNoaaMode) {
		gDualWatchCountdown = 7;
	} else
#endif
		gDualWatchCountdown = 10;
}

void APP_CheckRadioInterrupts(void)
{
	if (gScreenToDisplay == DISPLAY_SCANNER) {
		return;
	}

	while (BK4819_ReadRegister(BK4819_REG_0C) & 1U) {
		uint16_t Mask;

		BK4819_WriteRegister(BK4819_REG_02, 0);

		// fetch the interrupt status bits
		interrupt_status_bits = BK4819_ReadRegister(BK4819_REG_02);

		// 0 = no phase shift
		// 1 = 120deg phase shift
		// 2 = 180deg phase shift
		// 3 = 240deg phase shift
//		const uint8_t ctcss_shift = BK4819_GetCTCShift();
//		if (ctcss_shift > 0)
//			g_CTCSS_Lost = true;

		if (interrupt_status_bits & BK4819_REG_02_DTMF_5TONE_FOUND)
		{	// save the new DTMF RX'ed character

			// fetch the RX'ed char
			const char c = DTMF_GetCharacter(BK4819_GetDTMF_5TONE_Code());
			if (c != 0xff)
			{
				gDTMF_RequestPending = true;
				gDTMF_RecvTimeout = 5;
				if (gDTMF_WriteIndex >= sizeof(gDTMF_Received))
					memmove(gDTMF_Received, &gDTMF_Received[1], gDTMF_WriteIndex-- - 1);
				gDTMF_Received[gDTMF_WriteIndex++] = c;

				if (gCurrentFunction == FUNCTION_RECEIVE)
				{
					{	// live DTMF decoder
						gDTMF_RecvTimeoutSaved = 40;
						size_t len = strlen(gDTMF_ReceivedSaved);
						// shift the RX buffer down one - if need be
						if (len >= (sizeof(gDTMF_ReceivedSaved) - 1))
							memmove(gDTMF_ReceivedSaved, &gDTMF_ReceivedSaved[1], len--);
						gDTMF_ReceivedSaved[len++] = c;
						gDTMF_ReceivedSaved[len]   = '\0';
						gUpdateDisplay = true;
					}

					DTMF_HandleRequest();
				}
			}
		}
		if (Mask & BK4819_REG_02_CxCSS_TAIL) {
			g_CxCSS_TAIL_Found = true;
		}
		if (Mask & BK4819_REG_02_CDCSS_LOST) {
			g_CDCSS_Lost = true;
			gCDCSSCodeType = BK4819_GetCDCSSCodeType();
		}
		if (Mask & BK4819_REG_02_CDCSS_FOUND) {
			g_CDCSS_Lost = false;
		}
		if (Mask & BK4819_REG_02_CTCSS_LOST) {
			g_CTCSS_Lost = true;
		}
		if (Mask & BK4819_REG_02_CTCSS_FOUND) {
			g_CTCSS_Lost = false;
		}
		if (Mask & BK4819_REG_02_VOX_LOST) {
			g_VOX_Lost = true;
			gVoxPauseCountdown = 10;
			if (gEeprom.VOX_SWITCH) {
				if (gCurrentFunction == FUNCTION_POWER_SAVE && !gRxIdleMode) {
					gBatterySave = 20;
					gBatterySaveCountdownExpired = 0;
				}
				if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF && (gScheduleDualWatch || gDualWatchCountdown < 20)) {
					gDualWatchCountdown = 20;
					gScheduleDualWatch = false;

					// let the user see DW is not active
					gDualWatchActive = false;
					gUpdateStatus    = true;
				}
			}
		}
		if (Mask & BK4819_REG_02_VOX_FOUND) {
			g_VOX_Lost = false;
			gVoxPauseCountdown = 0;
		}
		if (Mask & BK4819_REG_02_SQUELCH_LOST) {
			g_SquelchLost = true;
			BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_GREEN, true);
		}
		if (Mask & BK4819_REG_02_SQUELCH_FOUND) {
			g_SquelchLost = false;
			BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_GREEN, false);
		}
#if defined(ENABLE_AIRCOPY)
		if (Mask & BK4819_REG_02_FSK_FIFO_ALMOST_FULL && gScreenToDisplay == DISPLAY_AIRCOPY && gAircopyState == AIRCOPY_TRANSFER && gAirCopyIsSendMode == 0) {
			uint8_t i;

			for (i = 0; i < 4; i++) {
				g_FSK_Buffer[gFSKWriteIndex++] = BK4819_ReadRegister(BK4819_REG_5F);
			}
			AIRCOPY_StorePacket();
		}
#endif
	}
}

void APP_EndTransmission(void)
{	// back to RX mode

	RADIO_SendEndOfTransmission();

	if (gCurrentVfo->pTX->CodeType != CODE_TYPE_OFF)
	{	// CTCSS/DCS is enabled

		//if (gEeprom.TAIL_NOTE_ELIMINATION && gEeprom.REPEATER_TAIL_TONE_ELIMINATION > 0)
		if (gEeprom.TAIL_NOTE_ELIMINATION)
		{	// send the CTCSS/DCS tail tone - allows the receivers to mute the usual FM squelch tail/crash
			RADIO_EnableCxCSS();
		}
		#if 0
			else
			{	// TX a short blank carrier
				// this gives the receivers time to mute RX audio before we drop carrier
				BK4819_ExitSubAu();
				SYSTEM_DelayMs(200);
			}
		#endif
	}

	RADIO_SetupRegisters(false);
}

static void APP_HandleVox(void)
{
	if (!gSetting_KILLED) {
		if (gVoxResumeCountdown == 0) {
			if (gVoxPauseCountdown) {
				return;
			}
		} else {
			g_VOX_Lost = false;
			gVoxPauseCountdown = 0;
		}
		if (gCurrentFunction != FUNCTION_RECEIVE && gCurrentFunction != FUNCTION_MONITOR && gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF
#if defined(ENABLE_FMRADIO)
			&& !gFmRadioMode
#endif
			) {
			if (gVOX_NoiseDetected) {
				if (g_VOX_Lost) {
					gVoxStopCountdown = 100;
				} else if (gVoxStopCountdown == 0) {
					gVOX_NoiseDetected = false;
				}
				if (gCurrentFunction == FUNCTION_TRANSMIT && !gPttIsPressed && !gVOX_NoiseDetected) {
					if (gFlagEndTransmission) {
						FUNCTION_Select(FUNCTION_FOREGROUND);
					} else {
						APP_EndTransmission();
						if (gEeprom.REPEATER_TAIL_TONE_ELIMINATION == 0) {
							FUNCTION_Select(FUNCTION_FOREGROUND);
						} else {
							gRTTECountdown = gEeprom.REPEATER_TAIL_TONE_ELIMINATION * 10;
						}
					}
					gUpdateDisplay = true;
					gFlagEndTransmission = false;
					return;
				}
			} else if (g_VOX_Lost) {
				gVOX_NoiseDetected = true;
				if (gCurrentFunction == FUNCTION_POWER_SAVE) {
					FUNCTION_Select(FUNCTION_FOREGROUND);
				}
				if (gCurrentFunction != FUNCTION_TRANSMIT) {
					gDTMF_ReplyState = DTMF_REPLY_NONE;
					RADIO_PrepareTX();
					gUpdateDisplay = true;
				}
			}
		}
	}
}

void APP_Update(void)
{
#ifdef ENABLE_VOICE
	if (gFlagPlayQueuedVoice) {
		AUDIO_PlayQueuedVoice();
		gFlagPlayQueuedVoice = false;
	}
#endif

	if (gCurrentFunction == FUNCTION_TRANSMIT && gTxTimeoutReached) {
		gTxTimeoutReached = false;
		gFlagEndTransmission = true;
		APP_EndTransmission();
		AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);
		RADIO_SetVfoState(VFO_STATE_TIMEOUT);
		GUI_DisplayScreen();
	}

	if (gReducedService) {
		return;
	}
	if (gCurrentFunction != FUNCTION_TRANSMIT) {
		APP_HandleFunction();
	}
#if defined(ENABLE_FMRADIO)
	if (gFmRadioMode && gFmRadioCountdown) {
		return;
	}
#endif

	#ifdef ENABLE_VOICE
		if (gScreenToDisplay != DISPLAY_SCANNER && gScanState != SCAN_OFF && gScheduleScanListen && !gPttIsPressed && gVoiceWriteIndex == 0)
	#else
		if (gScreenToDisplay != DISPLAY_SCANNER && gScanState != SCAN_OFF && gScheduleScanListen && !gPttIsPressed)
	#endif
	{
		if (IS_FREQ_CHANNEL(gNextMrChannel)) {
			if (gCurrentFunction == FUNCTION_INCOMING) {
				APP_StartListening(gMonitor ? FUNCTION_MONITOR : FUNCTION_RECEIVE, true);
			} else {
				FREQ_NextChannel();
			}
		} else {
			if (gCurrentCodeType == CODE_TYPE_OFF && gCurrentFunction == FUNCTION_INCOMING) {
				APP_StartListening(gMonitor ? FUNCTION_MONITOR : FUNCTION_RECEIVE, true);
			} else {
				MR_NextChannel();
			}
		}
		gScanPauseMode = false;
		gRxReceptionMode = RX_MODE_NONE;
		gScheduleScanListen = false;
	}

	#ifdef ENABLE_VOICE
		if (gCssScanMode == CSS_SCAN_MODE_SCANNING && gScheduleScanListen && gVoiceWriteIndex == 0)
	#else
		if (gCssScanMode == CSS_SCAN_MODE_SCANNING && gScheduleScanListen)
	#endif
	{
		MENU_SelectNextCode();
		gScheduleScanListen = false;
	}

#if defined(ENABLE_NOAA)
		#ifdef ENABLE_VOICE
			if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF && gIsNoaaMode && gScheduleNOAA && gVoiceWriteIndex == 0)
		#else
			if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF && gIsNoaaMode && gScheduleNOAA)
		#endif
		NOAA_NextChannel();
		RADIO_SetupRegisters(false);
		gScheduleNOAA = false;
		gNOAA_Countdown = 7;
	}
#endif

	if (gScreenToDisplay != DISPLAY_SCANNER && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF) {
		#ifdef ENABLE_VOICE
			if (gScheduleDualWatch && gVoiceWriteIndex == 0)
		#else
			if (gScheduleDualWatch)
		#endif
			if (gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF) {
				if (!gPttIsPressed
#if defined(ENABLE_FMRADIO)
						&& !gFmRadioMode
#endif
						&& gDTMF_CallState == DTMF_CALL_STATE_NONE
						&& gCurrentFunction != FUNCTION_POWER_SAVE) {
					DUALWATCH_Alternate();
					if (gRxVfoIsActive && gScreenToDisplay == DISPLAY_MAIN) {
						GUI_SelectNextDisplay(DISPLAY_MAIN);
					}
					gRxVfoIsActive = false;
					gScanPauseMode = false;
					gRxReceptionMode = RX_MODE_NONE;
					gScheduleDualWatch = false;
				}
			}
		}
	}

#if defined(ENABLE_FMRADIO)
	if (gFM_ScanState != FM_SCAN_OFF && gScheduleFM && gCurrentFunction != FUNCTION_MONITOR && gCurrentFunction != FUNCTION_RECEIVE && gCurrentFunction != FUNCTION_TRANSMIT) {
		FM_Play();
		gScheduleFM = false;
	}
#endif

	if (gEeprom.VOX_SWITCH) {
		APP_HandleVox();
	}

	if (gSchedulePowerSave) {
		if (gEeprom.BATTERY_SAVE == 0 || gScanState != SCAN_OFF || gCssScanMode != CSS_SCAN_MODE_OFF
#if defined(ENABLE_FMRADIO)
				|| gFmRadioMode
#endif
				|| gPttIsPressed || gScreenToDisplay != DISPLAY_MAIN || gKeyBeingHeld
				|| gDTMF_CallState != DTMF_CALL_STATE_NONE
				) {
			gBatterySaveCountdown = 1000;
		} else {
			if ((IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[0]) && IS_NOT_NOAA_CHANNEL(gEeprom.ScreenChannel[1]))
#if defined(ENABLE_NOAA)
				|| !gIsNoaaMode
#endif
				) {
				FUNCTION_Select(FUNCTION_POWER_SAVE);
			} else {
				gBatterySaveCountdown = 1000;
			}
		}
		gSchedulePowerSave = false;
	}

	#ifdef ENABLE_VOICE
		if (gPowerSaveCountdownExpired && gCurrentFunction == FUNCTION_POWER_SAVE && gVoiceWriteIndex == 0)
	#else
		if (gPowerSaveCountdownExpired && gCurrentFunction == FUNCTION_POWER_SAVE)
	#endif
		if (gRxIdleMode) {
			BK4819_Conditional_RX_TurnOn_and_GPIO6_Enable();
			if (gEeprom.VOX_SWITCH) {
				BK4819_EnableVox(gEeprom.VOX1_THRESHOLD, gEeprom.VOX0_THRESHOLD);
			}
			if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF && gScanState == SCAN_OFF && gCssScanMode == CSS_SCAN_MODE_OFF) {
				DUALWATCH_Alternate();
				gUpdateRSSI = false;
			}
			FUNCTION_Init();
			gBatterySave = 10;
			gRxIdleMode = false;
		} else if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF || gScanState != SCAN_OFF || gCssScanMode != CSS_SCAN_MODE_OFF || gUpdateRSSI) {
			updateRSSI(gEeprom.RX_CHANNEL);
			gBatterySave = gEeprom.BATTERY_SAVE * 10;
			gRxIdleMode = true;
			BK4819_DisableVox();
			BK4819_Sleep();
			BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2, false);
			// Authentic device checked removed
		} else {
			DUALWATCH_Alternate();
			gUpdateRSSI = true;
			gBatterySave = 10;
		}
		gBatterySaveCountdownExpired = false;
	}
}

void APP_CheckKeys(void)
{
	KEY_Code_t Key;

	if (gSetting_KILLED
#if defined(ENABLE_AIRCOPY)
		|| (gScreenToDisplay == DISPLAY_AIRCOPY && gAircopyState != AIRCOPY_READY)
#endif
		) {
		return;
	}

	if (gPttIsPressed) {
		if (GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT)) {
				if (++gPttDebounceCounter >= 3)	    // 30ms
				{	// stop transmitting
					APP_ProcessKey(KEY_PTT, false, false);
					gPttIsPressed = false;
					if (gKeyReading1 != KEY_INVALID)
						gPttWasReleased = true;
				}
			}
		else
			gPttDebounceCounter = 0;
		}
	} else {
		if (!GPIO_CheckBit(&GPIOC->DATA, GPIOC_PIN_PTT)) {
			if (++gPttDebounceCounter >= 3)	    // 30ms
			{	// start transmitting
				boot_counter_10ms   = 0;
				gPttDebounceCounter = 0;
				gPttIsPressed       = true;
				APP_ProcessKey(KEY_PTT, true, false);
			}
		}
		else
			gPttDebounceCounter = 0;

	}
	Key = KEYBOARD_Poll();

	if (Key != KEY_INVALID)
		boot_counter_10ms = 0;   // cancel boot screen/beeps if any key pressed

	if (gKeyReading0 != Key) {
		if (gKeyReading0 != KEY_INVALID && Key != KEY_INVALID) {
			APP_ProcessKey(gKeyReading1, false, gKeyBeingHeld);
		}
		gKeyReading0 = Key;
		gDebounceCounter = 0;
		return;
	}
	gDebounceCounter++;
	if (gDebounceCounter == 2) {
		if (Key == KEY_INVALID) {
			if (gKeyReading1 != KEY_INVALID) {
				APP_ProcessKey(gKeyReading1, false, gKeyBeingHeld);
				gKeyReading1 = KEY_INVALID;
			}
		} else {
			gKeyReading1 = Key;
			APP_ProcessKey(Key, true, false);
		}
		gKeyBeingHeld = false;
	} else if (gDebounceCounter == 128) {
	{	// initial delay after pressed
		if (Key == KEY_STAR  ||
		    Key == KEY_F     ||
		    Key == KEY_SIDE2 ||
		    Key == KEY_SIDE1 ||
		    Key == KEY_UP    ||
		    Key == KEY_DOWN  ||
		    Key == KEY_EXIT  ||
		    Key == KEY_MENU
		    #ifdef ENABLE_MAIN_KEY_HOLD
		        || Key <= KEY_9       // keys 0-9 can be held down to bypass pressing the F-Key
		    #endif
			)
		{
			gKeyBeingHeld = true;
			APP_ProcessKey(Key, true, true);
		}
	} else if (gDebounceCounter > 128) {
		if (Key == KEY_UP || Key == KEY_DOWN) {
			gKeyBeingHeld = true;
			if ((gDebounceCounter & 15) == 0) {
				APP_ProcessKey(Key, true, true);
			}
		}
		if (gDebounceCounter != 0xFFFF) {
			return;
		}
		gDebounceCounter = 128;
	}
}

void APP_TimeSlice10ms(void)
{
	gFlashLightBlinkCounter++;

	#ifdef ENABLE_BOOT_BEEPS
		if (boot_counter_10ms > 0)
			if ((boot_counter_10ms % 25) == 0)
				AUDIO_PlayBeep(BEEP_880HZ_40MS_OPTIONAL);
	#endif

	#ifdef ENABLE_AM_FIX
		if (gEeprom.VfoInfo[gEeprom.RX_CHANNEL].IsAM && gSetting_AM_fix)
			AM_fix_10ms(gEeprom.RX_CHANNEL);
	#endif

#if defined(ENABLE_UART)
	if (UART_IsCommandAvailable()) {
		__disable_irq();
		UART_HandleCommand();
		__enable_irq();
	}
#endif

	if (gReducedService) {
		return;
	}

	if (gCurrentFunction != FUNCTION_POWER_SAVE || !gRxIdleMode) {
		APP_CheckRadioInterrupts();
	}

	if (gCurrentFunction != FUNCTION_TRANSMIT) {
		if (gUpdateStatus)
			UI_DisplayStatus(false);

		if (gUpdateDisplay)
		{
			gUpdateDisplay = false;
			GUI_DisplayScreen();
		}
	}
	else
	{	// transmitting
		#ifdef ENABLE_AUDIO_BAR
			if (gSetting_mic_bar && (gFlashLightBlinkCounter % (150 / 10)) == 0) // once every 150ms
				UI_DisplayAudioBar();
		#endif

		if (gUpdateDisplay)
		{
			gUpdateDisplay = false;
			GUI_DisplayScreen();
		}

		if (gUpdateStatus)
			UI_DisplayStatus(false);
	}

	// Skipping authentic device checks

#if defined(ENABLE_FMRADIO)
	if (gFmRadioMode && gFmRadioCountdown) {
		return;
	}
#endif

	if (gFlashLightState == FLASHLIGHT_BLINK && (gFlashLightBlinkCounter & 15U) == 0) {
		GPIO_FlipBit(&GPIOC->DATA, GPIOC_PIN_FLASHLIGHT);
	}
	if (gVoxResumeCountdown) {
		gVoxResumeCountdown--;
	}
	if (gVoxPauseCountdown) {
		gVoxPauseCountdown--;
	}
	if (gCurrentFunction == FUNCTION_TRANSMIT) {
#if defined(ENABLE_ALARM)
		if (gAlarmState == ALARM_STATE_TXALARM || gAlarmState == ALARM_STATE_ALARM) {
			uint16_t Tone;

			gAlarmRunningCounter++;
			gAlarmToneCounter++;

			Tone = 500 + (gAlarmToneCounter * 25);
			if (Tone > 1500) {
				Tone = 500;
				gAlarmToneCounter = 0;
			}
			BK4819_SetScrambleFrequencyControlWord(Tone);
			if (gEeprom.ALARM_MODE == ALARM_MODE_TONE && gAlarmRunningCounter == 512) {
				gAlarmRunningCounter = 0;
				if (gAlarmState == ALARM_STATE_TXALARM) {
					gAlarmState = ALARM_STATE_ALARM;
					RADIO_EnableCxCSS();
					BK4819_SetupPowerAmplifier(0, 0);
					BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1, false);
					BK4819_Enable_AfDac_DiscMode_TxDsp();
					BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_RED, false);
					GUI_DisplayScreen();
				} else {
					gAlarmState = ALARM_STATE_TXALARM;
					GUI_DisplayScreen();
					BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_RED, true);
					RADIO_SetTxParameters();
					BK4819_TransmitTone(true, 500);
					SYSTEM_DelayMs(2);
					GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
					gEnableSpeaker = true;
					gAlarmToneCounter = 0;
				}
			}
		}
#endif
		if (gRTTECountdown) {
			gRTTECountdown--;
			if (gRTTECountdown == 0) {
				FUNCTION_Select(FUNCTION_FOREGROUND);
				gUpdateDisplay = true;
			}
		}
	}
#if defined(ENABLE_FMRADIO)
	if (gFmRadioMode && gFM_RestoreCountdown) {
		gFM_RestoreCountdown--;
		if (gFM_RestoreCountdown == 0) {
			FM_Start();
			GUI_SelectNextDisplay(DISPLAY_FM);
		}
	}
#endif
	if (gScreenToDisplay == DISPLAY_SCANNER) {
		uint32_t Result;
		int32_t Delta;
		BK4819_CssScanResult_t ScanResult;
		uint16_t CtcssFreq;

		if (gScanDelay) {
			gScanDelay--;
			APP_CheckKeys();
			return;
		}
		if (gScannerEditState != 0) {
			APP_CheckKeys();
			return;
		}

		switch (gScanCssState) {
		case SCAN_CSS_STATE_OFF:
			if (!BK4819_GetFrequencyScanResult(&Result)) {
				break;
			}

			Delta = Result - gScanFrequency;
			gScanFrequency = Result;
			if (Delta < 0) {
				Delta = -Delta;
			}
			if (Delta < 100) {
				gScanHitCount++;
			} else {
				gScanHitCount = 0;
			}
			BK4819_DisableFrequencyScan();
			if (gScanHitCount < 3) {
				BK4819_EnableFrequencyScan();
			} else {
				BK4819_SetScanFrequency(gScanFrequency);
				gScanCssResultCode = 0xFF;
				gScanCssResultType = 0xFF;
				gScanHitCount = 0;
				gScanUseCssResult = false;
				gScanProgressIndicator = 0;
				gScanCssState = SCAN_CSS_STATE_SCANNING;
				GUI_SelectNextDisplay(DISPLAY_SCANNER);
			}
			gScanDelay = 21;
			break;

		case SCAN_CSS_STATE_SCANNING:
			ScanResult = BK4819_GetCxCSSScanResult(&Result, &CtcssFreq);
			if (ScanResult == BK4819_CSS_RESULT_NOT_FOUND) {
				break;
			}
			BK4819_Disable();
			if (ScanResult == BK4819_CSS_RESULT_CDCSS) {
				uint8_t Code;

				Code = DCS_GetCdcssCode(Result);
				if (Code != 0xFF) {
					gScanCssResultCode = Code;
					gScanCssResultType = CODE_TYPE_DIGITAL;
					gScanCssState = SCAN_CSS_STATE_FOUND;
					gScanUseCssResult = true;
				}
			} else if (ScanResult == BK4819_CSS_RESULT_CTCSS) {
				uint8_t Code;

				Code = DCS_GetCtcssCode(CtcssFreq);
				if (Code != 0xFF) {
					if (Code == gScanCssResultCode && gScanCssResultType == CODE_TYPE_CONTINUOUS_TONE) {
						gScanHitCount++;
						if (gScanHitCount >= 2) {
							gScanCssState = SCAN_CSS_STATE_FOUND;
							gScanUseCssResult = true;
							gUpdateStatus     = true;
						}
					} else {
						gScanHitCount = 0;
					}
					gScanCssResultType = CODE_TYPE_CONTINUOUS_TONE;
					gScanCssResultCode = Code;
				}
			}
			if (gScanCssState < SCAN_CSS_STATE_FOUND) {
				BK4819_SetScanFrequency(gScanFrequency);
				gScanDelay = 21;
				break;
			}
			GUI_SelectNextDisplay(DISPLAY_SCANNER);
			break;
		default:
			break;
		}
	}

#if defined(ENABLE_AIRCOPY)
	if (gScreenToDisplay == DISPLAY_AIRCOPY && gAircopyState == AIRCOPY_TRANSFER && gAirCopyIsSendMode == 1) {
		if (gAircopySendCountdown) {
			gAircopySendCountdown--;
			if (gAircopySendCountdown == 0) {
				AIRCOPY_SendMessage();
				GUI_DisplayScreen();
			}
		}
	}
#endif

	APP_CheckKeys();
}

void cancelUserInputModes(void)
{
	gKeyInputCountdown = 0;
	if (gDTMF_InputMode || gInputBoxIndex > 0)
	{
		gDTMF_InputMode       = false;
		gDTMF_InputIndex      = 0;
		memset(gDTMF_String, 0, sizeof(gDTMF_String));
		gInputBoxIndex        = 0;
		gRequestDisplayScreen = DISPLAY_MAIN;
		gBeepToPlay           = BEEP_1KHZ_60MS_OPTIONAL;
	}
}

void APP_TimeSlice500ms(void)
{
	// Skipped authentic device check

	if (gKeypadLocked) {
		gKeypadLocked--;
		if (!gKeypadLocked) {
			gUpdateDisplay = true;
		}
	}
	if (gKeyInputCountdown > 0) {
		gKeyInputCountdown--;
		if (gKeyInputCountdown == 0)
			cancelUserInputModes();
	}

	// Skipped authentic device check

#if defined(ENABLE_FMRADIO)
	if (gFmRadioCountdown) {
		gFmRadioCountdown--;
		if (gFmRadioMode)           // 1of11
			return;
	}
#endif

	if (gReducedService) {
		BOARD_ADC_GetBatteryInfo(&gBatteryCurrentVoltage, &gBatteryCurrent);
		if (gBatteryCurrent > 500 || gBatteryCalibration[3] < gBatteryCurrentVoltage) {
#if defined(ENABLE_OVERLAY)
			overlay_FLASH_RebootToBootloader();
#else
			NVIC_SystemReset();
#endif
		}
		return;
	}

	gBatteryCheckCounter++;

	// Skipped authentic device check

	if (gCurrentFunction != FUNCTION_TRANSMIT) {
		if ((gBatteryCheckCounter & 1) == 0) {
			BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryVoltageIndex++], &gBatteryCurrent);
			if (gBatteryVoltageIndex > 3) {
				gBatteryVoltageIndex = 0;
			}
			BATTERY_GetReadings(true);
		}

		// regular statusbar updates (once every 2 sec) if need be
		if ((gBatteryCheckCounter & 3) == 0)
			if (gChargingWithTypeC || gSetting_battery_text > 0)
				gUpdateStatus = true;

		if (gCurrentFunction != FUNCTION_POWER_SAVE) {
			updateRSSI(gEeprom.RX_CHANNEL);
		}
		if (
#if defined(ENABLE_FMRADIO)
			(gFM_ScanState == FM_SCAN_OFF || gAskToSave) &&
#endif
			gCssScanMode == CSS_SCAN_MODE_OFF) {
			if (gBacklightCountdown) {
				if (gScreenToDisplay != DISPLAY_MENU || gMenuCursor != MENU_ABR) { // don't turn off backlight if user is in backlight menu option
					gBacklightCountdown--;
					if (gBacklightCountdown == 0) {
						if (gEeprom.BACKLIGHT < (ARRAY_SIZE(gSubMenu_BACKLIGHT) - 1))
							GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);
					}
				}
			}

			#ifdef ENABLE_AIRCOPY
				if (gScanState == SCAN_OFF && gScreenToDisplay != DISPLAY_AIRCOPY && (gScreenToDisplay != DISPLAY_SCANNER || gScanCssState >= SCAN_CSS_STATE_FOUND))
			#else
				if (gScanState == SCAN_OFF && (gScreenToDisplay != DISPLAY_SCANNER || gScanCssState >= SCAN_CSS_STATE_FOUND))
			#endif
			{
				if (gEeprom.AUTO_KEYPAD_LOCK && gKeyLockCountdown && !gDTMF_InputMode) {
					gKeyLockCountdown--;
					if (gKeyLockCountdown == 0) {
						gEeprom.KEY_LOCK = true;
					}
					gUpdateStatus = true;
				}
				if (gVoltageMenuCountdown) {
					gVoltageMenuCountdown--;
					if (gVoltageMenuCountdown == 0) {
						if (gEeprom.BACKLIGHT == 0 && gScreenToDisplay == DISPLAY_MENU)
						{
							gBacklightCountdown = 0;
							GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);	// turn the backlight OFF
						}
						if (gInputBoxIndex || gDTMF_InputMode || gScreenToDisplay == DISPLAY_MENU) {
							AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
						}
						if (gScreenToDisplay == DISPLAY_SCANNER) {
							BK4819_StopScan();
							RADIO_ConfigureChannel(0, 2);
							RADIO_ConfigureChannel(1, 2);
							RADIO_SetupRegisters(true);
						}
						gWasFKeyPressed = false;
						gUpdateStatus = true;
						gInputBoxIndex = 0;
						gDTMF_InputMode = false;
						gDTMF_InputIndex = 0;
						gAskToSave = false;
						gAskToDelete = false;

						#ifdef ENABLE_FMRADIO
							if (gFmRadioMode &&
							    gCurrentFunction != FUNCTION_RECEIVE &&
								gCurrentFunction != FUNCTION_MONITOR &&
								gCurrentFunction != FUNCTION_TRANSMIT)
							{
								GUI_SelectNextDisplay(DISPLAY_FM);
							}
							else
						#endif
						#ifdef ENABLE_NO_SCAN_TIMEOUT
							if (gScreenToDisplay != DISPLAY_SCANNER)
						#endif
								GUI_SelectNextDisplay(DISPLAY_MAIN);
					}
				}
			}
		}
	}

#if defined(ENABLE_FMRADIO)
	if (!gPttIsPressed && gFM_ResumeCountdown) {
		gFM_ResumeCountdown--;
		if (gFM_ResumeCountdown == 0) {
			RADIO_SetVfoState(VFO_STATE_NORMAL);
			if (gCurrentFunction != FUNCTION_RECEIVE && gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_MONITOR && gFmRadioMode) {
				FM_Start();
				GUI_SelectNextDisplay(DISPLAY_FM);
			}
		}
	}
#endif

	if (gLowBattery) {
		gLowBatteryBlink = ++gLowBatteryCountdown & 1;
		UI_DisplayBattery(0, gLowBatteryBlink);
		if (gCurrentFunction != FUNCTION_TRANSMIT) {
			if (gLowBatteryCountdown < 30) {
				if (gLowBatteryCountdown == 29 && !gChargingWithTypeC) {
					AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);
				}
			} else {
				gLowBatteryCountdown = 0;
				if (!gChargingWithTypeC) {
					AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP);
#ifdef ENABLE_VOICE
					AUDIO_SetVoiceID(0, VOICE_ID_LOW_VOLTAGE);
#endif
					if (gBatteryDisplayLevel == 0) {
						#ifdef ENABLE_VOICE
						AUDIO_PlaySingleVoice(true);
						#endif
						gReducedService = true;
						FUNCTION_Select(FUNCTION_POWER_SAVE);
						ST7565_Configure_GPIO_B11();
						if (gEeprom.BACKLIGHT < (ARRAY_SIZE(gSubMenu_BACKLIGHT) - 1))
							GPIO_ClearBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);
					} 
#ifdef ENABLE_VOICE
					else {
						AUDIO_PlaySingleVoice(false);
					}
#endif
				}
			}
		}
	}

	if (gScreenToDisplay == DISPLAY_SCANNER && gScannerEditState == 0 && gScanCssState < SCAN_CSS_STATE_FOUND) {
		gScanProgressIndicator++;
#ifndef ENABLE_NOSCANTIMEOUT					
		if (gScanProgressIndicator > 32) {
			if (gScanCssState == SCAN_CSS_STATE_SCANNING && !gScanSingleFrequency) {
				gScanCssState = SCAN_CSS_STATE_FOUND;
			} else {
				gScanCssState = SCAN_CSS_STATE_FAILED;
			}
		}
#endif	  
		gUpdateDisplay = true;
	}

	if (gDTMF_CallState != DTMF_CALL_STATE_NONE && gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE) {
		if (gDTMF_AUTO_RESET_TIME) {
			gDTMF_AUTO_RESET_TIME--;
			if (gDTMF_AUTO_RESET_TIME == 0) {
				gDTMF_CallState = DTMF_CALL_STATE_NONE;
				gUpdateDisplay = true;
			}
		}
		if (gDTMF_DecodeRing && gDTMF_DecodeRingCountdown) {
			gDTMF_DecodeRingCountdown--;
			if ((gDTMF_DecodeRingCountdown % 3) == 0) {
				AUDIO_PlayBeep(BEEP_440HZ_500MS);
			}
			if (gDTMF_DecodeRingCountdown == 0) {
				gDTMF_DecodeRing = false;
			}
		}
	}

	if (gDTMF_IsTx && gDTMF_TxStopCountdown) {
		gDTMF_TxStopCountdown--;
		if (gDTMF_TxStopCountdown == 0) {
			gDTMF_IsTx = false;
			gUpdateDisplay = true;
		}
	}

	if (gDTMF_RecvTimeout) {
		gDTMF_RecvTimeout--;
		if (gDTMF_RecvTimeout == 0) {
			gDTMF_WriteIndex = 0;
			memset(gDTMF_Received, 0, sizeof(gDTMF_Received));
		}
	}
	if (gDTMF_RecvTimeoutSaved > 0)
	{
		gDTMF_RecvTimeoutSaved--;
		if (gDTMF_RecvTimeoutSaved == 0)
		{
			gDTMF_ReceivedSaved[0] = '\0';
			gUpdateDisplay = true;
		}
	}
}

#if defined(ENABLE_ALARM) || defined(ENABLE_TX1750)
static void ALARM_Off(void)
{
	gAlarmState = ALARM_STATE_OFF;
	GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
	gEnableSpeaker = false;
	if (gEeprom.ALARM_MODE == ALARM_MODE_TONE) {
		RADIO_SendEndOfTransmission();
		RADIO_EnableCxCSS();
	}
	gVoxResumeCountdown = 0x50;
	SYSTEM_DelayMs(5);
	RADIO_SetupRegisters(true);
	gRequestDisplayScreen = DISPLAY_MAIN;
}
#endif

void CHANNEL_Next(bool bBackup, int8_t Direction)
{
	RADIO_SelectVfos();
	gNextMrChannel = gRxVfo->CHANNEL_SAVE;
	gCurrentScanList = 0;
	gScanState = Direction;
	if (IS_MR_CHANNEL(gNextMrChannel)) {
		if (bBackup) {
			gRestoreMrChannel = gNextMrChannel;
		}
		MR_NextChannel();
	} else {
		if (bBackup) {
			gRestoreFrequency = gRxVfo->ConfigRX.Frequency;
		}
		FREQ_NextChannel();
	}
	ScanPauseDelayIn10msec = 50;
	gScheduleScanListen = false;
	gRxReceptionMode = RX_MODE_NONE;
	gScanPauseMode = false;
	bScanKeepFrequency = false;
}

static void APP_ProcessKey(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	bool bFlag = false;

	const bool backlight_was_on = GPIO_CheckBit(&GPIOB->DATA, GPIOB_PIN_BACKLIGHT);

	if (Key == KEY_EXIT && !backlight_was_on && gEeprom.BACKLIGHT > 0)
	{	// just turn the light on for now so the user can see what's what
		BACKLIGHT_TurnOn();
		gBeepToPlay = BEEP_NONE;
		return;
	}

	if (gCurrentFunction == FUNCTION_POWER_SAVE) {
		FUNCTION_Select(FUNCTION_FOREGROUND);
	}
	gBatterySaveCountdown = 1000;
	if (gEeprom.AUTO_KEYPAD_LOCK) {
		gKeyLockCountdown = 30;
	}
	
	if (Key == KEY_EXIT && bKeyPressed && bKeyHeld && gDTMF_ReceivedSaved[0] > 0)
	{	// clear the live DTMF decoder if the EXIT key is held
		gDTMF_RecvTimeoutSaved = 0;
		gDTMF_ReceivedSaved[0] = '\0';
		gUpdateDisplay         = true;
	}
	if (!bKeyPressed) {
		if (gFlagSaveVfo) {
			SETTINGS_SaveVfoIndices();
			gFlagSaveVfo = false;
		}
		if (gFlagSaveSettings) {
			SETTINGS_SaveSettings();
			gFlagSaveSettings = false;
		}
#if defined(ENABLE_FMRADIO)
		if (gFlagSaveFM) {
			SETTINGS_SaveFM();
			gFlagSaveFM = false;
		}
#endif
		if (gFlagSaveChannel) {
			SETTINGS_SaveChannel(
				gTxVfo->CHANNEL_SAVE,
				gEeprom.TX_CHANNEL,
				gTxVfo,
				gFlagSaveChannel);
			gFlagSaveChannel = false;
			RADIO_ConfigureChannel(gEeprom.TX_CHANNEL, 1);
			RADIO_SetupRegisters(true);
			GUI_SelectNextDisplay(DISPLAY_MAIN);
		}
	} else {
		if (Key != KEY_PTT) {
			gVoltageMenuCountdown = 0x10;
		}
		BACKLIGHT_TurnOn();
		if (gDTMF_DecodeRing) {
			gDTMF_DecodeRing = false;
			AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);
			if (Key != KEY_PTT) {
				gPttWasReleased = true;
				return;
			}
		}
	}

	if (gEeprom.KEY_LOCK && gCurrentFunction != FUNCTION_TRANSMIT && Key != KEY_PTT)
	{	// keyboard is locked

		if (Key == KEY_F)
		{	// function/key-lock key

			if (!bKeyPressed)
				return;

			if (!bKeyHeld)
			{
				// keypad is locked, tell the user
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
				gKeypadLocked  = 4;      // 2 seconds
				gUpdateDisplay = true;
				return;
			}
		}
		else
		if (Key != KEY_SIDE1 && Key != KEY_SIDE2)
		{
			if (!bKeyPressed || bKeyHeld)
				return;

			// keypad is locked, tell the user
			AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
			gKeypadLocked  = 4;          // 2 seconds
			gUpdateDisplay = true;
			return;
		}
	}

	if (Key != KEY_PTT   &&
	    Key != KEY_UP    &&
	    Key != KEY_DOWN  &&
	    Key != KEY_EXIT  &&
	    Key != KEY_SIDE1 &&
	    Key != KEY_SIDE2 &&
	    Key != KEY_STAR)
	{
		if (gScanState != SCAN_OFF || gCssScanMode != CSS_SCAN_MODE_OFF)
		{	// frequency or CTCSS/DCS scanning
			if (bKeyPressed && !bKeyHeld)
				AUDIO_PlayBeep(BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL);
			return;
		}
	}


	if (gPttWasPressed && Key == KEY_PTT) {
		bFlag = bKeyHeld;
		if (!bKeyPressed) {
			bFlag = true;
			gPttWasPressed = false;
		}
	}

	if (gPttWasReleased && Key != KEY_PTT) {
		if (bKeyHeld) {
			bFlag = true;
		}
		if (!bKeyPressed) {
			bFlag = true;
			gPttWasReleased = false;
		}
	}

	if (gWasFKeyPressed && Key > KEY_9 && Key != KEY_F && Key != KEY_STAR && Key != KEY_MENU)
	{
		gWasFKeyPressed = false;
		gUpdateStatus = true;
	}

	if (gF_LOCK && (Key == KEY_PTT || Key == KEY_SIDE2 || Key == KEY_SIDE1))
		return;

	if (!bFlag) {
		if (gCurrentFunction == FUNCTION_TRANSMIT) {
#if defined(ENABLE_ALARM) || defined(ENABLE_TX1750)

			#ifdef ENABLE_ALARM
				if (gAlarmState == ALARM_STATE_OFF)
			#endif
			{
				if (Key == KEY_PTT) {
					GENERIC_Key_PTT(bKeyPressed);
				} else {
					char Code;

					if (Key == KEY_SIDE2) {
						Code = 0xFE;
					} else {
						Code = DTMF_GetCharacter(Key);
						if (Code == 0xFF) {
							goto Skip;
						}
					}

					if (bKeyHeld || !bKeyPressed) {
						if (!bKeyPressed) {
							GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
							gEnableSpeaker = false;
							BK4819_ExitDTMF_TX(false);
							if (gCurrentVfo->SCRAMBLING_TYPE == 0 || !gSetting_ScrambleEnable) {
								BK4819_DisableScramble();
							} else {
								BK4819_EnableScramble(gCurrentVfo->SCRAMBLING_TYPE - 1);
							}
						}
					} else {
						if (gEeprom.DTMF_SIDE_TONE) {
							GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_AUDIO_PATH);
							gEnableSpeaker = true;
						}
						BK4819_DisableScramble();
						if (Code == 0xFE) {
							BK4819_TransmitTone(gEeprom.DTMF_SIDE_TONE, 1750);
						} else {
							BK4819_PlayDTMFEx(gEeprom.DTMF_SIDE_TONE, Code);
						}
					}
				}
			}
#if defined(ENABLE_ALARM) || defined(ENABLE_TX1750)
			else if (!bKeyHeld && bKeyPressed) {

				ALARM_Off();
				if (gEeprom.REPEATER_TAIL_TONE_ELIMINATION == 0) {
					FUNCTION_Select(FUNCTION_FOREGROUND);
				} else {
					gRTTECountdown = gEeprom.REPEATER_TAIL_TONE_ELIMINATION * 10;
				}
				if (Key == KEY_PTT) {
					gPttWasPressed = true;
				} else {
					gPttWasReleased = true;
				}
#endif
			}
		} else if (Key != KEY_SIDE1 && Key != KEY_SIDE2) {
			switch (gScreenToDisplay) {
			case DISPLAY_MAIN:
				MAIN_ProcessKeys(Key, bKeyPressed, bKeyHeld);
#ifdef ENABLE_MAIN_KEY_HOLD
				bKeyHeld = false;	// allow the channel setting to be saved
#endif
				break;
#if defined(ENABLE_FMRADIO)
			case DISPLAY_FM:
				FM_ProcessKeys(Key, bKeyPressed, bKeyHeld);
				break;
#endif
			case DISPLAY_MENU:
				MENU_ProcessKeys(Key, bKeyPressed, bKeyHeld);
				break;
			case DISPLAY_SCANNER:
				SCANNER_ProcessKeys(Key, bKeyPressed, bKeyHeld);
				break;
#if defined(ENABLE_AIRCOPY)
			case DISPLAY_AIRCOPY:
				AIRCOPY_ProcessKeys(Key, bKeyPressed, bKeyHeld);
				break;
#endif
			default:
				break;
			}
		} else if (gScreenToDisplay != DISPLAY_SCANNER
#if defined(ENABLE_AIRCOPY)
				&& gScreenToDisplay != DISPLAY_AIRCOPY
#endif
			) {
			ACTION_Handle(Key, bKeyPressed, bKeyHeld);
		} else if (!bKeyHeld && bKeyPressed) {
			gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		}
	}
	else
	{
		if (Key == KEY_EXIT && bKeyHeld)
			cancelUserInputModes();
	}

Skip:
	if (gBeepToPlay) {
		AUDIO_PlayBeep(gBeepToPlay);
		gBeepToPlay = BEEP_NONE;
	}

	if (gFlagAcceptSetting) {
		MENU_AcceptSetting();
		gFlagRefreshSetting = true;
		gFlagAcceptSetting = false;
	}
	if (gFlagStopScan) {
		BK4819_StopScan();
		gFlagStopScan = false;
	}
	if (gRequestSaveSettings) {
		if (bKeyHeld == 0) {
			SETTINGS_SaveSettings();
		} else {
			gFlagSaveSettings = 1;
		}
		gRequestSaveSettings = false;
		gUpdateStatus = true;
	}
#if defined(ENABLE_FMRADIO)
	if (gRequestSaveFM) {
		if (!bKeyHeld) {
			SETTINGS_SaveFM();
		} else {
			gFlagSaveFM = true;
		}
		gRequestSaveFM = false;
	}
#endif
	if (gRequestSaveVFO) {
		if (!bKeyHeld) {
			SETTINGS_SaveVfoIndices();
		} else {
			gFlagSaveVfo = true;
		}
		gRequestSaveVFO = false;
	}
	if (gRequestSaveChannel) {
		if (!bKeyHeld) {
			SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, gEeprom.TX_CHANNEL, gTxVfo, gRequestSaveChannel);
			if (gScreenToDisplay != DISPLAY_SCANNER) {
				gVfoConfigureMode = VFO_CONFIGURE_1;
			}
		} else {
			gFlagSaveChannel = gRequestSaveChannel;
			if (gRequestDisplayScreen == DISPLAY_INVALID) {
				gRequestDisplayScreen = DISPLAY_MAIN;
			}
		}
		gRequestSaveChannel = 0;
	}


	if (gVfoConfigureMode != VFO_CONFIGURE_0) {
		if (gFlagResetVfos) {
			RADIO_ConfigureChannel(0, gVfoConfigureMode);
			RADIO_ConfigureChannel(1, gVfoConfigureMode);
		} else {
			RADIO_ConfigureChannel(gEeprom.TX_CHANNEL, gVfoConfigureMode);
		}
		if (gRequestDisplayScreen == DISPLAY_INVALID) {
			gRequestDisplayScreen = DISPLAY_MAIN;
		}
		gFlagReconfigureVfos = true;
		gVfoConfigureMode = VFO_CONFIGURE_0;
		gFlagResetVfos = false;
	}

	if (gFlagReconfigureVfos) {
		RADIO_SelectVfos();
#if defined(ENABLE_NOAA)
		RADIO_ConfigureNOAA();
#endif
		RADIO_SetupRegisters(true);
		gDTMF_AUTO_RESET_TIME = 0;
		gDTMF_CallState = DTMF_CALL_STATE_NONE;
		gDTMF_TxStopCountdown = 0;
		gDTMF_IsTx = false;
		gVFO_RSSI_Level[0] = 0;
		gVFO_RSSI_Level[1] = 0;
		gFlagReconfigureVfos = false;
		if (gMonitor)
			ACTION_Monitor();   // 1of11
	}

	if (gFlagRefreshSetting) {
		MENU_ShowCurrentSetting();
		gFlagRefreshSetting = false;
	}
	if (gFlagStartScan) {
		gMonitor = false;
#ifdef ENABLE_VOICE
		AUDIO_SetVoiceID(0, VOICE_ID_SCANNING_BEGIN);
		AUDIO_PlaySingleVoice(true);
#endif
		SCANNER_Start();
		gRequestDisplayScreen = DISPLAY_SCANNER;
		gFlagStartScan = false;
	}
	if (gFlagPrepareTX) {
		RADIO_PrepareTX();
		gFlagPrepareTX = false;
	}
#ifdef ENABLE_VOICE
	if (gAnotherVoiceID != VOICE_ID_INVALID) {
		if (gAnotherVoiceID < 76) {
			AUDIO_SetVoiceID(0, gAnotherVoiceID);
		}
		AUDIO_PlaySingleVoice(false);
		gAnotherVoiceID = VOICE_ID_INVALID;
	}
#endif
	GUI_SelectNextDisplay(gRequestDisplayScreen);
	gRequestDisplayScreen = DISPLAY_INVALID;
}

