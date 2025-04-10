//
// userinterface.cpp
//
// MiniDexed - Dexed FM synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "userinterface.h"
#include "minijv880.h"
#include "emulator/mcu.h"
#include <circle/logger.h>
#include <circle/string.h>
#include <circle/startup.h>
#include <circle/timer.h>
#include <string.h>
#include <assert.h>
#include <chrono>

LOGMODULE ("ui");

void set_pixel(unsigned char *screen, int x, int y, bool value) {
  if (!value)
    screen[(y / 8) * 128 + x] |= 1 << (y % 8);
  else
    screen[(y / 8) * 128 + x] &= ~(1 << (y % 8));
}

CUserInterface::CUserInterface (CMiniJV880 *pMiniJV880, CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster, CSPIMaster *pSPIMaster, CConfig *pConfig)
:	m_pMiniJV880 (pMiniJV880),
	m_pGPIOManager (pGPIOManager),
	m_pI2CMaster (pI2CMaster),
	m_pSPIMaster (pSPIMaster),
	m_pConfig (pConfig),
	m_pLCD (0),
	m_pLCDBuffered (0),
	m_pUIButtons (0),
	m_pRotaryEncoder (0),
	m_bSwitchPressed (false),
	m_lastTick (0)
{
	LOGNOTE("UI Constructor");
	if (!pGPIOManager || !pI2CMaster || !pSPIMaster || !pConfig) {
        LOGNOTE("Invalid dependencies!");
	}
	screen_buffer = (u8 *)malloc(512);
}

CUserInterface::~CUserInterface (void)
{
	delete m_pRotaryEncoder;
	delete m_pUIButtons;
	delete m_pLCDBuffered;
	delete m_pLCD;
}

bool CUserInterface::Initialize (void)
{
	assert (m_pConfig);

	if (m_pConfig->GetLCDEnabled ())
	{
		unsigned i2caddr = m_pConfig->GetLCDI2CAddress ();
		unsigned ssd1306addr = m_pConfig->GetSSD1306LCDI2CAddress ();
		bool st7789 = m_pConfig->GetST7789Enabled ();
		if (ssd1306addr != 0) {
			m_pSSD1306 = new CSSD1306Device (m_pConfig->GetSSD1306LCDWidth (), m_pConfig->GetSSD1306LCDHeight (),
											 m_pI2CMaster, ssd1306addr,
											 m_pConfig->GetSSD1306LCDRotate (), m_pConfig->GetSSD1306LCDMirror ());
			if (!m_pSSD1306->Initialize ())
			{
				LOGDBG("LCD: SSD1306 initialization failed");
				return false;
			}
			LOGDBG ("LCD: SSD1306");
			m_pLCD = m_pSSD1306;
		}
		else if (st7789)
		{
			if (m_pSPIMaster == nullptr)
			{
				LOGDBG("LCD: ST7789 Enabled but SPI Initialisation Failed");
				return false;
			}

			unsigned long nSPIClock = 1000 * m_pConfig->GetSPIClockKHz();
			unsigned nSPIMode = m_pConfig->GetSPIMode();
			unsigned nCPHA = (nSPIMode & 1) ? 1 : 0;
			unsigned nCPOL = (nSPIMode & 2) ? 1 : 0;
			LOGDBG("SPI: CPOL=%u; CPHA=%u; CLK=%u",nCPOL,nCPHA,nSPIClock);
			m_pST7789Display = new CST7789Display (m_pSPIMaster,
							m_pConfig->GetST7789Data(),
							m_pConfig->GetST7789Reset(),
							m_pConfig->GetST7789Backlight(),
							m_pConfig->GetST7789Width(),
							m_pConfig->GetST7789Height(),
							nCPOL, nCPHA, nSPIClock,
							m_pConfig->GetST7789Select());
			if (m_pST7789Display->Initialize())
			{
				m_pST7789Display->SetRotation (m_pConfig->GetST7789Rotation());
				bool bLargeFont = !(m_pConfig->GetST7789SmallFont());
				m_pST7789 = new CST7789Device (m_pSPIMaster, m_pST7789Display, m_pConfig->GetLCDColumns (), m_pConfig->GetLCDRows (), bLargeFont, bLargeFont);
				if (m_pST7789->Initialize())
				{
					LOGDBG ("LCD: ST7789");
					m_pLCD = m_pST7789;
				}
				else
				{
					LOGDBG ("LCD: Failed to initalize ST7789 character device");
					delete (m_pST7789);
					delete (m_pST7789Display);
					m_pST7789 = nullptr;
					m_pST7789Display = nullptr;
					return false;
				}
			}
			else
			{
				LOGDBG ("LCD: Failed to initialize ST7789 display");
				delete (m_pST7789Display);
				m_pST7789Display = nullptr;
				return false;
			}
		}
		else if (i2caddr == 0)
		{
			m_pHD44780 = new CHD44780Device (m_pConfig->GetLCDColumns (), m_pConfig->GetLCDRows (),
							 m_pConfig->GetLCDPinData4 (),
							 m_pConfig->GetLCDPinData5 (),
							 m_pConfig->GetLCDPinData6 (),
							 m_pConfig->GetLCDPinData7 (),
							 m_pConfig->GetLCDPinEnable (),
							 m_pConfig->GetLCDPinRegisterSelect (),
							 m_pConfig->GetLCDPinReadWrite ());
			if (!m_pHD44780->Initialize ())
			{
				LOGDBG("LCD: HD44780 initialization failed");
				return false;
			}
			LOGDBG ("LCD: HD44780");
			m_pLCD = m_pHD44780;
		}
		else
		{
			m_pHD44780 = new CHD44780Device (m_pI2CMaster, i2caddr,
							m_pConfig->GetLCDColumns (), m_pConfig->GetLCDRows ());
			if (!m_pHD44780->Initialize ())
			{
				LOGDBG("LCD: HD44780 (I2C) initialization failed");
				return false;
			}
			LOGDBG ("LCD: HD44780 I2C");
			m_pLCD = m_pHD44780;
		}
		assert (m_pLCD);

		m_pLCDBuffered = new CWriteBufferDevice (m_pLCD);
		assert (m_pLCDBuffered);

		LCDWrite ("\x1B[?25l\x1B""d+");		// cursor off, autopage mode
		LCDWrite ("MiniJV880\nLoading...");
		m_pLCDBuffered->Update ();

		LOGDBG ("LCD initialized");
	}

	m_pUIButtons = new CUIButtons (
				m_pConfig,
                  m_pConfig->GetButtonPinPreview (), m_pConfig->GetButtonActionPreview (),
									m_pConfig->GetButtonPinLeft (), m_pConfig->GetButtonActionRight (),
									m_pConfig->GetButtonPinRight (), m_pConfig->GetButtonActionRight (),
									m_pConfig->GetButtonPinData (), m_pConfig->GetButtonActionData (),
									m_pConfig->GetButtonPinToneSelect (), m_pConfig->GetButtonActionToneSelect (),
                  m_pConfig->GetButtonPinPatchPerform (), m_pConfig->GetButtonActionPatchPerform (),
									m_pConfig->GetButtonPinEdit (), m_pConfig->GetButtonActionEdit (),
									m_pConfig->GetButtonPinSystem (), m_pConfig->GetButtonActionSystem (),
									m_pConfig->GetButtonPinRhythm (), m_pConfig->GetButtonActionRhythm (),
									m_pConfig->GetButtonPinUtility (), m_pConfig->GetButtonActionUtility (),
                  m_pConfig->GetButtonPinMute (), m_pConfig->GetButtonActionMute (),
                  m_pConfig->GetButtonPinMonitor (), m_pConfig->GetButtonActionMonitor (),
                  m_pConfig->GetButtonPinCompare (), m_pConfig->GetButtonActionCompare (),
									m_pConfig->GetButtonPinEnter (), m_pConfig->GetButtonActionEnter (),
									m_pConfig->GetDoubleClickTimeout (), m_pConfig->GetLongPressTimeout ()
								  );
	assert (m_pUIButtons);

	if (!m_pUIButtons->Initialize ())
	{
		return false;
	}

	m_pUIButtons->RegisterEventHandler (UIButtonsEventStub, this);

	LOGDBG ("Button User Interface initialized");

	if (m_pConfig->GetEncoderEnabled ())
	{
		m_pRotaryEncoder = new CKY040 (m_pConfig->GetEncoderPinClock (),
					       m_pConfig->GetEncoderPinData (),
					       m_pConfig->GetButtonPinEnter (),
					       m_pGPIOManager);
		assert (m_pRotaryEncoder);

		if (!m_pRotaryEncoder->Initialize ())
		{
			return false;
		}

		m_pRotaryEncoder->RegisterEventHandler (EncoderEventStub, this);

		LOGDBG ("Rotary encoder initialized");
	}
    // TODO: Update LCD

	return true;
}

void CUserInterface::Process (void)
{
	uint32_t* lcd_buffer = m_pMiniJV880->mcu.lcd.LCD_Update();

	// for (size_t y = 0; y < lcd_height; y++) {
	// 	for (size_t x = 0; x < lcd_width; x++) {
	// 		m_pMiniJV880->screenUnbuffered->SetPixel(x + 700, y + 50, lcd_buffer[y * lcd_width + x]);
	// 	}
	// }

	if (m_pLCDBuffered)
	{
		m_pLCDBuffered->Update ();
	}
	if (m_pUIButtons)
	{
		m_pUIButtons->Update();
	}
	// for (size_t y = 0; y < 32; y++) {
  //     for (size_t x = 0; x < 128; x++) {
  //       int destX = (int)(((float)x / 128) * 820);
  //       int destY = (int)(((float)y / 32) * 100);
  //       int sum = 0;
  //       for (int py = -1; py <= 1; py++) {
  //         for (int px = -1; px <= 1; px++) {
  //           if ((destY + py) >= 0 && (destX + px) >= 0) {
  //             bool pixel = m_pMiniJV880->mcu.lcd.lcd_buffer[destY + py][destX + px] == lcd_col1;
  //             sum += pixel;
  //           }
  //         }
  //       }

  //       bool pixel = sum > 0;
  //       // bool pixel = mcu.lcd.lcd_buffer[destY][destX] == lcd_col1;
  //       set_pixel(screen_buffer, x, y, pixel);
	//
  //       // m_ScreenUnbuffered->SetPixel(x + 800, y + 300, pixel ? 0xFFFF : 0x0000);
  //     }
  //   }
	int displayCols = m_pConfig->GetLCDColumns();
	CString Msg ("\x1B[H\E[?25l");
	for (int i = 0; i < 2; i++)
	{
		unsigned long currentTime = CTimer::GetClockTicks();

		// Handle scrolling and pausing logic
    if (currentTime - m_lastScrollTime >= SCROLL_INTERVAL) {
			for (int r = 0; r < 2; r++) {
				if (isPaused[r]) {
					// Check if pause duration has elapsed
					if (currentTime - pauseStartTime[r] >= PAUSE_DURATION) {
						isPaused[r] = false;
						if (isAtEnd[r]) {
							// Reset to beginning after pause at end
							m_scrollPosition[r] = 0;
							isAtEnd[r] = false;
							isPaused[r] = true;  // Pause again at start
							pauseStartTime[r] = currentTime;
						}
					}
				} else {
					// Update scroll position
					m_scrollPosition[r]++;
					// Check if we've reached the end
					if (m_scrollPosition[r] >= ACTUAL_COLS - displayCols) {
						m_scrollPosition[r] = ACTUAL_COLS - displayCols;
						isPaused[r] = true;
						isAtEnd[r] = true;
						pauseStartTime[r] = currentTime;
					}
				}
			}
			m_lastScrollTime = currentTime;
		}


		// Calculate starting position for this row
    int startPos = m_scrollPosition[i];
		for (int j = 0; j < displayCols; j++)
		{
			int sourcePos = (j + startPos) % ACTUAL_COLS;
			uint8_t ch = m_pMiniJV880->mcu.lcd.LCD_Data[i * 40 + sourcePos];
			std::string str(1, ch);
			const char *pString = str.c_str();

			int jj = m_pMiniJV880->mcu.lcd.LCD_DD_RAM % 0x40;
    	int ii = m_pMiniJV880->mcu.lcd.LCD_DD_RAM / 0x40;
			if (i == ii && j == jj && ii < 2 && jj < ACTUAL_COLS && m_pMiniJV880->mcu.lcd.LCD_C) {
				// cursor
				Msg.Append("_");
			} else {
				Msg.Append(pString);
			}
		}
	}
	LCDWrite(Msg);
}

void CUserInterface::LCDWrite (const char *pString)
{
	if (m_pLCDBuffered)
	{
		m_pLCDBuffered->Write (pString, strlen (pString));
	}
}

void CUserInterface::EncoderEventHandler (CKY040::TEvent Event)
{
	uint32_t btn = 0;
	switch (Event)
	{
	case CKY040::EventSwitchDown:
		m_bSwitchPressed = true;
		break;

	case CKY040::EventSwitchUp:
		m_bSwitchPressed = false;
		break;

	case CKY040::EventClockwise:
		if (m_bSwitchPressed) {
			// We must reset the encoder switch button to prevent events from being
			// triggered after the encoder is rotated
			m_pUIButtons->ResetButton(m_pConfig->GetButtonPinEnter());
		} else {
      m_pMiniJV880->mcu.MCU_EncoderTrigger(1);
    }
		break;

	case CKY040::EventCounterclockwise:
		if (m_bSwitchPressed) {
			m_pUIButtons->ResetButton(m_pConfig->GetButtonPinEnter());
		} else {
			m_pMiniJV880->mcu.MCU_EncoderTrigger(0);
		}
		break;

	case CKY040::EventSwitchHold:
		if (m_pRotaryEncoder->GetHoldSeconds () >= 120)
		{
			delete m_pLCD;		// reset LCD

			reboot ();
		}
		break;

	default:
		break;
	}
}

void CUserInterface::EncoderEventStub (CKY040::TEvent Event, void *pParam)
{
	CUserInterface *pThis = static_cast<CUserInterface *> (pParam);
	assert (pThis != 0);

	pThis->EncoderEventHandler (Event);
}

void CUserInterface::UIButtonsEventHandler (CUIButton::BtnEvent Event)
{
	uint32_t btn = 0;
	if (Event == CUIButton::BtnEventPreview) {
		btn |= 1 << MCU_BUTTON_PREVIEW;
	} else {
		btn &= ~(1 << MCU_BUTTON_PREVIEW);
	}
	if (Event == CUIButton::BtnEventLeft) {
		btn |= 1 << MCU_BUTTON_CURSOR_L;
	} else {
		btn &= ~(1 << MCU_BUTTON_CURSOR_L);
	}
	if (Event == CUIButton::BtnEventRight) {
		btn |= 1 << MCU_BUTTON_CURSOR_R;
	} else {
		btn &= ~(1 << MCU_BUTTON_CURSOR_R);
	}
	if (Event == CUIButton::BtnEventData) {
		btn |= 1 << MCU_BUTTON_DATA;
	} else {
		btn &= ~(1 << MCU_BUTTON_DATA);
	}
	if (Event == CUIButton::BtnEventToneSelect) {
		btn |= 1 << MCU_BUTTON_TONE_SELECT;
	} else {
		btn &= ~(1 << MCU_BUTTON_TONE_SELECT);
	}
	if (Event == CUIButton::BtnEventPatchPerform) {
		btn |= 1 << MCU_BUTTON_PATCH_PERFORM;
	} else {
		btn &= ~(1 << MCU_BUTTON_PATCH_PERFORM);
	}
	if (Event == CUIButton::BtnEventEdit) {
		btn |= 1 << MCU_BUTTON_EDIT;
	} else {
		btn &= ~(1 << MCU_BUTTON_EDIT);
	}
	if (Event == CUIButton::BtnEventSystem) {
		btn |= 1 << MCU_BUTTON_SYSTEM;
	} else {
		btn &= ~(1 << MCU_BUTTON_SYSTEM);
	}
	if (Event == CUIButton::BtnEventRhythm) {
		btn |= 1 << MCU_BUTTON_RHYTHM;
	} else {
		btn &= ~(1 << MCU_BUTTON_RHYTHM);
	}
	if (Event == CUIButton::BtnEventUtility) {
		btn |= 1 << MCU_BUTTON_UTILITY;
	} else {
		btn &= ~(1 << MCU_BUTTON_UTILITY);
	}
	if (Event == CUIButton::BtnEventMute) {
			btn |= 1 << MCU_BUTTON_MUTE;
	} else {
		btn &= ~(1 << MCU_BUTTON_MUTE);
	}
	if (Event == CUIButton::BtnEventMonitor) {
		btn |= 1 << MCU_BUTTON_MONITOR;
	} else {
		btn &= ~(1 << MCU_BUTTON_MONITOR);
	}
	if (Event == CUIButton::BtnEventCompare) {
		btn |= 1 << MCU_BUTTON_COMPARE;
	} else {
		btn &= ~(1 << MCU_BUTTON_COMPARE);
	}
	if (Event == CUIButton::BtnEventEnter) {
		btn |= 1 << MCU_BUTTON_ENTER;
	} else {
		btn &= ~(1 << MCU_BUTTON_ENTER);
	}
	LOGNOTE("Button: %x", btn);
	m_pMiniJV880->mcu.mcu_button_pressed = btn;
}

void CUserInterface::UIButtonsEventStub (CUIButton::BtnEvent Event, void *pParam)
{
	CUserInterface *pThis = static_cast<CUserInterface *> (pParam);
	assert (pThis != 0);

	pThis->UIButtonsEventHandler (Event);
}
