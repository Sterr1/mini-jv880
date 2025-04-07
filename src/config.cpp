//
// config.cpp
//
// MiniDexed - Dexed FM synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
//
// Original author of this class:
//	R. Stange <rsta2@o2online.de>
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
#include "config.h"

CConfig::CConfig (FATFS *pFileSystem)
:	m_Properties ("minijv880.ini", pFileSystem)
{
}

CConfig::~CConfig (void)
{
}

void CConfig::Load (void)
{
	m_Properties.Load ();
	
	m_SoundDevice = m_Properties.GetString ("SoundDevice", "pwm");

	if (m_SoundDevice == "hdmi") {
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 384*6);
	}
	else
	{
#ifdef ARM_ALLOW_MULTI_CORE
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 256);  // 128 per channel
#else
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 1024);
#endif
	}
	m_nDACI2CAddress = m_Properties.GetNumber ("DACI2CAddress", 0);
	m_bChannelsSwapped = m_Properties.GetNumber ("ChannelsSwapped", 0) != 0;

	m_nMIDIBaudRate = m_Properties.GetNumber ("MIDIBaudRate", 31250);


	m_bLCDEnabled = m_Properties.GetNumber ("LCDEnabled", 0) != 0;
	m_nLCDPinEnable = m_Properties.GetNumber ("LCDPinEnable", 4);
	m_nLCDPinRegisterSelect = m_Properties.GetNumber ("LCDPinRegisterSelect", 27);
	m_nLCDPinReadWrite = m_Properties.GetNumber ("LCDPinReadWrite", 0);
	m_nLCDPinData4 = m_Properties.GetNumber ("LCDPinData4", 22);
	m_nLCDPinData5 = m_Properties.GetNumber ("LCDPinData5", 23);
	m_nLCDPinData6 = m_Properties.GetNumber ("LCDPinData6", 24);
	m_nLCDPinData7 = m_Properties.GetNumber ("LCDPinData7", 25);
	m_nLCDI2CAddress = m_Properties.GetNumber ("LCDI2CAddress", 0);

	m_nSSD1306LCDI2CAddress = m_Properties.GetNumber ("SSD1306LCDI2CAddress", 0);
	m_nSSD1306LCDWidth = m_Properties.GetNumber ("SSD1306LCDWidth", 128);
	m_nSSD1306LCDHeight = m_Properties.GetNumber ("SSD1306LCDHeight", 32);
	m_bSSD1306LCDRotate = m_Properties.GetNumber ("SSD1306LCDRotate", 0) != 0;
	m_bSSD1306LCDMirror = m_Properties.GetNumber ("SSD1306LCDMirror", 0) != 0;

	m_nSPIBus = m_Properties.GetNumber ("SPIBus", SPI_INACTIVE);  // Disabled by default
	m_nSPIMode = m_Properties.GetNumber ("SPIMode", SPI_DEF_MODE);
	m_nSPIClockKHz = m_Properties.GetNumber ("SPIClockKHz", SPI_DEF_CLOCK);

	m_bST7789Enabled = m_Properties.GetNumber ("ST7789Enabled", 0) != 0;
	m_nST7789Data = m_Properties.GetNumber ("ST7789Data", 0);
	m_nST7789Select = m_Properties.GetNumber ("ST7789Select", 0);
	m_nST7789Reset = m_Properties.GetNumber ("ST7789Reset", 0);  // optional
	m_nST7789Backlight = m_Properties.GetNumber ("ST7789Backlight", 0);  // optional
	m_nST7789Width = m_Properties.GetNumber ("ST7789Width", 240);
	m_nST7789Height = m_Properties.GetNumber ("ST7789Height", 240);
	m_nST7789Rotation = m_Properties.GetNumber ("ST7789Rotation", 0);
	m_bST7789SmallFont = m_Properties.GetNumber ("ST7789SmallFont", 0) != 0;

	m_nLCDColumns = m_Properties.GetNumber ("LCDColumns", 16);
	m_nLCDRows = m_Properties.GetNumber ("LCDRows", 2);

	m_nButtonPinPreview = m_Properties.GetNumber ("ButtonPinPreview", 0);
	m_nButtonPinLeft = m_Properties.GetNumber ("ButtonPinLeft", 0);
	m_nButtonPinRight = m_Properties.GetNumber ("ButtonPinRight", 0);
	m_nButtonPinData = m_Properties.GetNumber ("ButtonPinData", 0);
	m_nButtonPinToneSelect = m_Properties.GetNumber ("ButtonPinToneSelect", 0);
	m_nButtonPinPatchPerform = m_Properties.GetNumber ("ButtonPinPatchPerform", 0);
	m_nButtonPinEdit = m_Properties.GetNumber ("ButtonPinEdit", 0);
	m_nButtonPinSystem = m_Properties.GetNumber ("ButtonPinSystem", 0);
	m_nButtonPinRhythm = m_Properties.GetNumber ("ButtonPinRhythm", 0);
	m_nButtonPinUtility = m_Properties.GetNumber ("ButtonPinUtility", 0);
	m_nButtonPinMute = m_Properties.GetNumber ("ButtonPinMute", 0);
	m_nButtonPinMonitor = m_Properties.GetNumber ("ButtonPinMonitor", 0);
	m_nButtonPinCompare = m_Properties.GetNumber ("ButtonPinCompare", 0);
	m_nButtonPinEnter = m_Properties.GetNumber ("ButtonPinEnter", 11);

	m_ButtonActionPreview = m_Properties.GetString ("ButtonActionPreview", "");
	m_ButtonActionLeft = m_Properties.GetString ("ButtonActionLeft", "");
	m_ButtonActionRight = m_Properties.GetString ("ButtonActionRight", "");
	m_ButtonActionData = m_Properties.GetString ("ButtonActionData", "");
	m_ButtonActionToneSelect = m_Properties.GetString ("ButtonActionToneSelect", "");
	m_ButtonActionPatchPerform = m_Properties.GetString ("ButtonActionPatchPerform", "");
	m_ButtonActionEdit = m_Properties.GetString ("ButtonActionEdit", "");
	m_ButtonActionSystem = m_Properties.GetString ("ButtonActionSystem", "");
	m_ButtonActionRhythm = m_Properties.GetString ("ButtonActionRhythm", "");
	m_ButtonActionUtility = m_Properties.GetString ("ButtonActionUtility", "");
	m_ButtonActionMute = m_Properties.GetString ("ButtonActionMute", "");
	m_ButtonActionMonitor = m_Properties.GetString ("ButtonActionMonitor", "");
	m_ButtonActionCompare = m_Properties.GetString ("ButtonActionCompare", "");
	m_ButtonActionEnter = m_Properties.GetString("ButtonActionEnter", "click");

	m_nDoubleClickTimeout = m_Properties.GetNumber ("DoubleClickTimeout", 400);
	m_nLongPressTimeout = m_Properties.GetNumber ("LongPressTimeout", 600);

	m_bEncoderEnabled = m_Properties.GetNumber ("EncoderEnabled", 0) != 0;
	m_nEncoderPinClock = m_Properties.GetNumber ("EncoderPinClock", 10);
	m_nEncoderPinData = m_Properties.GetNumber ("EncoderPinData", 9);

	m_bProfileEnabled = m_Properties.GetNumber ("ProfileEnabled", 0) != 0;
}

const char *CConfig::GetSoundDevice (void) const
{
	return m_SoundDevice.c_str ();
}

unsigned CConfig::GetChunkSize (void) const
{
	return m_nChunkSize;
}

unsigned CConfig::GetDACI2CAddress (void) const
{
	return m_nDACI2CAddress;
}

bool CConfig::GetChannelsSwapped (void) const
{
	return m_bChannelsSwapped;
}

unsigned CConfig::GetMIDIBaudRate (void) const
{
	return m_nMIDIBaudRate;
}


bool CConfig::GetLCDEnabled (void) const
{
	return m_bLCDEnabled;
}

unsigned CConfig::GetLCDPinEnable (void) const
{
	return m_nLCDPinEnable;
}

unsigned CConfig::GetLCDPinRegisterSelect (void) const
{
	return m_nLCDPinRegisterSelect;
}

unsigned CConfig::GetLCDPinReadWrite (void) const
{
	return m_nLCDPinReadWrite;
}

unsigned CConfig::GetLCDPinData4 (void) const
{
	return m_nLCDPinData4;
}

unsigned CConfig::GetLCDPinData5 (void) const
{
	return m_nLCDPinData5;
}

unsigned CConfig::GetLCDPinData6 (void) const
{
	return m_nLCDPinData6;
}

unsigned CConfig::GetLCDPinData7 (void) const
{
	return m_nLCDPinData7;
}

unsigned CConfig::GetLCDI2CAddress (void) const
{
	return m_nLCDI2CAddress;
}

unsigned CConfig::GetSSD1306LCDI2CAddress (void) const
{
	return m_nSSD1306LCDI2CAddress;
}

unsigned CConfig::GetSSD1306LCDWidth (void) const
{
	return m_nSSD1306LCDWidth;
}

unsigned CConfig::GetSSD1306LCDHeight (void) const
{
	return m_nSSD1306LCDHeight;
}

bool CConfig::GetSSD1306LCDRotate (void) const
{
	return m_bSSD1306LCDRotate;
}

bool CConfig::GetSSD1306LCDMirror (void) const
{
	return m_bSSD1306LCDMirror;
}

unsigned CConfig::GetSPIBus (void) const
{
	return m_nSPIBus;
}

unsigned CConfig::GetSPIMode (void) const
{
	return m_nSPIMode;
}

unsigned CConfig::GetSPIClockKHz (void) const
{
	return m_nSPIClockKHz;
}

bool CConfig::GetST7789Enabled (void) const
{
	return m_bST7789Enabled;
}

unsigned CConfig::GetST7789Data (void) const
{
	return m_nST7789Data;
}

unsigned CConfig::GetST7789Select (void) const
{
	return m_nST7789Select;
}

unsigned CConfig::GetST7789Reset (void) const
{
	return m_nST7789Reset;
}

unsigned CConfig::GetST7789Backlight (void) const
{
	return m_nST7789Backlight;
}

unsigned CConfig::GetST7789Width (void) const
{
	return m_nST7789Width;
}

unsigned CConfig::GetST7789Height (void) const
{
	return m_nST7789Height;
}

unsigned CConfig::GetST7789Rotation (void) const
{
	return m_nST7789Rotation;
}

bool CConfig::GetST7789SmallFont (void) const
{
	return m_bST7789SmallFont;
}
unsigned CConfig::GetLCDColumns (void) const
{
	return m_nLCDColumns;
}

unsigned CConfig::GetLCDRows (void) const
{
	return m_nLCDRows;
}

unsigned CConfig::GetButtonPinPreview (void) const
{
	return m_nButtonPinPreview;
}

unsigned CConfig::GetButtonPinLeft (void) const
{
	return m_nButtonPinLeft;
}

unsigned CConfig::GetButtonPinRight (void) const
{
	return m_nButtonPinRight;
}

unsigned CConfig::GetButtonPinData (void) const
{
	return m_nButtonPinData;
}

unsigned CConfig::GetButtonPinToneSelect (void) const
{
	return m_nButtonPinToneSelect;
}

unsigned CConfig::GetButtonPinPatchPerform (void) const
{
	return m_nButtonPinPatchPerform;
}

unsigned CConfig::GetButtonPinEdit (void) const
{
	return m_nButtonPinEdit;
}

unsigned CConfig::GetButtonPinSystem (void) const
{
	return m_nButtonPinSystem;
}

unsigned CConfig::GetButtonPinRhythm (void) const
{
	return m_nButtonPinRhythm;
}

unsigned CConfig::GetButtonPinUtility (void) const
{
	return m_nButtonPinUtility;
}

unsigned CConfig::GetButtonPinMute (void) const
{
	return m_nButtonPinMute;
}

unsigned CConfig::GetButtonPinMonitor (void) const
{
	return m_nButtonPinMonitor;
}

unsigned CConfig::GetButtonPinCompare (void) const
{
	return m_nButtonPinCompare;
}

unsigned CConfig::GetButtonPinEnter (void) const
{
	return m_nButtonPinEnter;
}

const char *CConfig::GetButtonActionPreview (void) const
{
	return m_ButtonActionPreview.c_str();
}

const char *CConfig::GetButtonActionLeft (void) const
{
	return m_ButtonActionLeft.c_str();
}

const char *CConfig::GetButtonActionRight (void) const
{
	return m_ButtonActionRight.c_str();
}

const char *CConfig::GetButtonActionData (void) const
{
	return m_ButtonActionData.c_str();
}

const char *CConfig::GetButtonActionToneSelect (void) const
{
	return m_ButtonActionToneSelect.c_str();
}

const char *CConfig::GetButtonActionPatchPerform (void) const
{
	return m_ButtonActionPatchPerform.c_str();
}

const char *CConfig::GetButtonActionEdit (void) const
{
	return m_ButtonActionEdit.c_str();
}

const char *CConfig::GetButtonActionSystem (void) const
{
	return m_ButtonActionSystem.c_str();
}

const char *CConfig::GetButtonActionRhythm (void) const
{
	return m_ButtonActionRhythm.c_str();
}

const char *CConfig::GetButtonActionUtility (void) const
{
	return m_ButtonActionUtility.c_str();
}

const char *CConfig::GetButtonActionMute (void) const
{
	return m_ButtonActionMute.c_str();
}

const char *CConfig::GetButtonActionMonitor (void) const
{
	return m_ButtonActionMonitor.c_str();
}

const char *CConfig::GetButtonActionCompare (void) const
{
	return m_ButtonActionCompare.c_str();
}

const char *CConfig::GetButtonActionEnter (void) const
{
	return m_ButtonActionEnter.c_str();
}

bool CConfig::GetEncoderEnabled (void) const
{
	return m_bEncoderEnabled;
}

unsigned CConfig::GetEncoderPinClock (void) const
{
	return m_nEncoderPinClock;
}

unsigned CConfig::GetEncoderPinData (void) const
{
	return m_nEncoderPinData;
}

unsigned CConfig::GetDoubleClickTimeout (void) const
{
	return m_nDoubleClickTimeout;
}

unsigned CConfig::GetLongPressTimeout (void) const
{
	return m_nLongPressTimeout;
}

bool CConfig::GetProfileEnabled (void) const
{
	return m_bProfileEnabled;
}
unsigned CConfig::GetMIDIButtonCh (void) const
{
	return m_nMIDIButtonCh;
}

unsigned CConfig::GetMIDIButtonNotes (void) const
{
	return m_nMIDIButtonNotes;
}

unsigned CConfig::GetMIDIButtonPrev (void) const
{
	return m_nMIDIButtonPrev;
}

unsigned CConfig::GetMIDIButtonNext (void) const
{
	return m_nMIDIButtonNext;
}

unsigned CConfig::GetMIDIButtonBack (void) const
{
	return m_nMIDIButtonBack;
}

unsigned CConfig::GetMIDIButtonSelect (void) const
{
	return m_nMIDIButtonSelect;
}

unsigned CConfig::GetMIDIButtonHome (void) const
{
	return m_nMIDIButtonHome;
}

unsigned CConfig::GetMIDIButtonPgmUp (void) const
{
	return m_nMIDIButtonPgmUp;
}

unsigned CConfig::GetMIDIButtonPgmDown (void) const
{
	return m_nMIDIButtonPgmDown;
}

unsigned CConfig::GetMIDIButtonBankUp (void) const
{
	return m_nMIDIButtonBankUp;
}

unsigned CConfig::GetMIDIButtonBankDown (void) const
{
	return m_nMIDIButtonBankDown;
}

unsigned CConfig::GetMIDIButtonTGUp (void) const
{
	return m_nMIDIButtonTGUp;
}

unsigned CConfig::GetMIDIButtonTGDown (void) const
{
	return m_nMIDIButtonTGDown;
}