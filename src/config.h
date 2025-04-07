//
// config.h
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
#ifndef _config_h
#define _config_h

#include <fatfs/ff.h>
#include <Properties/propertiesfatfsfile.h>
#include <circle/sysconfig.h>
#include <string>

#define SPI_INACTIVE	255
#define SPI_DEF_CLOCK	15000	// kHz
#define SPI_DEF_MODE	0		// Default mode (0,1,2,3)

class CConfig		// Configuration for MiniJV880
{
public:

	static const unsigned MaxChunkSize = 4096;

#if RASPPI <= 3
	static const unsigned MaxUSBMIDIDevices = 2;
#else
	static const unsigned MaxUSBMIDIDevices = 4;
#endif

// TODO - Leave this for uimenu.cpp for now, but it will need to be dynamic at some point...
	static const unsigned LCDColumns = 16;		// HD44780 LCD
	static const unsigned LCDRows = 2;

public:
	CConfig (FATFS *pFileSystem);
	~CConfig (void);

	void Load (void);
	
	// Sound device
	const char *GetSoundDevice (void) const;
	unsigned GetChunkSize (void) const;
	unsigned GetDACI2CAddress (void) const;		// 0 for auto probing
	bool GetChannelsSwapped (void) const;

	// MIDI
	unsigned GetMIDIBaudRate (void) const;

	// HD44780 LCD
	// GPIO pin numbers are chip numbers, not header positions
	bool GetLCDEnabled (void) const;
	unsigned GetLCDPinEnable (void) const;
	unsigned GetLCDPinRegisterSelect (void) const;
	unsigned GetLCDPinReadWrite (void) const;	// set to 0 if not connected
	unsigned GetLCDPinData4 (void) const;
	unsigned GetLCDPinData5 (void) const;
	unsigned GetLCDPinData6 (void) const;
	unsigned GetLCDPinData7 (void) const;
	unsigned GetLCDI2CAddress (void) const;
	
	// SSD1306 LCD
	unsigned GetSSD1306LCDI2CAddress (void) const;
	unsigned GetSSD1306LCDWidth (void) const;
	unsigned GetSSD1306LCDHeight (void) const;
	bool     GetSSD1306LCDRotate (void) const;
	bool     GetSSD1306LCDMirror (void) const;

	// SPI support
	unsigned GetSPIBus (void) const;
	unsigned GetSPIMode (void) const;
	unsigned GetSPIClockKHz (void) const;

	// ST7789 LCD
	bool     GetST7789Enabled (void) const;
	unsigned GetST7789Data (void) const;
	unsigned GetST7789Select (void) const;
	unsigned GetST7789Reset (void) const;
	unsigned GetST7789Backlight (void) const;
	unsigned GetST7789Width (void) const;
	unsigned GetST7789Height (void) const;
	unsigned GetST7789Rotation (void) const;
	bool     GetST7789SmallFont (void) const;

	unsigned GetLCDColumns (void) const;
	unsigned GetLCDRows (void) const;

	// GPIO Button Navigation
	// GPIO pin numbers are chip numbers, not header positions
	unsigned GetButtonPinPreview (void) const;
	unsigned GetButtonPinLeft (void) const;
	unsigned GetButtonPinRight (void) const;
	unsigned GetButtonPinData (void) const;
	unsigned GetButtonPinToneSelect (void) const;
	unsigned GetButtonPinPatchPerform (void) const;
	unsigned GetButtonPinEdit (void) const;
	unsigned GetButtonPinSystem (void) const;
	unsigned GetButtonPinRhythm (void) const;
	unsigned GetButtonPinUtility (void) const;
	unsigned GetButtonPinMute (void) const;
	unsigned GetButtonPinMonitor (void) const;
	unsigned GetButtonPinCompare (void) const;
	unsigned GetButtonPinEnter (void) const;

	// Action type for buttons: "click", "doubleclick", "longpress", ""
	const char *GetButtonActionPreview (void) const;
	const char *GetButtonActionLeft (void) const;
	const char *GetButtonActionRight (void) const;
	const char *GetButtonActionData (void) const;
	const char *GetButtonActionToneSelect (void) const;
	const char *GetButtonActionPatchPerform (void) const;
	const char *GetButtonActionEdit (void) const;
	const char *GetButtonActionSystem (void) const;
	const char *GetButtonActionRhythm (void) const;
	const char *GetButtonActionUtility (void) const;
	const char *GetButtonActionMute (void) const;
	const char *GetButtonActionMonitor (void) const;
	const char *GetButtonActionCompare (void) const;
	const char *GetButtonActionEnter (void) const;
	
	// Timeouts for button events in milliseconds
	unsigned GetDoubleClickTimeout (void) const;
	unsigned GetLongPressTimeout (void) const;

	// MIDI Button Navigation
	unsigned GetMIDIButtonCh   (void) const;
	unsigned GetMIDIButtonNotes (void) const;
	unsigned GetMIDIButtonPrev (void) const;
	unsigned GetMIDIButtonNext (void) const;
	unsigned GetMIDIButtonBack (void) const;
	unsigned GetMIDIButtonSelect (void) const;
	unsigned GetMIDIButtonHome (void) const;

	// MIDI Button Program and TG Selection
	unsigned GetMIDIButtonPgmUp (void) const;
	unsigned GetMIDIButtonPgmDown (void) const;
	unsigned GetMIDIButtonBankUp (void) const;
	unsigned GetMIDIButtonBankDown (void) const;
	unsigned GetMIDIButtonTGUp (void) const;
	unsigned GetMIDIButtonTGDown (void) const;

	// KY-040 Rotary Encoder
	// GPIO pin numbers are chip numbers, not header positions
	bool GetEncoderEnabled (void) const;
	unsigned GetEncoderPinClock (void) const;
	unsigned GetEncoderPinData (void) const;

	bool GetProfileEnabled (void) const;

private:
	CPropertiesFatFsFile m_Properties;
	
	std::string m_SoundDevice;
	unsigned m_nChunkSize;
	unsigned m_nDACI2CAddress;
	bool m_bChannelsSwapped;
	unsigned m_EngineType;

	unsigned m_nMIDIBaudRate;


	bool m_bLCDEnabled;
	unsigned m_nLCDPinEnable;
	unsigned m_nLCDPinRegisterSelect;
	unsigned m_nLCDPinReadWrite;
	unsigned m_nLCDPinData4;
	unsigned m_nLCDPinData5;
	unsigned m_nLCDPinData6;
	unsigned m_nLCDPinData7;
	unsigned m_nLCDI2CAddress;
	
	unsigned m_nSSD1306LCDI2CAddress;
	unsigned m_nSSD1306LCDWidth;
	unsigned m_nSSD1306LCDHeight;
	bool     m_bSSD1306LCDRotate;
	bool     m_bSSD1306LCDMirror;

	unsigned m_nSPIBus;
	unsigned m_nSPIMode;
	unsigned m_nSPIClockKHz;

	bool     m_bST7789Enabled;
	unsigned m_nST7789Data;
	unsigned m_nST7789Select;
	unsigned m_nST7789Reset;
	unsigned m_nST7789Backlight;
	unsigned m_nST7789Width;
	unsigned m_nST7789Height;
	unsigned m_nST7789Rotation;
	unsigned m_bST7789SmallFont;

	unsigned m_nLCDColumns;
	unsigned m_nLCDRows;

	unsigned m_nButtonPinPreview;
	unsigned m_nButtonPinLeft;
	unsigned m_nButtonPinRight;
	unsigned m_nButtonPinData;
	unsigned m_nButtonPinToneSelect;
	unsigned m_nButtonPinPatchPerform;
	unsigned m_nButtonPinEdit;
	unsigned m_nButtonPinSystem;
	unsigned m_nButtonPinRhythm;
	unsigned m_nButtonPinUtility;
	unsigned m_nButtonPinMute;
	unsigned m_nButtonPinMonitor;
	unsigned m_nButtonPinCompare;
	unsigned m_nButtonPinEnter;

	std::string m_ButtonActionPreview;
	std::string m_ButtonActionLeft;
	std::string m_ButtonActionRight;
	std::string m_ButtonActionData;
	std::string m_ButtonActionToneSelect;
	std::string m_ButtonActionPatchPerform;
	std::string m_ButtonActionEdit;
	std::string m_ButtonActionSystem;
	std::string m_ButtonActionRhythm;
	std::string m_ButtonActionUtility;
	std::string m_ButtonActionMute;
	std::string m_ButtonActionMonitor;
	std::string m_ButtonActionCompare;
	std::string m_ButtonActionEnter;
	
	unsigned m_nDoubleClickTimeout;
	unsigned m_nLongPressTimeout;
	
	unsigned m_nMIDIButtonCh;
	unsigned m_nMIDIButtonNotes;
	unsigned m_nMIDIButtonPrev;
	unsigned m_nMIDIButtonNext;
	unsigned m_nMIDIButtonBack;
	unsigned m_nMIDIButtonSelect;
	unsigned m_nMIDIButtonHome;
	unsigned m_nMIDIButtonPgmUp;
	unsigned m_nMIDIButtonPgmDown;
	unsigned m_nMIDIButtonBankUp;
	unsigned m_nMIDIButtonBankDown;
	unsigned m_nMIDIButtonTGUp;
	unsigned m_nMIDIButtonTGDown;

	bool m_bEncoderEnabled;
	unsigned m_nEncoderPinClock;
	unsigned m_nEncoderPinData;

	bool m_bProfileEnabled;
};

#endif
