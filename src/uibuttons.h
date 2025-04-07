//
// uibuttons.h
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
#ifndef _uibuttons_h
#define _uibuttons_h

#include <circle/gpiopin.h>
#include <circle/types.h>
#include "config.h"
#include "midipin.h"


#define BUTTONS_UPDATE_NUM_TICKS 100
#define DEBOUNCE_TIME 20
#define MAX_GPIO_BUTTONS 14  // 14 UI buttons
#define MAX_BUTTONS (MAX_GPIO_BUTTONS)

class CUIButtons;

class CUIButton
{
public:
	enum BtnTrigger
	{
		BtnTriggerNone = 0,
		BtnTriggerClick = 1,
		BtnTriggerDoubleClick = 2,
		BtnTriggerLongPress = 3
	};

	enum BtnEvent
	{
		BtnEventNone = 0,
		BtnEventPreview = 1,
		BtnEventLeft = 2,
		BtnEventRight = 3,
		BtnEventData = 4,
		BtnEventToneSelect = 5,
		BtnEventPatchPerform = 6,
		BtnEventEdit = 7,
		BtnEventSystem = 8,
		BtnEventRhythm = 9,
		BtnEventUtility = 10,
		BtnEventMute = 11,
		BtnEventMonitor = 12,
		BtnEventCompare = 13,
		BtnEventEnter = 14,
		BtnEventUnknown = 15
	};
	
	CUIButton (void);
	~CUIButton (void);
	
	void reset (void);
	boolean Initialize (unsigned pinNumber, unsigned doubleClickTimeout, unsigned longPressTimeout);

	void setClickEvent(BtnEvent clickEvent);
	void setDoubleClickEvent(BtnEvent doubleClickEvent);
	void setLongPressEvent(BtnEvent longPressEvent);

	unsigned getPinNumber(void);
	
	BtnTrigger ReadTrigger (void);
	BtnEvent Read (void);
	void Write (unsigned nValue); // MIDI buttons only!

	static BtnTrigger triggerTypeFromString(const char* triggerString);
	
private:
	CMIDIPin *m_midipin;
	// Pin number
	unsigned m_pinNumber;
	// GPIO pin
	CGPIOPin *m_pin;
	// The value of the pin at the end of the last loop
	unsigned m_lastValue;
	// Set to 0 on press, increment each read, use to trigger events
	uint16_t m_timer;
	// Debounce timer
	uint16_t m_debounceTimer;
	// Number of clicks recorded since last timer reset
	uint8_t m_numClicks;
	// Event to fire on click
	BtnEvent m_clickEvent;
	// Event to fire on double click
	BtnEvent m_doubleClickEvent;
	// Event to fire on long press
	BtnEvent m_longPressEvent;
	
	// Timeout for double click in tenths of a millisecond
	unsigned m_doubleClickTimeout;
	// Timeout for long press in tenths of a millisecond
	unsigned m_longPressTimeout;
};

class CUIButtons
{
public:
	typedef void BtnEventHandler (CUIButton::BtnEvent Event, void *param);
	

public:
	CUIButtons (
			CConfig *pConfig,
			unsigned previewPin, const char *previewAction,
			unsigned leftPin, const char *leftAction,
			unsigned rightPin, const char *rightAction,
			unsigned dataPin, const char *dataAction,
			unsigned toneSelectPin, const char *toneSelectAction,
			unsigned patchPerformPin, const char *patchPerformAction,
			unsigned editPin, const char *editAction,
			unsigned systemPin, const char *systemAction,
			unsigned rhythmPin, const char *rhythmAction,
			unsigned utilityPin, const char *utilityAction,
			unsigned mutePin, const char *muteAction,
			unsigned monitorPin, const char *monitorAction,
			unsigned comparePin, const char *compareAction,
			unsigned enterPin, const char *enterAction,
			unsigned doubleClickTimeout, unsigned longPressTimeout
	);
	~CUIButtons (void);
	
	boolean Initialize (void);
	
	void RegisterEventHandler (BtnEventHandler *handler, void *param = 0);
	
	void Update (void);

	void ResetButton (unsigned pinNumber);
	
private:

	CConfig *m_pConfig;
	// Array of normal GPIO buttons and "MIDI buttons"
	CUIButton m_buttons[MAX_BUTTONS];
	
	
	// Timeout for double click in tenths of a millisecond
	unsigned m_doubleClickTimeout;
	// Timeout for long press in tenths of a millisecond
	unsigned m_longPressTimeout;
	
	// Configuration for buttons
	unsigned m_previewPin;
	CUIButton::BtnTrigger m_previewAction;
	unsigned m_leftPin;
	CUIButton::BtnTrigger m_leftAction;
	unsigned m_rightPin;
	CUIButton::BtnTrigger m_rightAction;
	unsigned m_dataPin;
	CUIButton::BtnTrigger m_dataAction;
	unsigned m_toneSelectPin;
	CUIButton::BtnTrigger m_toneSelectAction;
	unsigned m_patchPerformPin;
	CUIButton::BtnTrigger m_patchPerformAction;
	unsigned m_editPin;
	CUIButton::BtnTrigger m_editAction;
	unsigned m_systemPin;
	CUIButton::BtnTrigger m_systemAction;
	unsigned m_rhythmPin;
	CUIButton::BtnTrigger m_rhythmAction;
	unsigned m_utilityPin;
	CUIButton::BtnTrigger m_utilityAction;
	unsigned m_mutePin;
	CUIButton::BtnTrigger m_muteAction;
	unsigned m_monitorPin;
	CUIButton::BtnTrigger m_monitorAction;
	unsigned m_comparePin;
	CUIButton::BtnTrigger m_compareAction;
	unsigned m_enterPin;
	CUIButton::BtnTrigger m_enterAction;

	unsigned m_notesMidi;
	unsigned m_prevMidi;
	unsigned m_nextMidi;
	unsigned m_backMidi;
	unsigned m_selectMidi;
	unsigned m_homeMidi;
	
	unsigned m_pgmUpMidi;
	unsigned m_pgmDownMidi;
	unsigned m_BankUpMidi;
	unsigned m_BankDownMidi;
	unsigned m_TGUpMidi;
	unsigned m_TGDownMidi;

	BtnEventHandler *m_eventHandler;
	void *m_eventParam;

	unsigned m_lastTick;

	void bindButton(unsigned pinNumber, CUIButton::BtnTrigger trigger, CUIButton::BtnEvent event);
};

#endif
