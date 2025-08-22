//
// minidexed.h
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
#ifndef _minijv880_h
#define _minijv880_h

#ifndef ARM_ALLOW_MULTI_CORE
#define ARM_ALLOW_MULTI_CORE
#endif

#include "config.h"
#include "userinterface.h"
#include "emulator/mcu.h"
#include "sync.h"
#include <circle/gpiomanager.h>
#include <circle/i2cmaster.h>
#include <circle/interrupt.h>
#include <circle/multicore.h>
#include <circle/screen.h>
#include <circle/sound/soundbasedevice.h>
#include <circle/spimaster.h>
#include <circle/spinlock.h>
#include <circle/types.h>
// #include <circle/usb/usbkompletekontrol.h>
#include <circle/usb/usbmidi.h>
#include <fatfs/ff.h>
#include <stdint.h>
#include <atomic>



class CMiniJV880 : public CMultiCoreSupport {
public:
  CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
             CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster, CSPIMaster *pSPIMaster,
             FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered);

  bool Initialize(void);
  void Process(bool bPlugAndPlayUpdated);

  virtual void Run(unsigned nCore) override;

  static void USBMIDIMessageHandler(unsigned nCable, u8 *pPacket,
                                    unsigned nLength);
  static void DeviceRemovedHandler(CDevice *pDevice, void *pContext);

  MCU mcu;



private:
  CConfig *m_pConfig;
  FATFS *m_pFileSystem;

  CUSBMIDIDevice *volatile m_pMIDIDevice = 0;
  // CUSBKompleteKontrolDevice *volatile m_KompleteKontrol = 0;
  int lastEncoderPos = 0;

  CSoundBaseDevice *m_pSoundDevice;
  CScreenDevice *screenUnbuffered;
  bool m_bChannelsSwapped;
  unsigned m_nQueueSizeFrames;

  CUserInterface m_UI;

  unsigned m_lastTick;
  unsigned m_lastTick1;

  static CMiniJV880 *s_pThis;

    static Barrier             s_Barrier;       // барьер на 2 участника
    static std::atomic<uint64_t> s_CycleTarget; // цель по MCU-циклам для PCM
};

#endif
