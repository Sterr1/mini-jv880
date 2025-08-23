//
// minidexed.cpp
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
#include "minijv880.h"
#include <assert.h>
#include <circle/devicenameservice.h>
#include <circle/gpiopin.h>
#include <circle/logger.h>
#include <circle/memory.h>
#include <circle/sound/hdmisoundbasedevice.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <circle/usb/usbmidihost.h>
#include <stdio.h>
#include <string.h>
#include <cstring>   // memcpy / memmove
#include <algorithm>


// конфиг: сколько циклов MCU считаем за один квант (подбирай)
static constexpr uint64_t CYCLES_PER_INSTR = 12ull;   // твоё допущение
static constexpr unsigned   INSTRS_PER_QUANTUM = 256u;
static constexpr uint64_t   CYCLES_PER_QUANTUM = CYCLES_PER_INSTR * INSTRS_PER_QUANTUM;

CMiniJV880 *CMiniJV880::s_pThis = 0;

LOGMODULE("minijv880");

CMiniJV880::CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
                       CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster, CSPIMaster *pSPIMaster,
                       FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered)
    : CMultiCoreSupport(CMemorySystem::Get()), m_pConfig(pConfig),
      m_pFileSystem(pFileSystem), 
      m_Serial(pInterrupt, TRUE),
      m_pSoundDevice(0),
      screenUnbuffered(mScreenUnbuffered),
      m_bChannelsSwapped(pConfig->GetChannelsSwapped()),
      m_UI(this, pGPIOManager, pI2CMaster, pSPIMaster, pConfig),
      m_lastTick(0),
      m_lastTick1(0) {
  
      assert(m_pConfig);

      s_pThis = this;

  // select the sound device
  const char *pDeviceName = pConfig->GetSoundDevice();
  if (strcmp(pDeviceName, "i2s") == 0) {
    LOGNOTE("I2S mode");
    m_pSoundDevice = new CI2SSoundBaseDevice(
        pInterrupt, 32000, pConfig->GetChunkSize(), false, pI2CMaster,
        pConfig->GetDACI2CAddress(), CI2SSoundBaseDevice::DeviceModeTXOnly,
        2); // 2 channels - L+R
  } else if (strcmp(pDeviceName, "hdmi") == 0) {
#if RASPPI == 5
    LOGNOTE("HDMI mode NOT supported on RPI 5.");
#else
    LOGNOTE("HDMI mode");

    m_pSoundDevice =
        new CHDMISoundBaseDevice(pInterrupt, 32000, pConfig->GetChunkSize());

    // The channels are swapped by default in the HDMI sound driver.
    // TODO: Remove this line, when this has been fixed in the driver.
    m_bChannelsSwapped = !m_bChannelsSwapped;
#endif
  } else {
    LOGNOTE("PWM mode");

    m_pSoundDevice =
        new CPWMSoundBaseDevice(pInterrupt, 32000, pConfig->GetChunkSize());
  }
};

bool CMiniJV880::Initialize(void) {
  assert(m_pConfig);
  assert(m_pSoundDevice);

  if (!m_UI.Initialize ())
	{
    LOGERR("Failed to initialize UI");
		return false;
	}


	assert (m_pConfig);
	if (!m_Serial.Initialize(m_pConfig->GetMIDIBaudRate ())) 
    {
        LOGERR("Failed to initialize Serial MIDI");
        return false;
    }
	unsigned ser_options = m_Serial.GetOptions();
	// Ensure CR->CRLF translation is disabled for MIDI links
	ser_options &= ~(SERIAL_OPTION_ONLCR);
	m_Serial.SetOptions(ser_options);
  LOGNOTE("Serial MIDI Initialized");
  
  LOGNOTE("Loading emu files");
  uint8_t *rom1 = (uint8_t *)malloc(ROM1_SIZE);
  uint8_t *rom2 = (uint8_t *)malloc(ROM2_SIZE);
  uint8_t *nvram = (uint8_t *)malloc(NVRAM_SIZE);
  uint8_t *pcm1 = (uint8_t *)malloc(0x200000);
  uint8_t *pcm2 = (uint8_t *)malloc(0x200000);

  FIL f;
  unsigned int nBytesRead = 0;

  if (f_open(&f, "jv880_rom1.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, rom1, ROM1_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_rom2.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom2.bin");
    return false;
  }
  f_read(&f, rom2, ROM2_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_nvram.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_nvram.bin");
    return false;
  }
  f_read(&f, nvram, NVRAM_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_waverom1.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_waverom1.bin");
    return false;
  }
  f_read(&f, pcm1, 0x200000, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_waverom2.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_waverom2.bin");
    return false;
  }
  f_read(&f, pcm2, 0x200000, &nBytesRead);
  f_close(&f);
  LOGNOTE("Emu files loaded");

  int ret = mcu.startSC55(rom1, rom2, pcm1, pcm2, nvram);
  LOGNOTE("startSC55 returned: %d", ret);
  free(rom1);
  free(rom2);
  free(nvram);
  free(pcm1);
  free(pcm2);

  // setup and start the sound device
  int Channels = 2; // 16-bit Stereo
  // Need 2 x ChunkSize / Channel queue frames as the audio driver uses
  // two DMA channels each of ChunkSize and one single single frame
  // contains a sample for each of all the channels.
  //
  // See discussion here: https://github.com/rsta2/circle/discussions/453
  if (!m_pSoundDevice->AllocateQueueFrames(2 * m_pConfig->GetChunkSize() /
                                           Channels)) {
    LOGERR("Cannot allocate sound queue");

    return false;
  }

  m_pSoundDevice->SetWriteFormat(SoundFormatSigned16, Channels);

  m_nQueueSizeFrames = m_pSoundDevice->GetQueueSizeFrames();

  m_pSoundDevice->Start();

  CMultiCoreSupport::Initialize();
  LOGNOTE("initialised");

  return true;
}

void CMiniJV880::Process(bool bPlugAndPlayUpdated) {

  m_UI.Process ();

  if (!bPlugAndPlayUpdated)
    return;

  if (m_pMIDIDevice == 0) {
    m_pMIDIDevice =
        (CUSBMIDIDevice *)CDeviceNameService::Get()->GetDevice("umidi1", FALSE);
    if (m_pMIDIDevice != 0) {
      m_pMIDIDevice->RegisterPacketHandler(USBMIDIMessageHandler);
      m_pMIDIDevice->RegisterRemovedHandler(DeviceRemovedHandler, this);
    }
  }
}

void CMiniJV880::USBMIDIMessageHandler(unsigned nCable, u8 *pPacket,
                                       unsigned nLength) {
  // LOGERR("CMiniJV880::USBMIDIMessageHandler");
  CMiniJV880 *pThis = static_cast<CMiniJV880 *>(s_pThis);
  if (!pPacket || nLength == 0) return;
  ParseMIDIData(pThis, pPacket, nLength);
  //pThis->mcu.postMidiSC55(pPacket, nLength);
}

void CMiniJV880::ParseMIDIData(CMiniJV880* pThis, const u8* pData, unsigned nLength)
{

    for (unsigned i = 0; i < nLength; i++)
    {

        if ((pData[i] & 0xF0) == 0xB0 && i + 2 < nLength) 
        {
            u8 ccNumber = pData[i + 1];
            u8 ccValue = pData[i + 2];
            
            switch (ccNumber)
            {
                case 0:   // Volume
                    LOGDBG("Bank Select: %d", ccValue);
                    break;
                    
                case 10:  // Pan
                    LOGDBG("Pan CC: %d", ccValue);
                    break;
                    
            }
        }
    }
    
    pThis->mcu.postMidiSC55(pData, nLength);
}

void CMiniJV880::DeviceRemovedHandler(CDevice *pDevice, void *pContext) {
  LOGERR("CMiniJV880::DeviceRemovedHandler");

  CMiniJV880 *pThis = static_cast<CMiniJV880 *>(pContext);
  assert(pThis != 0);

  if (pDevice == pThis->m_pMIDIDevice)
    pThis->m_pMIDIDevice = 0;
}

// double avg = 0;
// int cnt = 0;


void CMiniJV880::Run(unsigned nCore) {
  assert(1 <= nCore && nCore < CORES);
  int nSamples = 0;
  u8 buffer[64];

  if (nCore == 1) { // ------------------- 1st core ----- serial MIDI
    while (true) {
      int nRead = m_Serial.Read(buffer, sizeof(buffer));
      if (nRead > 0)
      {
          ParseMIDIData(this, buffer, nRead);
      }
      CTimer::SimpleMsDelay(1);
    }
  } else if (nCore == 2) { // ------------ 2nd core -----  MCU code
    // emulator
    while (true) {
      unsigned nFrames =
          m_nQueueSizeFrames - m_pSoundDevice->GetQueueFramesAvail();
      if (nFrames >= m_nQueueSizeFrames / 2) {
        // unsigned int startT = CTimer::GetClockTicks();

        nSamples = (int)nFrames * 2;
        // mcu.updateSC55(nSamples);

        mcu.sample_write_ptr = 0;
        while (mcu.sample_write_ptr < nSamples) {
          if (!mcu.mcu.ex_ignore)
            mcu.MCU_Interrupt_Handle();
          else
            mcu.mcu.ex_ignore = 0;

          if (!mcu.mcu.sleep)
            mcu.MCU_ReadInstruction();

          mcu.mcu.cycles += 10; // FIXME: assume 12 cycles per instruction

          mcu.TIMER_Clock(mcu.mcu.cycles);
          mcu.MCU_UpdateUART_RX();
          mcu.MCU_UpdateUART_TX();
          mcu.MCU_UpdateAnalog(mcu.mcu.cycles);

          // mcu.pcm.PCM_Update(mcu.mcu.cycles);
        }

        // unsigned int endT = CTimer::GetClockTicks();
        // avg = avg == 0 ? (endT - startT) : avg * 0.99 + (endT - startT) *
        // 0.01; if (cnt++ == 100) {
        //   LOGNOTE("%f\n", avg);
        //   cnt = 0;
        // }

        int len = nSamples * sizeof(int16_t);
        if (m_pSoundDevice->Write(mcu.sample_buffer, len) != len) {
          LOGERR("Sound data dropped");
        }
      }
    }
    // LOGNOTE("%d samples in %d time", nFrames, m_GetChunkTimer);
  } else if (nCore == 3) { // ----------------------  3rd core ----- PCM Update
    // pcm chip
    while (true) {
      // while (mcu.sample_write_ptr >= nSamples) {
      // }
      mcu.pcm.PCM_Update(mcu.mcu.cycles);
    }
  }
}


/*void CMiniJV880::Run(unsigned nCore)
{
    assert(1 <= nCore && nCore < CORES);

    if (nCore == 1)
    {
        // ядро 1 — оставляем как в оригинале
        return;
    }
    else if (nCore == 2)
    {
        // ядро 2 — эмуляция и генерация звука
        while (true)
        {
            unsigned nFrames = m_nQueueSizeFrames - m_pSoundDevice->GetQueueFramesAvail();
            if (nFrames >= m_nQueueSizeFrames / 2)
            {
                int nSamples = (int)nFrames * 2; // стерео
                mcu.sample_write_ptr = 0;

                // Генерируем звук в PCM буфер
                while (mcu.sample_write_ptr < nSamples)
                {
                    if (!mcu.mcu.ex_ignore)
                        mcu.MCU_Interrupt_Handle();
                    else
                        mcu.mcu.ex_ignore = 0;

                    if (!mcu.mcu.sleep)
                        mcu.MCU_ReadInstruction();

                    mcu.mcu.cycles += 12;
                    mcu.TIMER_Clock(mcu.mcu.cycles);
                    mcu.MCU_UpdateUART_RX();
                    mcu.MCU_UpdateUART_TX();
                    mcu.MCU_UpdateAnalog(mcu.mcu.cycles);
                    mcu.pcm.PCM_Update(mcu.mcu.cycles);
                }

                // Пишем в аудиоустройство пакетами
                m_pSoundDevice->Write(mcu.sample_buffer, mcu.sample_write_ptr * sizeof(int16_t));
            }
        }
    }
}*/

/*void CMiniJV880::Run(unsigned nCore)
{
    assert(1 <= nCore && nCore < CORES);

    if (nCore == 2) {
        // === ЯДРО 2: эмулятор MCU + управление выводом аудио ===
        std::vector<int16_t> tmpBuf; // временный буфер для Write()

        while (true) {
            // сколько кадров нам нужно заполнить для звукового устройства
            unsigned framesAvail = m_pSoundDevice->GetQueueFramesAvail();
            unsigned needFrames  = (framesAvail < m_nQueueSizeFrames) ? (m_nQueueSizeFrames - framesAvail) : 0;

            if (needFrames < m_nQueueSizeFrames / 2) {
                // очередь почти полная — уступим CPU (yield)
                asm volatile("yield" ::: "memory");
                continue;
            }

            // будем генерировать столько сэмплов, сколько требуется
            const int targetSamples = int(needFrames) * 2; // стерео -> samples (int16)
            // начинаем собирать новый блок: обнуляем указатель, чтобы PCM писал с начала
            mcu.sample_write_ptr = 0;

            while (mcu.sample_write_ptr < targetSamples) {
                // 1) эмулируем MCU до следующего квант-target
                uint64_t targetCycles = mcu.mcu.cycles + CYCLES_PER_QUANTUM;

                // выполняем инструкции MCU (никакого PCM_Update здесь)
                while (mcu.mcu.cycles < targetCycles) {
                    if (!mcu.mcu.ex_ignore)
                        mcu.MCU_Interrupt_Handle();
                    else
                        mcu.mcu.ex_ignore = 0;

                    if (!mcu.mcu.sleep)
                        mcu.MCU_ReadInstruction();

                    mcu.mcu.cycles += CYCLES_PER_INSTR;
                    mcu.TIMER_Clock(mcu.mcu.cycles);
                    mcu.MCU_UpdateUART_RX();
                    mcu.MCU_UpdateUART_TX();
                    mcu.MCU_UpdateAnalog(mcu.mcu.cycles);
                }

                // 2) опубликовать цель и запустить PCM: ПАРА BARRIER WAIT'ов
                s_CycleTarget.store(targetCycles, std::memory_order_release);

                // Первый Wait: совпадёт с PCM'ом, который тоже вызывает Wait перед своей работой.
                s_Barrier.Wait();

                // Второй Wait: CPU блокируется здесь, пока PCM не закончится и не вызовет второй Wait.
                s_Barrier.Wait();

                // после второго Wait PCM гарантированно окончил писать сэмплы в mcu.sample_buffer
                // и обновил mcu.sample_write_ptr — можно проверять/копировать
            }

            // Отправляем в аудиоустройство все готовые сэмплы
            int producedSamples = mcu.sample_write_ptr; // int16 samples (L,R interleaved)
            if (producedSamples > 0) {
                // безопасность: не писать больше, чем хотели
                int toSendSamples = std::min(producedSamples, targetSamples);
                int lenBytes = toSendSamples * sizeof(int16_t);

                // Временный буфер, чтобы Write() мог читать непрерывную память (если нужно)
                tmpBuf.resize(toSendSamples);
                std::memcpy(tmpBuf.data(), mcu.sample_buffer, lenBytes);

                if (m_pSoundDevice->Write(tmpBuf.data(), lenBytes) != lenBytes) {
                    LOGERR("Sound data dropped");
                }

                // если PCM написал больше, чем мы отправили, сдвигаем хвост в начало
                int tail = producedSamples - toSendSamples;
                if (tail > 0) {
                    std::memmove(mcu.sample_buffer,
                                 mcu.sample_buffer + toSendSamples,
                                 tail * sizeof(int16_t));
                }
                mcu.sample_write_ptr = tail;
            }
        }
    }
    else if (nCore == 3) {
        // === ЯДРО 3: PCM (только тут вызываем PCM_Update) ===
        while (true) {
            // Первый Wait: CPU опубликовал s_CycleTarget и ждёт — мы sync'имся сюда.
            s_Barrier.Wait();

            // Читаем цель (до какого цикла считать PCM)
            uint64_t target = s_CycleTarget.load(std::memory_order_acquire);

            // МОНОПОЛИЯ ядра 3 на PCM_Update: считаем звук до target
            // (PCM_Update должен писать в mcu.sample_buffer и изменять mcu.sample_write_ptr)
            mcu.pcm.PCM_Update(target);

            // Второй Wait: сигнализируем CPU, что подсчёт завершён
            s_Barrier.Wait();
        }
    }
    else {
        // ядро 1 — MIDI/прочее: просто yield в цикле (не участвует в барьере)
        while (true) {
           // if (m_pMIDIDevice && m_pMIDIDevice->hostDevice)
           //     m_pMIDIDevice->hostDevice->Update();
            asm volatile("yield" ::: "memory");
        }
    }
}*/