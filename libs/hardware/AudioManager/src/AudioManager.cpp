#include "AudioManager.h"

#include <BoardConfig.h>

// Capability-gated like FrontlightManager: devices without FREEINK_CAP_AUDIO
// compile the stub bodies at the bottom and link no I2S/codec code.
#if FREEINK_CAP_AUDIO

#include <Wire.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

#include <memory>

namespace freeink {

namespace {

constexpr uint32_t CODEC_I2C_HZ = 100000;  // OEM bus speed (shared with touch)
constexpr size_t READ_CHUNK = 4096;        // PCM bytes per loop pass (matched to the MP3 decoder's 4 KiB SD read / decode batch, so a single decode pass fuels many loop iterations instead of being discarded every pass)

// ES8388 playback init recovered from the Murphy OEM firmware — exact register
// order matters (staged mute -> clocks/format -> mixers -> power -> unmute).
struct RegVal {
  uint8_t reg, val;
};
constexpr RegVal ES8388_INIT[] = {
    {0x19, 0x04}, {0x01, 0x50}, {0x02, 0x00}, {0x08, 0x00}, {0x04, 0x3e}, {0x00, 0x12},
    {0x17, 0x18}, {0x18, 0x02}, {0x26, 0x1b}, {0x27, 0x90}, {0x2a, 0x90}, {0x2b, 0x80},
    {0x2d, 0x00}, {0x1b, 0x00}, {0x1a, 0x00}, {0x03, 0xff}, {0x09, 0x88}, {0x0a, 0xf0},
    {0x0b, 0x80}, {0x0c, 0x0e}, {0x0d, 0x02}, {0x10, 0x20}, {0x11, 0x20}, {0x2e, 0x1e},
    {0x2f, 0x1e}, {0x30, 0x1e}, {0x31, 0x1e}, {0x04, 0x3c}, {0x19, 0x00},
};

constexpr uint8_t ES8388_VOL_MAX_REG = 0x21;  // register full-scale for OUT volumes

// ES8311 playback init, mirroring M5Unified's PaperColor speaker bring-up
// (_speaker_enabled_cb_papercolor). The codec derives its internal MCLK from
// BCLK (reg 0x01 bit7 + reg 0x02 MULT_PRE=3, i.e. 32*fs * 8 = 256*fs), so no
// MCLK line is needed and the same init covers every sample rate. The 16-bit
// I2S SDP format (0x09) is set explicitly — M5Unified relies on the default.
constexpr RegVal ES8311_INIT[] = {
    {0x00, 0x80},  // RESET: CSM power on, slave mode
    {0x01, 0xB5},  // CLK_MANAGER: internal MCLK from BCLK pin, clocks on
    {0x02, 0x18},  // CLK_MANAGER: MULT_PRE x8 -> internal MCLK = 256*fs
    {0x09, 0x0C},  // SDP-in: I2S format, 16-bit
    {0x0D, 0x01},  // SYSTEM: power up analog circuitry
    {0x12, 0x00},  // SYSTEM: power up DAC
    {0x13, 0x10},  // SYSTEM: enable output to HP drive
    {0x32, 0xCF},  // DAC volume +16 dB (M5's default for the 1W speaker)
    {0x37, 0x08},  // DAC: bypass equalizer
};

// Waveshare 397 uses a dedicated external MCLK line (GPIO13) driven by the
// ESP32-S3 I2S peripheral at 256×Fs. With MCLK always at 256×Fs, the ES8311's
// clock-divider registers are CONSTANTS (per the Waveshare driver coefficient
// table — every row where MCLK=256×Fs collapses to the same set), and the
// sample rate is set by the I2S LRCK alone. The codec therefore needs no
// re-tuning to switch sample rates: 8/11.025/12/16/22.05/24/32/44.1/48/64 kHz
// all work with this one init sequence.
//
//   reg 0x01 = 0x3F  CLK_MANAGER: MCLK from MCLK pin (MCLK_SEL=0), clocks on
//   reg 0x02 = 0x00  DIV_PRE=0, MULT_PRE=0 -> dig_mclk = MCLK (256×Fs)
//   reg 0x03 = 0x10  ADC_OSR = 64×fs
//   reg 0x04 = 0x10  DAC_OSR = 64×fs
//   reg 0x05 = 0x00  ADC_DIV=0, DAC_DIV=0 -> adc/dac_mclk = dig_mclk
//   reg 0x06 = 0x03  BCLK divider (master-mode only; we are slave)
//   reg 0x07 = 0x00  LRCLK divider high
//   reg 0x08 = 0xFF  LRCLK divider low
constexpr RegVal ES8311_MCLK_16K_INIT[] = {
    {0x00, 0x80}, {0x01, 0x3F}, {0x02, 0x00}, {0x03, 0x10}, {0x04, 0x10},
    {0x05, 0x00}, {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xFF}, {0x09, 0x0C},
    {0x0D, 0x01}, {0x12, 0x00}, {0x13, 0x10},
    // DAC mute / volume. 0x31 is the DAC mute control: codecMute() writes 0x60 to
    // silence it and 0x00 to open it, so an init table that omits 0x31 inherits
    // whatever the chip powered up with — and "everything succeeds, position
    // climbs, speaker silent" is precisely what a muted DAC looks like. The
    // ES8388 table spells the equivalent explicitly (0x19 0x04 -> 0x19 0x00);
    // this one was trimmed and lost the line.
    {0x31, 0x00},
    {0x32, 0xCF}, {0x37, 0x08},
};

constexpr uint8_t ES8311_VOL_MAX_REG = 0xCF;  // +16 dB, 0.5 dB/step (0xBF = 0 dB)

uint32_t readLE32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint16_t readLE16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

bool isEs8311(BoardConfig::AudioOutput output) {
  return output == BoardConfig::AudioOutput::I2sEs8311 ||
         output == BoardConfig::AudioOutput::I2sEs8311Mclk16k;
}

}  // namespace

bool AudioManager::present() const {
  return BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sEs8388 ||
         isEs8311(BoardConfig::ACTIVE.audio.output) ||
         BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sDac;
}

bool AudioManager::codecWrite(uint8_t reg, uint8_t value) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  Wire.beginTransmission(cfg.codecAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool AudioManager::codecRead(uint8_t reg, uint8_t* value) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  Wire.beginTransmission(cfg.codecAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(cfg.codecAddr, (uint8_t)1) != 1) return false;
  *value = (uint8_t)Wire.read();
  return true;
}

bool AudioManager::codecInit() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return true;  // plain I2S DAC, nothing to configure

  const RegVal* seq;
  size_t seqLen;
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k) {
    seq = ES8311_MCLK_16K_INIT;
    seqLen = sizeof(ES8311_MCLK_16K_INIT) / sizeof(ES8311_MCLK_16K_INIT[0]);
  } else if (cfg.output == BoardConfig::AudioOutput::I2sEs8311) {
    seq = ES8311_INIT;
    seqLen = sizeof(ES8311_INIT) / sizeof(ES8311_INIT[0]);
  } else {
    seq = ES8388_INIT;
    seqLen = sizeof(ES8388_INIT) / sizeof(ES8388_INIT[0]);
  }

  Wire.begin(cfg.codecSda, cfg.codecScl, CODEC_I2C_HZ);

  // The OEM firmware retries until the codec ACKs; three attempts is plenty
  // for a codec already powered.
  for (int attempt = 0; attempt < 3; ++attempt) {
    bool ok = true;
    for (size_t i = 0; i < seqLen; ++i) {
      if (!codecWrite(seq[i].reg, seq[i].val)) {
        ok = false;
        break;
      }
    }
    if (ok) {
      // Diagnose the actual codec clock registers after init: read back the
      // divider/OSR values so a wrong write (vs a wrong rate) is distinguishable.
      if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k) {
        uint8_t r01 = 0, r02 = 0, r03 = 0, r04 = 0, r05 = 0, r08 = 0;
        uint8_t r0d = 0, r12 = 0, r13 = 0, r31 = 0, r32 = 0;
        codecRead(0x01, &r01);
        codecRead(0x02, &r02);
        codecRead(0x03, &r03);
        codecRead(0x04, &r04);
        codecRead(0x05, &r05);
        codecRead(0x08, &r08);
        codecRead(0x0D, &r0d);
        codecRead(0x12, &r12);
        codecRead(0x13, &r13);
        codecRead(0x31, &r31);
        codecRead(0x32, &r32);
        // Read back the whole signal path, not just the clock registers. The
        // output stage is where a "decoding fine but silent" fault shows up, and
        // 0x31 in particular: it is the DAC mute bit, and an init table that
        // omits it inherits whatever the chip powered up with.
        Serial.printf("[AUDIO] ES8311 clk 01=%02x 02=%02x 03=%02x 04=%02x 05=%02x 08=%02x\n", r01,
                      r02, r03, r04, r05, r08);
        Serial.printf("[AUDIO] ES8311 out 0d=%02x 12=%02x 13=%02x 31=%02x 32=%02x\n", r0d, r12,
                      r13, r31, r32);
        if (r31 != 0x00) Serial.printf("[AUDIO] WARNING: DAC mute set (0x31=%02x)\n", r31);
      }
      return true;
    }
    delay(100);
  }
  return false;
}

bool AudioManager::begin() {
  if (begun_) return true;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!present()) return false;

  if (cfg.ampEnable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.ampEnable, OUTPUT);
    digitalWrite(cfg.ampEnable, LOW);  // amp comes up only during playback
  }
  if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.enable, OUTPUT);
    digitalWrite(cfg.enable, cfg.enableActiveHigh ? HIGH : LOW);
    delay(10);  // codec rail ramp before the first I2C access
  }

  // Bring the I2S channel up once so the codec sees a valid clock before init,
  // and so the first playback does not pay for channel creation. The rate here
  // is only a starting point: ensureI2s() rebuilds the channel whenever the
  // requested rate differs (see currentRate_), so 44.1 kHz covers the common
  // MP3 case without a conversion.
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k && !ensureI2s(44100)) {
    log_e("audio mclk setup failed");
    rollbackBegin();
    return false;
  }

  if (!codecInit()) {
    log_e("audio codec init failed");
    rollbackBegin();
    return false;
  }
  begun_ = true;
  return true;
}

void AudioManager::setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (isEs8311(cfg.output)) {
    // ES8311 register 0x32 is a 0.5 dB-per-step attenuator, NOT a linear
    // percentage: 0x00 = -95.5 dB (silence), 0xBF = 0 dB, 0xCF = +16 dB.
    //
    // Mapping percent straight onto the register (percent * 0xCF / 100) put the
    // usable range in completely the wrong place. Its actual output was:
    //   100% -> +8 dB     80% -> -13 dB     60% -> -33.5 dB
    //    40% -> -54.5 dB  20% -> -75 dB
    // i.e. the whole bottom half of the slider was already inaudible, and the
    // top fifth threw away 21 dB. That is why "80" sounded like what you would
    // expect from "60": almost all of the travel sat below 0 dB.
    //
    // Fix: interpolate in dB (which is what ears hear logarithmically) across a
    // range that is actually useful, then convert back to 0.5 dB steps. The top
    // stays at the codec init value so nothing gets louder than the speaker was
    // tuned for; the floor is -55 dB, and 0 % is a hard mute.
    //
    // Resulting curve — a near-constant ~6.5 dB per 10 % step:
    //   100% +8 dB | 80% -1.5 dB | 60% -15 dB | 40% -28.5 dB | 20% -42 dB | 0% mute
    constexpr int kZeroDbReg = 0xBF;         // register value for 0 dB
    constexpr int kVolFloorDbTimes10 = -550; // -55.0 dB at 1 %
    constexpr int kVolTopDbTimes10 = 120;    // +12.0 dB at 100 %

    if (percent == 0) {
      // Hard mute rather than a very quiet signal: 0x00 is -95.5 dB, which the
      // speaker/amp combination renders as silence.
      codecWrite(0x32, 0x00);
      return;
    }

    const int dbTimes10 =
        kVolFloorDbTimes10 + (kVolTopDbTimes10 - kVolFloorDbTimes10) * percent / 100;
    // One register step is 0.5 dB, and dbTimes10 is in tenths of a dB,
    // so steps = db / 0.5 = dbTimes10 * 2 / 10 = dbTimes10 / 5.
    int reg = kZeroDbReg + dbTimes10 / 5;
    if (reg < 0x01) reg = 0x01;  // never land on the mute code by accident
    if (reg > (int)ES8311_VOL_MAX_REG) reg = ES8311_VOL_MAX_REG;
    codecWrite(0x32, (uint8_t)reg);
    return;
  }
  const uint8_t reg = (uint8_t)((uint16_t)percent * ES8388_VOL_MAX_REG / 100);
  // OUT1 (0x2e/0x2f) and OUT2 (0x30/0x31) pairs, like the OEM volume path.
  codecWrite(0x2e, reg);
  codecWrite(0x2f, reg);
  codecWrite(0x30, reg);
  codecWrite(0x31, reg);
}

// DAC mute between alarms so nothing residual reaches the output.
void AudioManager::codecMute(bool mute) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (isEs8311(cfg.output)) {
    codecWrite(0x31, mute ? 0x60 : 0x00);  // DAC_MUTE bits
  } else {
    codecWrite(0x19, mute ? 0x04 : 0x00);  // ES8388 DACCONTROL3 soft mute
  }
}

void AudioManager::setAmp(bool on) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.ampEnable == BoardConfig::PIN_UNASSIGNED) return;
  digitalWrite(cfg.ampEnable, on ? HIGH : LOW);
}

// Public form of the above. Unlike the private one it does not assume begin()
// has already run, so a caller can hand the amp back at any point.
void AudioManager::setAmpEnable(bool on) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.ampEnable == BoardConfig::PIN_UNASSIGNED) return;
  pinMode(cfg.ampEnable, OUTPUT);
  digitalWrite(cfg.ampEnable, on ? HIGH : LOW);
  // Serial.printf, not log_i: this firmware builds at CORE_DEBUG_LEVEL 0, where
  // log_i() compiles to nothing — its silence in the log is what made a missing
  // amp enable look like a working one. Logging.h is not an option here either,
  // this library does not link the custom serial implementation it needs.
  // Only the transition is reported: playFile() calls this on every track.
  static bool sLast = false;
  if (on != sLast) {
    sLast = on;
    Serial.printf("[AUDIO] amp %s (gpio %d)\n", on ? "on" : "off", (int)cfg.ampEnable);
  }
}

void AudioManager::rollbackBegin() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  setAmp(false);
  teardownI2s();
  if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
    digitalWrite(cfg.enable, cfg.enableActiveHigh ? LOW : HIGH);
  }
}

void AudioManager::powerDown() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!begun_) return;
  stop();
  teardownI2s();
  if (isEs8311(cfg.output)) {
    // Like M5Unified's disable path: drop the amp and the codec rail.
    setAmp(false);
    if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
      digitalWrite(cfg.enable, cfg.enableActiveHigh ? LOW : HIGH);
    }
  } else if (cfg.codecAddr != 0) {
    codecWrite(0x02, 0xff);  // ES8388 CHIPPOWER: everything off
  }
  begun_ = false;
}

bool AudioManager::parseWavHeader(const WavSource& source, WavInfo& info) {
  uint8_t hdr[12];
  if (!source.seek(0)) return false;
  if (source.read(hdr, 12) != 12) return false;
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  size_t pos = 12;
  bool haveFmt = false;
  for (int guard = 0; guard < 32; ++guard) {
    uint8_t chunk[8];
    if (!source.seek(pos) || source.read(chunk, 8) != 8) return false;
    const uint32_t size = readLE32(chunk + 4);
    pos += 8;

    if (memcmp(chunk, "fmt ", 4) == 0) {
      uint8_t fmt[16];
      if (size < 16 || source.read(fmt, 16) != 16) return false;
      const uint16_t audioFormat = readLE16(fmt);
      info.channels = readLE16(fmt + 2);
      info.sampleRate = readLE32(fmt + 4);
      info.bitsPerSample = readLE16(fmt + 14);
      if (audioFormat != 1) return false;  // PCM only
      haveFmt = true;
    } else if (memcmp(chunk, "data", 4) == 0) {
      info.dataStart = pos;
      info.dataLength = size;
      break;
    }
    pos += size + (size & 1);  // chunks are word-aligned
  }

  return haveFmt && info.dataStart != 0 && info.bitsPerSample == 16 &&
         (info.channels == 1 || info.channels == 2) && info.sampleRate >= 8000 &&
         info.sampleRate <= 48000;
}

bool AudioManager::ensureI2s(uint32_t sampleRate) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  // ES8311 with external MCLK accepts any standard rate, because MCLK is always
  // driven at 256 x Fs and the codec's divider registers are the same for every
  // rate (see the table note above ES8311_MCLK_16K_INIT: 8/11.025/12/16/22.05/
  // 24/32/44.1/48/64 kHz all use the identical init sequence). The old guard
  // that rejected anything but 16 kHz was a leftover assumption, not a hardware
  // limit — it is what forced every MP3 through a 44.1 -> 16 kHz resampler.
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k) {
    const bool supported = sampleRate == 16000 || sampleRate == 22050 || sampleRate == 24000 ||
                           sampleRate == 32000 || sampleRate == 44100 || sampleRate == 48000;
    if (!supported) {
      log_e("external-mclk ES8311: unsupported PCM rate %u", (unsigned)sampleRate);
      return false;
    }
  }
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;

  if (tx && currentRate_ == sampleRate) {
    // Channel exists but was disabled when the last playback drained.
    if (!chanEnabled_) {
      if (i2s_channel_enable(tx) != ESP_OK) return false;
      chanEnabled_ = true;
    }
    return true;
  }

  if (tx) {
    if (chanEnabled_) i2s_channel_disable(tx);
    chanEnabled_ = false;
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
    clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // codec runs 256x MCLK/LRCK
    if (i2s_channel_reconfig_std_clock(tx, &clk) != ESP_OK) return false;
    if (i2s_channel_enable(tx) != ESP_OK) return false;
    chanEnabled_ = true;
    currentRate_ = sampleRate;
    return true;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // Without auto_clear the DMA replays its last buffers on underrun — heard
  // as a looping stutter after playback stops.
  chanCfg.auto_clear = true;
  // DMA depth is the whole reason audio used to cut out, so size it deliberately.
  //
  // The pool must outlast one decode block, because decoding runs inline on this
  // task and nothing is written to I2S while it runs. A 4 KiB MP3 chunk costs
  // ~85 ms to decode here.
  //
  // The required depth scales with the sample rate: 8 x 512 = 4096 frames is
  // 256 ms at 16 kHz but only 93 ms at 44.1 kHz, which is too thin once the
  // resampler is removed and playback stays at the file's native rate. 16 x 512
  // = 8192 frames gives 186 ms at 44.1 kHz — about 2.2x a decode block.
  //
  // The pool comes out of one contiguous internal-SRAM block, so this cannot be
  // raised freely: 12 x 512 at 16 KiB already failed to allocate once (MaxAlloc
  // was ~54 KB), which surfaced as a dead play button. 16 x 512 = 32 KiB. If
  // i2s_new_channel() starts failing, lower dma_frame_num rather than desc_num.
  chanCfg.dma_desc_num = 16;
  chanCfg.dma_frame_num = 512;
  esp_err_t chanErr = i2s_new_channel(&chanCfg, &tx, nullptr);
  if (chanErr != ESP_OK) {
    // Diagnose rather than fail silently: a too-large DMA pool is the difference
    // between a working player and a dead play button, and the numbers tell us
    // immediately whether it was the request or the free space.
    log_e("i2s_new_channel failed: %s (desc=%u frames=%u, free=%u, maxAlloc=%u)",
          esp_err_to_name(chanErr), (unsigned)chanCfg.dma_desc_num, (unsigned)chanCfg.dma_frame_num,
          (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    tx = nullptr;
    return false;
  }

  i2s_std_config_t std = {};
  std.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
  std.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  std.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  std.gpio_cfg.mclk = cfg.mclk == BoardConfig::PIN_UNASSIGNED ? I2S_GPIO_UNUSED
                                                              : (gpio_num_t)cfg.mclk;
  std.gpio_cfg.bclk = (gpio_num_t)cfg.bclk;
  std.gpio_cfg.ws = (gpio_num_t)cfg.lrclk;
  std.gpio_cfg.dout = (gpio_num_t)cfg.dout;
  std.gpio_cfg.din = I2S_GPIO_UNUSED;

  if (i2s_channel_init_std_mode(tx, &std) != ESP_OK) {
    i2s_del_channel(tx);
    return false;
  }
  if (i2s_channel_enable(tx) != ESP_OK) {
    i2s_del_channel(tx);
    return false;
  }

  // Report the frequencies the driver actually programmed. Guessing at dividers
  // from the header macros has already been wrong twice, so log the measured
  // clock tree: MCLK / BCLK tell us immediately whether the requested sample
  // rate was honoured. For 16-bit stereo at Fs the expected values are
  //   MCLK = 256 x Fs, BCLK = 32 x Fs   (2 channels x 16 bits)
  {
    i2s_chan_info_t info = {};
    if (i2s_channel_get_info(tx, &info) == ESP_OK) {
      log_i("i2s clk: req=%u mclk=%u (expect %u) bclk=%u (expect %u) clk_src=%d",
            (unsigned)sampleRate, (unsigned)info.mclk_hz, (unsigned)(sampleRate * 256),
            (unsigned)info.bclk_hz, (unsigned)(sampleRate * 32), (int)info.clk_src);
    }
  }
  txChan_ = tx;
  chanEnabled_ = true;
  currentRate_ = sampleRate;
  return true;
}

void AudioManager::releaseI2s() {
  // Hand the I2S peripheral back without touching the codec or the amplifier.
  // A second audio backend (ESP32-audioI2S) needs I2S_NUM_0 for itself, and the
  // ESP32 only has two I2S peripherals — holding this one is what makes that
  // backend fail with "no available channel found".
  teardownI2s();
}

void AudioManager::teardownI2s() {
  if (!txChan_) return;
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  if (chanEnabled_) i2s_channel_disable(tx);
  i2s_del_channel(tx);
  txChan_ = nullptr;
  chanEnabled_ = false;
  currentRate_ = 0;
}

bool AudioManager::play(const WavSource& source, bool loop) {
  if (!begun_ && !begin()) return false;
  stop();

  WavInfo info;
  if (!parseWavHeader(source, info)) {
    log_e("unsupported WAV (need 16-bit PCM, 1-2ch, 8-48 kHz)");
    return false;
  }
  if (!ensureI2s(info.sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }
  if (!source.seek(info.dataStart)) return false;

  return startPlayback(source, info, loop);
}

bool AudioManager::playPcm(const PcmSource& source, uint32_t sampleRate, uint8_t channels) {
  if (!source.read || sampleRate < 8000 || sampleRate > 48000 || (channels != 1 && channels != 2)) {
    log_e("unsupported PCM (need 16-bit LE, 1-2ch, 8-48 kHz)");
    return false;
  }
  if (!begun_ && !begin()) return false;
  stop();
  if (source.seek && !source.seek(0)) {
    log_e("PCM source seek failed");
    return false;
  }
  if (!ensureI2s(sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }

  WavInfo info;
  info.sampleRate = sampleRate;
  info.channels = channels;
  info.bitsPerSample = 16;

  WavSource wavSource;
  wavSource.read = source.read;
  wavSource.seek = source.seek;
  return startPlayback(wavSource, info, false);
}

bool AudioManager::startPlayback(const WavSource& source, const WavInfo& info, bool loop) {
  // Unmute the DAC (stop() mutes it). Codec writes stay on the caller's core
  // so the shared I2C bus is never touched from the audio task. The speaker
  // amp comes up in the playback task once silence is flowing — enabling it
  // against an idle I2S line is an audible pop.
  codecMute(false);

  source_ = source;
  wav_ = info;
  loop_ = loop;
  playbackState_.store(PlaybackState::Playing, std::memory_order_release);

  // Same shape as the OEM "musicTask" (high priority, core 0 — the Arduino
  // loop owns core 1).
  //
  // 24 KiB stack, sized from measurement and from what internal SRAM can
  // actually hand out.
  //
  // Measured with uxTaskGetStackHighWaterMark: the peak is ~2.7 KiB. The
  // "Stack canary watchpoint triggered (audio_play)" reports were never the
  // stack running out — a decoder buffer overrun put a stray write into the
  // guard region, which FreeRTOS reports as a stack fault. Chasing that by
  // raising the stack only made things worse: 64 KiB does not even fit here.
  //
  // Internal SRAM free is ~70 KiB but the LARGEST CONTIGUOUS block is only
  // ~31 KiB, because the decoder's own buffers (pcm ring / OLA staging /
  // mp3 chunk = ~44 KiB) are interleaved with everything else. A 64 KiB
  // request fails outright and the play button goes dead:
  //   audio task create failed (stack=65536, freeInt=71884, maxAlloc=31732)
  //
  // 24 KiB both fits inside the largest free block and leaves ~9x headroom
  // over the measured peak. Do NOT raise this again without first checking
  // heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL).
  constexpr uint32_t kAudioTaskStackBytes = 24 * 1024;
  if (xTaskCreatePinnedToCore(taskEntry, "audio_play", kAudioTaskStackBytes, this, 10, &task_, 0) !=
      pdPASS) {
    log_e("audio task create failed (stack=%u, freeInt=%u, maxAlloc=%u)", (unsigned)kAudioTaskStackBytes,
          (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    task_ = nullptr;
    playbackState_.store(PlaybackState::Idle, std::memory_order_release);
    return false;
  }
  return true;
}

bool AudioManager::playBuffer(const uint8_t* data, size_t len, bool loop) {
  // Shared offset state lives in the lambdas; play() copies them.
  auto offset = std::make_shared<size_t>(0);
  WavSource src;
  src.read = [data, len, offset](uint8_t* dst, size_t want) -> int {
    const size_t left = len - *offset;
    const size_t n = want < left ? want : left;
    memcpy(dst, data + *offset, n);
    *offset += n;
    return (int)n;
  };
  src.seek = [len, offset](size_t pos) {
    if (pos > len) return false;
    *offset = pos;
    return true;
  };
  return play(src, loop);
}

bool AudioManager::requestRestart() {
  if (!source_.seek) return false;
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  for (;;) {
    if (state == PlaybackState::RestartPending) return true;
    if (state != PlaybackState::Playing) return false;
    if (playbackState_.compare_exchange_weak(state, PlaybackState::RestartPending, std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      return true;
    }
  }
}

void AudioManager::stop() {
  if (!begun_) return;
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  while (state == PlaybackState::Playing || state == PlaybackState::RestartPending) {
    if (playbackState_.compare_exchange_weak(state, PlaybackState::Stopping, std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      break;
    }
  }
  for (int i = 0; i < 200 && playbackState_.load(std::memory_order_acquire) != PlaybackState::Idle; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Drop the amp and mute the DAC so nothing residual reaches the output
  // between alarms.
  setAmp(false);
  codecMute(true);
}

void AudioManager::taskEntry(void* self) { static_cast<AudioManager*>(self)->taskLoop(); }

void AudioManager::taskLoop() {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  // Static (not stack) buffers: the libhelix decode call chain runs *inside*
  // read() on this task's stack, and 12 KiB of on-stack buffers here was
  // overflowing the 20 KiB task stack (stack canary watchpoint on large/high-
  // bitrate files). Moving them off-stack leaves the whole stack for decode.
  alignas(int16_t) static uint8_t inBuf[READ_CHUNK];
  // Mono is duplicated into both slots, so the out buffer is 2x.
  static int16_t outBuf[READ_CHUNK];

  // Prime the line with silence, then raise the amp: the AW8737A pops loudly
  // when enabled against an idle or just-started I2S line.
  memset(outBuf, 0, sizeof(outBuf));
  for (int i = 0; i < 2; ++i) {
    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
  }
  setAmp(true);
  if (BoardConfig::ACTIVE.audio.ampSettleMs > 0) {
    vTaskDelay(pdMS_TO_TICKS(BoardConfig::ACTIVE.audio.ampSettleMs));
  }

  size_t consumed = 0;
  bool draining = false;
  uint8_t drainBuffers = 0;
  for (;;) {
    PlaybackState state = playbackState_.load(std::memory_order_acquire);
    if (state == PlaybackState::RestartPending) {
      if (!source_.seek || !source_.seek(wav_.dataStart)) {
        log_e("PCM source restart failed");
        playbackState_.store(PlaybackState::Stopping, std::memory_order_release);
      } else {
        consumed = 0;
        draining = false;
        drainBuffers = 0;
        PlaybackState pending = PlaybackState::RestartPending;
        playbackState_.compare_exchange_strong(pending, PlaybackState::Playing, std::memory_order_acq_rel,
                                               std::memory_order_acquire);
      }
      continue;
    }
    if (state == PlaybackState::Stopping && !draining) {
      draining = true;
      drainBuffers = 0;
    }

    if (draining) {
      if (drainBuffers >= 6) {
        if (state == PlaybackState::Stopping) break;
        PlaybackState playing = PlaybackState::Playing;
        if (playbackState_.compare_exchange_strong(playing, PlaybackState::Stopping, std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
          break;
        }
        continue;
      }
      memset(outBuf, 0, sizeof(outBuf));
      size_t written = 0;
      if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) {
        playbackState_.store(PlaybackState::Stopping, std::memory_order_release);
        break;
      }
      ++drainBuffers;
      continue;
    }

    size_t want = READ_CHUNK;
    if (wav_.dataLength > 0) {
      const size_t left = wav_.dataLength - consumed;
      if (left == 0) {
        if (loop_ && source_.seek(wav_.dataStart)) {
          consumed = 0;
          continue;
        }
        draining = true;
        continue;
      }
      if (want > left) want = left;
    }
    const size_t frameBytes = static_cast<size_t>(wav_.channels) * sizeof(int16_t);
    want -= want % frameBytes;
    if (want == 0) {
      draining = true;
      continue;
    }

    const uint32_t t0 = micros();
    const int n = source_.read(inBuf, want);
    const uint32_t readUs = micros() - t0;
    if (n <= 0) {
      if (loop_ && source_.seek(wav_.dataStart)) {
        consumed = 0;
        continue;
      }
      draining = true;
      continue;
    }
    if (n > static_cast<int>(want) || static_cast<size_t>(n) % frameBytes != 0) {
      log_e("PCM source returned an invalid byte count");
      playbackState_.store(PlaybackState::Stopping, std::memory_order_release);
      break;
    }
    consumed += n;

    const int16_t* samples = (const int16_t*)inBuf;
    size_t outBytes;
    if (wav_.channels == 1) {
      const int frames = n / 2;
      for (int i = 0; i < frames; ++i) {
        outBuf[i * 2] = samples[i];
        outBuf[i * 2 + 1] = samples[i];
      }
      outBytes = (size_t)frames * 4;
    } else {
      memcpy(outBuf, inBuf, n);
      outBytes = (size_t)n;
    }

    size_t written = 0;
    const uint32_t w0 = micros();
    const esp_err_t wr = i2s_channel_write(tx, outBuf, outBytes, &written, pdMS_TO_TICKS(1000));
    const uint32_t writeUs = micros() - w0;
    if (++diagPass_ == 1 || readUs > 5000 || writeUs > 5000) {
      // Report the stack high-water mark too: it is the smallest amount of free
      // stack ever seen on this task, so it tells us how close the decode chain
      // runs to the edge. Without this we are guessing at stack sizes blind.
      Serial.printf("[AUDIO] pass n=%d read=%luus write=%luus\n", n, (unsigned long)readUs,
                    (unsigned long)writeUs);
      diagPass_ = 0;
    }
    if (wr != ESP_OK) {
      playbackState_.store(PlaybackState::Stopping, std::memory_order_release);
      break;
    }
  }

  setAmp(false);
  i2s_channel_disable(tx);
  chanEnabled_ = false;

  // Report how much of the stack the decode chain actually touched. The high
  // water mark is the smallest number of free bytes ever seen; if it comes back
  // small (say under 4 KiB) the task is running on fumes and any deeper call —
  // the OLA path at a non-1.0 rate, or a switch mid-decode — will overflow it.
  // Logged on the way out rather than periodically so it never lands in the
  // middle of a decode.
  {
    const UBaseType_t hwWords = uxTaskGetStackHighWaterMark(nullptr);
    log_i("audio task stack high-water: %u bytes free (of %u)", (unsigned)(hwWords * sizeof(StackType_t)),
          (unsigned)kAudioTaskStackBytes);
  }

  task_ = nullptr;
  playbackState_.store(PlaybackState::Idle, std::memory_order_release);
  vTaskDelete(nullptr);
}

}  // namespace freeink

#else  // !FREEINK_CAP_AUDIO — stubs so callers need no #ifdefs

namespace freeink {
bool AudioManager::present() const { return false; }
bool AudioManager::begin() { return false; }
void AudioManager::setVolume(uint8_t) {}
bool AudioManager::play(const WavSource&, bool) { return false; }
bool AudioManager::playPcm(const PcmSource&, uint32_t, uint8_t) { return false; }
bool AudioManager::playBuffer(const uint8_t*, size_t, bool) { return false; }
bool AudioManager::requestRestart() { return false; }
void AudioManager::stop() {}
void AudioManager::powerDown() {}
bool AudioManager::parseWavHeader(const WavSource&, WavInfo&) { return false; }
bool AudioManager::startPlayback(const WavSource&, const WavInfo&, bool) { return false; }
bool AudioManager::ensureI2s(uint32_t) { return false; }
void AudioManager::teardownI2s() {}
void AudioManager::releaseI2s() {}
bool AudioManager::codecInit() { return false; }
bool AudioManager::codecWrite(uint8_t, uint8_t) { return false; }
void AudioManager::codecMute(bool) {}
void AudioManager::setAmp(bool) {}
void AudioManager::rollbackBegin() {}
void AudioManager::taskEntry(void*) {}
void AudioManager::taskLoop() {}
}  // namespace freeink

#endif  // FREEINK_CAP_AUDIO
