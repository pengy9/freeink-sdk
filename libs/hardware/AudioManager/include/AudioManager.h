#pragma once

// FreeInk audio output.
//
// Drives the audio path described by BoardConfig::ACTIVE.audio: bring up the
// control codec over its (possibly shared) I2C bus, master the I2S bus with
// the new ESP-IDF i2s_std driver, and stream 16-bit PCM data from a
// caller-supplied byte source. Two codecs are supported, selected by
// AudioConfig::output: ES8388 (Murphy M3, OEM-recovered register sequence)
// and ES8311 (M5 PaperColor, mirroring M5Unified's speaker bring-up — the
// codec clocks itself from BCLK, plus the AW8737A amp on its ampEnable pin).
//
// Playback runs in a dedicated FreeRTOS task (priority above typical workers,
// like the OEM "musicTask"), so play() returns immediately; with loop=true the
// source is rewound and replayed until stop() is called — the alarm use case.
//
// The WAV source is a pair of callbacks instead of a FILE/Stream so the SDK
// stays storage-agnostic: firmware can serve bytes from LittleFS, SD, or a
// PROGMEM array with the same API.

#include <Arduino.h>

#include <atomic>
#include <functional>

namespace freeink {

class AudioManager {
  enum class PlaybackState : uint8_t { Idle, Playing, Stopping, RestartPending };

 public:
  struct WavSource {
    // Copy up to len bytes to dst, returning the count (0 = EOF, <0 = error).
    std::function<int(uint8_t* dst, size_t len)> read;
    // Absolute seek from the start of the WAV; used for chunk walking and loop
    // rewind. Return false if unsupported (loop and header re-parse then fail).
    std::function<bool(size_t pos)> seek;
  };

  struct PcmSource {
    // Copy interleaved signed 16-bit little-endian PCM to dst. The callback
    // may block; 0 ends playback and a negative result reports a source error.
    std::function<int(uint8_t* dst, size_t len)> read;
    // Optional absolute byte seek. playPcm() starts at zero; restart requests
    // require this callback.
    std::function<bool(size_t pos)> seek;
  };

  // Initializes the codec + enable pin. Returns false when the active board
  // has no audio path (callers can treat audio as absent).
  bool begin();
  bool present() const;

  // Analog output volume, 0-100 (maps onto the codec's OUT1/OUT2 registers).
  void setVolume(uint8_t percent);

  // Starts WAV playback (16-bit PCM, mono or stereo, 8-48 kHz). Stops any
  // current playback first. loop=true replays until stop().
  bool play(const WavSource& source, bool loop);

  // Starts headerless PCM playback. Mono samples are duplicated to both I2S
  // slots. Only 16-bit little-endian, 1-2 channels, and 8-48 kHz are accepted.
  bool playPcm(const PcmSource& source, uint32_t sampleRate, uint8_t channels);

  // Convenience: play from a memory buffer (e.g. an embedded default sound).
  bool playBuffer(const uint8_t* data, size_t len, bool loop);

  // Request a rewind at the next PCM buffer boundary. Returns false when no
  // active seekable source can accept it, so callers may start new playback.
  bool requestRestart();

  void stop();
  bool isPlaying() const { return playbackState_.load(std::memory_order_acquire) != PlaybackState::Idle; }

  // Codec power-down (CHIPPOWER off). begin() restores it.
  void powerDown();

  // Release the I2S channel while leaving the codec powered and the amplifier
  // enabled. The ESP32 has a fixed pool of I2S peripherals (two), and this class
  // holds one for as long as it lives; a second audio stack that needs its own
  // channel has to hand it back first. That is the case when ESP32-audioI2S
  // drives playback (see lib/hal/HalAudioOutput.cpp): it owns the PCM pipeline
  // and creates its own I2S channel, but still relies on this class for the
  // ES8311 register table and the speaker amp.
  void releaseI2s();

  // Speaker-amp enable (GPIO 39 on this board). begin() deliberately leaves the
  // amp off ("comes up only during playback") and the in-house playback path
  // turns it on by itself — but a caller that uses the codec from this class and
  // the PCM pipeline from somewhere else never passes through that path, so it
  // must switch the amp itself. Silent output with a healthy decode and a rising
  // playback position is exactly what a missing setAmp(true) looks like.
  void setAmpEnable(bool on);

 private:
  struct WavInfo {
    uint32_t sampleRate = 0;
    uint16_t channels = 0;
    uint16_t bitsPerSample = 0;
    size_t dataStart = 0;
    size_t dataLength = 0;
  };

  static void taskEntry(void* self);
  void taskLoop();
  bool parseWavHeader(const WavSource& source, WavInfo& info);
  bool startPlayback(const WavSource& source, const WavInfo& info, bool loop);
  bool ensureI2s(uint32_t sampleRate);
  void teardownI2s();

  bool codecInit();
  bool codecWrite(uint8_t reg, uint8_t value);
  bool codecRead(uint8_t reg, uint8_t* value);
  void codecMute(bool mute);
  void setAmp(bool on);
  void rollbackBegin();

  bool begun_ = false;
  std::atomic<PlaybackState> playbackState_{PlaybackState::Idle};
  TaskHandle_t task_ = nullptr;

  WavSource source_;
  WavInfo wav_;
  bool loop_ = false;

  void* txChan_ = nullptr;  // i2s_chan_handle_t (kept void* to slim the header)
  volatile bool chanEnabled_ = false;
  uint32_t currentRate_ = 0;
  uint32_t diagPass_ = 0;  // throttle for the read/write timing diagnostic
};

}  // namespace freeink

using AudioManager = freeink::AudioManager;
