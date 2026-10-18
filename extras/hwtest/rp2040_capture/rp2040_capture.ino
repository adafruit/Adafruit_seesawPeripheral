// STEMMA Friend I2C capture; passive except during an explicit T controller test.
// USB CDC retains Arduino reset/upload.
// PIO sampling follows Raspberry Pi's BSD-3-Clause pico-examples logic_analyser:
// Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
// https://github.com/raspberrypi/pico-examples/tree/master/pio/logic_analyser
#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/i2c.h>
#include <hardware/pio.h>
#include <hardware/regs/pio.h>

// Record both known connector revisions: SDA/SCL on 26/27 or 28/29.
const uint8_t firstPin = 26;
const uint8_t pinCount = 4;
const uint32_t bufferWords = 49152;
uint32_t samples[bufferWords];
PIO capturePIO = pio0;
uint stateMachine;
int dmaChannel;
uint programOffset;
uint32_t sampleRate = 10000000;
uint32_t capturedWords = 0;
uint32_t captureFaults = 0;
bool armed = false;
char command[48];
uint8_t commandLength = 0;

void setup() {
  // GPIO25 is tied to SDA on one hardware revision. Never drive it.
  for (uint8_t pin = 25; pin <= 29; pin++) {
    pinMode(pin, INPUT);
    gpio_disable_pulls(pin);
  }
  Serial.begin(115200);

  stateMachine = pio_claim_unused_sm(capturePIO, true);
  dmaChannel = dma_claim_unused_channel(true);
  const uint16_t instruction = pio_encode_in(pio_pins, pinCount);
  const pio_program program = {&instruction, 1, -1};
  programOffset = pio_add_program(capturePIO, &program);
  pio_sm_config config = pio_get_default_sm_config();
  sm_config_set_in_pins(&config, firstPin);
  sm_config_set_wrap(&config, programOffset, programOffset);
  sm_config_set_in_shift(&config, true, true, 32);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
  pio_sm_init(capturePIO, stateMachine, programOffset, &config);
  pio_sm_set_consecutive_pindirs(capturePIO, stateMachine, firstPin, pinCount, false);
}

void loop() {
  while (Serial.available()) {
    char next = Serial.read();
    if (next == '\n') {
      command[commandLength] = 0;
      handleCommand();
      commandLength = 0;
    } else if (next != '\r') {
      if (commandLength < sizeof(command) - 1) {
        command[commandLength++] = next;
      } else {
        commandLength = 0;
        Serial.print("ERR command too long\n\r");
      }
    }
  }
}

void stopCapture() {
  if (!armed) {
    return;
  }
  pio_sm_set_enabled(capturePIO, stateMachine, false);
  // Let DMA drain the short PIO FIFO, then save its count BEFORE abort clears it.
  delayMicroseconds(2);
  capturedWords = bufferWords - dma_hw->ch[dmaChannel].transfer_count;
  dma_channel_abort(dmaChannel);
  // FDEBUG bits are hardware W1C flags. Preserve the raw value in the capture.
  captureFaults = capturePIO->fdebug;
  armed = false;
}

void startCapture(uint32_t rate) {
  stopCapture();
  sampleRate = rate;
  capturedWords = 0;
  pio_sm_clear_fifos(capturePIO, stateMachine);
  pio_sm_restart(capturePIO, stateMachine);
  pio_sm_exec(capturePIO, stateMachine, pio_encode_jmp(programOffset));
  pio_sm_set_clkdiv(capturePIO, stateMachine,
                   (float)clock_get_hz(clk_sys) / sampleRate);
  capturePIO->fdebug = 0xFFFFFFFF; // Clear all hardware W1C debug flags.
  dma_channel_config config = dma_channel_get_default_config(dmaChannel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
  channel_config_set_read_increment(&config, false);
  channel_config_set_write_increment(&config, true);
  channel_config_set_dreq(&config, pio_get_dreq(capturePIO, stateMachine, false));
  dma_channel_configure(dmaChannel, &config, samples,
                       &capturePIO->rxf[stateMachine], bufferWords, true);
  armed = true;
  pio_sm_set_enabled(capturePIO, stateMachine, true);
}

void handleCommand() {
  if (command[0] == 'I') {
    Serial.printf("RP2040_CAPTURE 2 pins=26,27,28,29 inputs=%u%u%u%u\n\r",
                  gpio_get(26), gpio_get(27), gpio_get(28), gpio_get(29));
  } else if (command[0] == 'T') {
    unsigned int reg, length, clock;
    if (sscanf(command + 1, "%u %u %u", &reg, &length, &clock) != 3 ||
        reg < 1 || reg > 3 || length < 1 || length > 4 ||
        (clock != 100000 && clock != 400000)) {
      Serial.print("ERR test arguments\n\r");
      return;
    }
    // Direct 3.3 V open-drain controller on the verified GPIO28/29 connector.
    // Disconnect other controllers before issuing T. PIO still samples these pins.
    uint8_t prefix[2] = {0, (uint8_t)reg};
    uint8_t result[4];
    i2c_init(i2c0, clock);
    gpio_set_function(28, GPIO_FUNC_I2C);
    gpio_set_function(29, GPIO_FUNC_I2C);
    int written = i2c_write_timeout_us(i2c0, 0x49, prefix, 2, false, 25000);
    int received = 0;
    if (written == 2) {
      delayMicroseconds(2000);
      received = i2c_read_timeout_us(i2c0, 0x49, result, length, false, 25000);
    }
    // The final RX byte can arrive before the hardware finishes its STOP.
    // Let that STOP finish before disabling the controller and releasing pins.
    delayMicroseconds(50);
    i2c_deinit(i2c0);
    // Always release both wires, including all timeout and NACK paths.
    for (uint8_t pin = 28; pin <= 29; pin++) {
      pinMode(pin, INPUT);
      gpio_disable_pulls(pin);
    }
    if (written != 2 || received != (int)length) {
      Serial.printf("ERR direct write=%d read=%d\n\r", written, received);
      return;
    }
    Serial.print("DATA");
    for (unsigned int index = 0; index < length; index++) {
      Serial.printf(" %02X", result[index]);
    }
    Serial.print("\n\r");
  } else if (command[0] == 'A') {
    uint32_t rate = strtoul(command + 1, nullptr, 10);
    if (rate < 1000000 || rate > 20000000) {
      Serial.print("ERR rate must be 1000000..20000000\n\r");
      return;
    }
    startCapture(rate);
    Serial.print("ARMED\n\r");
  } else if (command[0] == 'S') {
    stopCapture();
    Serial.printf("STOPPED %lu %lu\n\r", capturedWords, captureFaults);
  } else if (command[0] == 'D') {
    stopCapture();
    Serial.printf("DATA %lu %lu %lu\n\r", capturedWords * 4, sampleRate,
                  captureFaults);
    Serial.write((uint8_t *)samples, capturedWords * 4);
    Serial.print("END\n\r");
  } else {
    Serial.print("ERR command\n\r");
  }
}
