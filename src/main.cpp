#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include "rmt_uart.h"
#include <NeoPixelBus.h>
#include <EEPROM.h>

#define EEPROM_SIZE 1

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod> rgb(1, 13);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define NUM_PORTS 7
#define RMT_START 3
#else
#define NUM_PORTS 6
#define RMT_START 2
#endif

uint8_t logNumber;
uint8_t buf[NUM_PORTS][80];
uint8_t bufPos[NUM_PORTS] = {0};
File file;
char filename[12]={0};
bool button_pressed = false;

void pressed() {
  button_pressed = true;
}

__attribute__((unused)) void setup() {
  memset(bufPos, 0, sizeof(bufPos));
  pinMode(0, INPUT);
  attachInterrupt(0, pressed, FALLING);

  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(921600);
  Serial.println("Starting...");
  Serial.flush();
  button_pressed = false;
  rgb.Begin();
  rgb.SetPixelColor(0, RgbColor(64, 0, 0));
  rgb.Show();
}

static void writeBuffer(const int port) {
  Serial.print(port+1);
  file.write('1' + port);
  file.write(',');
  file.write(buf[port], bufPos[port]);
  bufPos[port] = 0;
}

static void readSerial(const int port, HardwareSerial& serial) {
  int bytes = serial.available();
  if (bytes) {
    for (int i=0 ; i<bytes ; i++) {
      uint8_t byte = serial.read();
      buf[port][bufPos[port]++] = byte;
      if (byte == '\n') {
        writeBuffer(port);
      }
      if(bufPos[port] == 80) {
        bufPos[port] = 0;
      }
    }
  }
}

bool openSDCard() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // SPI0.begin();
  // SD.begin(SS, SPI0);
#else
  if (!SD.begin()) {
    return false;
  }
#endif
 EEPROM.get(0, logNumber);
 logNumber++;
 EEPROM.put(0, logNumber);
 EEPROM.commit();
 
 sprintf(filename,"/log%03d.csv",logNumber);
 File dataFile = SD.open (filename,FILE_WRITE);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  file = SD.open(filename, FILE_WRITE, true);
  if (!file) {
    Serial.println("Could not open SD Card");
    return false;
  }
  return true;
}

void startSerials() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  Serial0.begin(460800, SERIAL_8N1, 44, -1);
  Serial1.begin(460800, SERIAL_8N1, 15, -1);
  Serial2.begin(460800, SERIAL_8N1, 19, -1);
  Serial0.flush(false);
#else
  Serial1.begin(460800, SERIAL_8N1, 14, -1);
  Serial2.begin(460800, SERIAL_8N1, 27, -1);
#endif

  rmt_uart_config_t config = {
    .baud_rate = 460800,
    .mode = RMT_UART_MODE_RX_ONLY,
    .data_bits = RMT_UART_DATA_8_BITS,
    .parity = RMT_UART_PARITY_DISABLE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .tx_io_num = GPIO_NUM_NC,
    .rx_io_num = GPIO_NUM_NC,
    .buffer_size = 1024
  };
  config.rx_io_num = GPIO_NUM_26;
  rmt_uart_init(RMT_UART_NUM_0, &config);
  pinMode(config.rx_io_num, INPUT_PULLUP);
  config.rx_io_num = GPIO_NUM_25;
  rmt_uart_init(RMT_UART_NUM_1, &config);
  pinMode(config.rx_io_num, INPUT_PULLUP);
  config.rx_io_num = GPIO_NUM_33;
  rmt_uart_init(RMT_UART_NUM_2, &config);
  pinMode(config.rx_io_num, INPUT_PULLUP);
  config.rx_io_num = GPIO_NUM_32;
  rmt_uart_init(RMT_UART_NUM_3, &config);
  pinMode(config.rx_io_num, INPUT_PULLUP);
  Serial1.flush(false);
  Serial2.flush(false);
}

void stopSerials() {
  rmt_uart_deinit(RMT_UART_NUM_0);
  rmt_uart_deinit(RMT_UART_NUM_1);
  rmt_uart_deinit(RMT_UART_NUM_2);
  rmt_uart_deinit(RMT_UART_NUM_3);
}

void loop() {
  if (!file) {
    if (button_pressed) {
      button_pressed = false;
      if (!openSDCard()) {
        Serial.println("Could not open SD Card");
        rgb.SetPixelColor(0, RgbColor(64, 0, 0));
      } else {
        Serial.print("Started ");
        Serial.println(filename);
        rgb.SetPixelColor(0, RgbColor(0, 64, 0));
        rgb.Show();
        startSerials();
        return;
      }
    } else {
      Serial.println("Awaiting start...");
      rgb.SetPixelColor(0, RgbColor(0, 0, 64));
    }
    rgb.Show();
    delay(1000);
    return;
  }

  readSerial(0, Serial1);
  readSerial(1, Serial2);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  readSerial(2, Serial0);
#endif

  for (int uart = 0 ; uart<NUM_PORTS-RMT_START ; uart++) {
    auto bytes = rmt_uart_read_bytes(RMT_UART_NUM_0 + uart, &buf[uart + RMT_START][bufPos[uart + RMT_START]], 0);
    if (bytes) {
      int start = bufPos[uart+RMT_START];
      int end = start + bytes;
      bool written = false;
      for (int i=start ; i<end ; i++) {
        if (buf[uart+RMT_START][i] == '\n') {
          bufPos[uart+RMT_START] = i;
          writeBuffer(uart+RMT_START);
          bufPos[uart+RMT_START] = end - (i + 1);
          memcpy(buf[uart+RMT_START], &buf[uart+RMT_START][i + 1], bufPos[uart+RMT_START]);
          written = true;
          break;
        }
      }
      if (!written) {
        bufPos[uart+RMT_START] += bytes;
        if (bufPos[uart+RMT_START] == 80) {
          bufPos[uart+RMT_START] = 0;
        }
      }
    }
  }

  if (button_pressed) {
    button_pressed = false;
    stopSerials();
    file.close();
    SD.end();
    Serial.println("Closed SD card");
    delay(1000);
  }
}
