#include <Arduino.h>
#include <Joystick_ESP32S2.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <esp_timer.h>
#include <stdint.h>

#define CRSF_UART_NUM UART_NUM_1
#define CRSF_RX_PIN 16
#define CRSF_TX_PIN 17
#define CRSF_BAUDRATE 420000
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_BUFFER_SIZE 512
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

Joystick_ Joystick;

struct __attribute__((packed)) GamepadData {
    uint16_t ch[8];
    uint8_t sw;
};

static QueueHandle_t uart_queue;
static GamepadData gamepad;
static volatile bool frameReady = false;
static uint8_t rxbuf[CRSF_MAX_PACKET_LEN];
static int rxPos = 0;
static uint32_t lastFrameTime = 0;


const uint8_t SWITCH_THRESHOLDS[8] = {
    0x40,   // Switch 1
    0x40,   // Switch 2
    0x40,   // Switch 3
    0x40,   // Switch 4
    0x40,   // Switch 5
    0x40,   // Switch 6
    0x40,   // Switch 7
    0x40    // Switch 8
};



static void processPacket() {
    if (rxbuf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER && 
        rxbuf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        
        gamepad.sw = 0;
        gamepad.ch[0] = ((rxbuf[3] | rxbuf[4] << 8) & 0x07ff) << 5;
        gamepad.ch[1] = ((rxbuf[4] >> 3 | rxbuf[5] << 5) & 0x07ff) << 5;
        gamepad.ch[2] = ((rxbuf[5] >> 6 | rxbuf[6] << 2 | rxbuf[7] << 10) & 0x07ff) << 5;
        gamepad.ch[3] = ((rxbuf[7] >> 1 | rxbuf[8] << 7) & 0x07ff) << 5;
        
        // Decode switches (CH5-CH8)
        for (int i = 4; i < 8; i++) {
            uint16_t switchValue = ((rxbuf[9 + (i-4)*2] >> 7 | rxbuf[10 + (i-4)*2] << 1 | rxbuf[11 + (i-4)*2] << 9) & 0x07ff);
            if (switchValue > (1024 + SWITCH_THRESHOLDS[i])) {
                gamepad.sw |= (1 << (i-4));
            }
        }
        
        frameReady = true;
    }
}

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data;
    
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                while (uart_read_bytes(CRSF_UART_NUM, &data, 1, 0) > 0) {
                    rxbuf[rxPos++] = data;
                    
                    if (rxPos == 1) {
                        if (data != CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                            rxPos = 0;
                        }
                    }
                    else if (rxPos == 2) {
                        if (data > CRSF_MAX_PACKET_LEN) {
                            rxPos = 0;
                        }
                    }
                    else if (rxPos >= 26) {
                        processPacket();
                        rxPos = 0;
                    }
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush(CRSF_UART_NUM);
                xQueueReset(uart_queue);
                rxPos = 0;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB
    };
    
    ESP_ERROR_CHECK(uart_driver_install(CRSF_UART_NUM, CRSF_BUFFER_SIZE, 0, 32, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(CRSF_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_rx_timeout(CRSF_UART_NUM, 1));
    
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL, 1);
    
    Joystick.begin(false);
    Joystick.setXAxisRange(0, 65535);
    Joystick.setYAxisRange(0, 65535);
    Joystick.setZAxisRange(0, 65535);
    Joystick.setRxAxisRange(0, 65535);
    Joystick.setRyAxisRange(0, 65535);
    Joystick.setRzAxisRange(0, 65535);
    
    Serial.println("ESP32-S2 CRSF to USB Joystick started");
}

void loop() {
    if (frameReady && (micros() - lastFrameTime) >= 2000) {
        Joystick.setXAxis(gamepad.ch[0]);      // Aileron (Right stick X)
        Joystick.setYAxis(gamepad.ch[1]);      // Elevator (Right stick Y)
        Joystick.setZAxis(gamepad.ch[2]);      // Throttle (Left stick Y)
        Joystick.setRxAxis(gamepad.ch[3]);     // Rudder (Left stick X)

        // Set digital buttons based on switch values
        for (int i = 0; i < 8; i++) {
            Joystick.setButton(i, gamepad.sw & (1 << i));
        }

        Joystick.sendState();
        
        lastFrameTime = micros();
        frameReady = false;
    }
}
