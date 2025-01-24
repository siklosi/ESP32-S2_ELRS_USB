#include <Arduino.h>
#include <Joystick_ESP32S2.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <esp_timer.h>
#include <stdint.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "USB.h"
#include "USBHIDKeyboard.h"
USBHIDKeyboard Keyboard;

#define CRSF_UART_NUM UART_NUM_1
#define CRSF_RX_PIN 16
#define CRSF_TX_PIN 17
#define CRSF_BAUDRATE 420000
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_BUFFER_SIZE 512
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

Joystick_ Joystick;
AsyncWebServer server(80);


// Configuration structure
struct JoystickConfig {
    String assignments[16];
};

JoystickConfig currentConfig;
const char* configFile = "/joystick_config.json";

struct __attribute__((packed)) GamepadData {
    uint16_t ch[16];  
    uint8_t sw;
};

static QueueHandle_t uart_queue;
static GamepadData gamepad;
static volatile bool frameReady = false;
static uint8_t rxbuf[CRSF_MAX_PACKET_LEN];
static int rxPos = 0;
static uint32_t lastFrameTime = 0;

// Default configuration
const JoystickConfig defaultConfig = {
    {"XAxis", "YAxis", "ZAxis", "RxAxis", "None", "None", "None", "None", 
     "None", "None", "None", "None", "None", "None", "None", "None"}
};

// Load configuration from SPIFFS
void loadConfig() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        currentConfig = defaultConfig;
        return;
    }

    File configFile = SPIFFS.open("/joystick_config.json", "r");
    if (!configFile) {
        Serial.println("No config file, using default");
        currentConfig = defaultConfig;
        return;
    }

    // StaticJsonDocument<512> doc;
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        Serial.println("Failed to parse config file, using default");
        currentConfig = defaultConfig;
        return;
    }

    for (int i = 0; i < 16; i++) {
        currentConfig.assignments[i] = doc[String("channel-" + String(i))].as<String>();
    }
}

// Save configuration to SPIFFS
bool saveConfig(const JoystickConfig& config) {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return false;
    }

    File configFile = SPIFFS.open("/joystick_config.json", "w");
    if (!configFile) {
        Serial.println("Failed to open config file for writing");
        return false;
    }

    // StaticJsonDocument<512> doc;
    JsonDocument doc;
    for (int i = 0; i < 16; i++) {
        doc[String("channel-" + String(i))] = config.assignments[i];
    }

    if (serializeJson(doc, configFile) == 0) {
        Serial.println("Failed to write to config file");
        configFile.close();
        return false;
    }

    configFile.close();
    return true;
}


// void mapChannelToJoystick() {
//     for (int i = 0; i < 16; i++) {
//         String assignment = currentConfig.assignments[i];
        
//         if (assignment == "XAxis") Joystick.setXAxis(gamepad.ch[i]);
//         else if (assignment == "YAxis") Joystick.setYAxis(gamepad.ch[i]);
//         else if (assignment == "ZAxis") Joystick.setZAxis(gamepad.ch[i]);
//         else if (assignment == "RxAxis") Joystick.setRxAxis(gamepad.ch[i]);
//         else if (assignment == "RyAxis") Joystick.setRyAxis(gamepad.ch[i]);
//         else if (assignment == "RzAxis") Joystick.setRzAxis(gamepad.ch[i]);
//         else if (assignment == "Throttle") Joystick.setThrottle(gamepad.ch[i]);
//         else if (assignment == "Rudder") Joystick.setRudder(gamepad.ch[i]);
//         else if (assignment.startsWith("Button")) {
//             int buttonNum = assignment.substring(7).toInt();
//             if (buttonNum >= 1 && buttonNum <= 32) {
//                 bool buttonState = gamepad.ch[i] > 52428 || gamepad.ch[i] < 13107;
//                 Joystick.setButton(buttonNum-1, buttonState);
//             }
//         }
//     }
// }

// Map channel assignments to joystick axes/buttons or keyboard
void mapChannelToJoystick() {
    for (int i = 0; i < 16; i++) {
        String assignment = currentConfig.assignments[i];
        
        if (assignment == "XAxis") Joystick.setXAxis(gamepad.ch[i]);
        else if (assignment == "YAxis") Joystick.setYAxis(gamepad.ch[i]);
        else if (assignment == "ZAxis") Joystick.setZAxis(gamepad.ch[i]);
        else if (assignment == "RxAxis") Joystick.setRxAxis(gamepad.ch[i]);
        else if (assignment == "RyAxis") Joystick.setRyAxis(gamepad.ch[i]);
        else if (assignment == "RzAxis") Joystick.setRzAxis(gamepad.ch[i]);
        else if (assignment == "Throttle") Joystick.setThrottle(gamepad.ch[i]);
        else if (assignment == "Rudder") Joystick.setRudder(gamepad.ch[i]);
        else if (assignment.startsWith("Button")) {
            int buttonNum = assignment.substring(7).toInt();
            if (buttonNum >= 1 && buttonNum <= 32) {
                bool buttonState = gamepad.ch[i] > 52428 || gamepad.ch[i] < 13107;
                Joystick.setButton(buttonNum - 1, buttonState);
            }
        }
        else if (assignment.startsWith("Key")) {
            bool currentButtonState = gamepad.ch[i] > 52428 || gamepad.ch[i] < 13107;
            static bool lastButtonState[16] = {false};
            
            if (currentButtonState && !lastButtonState[i]) {
                char key = assignment.charAt(4);
                Keyboard.press(key);
                delay(10);
                Keyboard.release(key);
            }
            
            lastButtonState[i] = currentButtonState;
        }
    }
}


static void processPacket() {
    if (rxbuf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER && 
        rxbuf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        
        // Decode all 16 channels
        gamepad.ch[0]  = ((rxbuf[3]  | rxbuf[4]  << 8) & 0x07FF);
        gamepad.ch[1]  = ((rxbuf[4]  >> 3 | rxbuf[5]  << 5) & 0x07FF);
        gamepad.ch[2]  = ((rxbuf[5]  >> 6 | rxbuf[6]  << 2 | rxbuf[7]  << 10) & 0x07FF);
        gamepad.ch[3]  = ((rxbuf[7]  >> 1 | rxbuf[8]  << 7) & 0x07FF);
        gamepad.ch[4]  = ((rxbuf[8]  >> 4 | rxbuf[9]  << 4) & 0x07FF);
        gamepad.ch[5]  = ((rxbuf[9]  >> 7 | rxbuf[10] << 1 | rxbuf[11] << 9) & 0x07FF);
        gamepad.ch[6]  = ((rxbuf[11] >> 2 | rxbuf[12] << 6) & 0x07FF);
        gamepad.ch[7]  = ((rxbuf[12] >> 5 | rxbuf[13] << 3) & 0x07FF);
        gamepad.ch[8]  = ((rxbuf[14] | rxbuf[15] << 8) & 0x07FF);
        gamepad.ch[9]  = ((rxbuf[15] >> 3 | rxbuf[16] << 5) & 0x07FF);
        gamepad.ch[10] = ((rxbuf[16] >> 6 | rxbuf[17] << 2 | rxbuf[18] << 10) & 0x07FF);
        gamepad.ch[11] = ((rxbuf[18] >> 1 | rxbuf[19] << 7) & 0x07FF);
        gamepad.ch[12] = ((rxbuf[19] >> 4 | rxbuf[20] << 4) & 0x07FF);
        gamepad.ch[13] = ((rxbuf[20] >> 7 | rxbuf[21] << 1 | rxbuf[22] << 9) & 0x07FF);
        gamepad.ch[14] = ((rxbuf[22] >> 2 | rxbuf[23] << 6) & 0x07FF);
        gamepad.ch[15] = ((rxbuf[23] >> 5 | rxbuf[24] << 3) & 0x07FF);
        
        // Scale channels to 0-65535 range
        for (int i = 0; i < 16; i++) {
            gamepad.ch[i] = map(gamepad.ch[i], 0, 2047, 0, 65535);
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
    IPAddress local_ip(10, 0, 0, 1);
    IPAddress gateway(10, 0, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAP("CRSF Joystick Config", "conf1234");
    WiFi.softAPConfig(local_ip, gateway, subnet);
    // Ensure SPIFFS is mounted
    if(!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    // Web Server Endpoints
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        // StaticJsonDocument<512> doc;
        JsonDocument doc;
        
        for (int i = 0; i < 16; i++) {
            doc[String("channel-" + String(i))] = currentConfig.assignments[i];
        }
        
        serializeJson(doc, *response);
        request->send(response);
    });

    server.on("/save-config", HTTP_POST, [](AsyncWebServerRequest *request) {
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, (char*)data);
        
        if (!error) {
            for (int i = 0; i < 16; i++) {
                currentConfig.assignments[i] = doc[String("channel-" + String(i))].as<String>();
            }
            
            if (saveConfig(currentConfig)) {
                request->send(200, "application/json", "{\"success\":true}");
            } else {
                request->send(500, "application/json", "{\"success\":false}");
            }
        } else {
            request->send(400, "application/json", "{\"success\":false}");
        }
    });

    server.begin();

    // UART configuration
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
    
    // Load configuration
    loadConfig();
    // Keyboard initialization
    Keyboard.begin();
    // Joystick initialization
    Joystick.begin(false);
    Joystick.setXAxisRange(0, 65535);
    Joystick.setYAxisRange(0, 65535);
    Joystick.setZAxisRange(0, 65535);
    Joystick.setRxAxisRange(0, 65535);
    Joystick.setRyAxisRange(0, 65535);
    Joystick.setRzAxisRange(0, 65535);
    Joystick.setThrottleRange(0, 65535);
    Joystick.setRudderRange(0, 65535);

    Serial.println("ESP32-S2 CRSF to USB Joystick started");
}

void loop() {
    if (frameReady && (micros() - lastFrameTime) >= 2000) {
        mapChannelToJoystick();
        Joystick.sendState();
        
        lastFrameTime = micros();
        frameReady = false;
    }
}
