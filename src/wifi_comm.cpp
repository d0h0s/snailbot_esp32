#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <HTTPClient.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/semphr.h"
#include "esp_sntp.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <MPU6050_Filter.h>
#include "Magnet_Lifter.h"
#include "Differential_Driver.h"
#include "common.h"
#include "gpio.h"

//#define RECORD_QUAT
#define HEART_RATE 10*1000

const char* ssid     = "FreeSN";
const char* password = "freesn666";
const char* ntpServer = "ntp2.aliyun.com"; //aliyun

IPAddress *monitor_address = &INADDR_NONE;
IPAddress *lifter1_node_address = &INADDR_NONE;
IPAddress *lifter2_node_address = &INADDR_NONE;

bool is_ntp_syncd = false;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncUDP udp;
AsyncUDP udp_identify;
HTTPClient http;

TickType_t last_ping_tick = 0;
unsigned long last_send_state_time;
char imu_data_buffer[200];

extern byte MAJOR_CODE, MINOR_CODE;
extern int robot_id;
extern int hardware_code;
uint64_t robot_mac;
uint8_t robot_mac_[14];

extern int lifter1_node_id, lifter2_node_id;
extern bool lifter1_node_connected, lifter2_node_connected;
extern unsigned long lifter1_connected_time, lifter2_connected_time;

extern MagnetLifter lifter1;
extern MagnetLifter lifter2;

extern DifferentialDriver base1_driver; 
extern DifferentialDriver base2_driver;

extern MPU6050_Filter imu;

extern QueueHandle_t xSendKeyQueue;

unsigned long ota_progress_millis = 0;

extern bool cmd_parse(char *cmd);

void onOTAStart() {
//   // Log when OTA has started
//   Serial.println("OTA update started!");
//   // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
//   // Log every 1 second
//   if ((unsigned long)(millis() - ota_progress_millis) > 1000) {
//     ota_progress_millis = millis();
//     Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
//   }
//   // TODO stop control
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
//   if (success) {
//     Serial.println("OTA update finished successfully!");
//   } else {
//     Serial.println("There was an error during OTA update!");
//   }
}



char cmd_copy[120];
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        strcpy(cmd_copy, (char*)data);
        if(!cmd_parse(cmd_copy)){
#ifdef WS_SYNC
          ws.textAll((char*)data);
#endif
        }
        Serial.println((char*)data);
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
        // Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
        case WS_EVT_DISCONNECT:
        // Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
        case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
        case WS_EVT_PONG:
        client->ping();
        break;
        case WS_EVT_ERROR:
        break;
    }
}

void time_sync_notification_cb(struct timeval *tv)
{
    is_ntp_syncd = true;

    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    // Serial.printf("Time synchronization event, the current time is: %s\n", strftime_buf);
}

void network_setup(void)
{
    // Serial.println();
    robot_mac = ESP.getEfuseMac();
    uint8_t* mac = (uint8_t*) &robot_mac;
    robot_mac_[0] = 'B'; // freesn 用了 S/s和N/n，snailbot用不同标识符即可，上位机通过这个做区分
    for (int i=0; i<6; i++) robot_mac_[i+2] = mac[i];
    robot_mac_[8] = (hardware_code << 4) | (MAJOR_CODE & 0x0F); // high 3 bit hardware code, low 5 bit major code
    robot_mac_[9] = MINOR_CODE;
    char macStr[18] = { 0 };
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // Serial.print("MAC address: ");
    // Serial.println(String(macStr));
    // Serial.println();
    // Serial.print("Connecting to ");
    // Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);

    // while (WiFi.status() != WL_CONNECTED) {
    //     vTaskDelay(500 / portTICK_RATE_MS);
    //     Serial.print(".");
    // }

    // Serial.println("");
    // Serial.println("WiFi connected");
    // Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());

    ws.onEvent(onEvent); //init websocket
    server.addHandler(&ws);

    ElegantOTA.begin(&server, "SNAILBOT", "SNAILBOT");    // OTA加密，上位机同步更改，防止不同板子烧错固件
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);
    server.begin();

    // NTP setup
    sntp_servermode_dhcp(1);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, ntpServer);

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_set_sync_interval(60 * 1000);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);

    sntp_init();

    setenv("TZ", "CST-8", 1);
    tzset();

    last_ping_tick = 0;
    last_send_state_time = micros();
}

void network_loop(void)
{

    if ((xTaskGetTickCount() + HEART_RATE/portTICK_RATE_MS - last_ping_tick) * portTICK_RATE_MS > HEART_RATE){
        // notify monitor
        float voltage = (analogRead(V_SENSE)*3.3*3.87)/4095;
        memcpy(robot_mac_+10, &voltage, sizeof(float));
        udp.broadcastTo(robot_mac_, 14, 2255);
        last_ping_tick = xTaskGetTickCount() + HEART_RATE / portTICK_RATE_MS;
    }

    ws.cleanupClients();
    ElegantOTA.loop();

    if (robot_id != -1) {
        unsigned long current_time = micros();
        if ( (unsigned long)(current_time - last_send_state_time) > 100000) { // 100 ms
            sprintf(imu_data_buffer, "{\"id\":\"S%d\",\"rot\":[%.5f,%.5f,%.5f,%.5f],\"alpha\":%.5f,\"lifter\":[%.2f,%.2f]}", 
            robot_id, imu.q[0], imu.q[1], imu.q[2], imu.q[3], imu.get_yaw_angle(), lifter1.get_current_position(), lifter2.get_current_position());
            //Serial.println(imu_data_buffer);
            if (monitor_address != &INADDR_NONE)
                udp.writeTo((byte*)imu_data_buffer, strlen(imu_data_buffer), *monitor_address, 2266);

            //if (monitor_address != &INADDR_NONE)
            last_send_state_time = current_time;
        }
    }

    uint32_t task_key;
    if( xQueueReceive( xSendKeyQueue, &( task_key ), ( TickType_t ) 0 ) )
    { 
      ws.textAll("task_key " + String(task_key));
      Serial.println("task_key " + String(task_key));
    }

}
