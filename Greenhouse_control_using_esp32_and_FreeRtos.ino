#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2 // DHT sensor pin
#define DHTTYPE DHT11 //Type of DHT sensor I used
#define PRINTING_INTERVAL 5000 // Milliseconds
#define SENSOR_SAMPLING_DELAY 2000 // Milliseconds

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(sensor_data_t)

typedef struct sensor_data {
    float temperature;
    int humidity;
    float averageTemperature;
    uint32_t timestamp;
    float averagehumidity;
} sensor_data_t;

uint8_t queue_storage_array[QUEUE_LENGTH * ITEM_SIZE * 2];
StaticQueue_t queue_buffer;
QueueHandle_t sensorDataQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, queue_storage_array, &queue_buffer);

void sensorTask(void* pvParameters) {
    sensor_data_t data;
    DHT dht(DHTPIN, DHTTYPE);

    while (1) {
        float temperature = dht.readTemperature();
        int humidity = dht.readHumidity();

        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Error reading sensor data!");
            continue;
        }

        data.temperature = temperature;
        data.humidity = humidity;
        data.timestamp = millis();

        if (xQueueSend(sensorDataQueue, &data, pdMS_TO_TICKS(1000)) != pdPASS) {
            Serial.println("Failed to send data to queue!");
        }

        delay(SENSOR_SAMPLING_DELAY);
    }
}
//testing github actions
void processingTask(void *pvParameters) {
    sensor_data_t receivedData;
    float averageTemperature = 0.0;
    float averagehumidity = 0.0;
    uint32_t count = 0;
    uint32_t lastPrintTime = millis();
    const uint8_t HUMIDIFIER_PIN = 3; // Humdifier pin 

    pinMode(HUMIDIFIER_PIN, OUTPUT);

    while (1) {
        if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(10000)) == pdPASS) {
            averageTemperature += receivedData.temperature;
            averagehumidity += receivedData.humidity;
            count++;

            if (millis() - lastPrintTime >= PRINTING_INTERVAL) {
                if (count > 0) {
                    averageTemperature /= count;
                    averagehumidity /= count;

                    // Controlling  the humidifier based on average humidity
                    if (averagehumidity < 75) {
                        digitalWrite(HUMIDIFIER_PIN, HIGH); // Turning on the humidifier if humidity is below 75%
                    } else {
                        digitalWrite(HUMIDIFIER_PIN, LOW); // Turning off the humidifier if humidity is above or equal to 75%
                    }

                    
                    Serial.print("Average temperature of the greenhouse is: ");
                    Serial.println(averageTemperature);
                    Serial.print("Average humidity of the greenhouse is: ");
                    Serial.println(averagehumidity);
                } else {
                    Serial.println("No valid temperature or humidity data received yet.");
                }

                
                averageTemperature = 0.0;
                averagehumidity = 0.0;
                count = 0;
                lastPrintTime = millis();
            }
        }
    }
}

void setup() {
    Serial.begin(115200); 

    
    xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(processingTask, "Processing Task", 1024, NULL, 2, NULL, 0);
}
void loop(){
  
}