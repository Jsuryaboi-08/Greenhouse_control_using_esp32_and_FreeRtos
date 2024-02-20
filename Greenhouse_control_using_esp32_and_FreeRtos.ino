#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2

#define DHTTYPE DHT11

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(sensor_data_t)

typedef struct sensor_data {
  float temperature;
  int humidity;
  float averageTemperature; 
  uint32_t timestamp; 
  float averagehumidity;
} sensor_data_t;

uint8_t queue_storage_array[QUEUE_LENGTH * ITEM_SIZE];
uint32_t queue_buffer_size = sizeof(queue_storage_array);
QueueHandle_t sensorDataQueue = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, queue_storage_array, &queue_buffer_size);

void sensorTask(void*pvParameters){
  sensor_data_t data;

  DHT dht(DHT, DHTTYPE);

  while(1){
    float temperature = dht.readTemperature();
    int humidity = dht.readHumidity();

    if(isnan(temperature)||isnan(humidity)){
      Serial.println("error reading sensor data!");
      continue;
    }
    //data processing task i did here to check if the values are in range 
    if(humidity<85 && 24<=temeprature<=29){
    Serial.println("humidity and temperture is not balanced");
    }
    data.temeprature = temperature;
    data.humidity = humidity;

    data.timestamp = millis();

    if(xQueueSend(sensorDataQueue, &data, pdMS_TO_TICKS(1000)) != pdPASS){
      Serial.println("failed to send data to queue!");
    }

    delay(dht.getMinimumInterval());
  }

}

void processingTask(void *pvParameters) {
  sensor_data_t receivedData;
  float averageTemperature = 0.0;
  float averagehumidity=0.0;
  uint32_t count = 0;

  while (1) {

    if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(10000)) == pdPASS) {
      averageTemperature += receivedData.temperature;
      averagehumidity +=receivedData.humidity;
      count++;

      if(millis - lastPrintTime >= 5000){
        if(count>0){
          averageTemperature /=count;
          averagehumidity /=count;
          if(23<=averageTemperature<=29 && 75<=averagehumidity<=90){
            Serial.print("your Gree House is working Great!");
          }else{
            Serial.print("Anamaly!!!");
          }
          Serial.print("Average temerature of the Green house is:");
          Serial.println(averageTemperature);
          Serial.print("Average humidity of the Green house is:");
          Serial.print(averagehumidity);

          averagehumidity=0.0;
          averageTemperature=0.0
          lastPrintTime = millis();

        }

      }else{
        Serial.println(" No Temperature or Humidity Data recieved yet !!!");
      }

      

    }
  }
