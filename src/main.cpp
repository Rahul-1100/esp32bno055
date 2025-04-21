#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create custom I2C object using peripheral #0
TwoWire Custom_two_wire(0);  

// Initialize BNO055 with I2C object and address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Custom_two_wire);

// Sampling rate delay
int64_t BNO055_SAMPLERATE_DELAY_MS = 16;  // In milliseconds
int64_t BNO055_SAMPLERATE_DELAY_MuS = BNO055_SAMPLERATE_DELAY_MS * 1000;  // In microseconds

void printEvent(sensors_event_t* event);
void GetAndPrintData(void* PvParam);

void setup() {
    Serial.begin(921600);  // Start Serial communication
    while (!Serial) delay(10);  // Wait for Serial port to initialize

    Serial.println("Orientation Sensor Test");
    Serial.flush();  // Ensure message is transmitted before continuing

    int sda_pins[] = {21, 22};
    int scl_pins[] = {22, 21};
    int i = 0;

    // Try different I2C pin configurations until the sensor is found
    while (1) {
        Serial.print("Trying SDA: ");
        Serial.print(sda_pins[i]);
        Serial.print(", SCL: ");
        Serial.println(scl_pins[i]);

        Custom_two_wire.begin(sda_pins[i], scl_pins[i]);
        
        if (bno.begin(OPERATION_MODE_NDOF)) {
            Serial.println("BNO055 detected!");
            break;
        }

        Serial.println("No BNO055 detected. Swapping I2C pins...");
        Serial.flush();
        Custom_two_wire.end();
        i = (i + 1) % 2;  // Swap between 21-22 and 22-21
        delay(200);  // Short delay before retrying
    }

    // Configure axis remap
    Adafruit_BNO055::adafruit_bno055_axis_remap_config_t REMAP_CONFIG_P8 = static_cast<Adafruit_BNO055::adafruit_bno055_axis_remap_config_t>(0x09);
    bno.setAxisRemap(REMAP_CONFIG_P8);
    bno.setAxisSign(bno.REMAP_SIGN_P4);

    // Wait for the sensor to stabilize
    delay(1000);

    // Create a FreeRTOS task pinned to core 0 to fetch and print sensor data
    xTaskCreatePinnedToCore(GetAndPrintData, "Print BNO Data", 20480, NULL, 1, NULL, 0);
}

void loop() {
    // Main loop can remain empty; data is fetched by the FreeRTOS task
}

// Task function to fetch and print data periodically
void GetAndPrintData(void* PvParam) {
    int64_t new_time = esp_timer_get_time();
    int64_t old_time = 0;
    float fps; 
    while (1) {
        new_time = esp_timer_get_time();
        if (new_time - old_time > BNO055_SAMPLERATE_DELAY_MuS) {
            old_time = new_time;

            sensors_event_t angVelocityData, linearAccelData, magnetometerData, gravityData;
            uint8_t system, gyro, accel, mag;
            
            // Fetch calibration status
            bno.getCalibration(&system, &gyro, &accel, &mag);

            // Fetch various sensor readings
            bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
            // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
            bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
            bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

            // Print the data
            printEvent(&angVelocityData);
            // printEvent(&linearAccelData);
            // printEvent(&magnetometerData);
            // printEvent(&gravityData);

            // Print quaternion data
            imu::Quaternion quat = bno.getQuat();
            Serial.print(F("Quat: "));
            Serial.print((float)quat.w(), 4);
            Serial.print(F(", "));
            Serial.print((float)quat.x(), 4);
            Serial.print(F(", "));
            Serial.print((float)quat.y(), 4);
            Serial.print(F(", "));
            Serial.print((float)quat.z(), 4);
            Serial.println(";");

            // Print calibration status
            Serial.print("Cal: ");
            Serial.println(system);
            Serial.flush();  // Ensure all data is transmitted before next iteration

            // Print task execution time
            // Serial.println("-----");
            // Serial.print(">time:");
            // fps = (float)1/((float)(esp_timer_get_time() - old_time)/1000000.0);
            // Serial.println(fps);
            // Serial.flush();  // Ensure message is sent

            // Delay task to match the desired sampling rate
            // vTaskDelay(BNO055_SAMPLERATE_DELAY_MS / portTICK_PERIOD_MS);
        }
        // new_time = esp_timer_get_time();
    }
}

// Helper function to print sensor events
void printEvent(sensors_event_t* event) {
    // double x = -1000000, y = -1000000, z = -1000000;  // Default values
    float x,y,z;
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl: ");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag: ");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro: ");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Lin: ");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Grav: ");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else {
        Serial.print("Unknown: ");
    }

    // Print the values
    Serial.print(x, 4);
    Serial.print(", ");
    Serial.print(y, 4);
    Serial.print(", ");
    Serial.println(z, 4);
    // Serial.flush();  // Ensure the data is transmitted immediately
}
