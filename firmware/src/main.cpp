/*
Author: Nikolas Zingraf

Description: Firmware for the motorized volume knob with haptic feedback and BLE communication.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>
#include "driver/timer.h"

#define TIMER_DIVIDER         80    // Hardware timer clock divider (80MHz / 80 = 1MHz)
#define TIMER_SCALE           1000  // Convert to milliseconds (1MHz / 1000 = 1ms resolution)
#define TIMER_INTERVAL_MS     50   // 50ms timer interval

#define SCL 9
#define SDA 8

#define BUTTON_PIN 10

#define LED_PIN 7
#define NUM_LEDS 12
#define MAX_BRIGHTNESS 80

#define MAX_ANGLE 4.25f
#define MIN_ANGLE -0.5f

#define NUM_DETENTS_IN_LIMITS 100

#define ANGLE_EPSILON 0.005f

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RX_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Receive from phone/PC
#define TX_CHAR_UUID        "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // Send to phone/PC


BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

const char* BT_DEVICE_NAME = "VolumeKnob";

String rxBuffer = "";

// I2C sensor instance
TwoWire *sensor = new TwoWire(0);
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(0, 1, 2, 3);

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Timer handle
hw_timer_t *timer = NULL;

const uint16_t debounce_delay = 500;
volatile unsigned long last_button_time = 0;
volatile bool button_pressed = false;

bool connected = false;

float targetAngle = MAX_ANGLE;
bool angle_control = true;
bool find_vol_attractor = false;
bool startup = true;

volatile bool timer_flag = false;
volatile unsigned long timer_counter = 0;

int16_t breathing_brightness = 0;
bool breathing_direction = false;
const uint8_t breathing_step = 10;

uint8_t prev_vol = 0;
bool prev_muted = false;
bool is_muted = false;
bool over_endstop = false;

// https://community.simplefoc.com/t/haptic-textures/301/3

PIDController P_haptic{12, 0, 0, 100000, 8};   // P=12, I=0, D=0, ramp=100000 (no ramp), limit=8

//calculate how many detents are needed
const float angle_range = MAX_ANGLE - MIN_ANGLE;  // Total angle range in radians
const float full_revolution = 2 * PI;  // Full revolution in radians
const uint8_t num_detents_per_revolution = round(NUM_DETENTS_IN_LIMITS * (full_revolution / angle_range));
const float detent_angle = full_revolution / num_detents_per_revolution;  // Radians between detents
const float attractor_distance = detent_angle;  // Same as detent_angle for uniform spacing

float attract_angle = MIN_ANGLE;


uint8_t get_volume();
void send_volume(uint8_t vol);


// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

float volume_to_angle(uint8_t volume) {
    return map(volume, 0, 100, MAX_ANGLE * 100, MIN_ANGLE * 100) / 100.0f;
}

// BLE RX Characteristic Callback
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            rxBuffer = "";
            for (int i = 0; i < rxValue.length(); i++) {
                rxBuffer += rxValue[i];
            }
            rxBuffer.trim();
            
            if(rxBuffer.startsWith("mute")) {
                is_muted = true;
            } else if(rxBuffer.startsWith("unmute")) {
                send_volume(get_volume());
                is_muted = false;
            } else if(rxBuffer.startsWith("vol:")) {
                String volStr = rxBuffer.substring(4);
                int newVolume = volStr.toInt();
                targetAngle = volume_to_angle((uint8_t)newVolume);
                angle_control = true;
                find_vol_attractor = false;
                timer_counter = 0;
            }
        }
    }
};

// Timer ISR
void IRAM_ATTR onTimer() {
    timer_counter++;
    timer_flag = true;
}


void setLEDs(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void button_isr() {
    unsigned long current_time = millis();
    // Check if enough time has passed since last interrupt
    if (current_time - last_button_time > debounce_delay) {
        button_pressed = true;
        last_button_time = current_time;
    }
}

uint8_t get_volume() {
    float angle = motor.shaft_angle;
    if(angle < MIN_ANGLE) {
        angle = MIN_ANGLE;
    }else if (angle > MAX_ANGLE) {
        angle = MAX_ANGLE;
    }
    angle = angle * 100;
    uint8_t volume = map(angle, MIN_ANGLE * 100, MAX_ANGLE * 100, 100, 1);  // when volume goes to 0 windows mutes it
    return volume;
 }

void send_volume(uint8_t vol) {
    if(deviceConnected && pTxCharacteristic != NULL) {
        String volStr = String(vol);
        pTxCharacteristic->setValue(volStr.c_str());
        pTxCharacteristic->notify();
    }
}

void send_string(String str) {
    if(deviceConnected && pTxCharacteristic != NULL) {
        pTxCharacteristic->setValue(str.c_str());
        pTxCharacteristic->notify();
    }
}

void update_LEDs(Adafruit_NeoPixel &strip, uint8_t volume, bool muted) {
    uint32_t colorHue = map(volume, 100, 0, 0, 65535/3);
    if(muted) {
        colorHue = map(volume, 100, 0, 65535*5/6, 65535/3*2); // Red color when muted
    }
    strip.fill(strip.ColorHSV(colorHue, 255, MAX_BRIGHTNESS), 0, NUM_LEDS);
    strip.show();

}

float findAttractor(float current_angle){
    return round(current_angle/attractor_distance)*attractor_distance;
}

void haptic_feedback(float angle) {
    unsigned long start_time = millis();
    float old_angle = motor.shaft_angle;
    while (millis() - start_time < 10) {
        motor.move(angle);
        motor.loopFOC();
    }
    start_time = millis();
    while (millis() - start_time < 10) {
        motor.move(P_haptic(old_angle));
        motor.loopFOC();
    }

}


void change_to_torque_control(){
    motor.controller = MotionControlType::torque;
    motor.PID_velocity.P = DEF_PID_CURR_P;
    motor.PID_velocity.I = DEF_PID_CURR_I;
    motor.PID_velocity.D = DEF_PID_CURR_D;
    motor.LPF_velocity.Tf = DEF_VEL_FILTER_Tf;
    motor.P_angle.P = DEF_P_ANGLE_P;
    motor.LPF_angle.Tf = 0.0;
    motor.voltage_limit = 8.0;   // [V]
}

void change_to_angle_control(){
    motor.controller = MotionControlType::angle;
    motor.PID_velocity.P = 0.6;
    motor.PID_velocity.I = 1.5;
    motor.PID_velocity.D = 0.0;
    motor.LPF_velocity.Tf = 0.01;
    motor.P_angle.P = 10.0;
    motor.LPF_angle.Tf = 0.01;
    motor.voltage_limit = 3.0;   // [V]
}


void setup() {

    Serial.begin(115200);

    // Initialize BLE
    BLEDevice::init(BT_DEVICE_NAME);
    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create BLE Characteristics
    pTxCharacteristic = pService->createCharacteristic(
                        TX_CHAR_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pTxCharacteristic->addDescriptor(new BLE2902());
    pRxCharacteristic = pService->createCharacteristic(
                        RX_CHAR_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE device is now advertising");
    Serial.print("Device name: ");
    Serial.println(BT_DEVICE_NAME);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);

    strip.begin();
    strip.fill(strip.Color(0, 0, 0), 0, NUM_LEDS);
    strip.setBrightness(MAX_BRIGHTNESS);
    strip.show();

    timer = timerBegin(0, TIMER_DIVIDER, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, TIMER_INTERVAL_MS * 1000, true);
    timerAlarmEnable(timer);

    sensor->begin(SDA, SCL, 400000); 
    as5600.init(sensor);
    motor.linkSensor(&as5600);

    driver.voltage_power_supply = 8;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.controller = MotionControlType::torque;

    motor.velocity_limit = 10.0; // [rad/s]
    motor.current_limit = 0.11;  // [A] - Reduced to prevent strong bounces

    motor.init();
    motor.initFOC();
}

void loop() {

    motor.loopFOC();

    attract_angle = findAttractor(motor.shaft_angle);

    if ( attract_angle < MIN_ANGLE ) {
        attract_angle = MIN_ANGLE;
        over_endstop = true;
    } else if ( attract_angle > MAX_ANGLE ) {
        attract_angle = MAX_ANGLE;
        over_endstop = true;
    } else {
        over_endstop = false;
    }

    if (startup) {
        change_to_angle_control();
        motor.move(MIN_ANGLE);
        if (motor.shaft_angle >= MIN_ANGLE - ANGLE_EPSILON && motor.shaft_angle <= MIN_ANGLE + ANGLE_EPSILON) {
            startup = false;
            change_to_torque_control();
        }
    }

    if (angle_control) {
        change_to_angle_control();
        motor.move(targetAngle);
        if (motor.shaft_angle >= targetAngle - ANGLE_EPSILON && motor.shaft_angle <= targetAngle + ANGLE_EPSILON) {
            angle_control = false;
            find_vol_attractor = true;
            change_to_torque_control();
            timer_counter = 0;
        }
    } else if (find_vol_attractor) {
        float closest_attract_angle = findAttractor(targetAngle);
        motor.move(P_haptic(closest_attract_angle - motor.shaft_angle));
        if (timer_counter >= 5) {
            find_vol_attractor = false;
        }
    } else {
        motor.move(P_haptic(attract_angle - motor.shaft_angle));
    }


    // Handle disconnection
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    // Handle new connection
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Breathing effect when not connected
    if (!deviceConnected) {
        if (timer_flag) {    
            if (breathing_direction) {
                breathing_brightness -= breathing_step;
                if (breathing_brightness <= 0) {
                    breathing_brightness = 0;
                    breathing_direction = false;
                }
            } else {
                breathing_brightness += breathing_step;
                if (breathing_brightness >= 255) {
                    breathing_brightness = 255;
                    breathing_direction = true;
                }
            }
            strip.fill(strip.Color(0, 0, (uint8_t)breathing_brightness), 0, NUM_LEDS);
            strip.show();
            timer_flag = false;
        }
    }

    if(deviceConnected) {
        if (timer_flag) {       // only execute every 50 ms for better stability
            uint8_t current_vol = get_volume();
            update_LEDs(strip, current_vol, is_muted);
            if (!angle_control) {
                if(prev_vol != current_vol || prev_muted != is_muted) {
                    prev_vol = current_vol;
                    prev_muted = is_muted;
                    if (!is_muted && !find_vol_attractor) {
                        send_volume(current_vol);
                    }
                }
            } 

            if(button_pressed) {
                if(!angle_control && !over_endstop) {    // to compensate EMV interference when motor is moving
                    if (is_muted) {
                        send_string("unmute");
                        send_volume(current_vol);
                        is_muted = false;
                    } else {
                        send_string("mute");
                        is_muted = true;
                    }
                    // haptic_feedback(P_haptic((attract_angle + attractor_distance) - motor.shaft_angle));
                }
                button_pressed = false;
            }
            timer_flag = false;
        }
    }
}
