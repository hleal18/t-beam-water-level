#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>
#include <WiFi.h>
#include <MPU9255.h>
#include "esp_sleep.h"
#include "config.h"

#include "Message/Message.h"
#include "Message/MessageBuilder.h"
#include "Message/MessageBuilderManager.h"

// Configurar pin análogo donde el sensor está conectado.
const int water_level_sensor_pin = 35;
// Máxima cantidad de profundidad calibrada en sensor en CM (centímetros).
const int profundidad_calibrada = 18;

// Máximo voltaje V obtenido a la profundidad calibrada (no cambiar).
// Resolución 12 bits para ESP32 - 4095 ADC para 3.3v, queda en 3750 para un máximo de 3v.
const int max_v = 3750;
MessageBuilderManager msg_manager;

void os_getArtEui(u1_t *buf)
{
}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;
void do_send(osjob_t *j);
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
unsigned TX_INTERVAL = 900;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN, // was "14,"
    .dio = {26, 33, 32},
};

void enable_sleep()
{ //TODO implement  UBX-ACK
    do
    { //We cannot read UBX ack therefore try to sleep gps until it does not send data anymore
        Serial.println("try to sleep gps!");
        byte CFG_RST[12] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0F, 0x66};
        delay(600);                                                                                                                  //give some time to restart //TODO wait for ack
        const byte RXM_PMREQ[16] = {0xb5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x3b}; //power off until wakeup call
        Serial1.write(RXM_PMREQ, sizeof(RXM_PMREQ));
        unsigned long startTime = millis();
        unsigned long offTime = 1;
        Serial.println(offTime);

        while (millis() - startTime < 1000)
        { //wait for the last command to finish
            int c = Serial1.read();
            if (offTime == 1 && c == -1)
            { //check  if empty
                offTime = millis();
            }
            else if (c != -1)
            {
                offTime = 1;
            }
            if (offTime != 1 && millis() - offTime > 100)
            { //if gps chip does not send any commands for .. seconds it is sleeping
                Serial.println("sleeping gps!");
                return;
            }
        }
    } while (1);
}

void onEvent(ev_t ev)
{
    switch (ev)
    {

    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        digitalWrite(BUILTIN_LED, LOW);
        // Schedule next transmission
        esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
        esp_deep_sleep_start();

        //ESP.restart();
        //delay(TX_INTERVAL * 1000);
        do_send(&sendjob);
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void do_send(osjob_t *j)
{
    int sensor_value = analogRead(water_level_sensor_pin);
    Serial.print("ADC leído: ");
    Serial.print(sensor_value);
    Serial.print(" Voltaje leído: ");
    Serial.print(sensor_value * 3.3 / 4095);
    Serial.print(" Cantidad de agua: ");
    // Se calcula la cantidad de agua en base al parámetro calibrado
    int value = (sensor_value * profundidad_calibrada) / max_v;
    Serial.print(value);
    Serial.println(" cm");
    Message msg = msg_manager.createWaterLevelMessage(1, WATER_LEVEL, value);
    uint8_t *buffer = msg.getMessageArray();
    int buffer_size = msg.getMessageArraySize();

    LMIC_setTxData2(1, buffer, buffer_size, 0);
    Serial.println(F("Packet queued"));

    delete[] buffer;
}

void setup()
{
    Serial.begin(9600);

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();

    // Puts GPS to sleep.
    Serial1.begin(9600, SERIAL_8N1, 12, 15);
    enable_sleep();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    LMIC_selectSubBand(1);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    do_send(&sendjob);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
    os_runloop_once();
}