#include <Arduino.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <../lib/ArduinoSchedule/src/schedule.h>

#define analogPin A1
#define statusLedPin 13 // Use onboard LED
#define relayPin 10

#define RADIOHEAD_BAUD 2000 // Transmission Speed
#define RADIOHEAD_TX_PIN -1  // Pin of the 433MHz transmitter (not used)
#define RADIOHEAD_RX_PIN 5 // Pin of the 433MHz receiver

RH_ASK rf_driver(RADIOHEAD_BAUD, RADIOHEAD_RX_PIN, RADIOHEAD_TX_PIN);

// Schedules
schedule schedule_250_millis = {0, 250};
schedule schedule_1_seconds = {0, 1000};

// Core Vars
const float arduinoInputVoltage = 5; // Voltage input
const float voltagePerPoint = arduinoInputVoltage / 1024; // Convert voltage to vpp
const float ampMetreSensitivity = 0.068; // Sensitivity of the sensor
const int ampTriggerValue = 3; // Amp threshold
const int ampTriggerDuration = 10000; // Minimum duration to trigger mode change
const unsigned int durationBetweenRunOn = 10000; // When running, how long to stay on for, rotates between durationBetweenRunOn & durationBetweenRunOff
const unsigned int durationBetweenRunOff = 20000; // When running, how long to stay off for, rotates between durationBetweenRunOn & durationBetweenRunOff

// Vars
bool isRunning = false; // Do we believe the fridge is running?
bool pendingRunningModeChange = false; // Are we detecting a change in running mode?
int ampThresholdBreach = false; // Has the threshold breached?
unsigned long ampTriggerThresholdMillis = 0; // When did the threhold breach?
bool relayState = false; // Status of relay
unsigned long currentRunModeDuration = 0; // Duration since last run mode change

// Volts / Amps
float volts;
float normalisedVolts;
float amps;

void setup()
{
    // Setup mode input
    pinMode(analogPin, INPUT);
    pinMode(statusLedPin, OUTPUT);
    pinMode(relayPin, OUTPUT);

    // Debug
    Serial.begin(9600);

    // RF Init
    rf_driver.init();
}

void relayClosed()
{
    digitalWrite(relayPin, LOW);
    relayState = false;
    currentRunModeDuration = millis();

    // This is crap, there is other ways but for simplistic
    // This will cause a delay for the whole thread.
    //
    // We have to give the relay time to switch the current back on otherwise the 
    // reading will be taken before the relay has switched, causing a threshold breach.
    delay(500);

    Serial.println("RELAY CLOSED");
}

void relayOpen()
{
    digitalWrite(relayPin, HIGH);
    relayState = true;
    currentRunModeDuration = millis();

    Serial.println("RELAY OPEN");
}

void runModeEnable()
{
    isRunning = true;
    digitalWrite(statusLedPin, HIGH);
    currentRunModeDuration = millis();
    
    Serial.println("Running True");
}

void runModeDisable()
{
    isRunning = false;
    digitalWrite(statusLedPin, LOW);
    relayClosed(); // Ensure we return the relay to default position, allowing fridge to operate
    currentRunModeDuration = 0;

    Serial.println("Running False");
}

void schedule_250_millis_run()
{
    // Read input
    int readingPoint = analogRead(analogPin);

    // Calc volts / amps
    volts = readingPoint * voltagePerPoint;
    normalisedVolts = volts -= (arduinoInputVoltage / 2);
    amps = normalisedVolts / ampMetreSensitivity;

    // Only allow threshold changes to occur if it was not the relaying cutting the power
    // Otherwise cuttin the relay to cut the power to the fridge would in it self cause the 
    // threshold to be triggered
    if (!isRunning || (isRunning && !relayState)) {
        // There has been a change in threshold
        if ((amps >= ampTriggerValue && !ampThresholdBreach) || (amps < ampTriggerValue && ampThresholdBreach)) {
            ampTriggerThresholdMillis = millis();
            pendingRunningModeChange = true;
            Serial.println("THRESHOLD CHANGE");
        }
    }

    // Swap between run modes
    if (pendingRunningModeChange) {
        // Rapid satus LED for running mode change
        digitalWrite(statusLedPin, !digitalRead(statusLedPin));
        ampThresholdBreach = amps >= ampTriggerValue;

        if (millis() - ampTriggerThresholdMillis > ampTriggerDuration) {
            pendingRunningModeChange = false;
            Serial.println("RUN CHANGE MODE");

            if (isRunning && amps < ampTriggerValue) {
                runModeDisable();
            } else if(!isRunning && amps >= ampTriggerValue) {
                runModeEnable();
            }
        }
    }

    // Relay Control
    if (isRunning) {
        if (relayState && millis() - currentRunModeDuration > durationBetweenRunOff) {
            relayClosed();
        } else if (!relayState && millis() - currentRunModeDuration > durationBetweenRunOn) {
            relayOpen();
        }
    }
}

void schedule_1_second_run()
{
    // Debug
    Serial.println("Volts: " + String(volts));
    Serial.println("Amps: " + String(amps));

    // Steady flash for operating
    if (isRunning && !pendingRunningModeChange) {
        digitalWrite(statusLedPin, !digitalRead(statusLedPin));
    }
}

void loop()
{
    // 250 millis Schedule
    if (scheduleCheck(&schedule_250_millis)) {
        schedule_250_millis_run();
        scheduleRun(&schedule_250_millis);
    }

    // 1 Second Schedule
    if (scheduleCheck(&schedule_1_seconds)) {
        schedule_1_second_run();
        scheduleRun(&schedule_1_seconds);
    }

    // RF Buffer Messages
    uint8_t buf[99];
    uint8_t buflen = sizeof(buf);

    if (rf_driver.recv(buf, &buflen)) {
        Serial.print("Message: ");
        Serial.print((char*)buf);
    }

}