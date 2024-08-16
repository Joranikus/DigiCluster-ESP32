#include <Arduino.h>

//////////////////// SETTINGS ////////////////////
int dimmer_low_range = 2700;
int dimmer_high_range = 3300;
bool test = false;
//////////////////////////////////////////////////

const int timeout = 5000;

enum Pin {
    RPM_PIN = 13,
    DIMMER_PIN = 12,
    LOW_OIL_PRESSURE_PIN = 33,
    TURN_SIGNAL_PIN = 14,
    PARKING_LIGHT_PIN = 27,
    HIGH_BEAM_PIN = 26,
    LOW_BEAM_PIN = 25,
    VOLTAGE_PIN = 39,
    FUEL_LEVEL_PIN = 35,
    COOLANT_TEMP_PIN = 34
};

//Sensor Class

class Sensor {
protected:
    String name;
    unsigned long update_interval;
    unsigned long last_update_time;
    float value;

public:
    Sensor(const String& sensor_name, unsigned long interval)
            : name(sensor_name), update_interval(interval), last_update_time(0), value(0.0) {}

    virtual void update() = 0;
    float getValue() const { return value; }
};

//Analog Sensor Class

class AnalogSensor : public Sensor {
private:
    int pin;
    int min_range;
    int max_range;
    int min_value;
    int max_value;

public:
    AnalogSensor(const String& sensor_name, int sensor_pin, unsigned long interval, int min_r, int max_r, int min_v, int max_v)
            : Sensor(sensor_name, interval), pin(sensor_pin), min_range(min_r), max_range(max_r), min_value(min_v), max_value(max_v) {}

    void update() override {
        unsigned long current_time = millis();
        if (current_time - last_update_time >= update_interval) {
            value = map(analogRead(pin), min_range, max_range, min_value, max_value);
            last_update_time = current_time;
        }
    }
};

//Digital Sensor Clas

class DigitalSensor : public Sensor {
private:
    int pin;
    bool inverted;

public:
    DigitalSensor(const String& sensor_name, int sensor_pin, unsigned long interval, bool inv = false)
            : Sensor(sensor_name, interval), pin(sensor_pin), inverted(inv) {}

    void update() override {
        unsigned long current_time = millis();
        if (current_time - last_update_time >= update_interval) {
            value = digitalRead(pin);
            if (inverted) value = !value;
            last_update_time = current_time;
        }
    }
};

//RPM Sensor Class

class RPM_Sensor : public Sensor {
private:
    volatile unsigned long last_time;
    volatile unsigned long delta_time;
    volatile float frequency;

    static void IRAM_ATTR isrWrapper() {
        RPM_Sensor::getInstance().isr();
    }

    RPM_Sensor() : Sensor("RPM", 100), last_time(0), delta_time(0), frequency(0) {
        attachInterrupt(digitalPinToInterrupt(Pin::RPM_PIN), isrWrapper, RISING);
    }

    void isr() {
        unsigned long current_time = micros();
        delta_time = current_time - last_time;
        last_time = current_time;
        if (delta_time != 0) {
            frequency = 1000000.0 / delta_time;
        }
    }

public:
    static RPM_Sensor& getInstance() {
        static RPM_Sensor instance;
        return instance;
    }

    void update() override {
        value = frequency * 30;
    }

    short getValue() const {
        return static_cast<short>(value);
    }
};

//Voltage Sensor Class

class VoltageSensor : public AnalogSensor {
public:
    VoltageSensor() : AnalogSensor("Voltage", Pin::VOLTAGE_PIN, 200, 0, 4095, 0, 100) {}

    void update() override {
        unsigned long current_time = millis();
        if (current_time - last_update_time >= update_interval) {
            value = floor((analogRead(Pin::VOLTAGE_PIN) * ((13000 + 3900) / 3900)) * 10);
            last_update_time = current_time;
        }
    }
};

//Coolant Temp Sensor Class

class CoolantTempSensor : public AnalogSensor {
public:
    CoolantTempSensor() : AnalogSensor("Coolant Temp", Pin::COOLANT_TEMP_PIN, 20000, 0, 4095, -40, 150) {}

    void update() override {
        unsigned long current_time = millis();
        if (current_time - last_update_time >= update_interval) {
            float vout = (3.3 / 4096) * analogRead(Pin::COOLANT_TEMP_PIN);
            float R2 = (vout * 820) / (5 - vout);
            value = -81.2439 + (233789 - (-81.2439)) / (1 + pow((R2 / 1.55134e-14), 0.19926));
            last_update_time = current_time;
        }
    }
};

void setup() {
    Serial.begin(115200);

    pinMode(Pin::RPM_PIN, INPUT_PULLDOWN);
    pinMode(Pin::DIMMER_PIN, INPUT_PULLDOWN);
    pinMode(Pin::LOW_OIL_PRESSURE_PIN, INPUT_PULLDOWN);
    pinMode(Pin::TURN_SIGNAL_PIN, INPUT_PULLDOWN);
    pinMode(Pin::PARKING_LIGHT_PIN, INPUT_PULLDOWN);
    pinMode(Pin::HIGH_BEAM_PIN, INPUT_PULLDOWN);
    pinMode(Pin::LOW_BEAM_PIN, INPUT_PULLDOWN);
    pinMode(Pin::VOLTAGE_PIN, INPUT_PULLDOWN);
    pinMode(Pin::FUEL_LEVEL_PIN, INPUT_PULLDOWN);
    pinMode(Pin::COOLANT_TEMP_PIN, INPUT_PULLDOWN);
}

RPM_Sensor& rpm_sensor = RPM_Sensor::getInstance();
AnalogSensor dimmer_sensor("Dimmer", Pin::DIMMER_PIN, 200, dimmer_low_range, dimmer_high_range, 1, 4);
DigitalSensor oil_pressure_sensor("Low Oil Pressure", Pin::LOW_OIL_PRESSURE_PIN, 100);
DigitalSensor turn_signal_sensor("Turn Signal", Pin::TURN_SIGNAL_PIN, 100, true);
DigitalSensor parking_light_sensor("Parking Light", Pin::PARKING_LIGHT_PIN, 100);
DigitalSensor low_beam_sensor("Low Beam", Pin::LOW_BEAM_PIN, 100);
DigitalSensor high_beam_sensor("High Beam", Pin::HIGH_BEAM_PIN, 100);
VoltageSensor voltage_sensor;
CoolantTempSensor coolant_temp_sensor;
AnalogSensor fuel_level_sensor("Fuel Level", Pin::FUEL_LEVEL_PIN, 20000, 1621, 288, 0, 100);

byte status_byte = 0x00;
unsigned long data_last_time = 0;
bool should_print = false;
byte data_from_rpi;

void loop() {
    rpm_sensor.update();
    dimmer_sensor.update();
    oil_pressure_sensor.update();
    turn_signal_sensor.update();
    parking_light_sensor.update();
    low_beam_sensor.update();
    high_beam_sensor.update();
    voltage_sensor.update();
    coolant_temp_sensor.update();
    fuel_level_sensor.update();

    if (test) {
        Serial.print("RPM: "); Serial.print(rpm_sensor.getValue());
        Serial.print(" | DIMMER: "); Serial.print(dimmer_sensor.getValue());
        Serial.print(" | OIL_PS: "); Serial.print(oil_pressure_sensor.getValue());
        Serial.print(" | TURN SIG: "); Serial.print(turn_signal_sensor.getValue());
        Serial.print(" | PARK LIGHT: "); Serial.print(parking_light_sensor.getValue());
        Serial.print(" | LOW BEAM: "); Serial.print(low_beam_sensor.getValue());
        Serial.print(" | HIGH BEAM: "); Serial.print(high_beam_sensor.getValue());
        Serial.print(" | FUEL LEVEL: "); Serial.print(fuel_level_sensor.getValue());
        Serial.print(" | COOLANT TEMP: "); Serial.print(coolant_temp_sensor.getValue());
        Serial.print(" | VOLTAGE: "); Serial.println(voltage_sensor.getValue());
    }

    if (Serial.available() > 0) {
        data_from_rpi = Serial.read();
        if (data_from_rpi == 1) {
            data_last_time = millis();
            should_print = true;
        }
    }

    unsigned long data_current_time = millis();
    unsigned long data_delta_time = data_current_time - data_last_time;

    if (should_print && data_delta_time <= timeout) {
        Serial.write(static_cast<byte>(rpm_sensor.getValue() >> 8));
        Serial.write(static_cast<byte>(rpm_sensor.getValue()));
        Serial.write(static_cast<byte>(dimmer_sensor.getValue()));
        Serial.write(static_cast<byte>(fuel_level_sensor.getValue()));
        Serial.write(static_cast<byte>(coolant_temp_sensor.getValue()));
        Serial.write(static_cast<byte>(voltage_sensor.getValue()));

        bitWrite(status_byte, 0, oil_pressure_sensor.getValue());
        bitWrite(status_byte, 1, turn_signal_sensor.getValue());
        bitWrite(status_byte, 2, parking_light_sensor.getValue());
        bitWrite(status_byte, 3, low_beam_sensor.getValue());
        bitWrite(status_byte, 4, high_beam_sensor.getValue());
        Serial.write(status_byte);
    } else {
        should_print = false;
    }

    delay(20);
}
