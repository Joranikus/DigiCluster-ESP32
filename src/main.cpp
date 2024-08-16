#include <Arduino.h>

//////////////////// SETTINGS ////////////////////

int dimmer_low_range = 2700;
int dimmer_high_range = 3300;
bool test = false;

//////////////////////////////////////////////////

const int RPM_pin = 13;
const int dimmer_pin = 12;
const int low_oil_pressure_pin = 33;
const int turn_signal_pin = 14;
const int parking_light_signal = 27;
const int high_beam_signal = 26;
const int low_beam_signal = 25;
const int voltage_pin = 39;
const int fuel_level = 35;
const int coolant_temp = 34;

volatile unsigned long data_last_time = 0;
volatile unsigned long data_current_time = 0;
volatile unsigned long data_delta_time = 0;
bool should_print = false;
int timeout = 5000;
byte status_byte = 0x00;
byte data_from_rpi;

volatile unsigned long last_time = 0;
volatile unsigned long current_time = 0;
volatile unsigned long delta_time = 0;
volatile float frequency;

void IRAM_ATTR isr() {
    current_time = micros();
    delta_time = current_time - last_time;
    last_time = current_time;
    if (delta_time != 0) {
        frequency = 1000000.0 / delta_time; // Calculate frequency in Hz based on time between pulses
    }
}

void setup() {
    pinMode(RPM_pin, INPUT_PULLDOWN);
    pinMode(dimmer_pin, INPUT_PULLDOWN);
    pinMode(low_oil_pressure_pin, INPUT_PULLDOWN);
    pinMode(turn_signal_pin, INPUT_PULLDOWN);
    pinMode(parking_light_signal, INPUT_PULLDOWN);
    pinMode(high_beam_signal, INPUT_PULLDOWN);
    pinMode(low_beam_signal, INPUT_PULLDOWN);
    pinMode(fuel_level, INPUT_PULLDOWN);
    pinMode(coolant_temp, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(RPM_pin), isr, RISING);

    Serial.begin(115200);
}

short read_RPM() {
    return short(frequency*30);
}

int read_dimmer() {
    return map(analogRead(dimmer_pin), dimmer_low_range, dimmer_high_range, 1, 4);
}

bool read_low_oil_pressure() {
    return digitalRead(low_oil_pressure_pin);
}

bool read_turn_signal() {
    return !digitalRead(turn_signal_pin);
}

bool read_parking_light() {
    return digitalRead(parking_light_signal);
}

bool read_low_beam() {
    return digitalRead(low_beam_signal);
}

bool read_high_beam() {
    return digitalRead(high_beam_signal);
}

int read_fuel_level() {
    return map(analogRead(fuel_level), 1621, 288, 0, 100);
}

int read_coolant_temp() {
    float vout = (3.3 / 4096) * (analogRead(coolant_temp));
    float R2 = (vout * 820) / (5 - vout);
    float temp = -81.2439 + (233789 - (-81.2439)) / (1 + pow((R2/1.55134e-14), 0.19926));
    return temp;
}

short read_voltage() {
    return static_cast<short>(floor((analogRead(voltage_pin) * ((13000 + 3900) / 3900)) * 10));
}

class Signal {
private:
    float value;
    unsigned long last_update_time;
    unsigned long update_interval;
    bool initialized = false;
    String name;
    float initialization_threshold;
    float spike_threshold;

public:
    Signal(String signal_name, float initial_value, unsigned long interval, float threshold, float initialization)
            : name(signal_name), value(initial_value), update_interval(interval), last_update_time(0), spike_threshold(threshold), initialization_threshold(initialization) {}

    void update(float new_value) {
        unsigned long current_time = millis();
        if (current_time - last_update_time >= update_interval) {
            if (!initialized) {
                delay(500);
                if (new_value < initialization_threshold) {
                    value = new_value;
                    initialized = true;
                }
            } else {
                if (abs(new_value - value) <= spike_threshold) {
                    value = new_value;
                }
            }
            last_update_time = current_time;
        }
    }

    float get_value() const {
        return value;
    }
};

Signal rpm_signal("RPM", 0.0, 20, 300.0, 5000);
Signal dimmer_signal("Dimmer", 0.0, 200, 2.0, 20);
Signal voltage_signal("Voltage", 0.0, 200, 5.0, 20);
Signal coolant_signal("Coolant Temp", 0.0, 200, 5.0, 200);
Signal fuel_signal("Fuel Level", 0.0, 200, 5.0, 200);

void loop() {

    rpm_signal.update(read_RPM());
    dimmer_signal.update(read_dimmer());
    voltage_signal.update(read_voltage());
    coolant_signal.update(read_coolant_temp());
    fuel_signal.update(read_fuel_level());

    short rpm = static_cast<short>(rpm_signal.get_value());
    int dimmer = static_cast<int>(dimmer_signal.get_value());
    short voltage = static_cast<short>(voltage_signal.get_value());
    int coolant = static_cast<int>(coolant_signal.get_value());
    int fuel_signal_ = static_cast<int>(fuel_signal.get_value());

    if (test) {
        Serial.print("RPM: ");
        Serial.print(rpm);
        Serial.print(" | DIMMER: ");
        Serial.print(dimmer);
        Serial.print(" | OIL_PS: ");
        Serial.print(read_low_oil_pressure());
        Serial.print(" | TURN SIG: ");
        Serial.print(read_turn_signal());
        Serial.print(" | PARK LIGHT: ");
        Serial.print(read_parking_light());
        Serial.print(" | LOW BEAM: ");
        Serial.print(read_low_beam());
        Serial.print(" | HIGH BEAM: ");
        Serial.print(read_high_beam());
        Serial.print(" | FUEL LEVEL: ");
        Serial.print(fuel_signal_);
        Serial.print(" | COOLANT TEMP: ");
        Serial.print(coolant);
        Serial.print(" | VOLTAGE: ");
        Serial.println(voltage);
    }

    if (Serial.available() > 0) {
        data_from_rpi = Serial.read();
        if (data_from_rpi == 1) {
            data_last_time = millis();
            should_print = true;
        }
    }

    data_current_time = millis();
    data_delta_time = data_current_time - data_last_time;

    if (should_print) {
        if (data_delta_time <= timeout) {

            Serial.write((rpm >> 8) & 0xFF);
            Serial.write((rpm) & 0xFF);
            Serial.write(static_cast<byte>(dimmer));
            Serial.write(static_cast<byte>(fuel_signal_));
            Serial.write(static_cast<byte>(coolant));
            Serial.write(static_cast<byte>(voltage));

            bitWrite(status_byte, 0, read_low_oil_pressure());
            bitWrite(status_byte, 1, read_turn_signal());
            bitWrite(status_byte, 2, read_parking_light());
            bitWrite(status_byte, 3, read_low_beam());
            bitWrite(status_byte, 4, read_high_beam());
            Serial.write(status_byte);

            /*
             * RPM
             * Dimmer
             * Fuel Level
             * Coolant Temp
             * Voltage
             *
             * Status Byte:
             * 0 - Low Oil Pressure
             * 1 - Turn Signal
             * 2 - Parking Light
             * 3 - Low Beam
             * 4 - HighBeam
             */

        } else {
            should_print = false;
        }
    }
    delay(20);
}