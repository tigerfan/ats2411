/*
 * ATS Automatic Measurement Device Main Control Program
 * Author: tigerfan
 * Version: 2024.11
 * Description: Main control program for ATS automatic measurement device
 */

// Standard libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// RTOS
#include <STM32FreeRTOS.h>

// Communication protocols
#include <Ethernet2.h>
#include <Vrekrer_scpi_parser.h>

// Sensor and peripheral libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MCP_ADC.h>
#include <MCP4725.h>

// Local headers
#include "TCA6424A.h"

// Hardware Configuration
namespace HW_CONFIG {
    // Communication Pins
    constexpr uint8_t W5500_CS = PB12;    // Ethernet CS
    constexpr uint8_t W5500_RST = PA12;   // Ethernet Reset
    constexpr uint8_t MCP3201_CS = PA4;   // ADC CS (SPI2)
    constexpr uint8_t ONE_WIRE_BUS = PA10; // Temperature sensor
    constexpr uint8_t DEVICE_PRESENT = PB4;// Device presence detection
    constexpr uint8_t TCA6424_RST = PB3;  // TCA6424A Reset
    constexpr uint8_t MEASURE_LED = PB5;   // Measurement status LED (Active Low)

    // Channel Selection
    namespace CS {
        constexpr uint8_t PIN1 = PB2;     // Chip select bit 0
        constexpr uint8_t PIN2 = PB1;     // Chip select bit 1
        constexpr uint8_t PIN3 = PB0;     // Chip select bit 2
    }

    namespace MUX {
        constexpr uint8_t PIN1 = PA1;     // Channel select bit 0
        constexpr uint8_t PIN2 = PA2;     // Channel select bit 1
        constexpr uint8_t PIN3 = PA3;     // Channel select bit 2
    }

    // I2C Bus Configuration
    namespace I2C {
        constexpr uint8_t I2C1_SCL = PB6;
        constexpr uint8_t I2C1_SDA = PB7;
        constexpr uint8_t I2C2_SCL = PB10;
        constexpr uint8_t I2C2_SDA = PB11;
    }

    // I2C Device Addresses
    namespace I2C_ADDR {
        constexpr uint8_t TCA6424A_DRIVE = 0x23;  // A0=0, Drive control
        constexpr uint8_t TCA6424A_LOAD = 0x22;   // A0=1, Load selector
        constexpr uint8_t MCP4725 = 0x60;         // DAC address
    }

    // Load Resistance Pins
    namespace LOAD_RES {
        constexpr uint8_t PIN1 = PA8;
        constexpr uint8_t PIN2 = PA9;
    }
}

// System Configuration
namespace SYS_CONFIG {
    // Channel Configuration
    constexpr uint8_t NUM_CHIPS = 3;
    constexpr uint8_t NUM_LINES = 8;
    constexpr uint8_t NUM_INPUTS = 20;
    constexpr uint8_t NUM_LOADS = 20;
    constexpr uint8_t NUM_DRIVES = 20;
    constexpr uint8_t NUM_LOGICAL_CHANNELS = NUM_INPUTS;
    constexpr uint8_t ADC_CHANNELS_PER_CHIP = 8;
    constexpr uint8_t INVALID_CHANNEL = 0xFF;

    // Sampling Configuration
    constexpr uint16_t MIN_SAMPLES = 1000;
    constexpr uint16_t MAX_SAMPLES = 10000;
    constexpr uint16_t DEFAULT_SAMPLES = 1000;
    constexpr uint16_t SAMPLE_INTERVAL_US = 100;  // 100us = 10kHz

    // Timer Configuration
    constexpr uint16_t TIMER_PRESCALER = 72;      // 72MHz / 72 = 1MHz
    constexpr uint16_t TIMER_PERIOD = 100;        // 1MHz / 100 = 10kHz

    // Calibration Configuration
    constexpr uint8_t CAL_CHANNEL = 24;
    constexpr uint16_t CAL_STEPS = 100;
    constexpr uint16_t CAL_START_MV = 0;
    constexpr uint16_t CAL_END_MV = 500;
}

// Load resistance type enumeration
enum LoadResistance {
    OHM_22_5 = 0,  // Both pins high - 22.5Ω
    OHM_20_0 = 1,  // PA8 low, PA9 high - 20.0Ω
    OHM_18_5 = 2   // PA8 high, PA9 low - 18.5Ω
};

// Forward declarations
bool setLoadResistance(LoadResistance resistance);
void resetToDefault();
bool configureTestChannelNoDrive(uint16_t logical_channel);

// Global variables
uint8_t current_drive_num = 0;  // Current selected drive number

// Channel mapping tables
const uint8_t channelLoadMap[SYS_CONFIG::NUM_LOGICAL_CHANNELS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19
};

const uint8_t channelInputMap[SYS_CONFIG::NUM_LOGICAL_CHANNELS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19
};

const uint8_t channelDriveMap[SYS_CONFIG::NUM_LOGICAL_CHANNELS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19
};

// Channel to resistance mapping table
const LoadResistance channelResistanceMap[SYS_CONFIG::NUM_LOGICAL_CHANNELS] = {
    OHM_22_5, OHM_22_5, OHM_22_5, OHM_22_5, OHM_22_5,  // Channels 0-4
    OHM_20_0, OHM_20_0, OHM_20_0, OHM_20_0, OHM_20_0,  // Channels 5-9
    OHM_18_5, OHM_18_5, OHM_18_5, OHM_18_5, OHM_18_5,  // Channels 10-14
    OHM_22_5, OHM_22_5, OHM_22_5, OHM_22_5, OHM_22_5   // Channels 15-19
};

// Current selected load resistance
LoadResistance current_resistance = OHM_22_5;

// Input channel mapping (physical to logical)
const uint8_t inputChannelMap[SYS_CONFIG::NUM_INPUTS][2] = {
    // {chip_number, line_number}
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 1}, {1, 0}, {1, 2}, {1, 3},  // Chip 0, 8 channels
    {0, 4}, {0, 5}, {0, 6}, {0, 7}, {1, 5}, {1, 4}, {1, 6}, {1, 7},  // Chip 1, 8 channels
    {2, 3}, {2, 2}, {2, 1}, {2, 0}                                     // Chip 2, 4 channels
};

// Load and Drive pin mapping - 18-to-1
const uint8_t loadPins[SYS_CONFIG::NUM_LOADS] = {
    0,  1,  2,  3,  4,  5, 6, 7,   // Port 0 pins
    8,  9,  10, 11, 12, 13, 14, 15,  // Port 1 pins
    16, 17, 18, 19   // Port 2 pins
};

const uint8_t drivePins[SYS_CONFIG::NUM_DRIVES] = {
    18,  16,  14,  12,  0,  1, 19, 17,   // Port 0 pins
    15,  13,  11,  9,  7,  5,  2,  3,  // Port 1 pins
    10, 8, 6, 4   // Port 2 pins
};

// Channel select pins arrays
const uint8_t chipSelectPins[3] = {
    HW_CONFIG::CS::PIN1, HW_CONFIG::CS::PIN2, HW_CONFIG::CS::PIN3
};

const uint8_t muxSelectPins[3] = {
    HW_CONFIG::MUX::PIN1, HW_CONFIG::MUX::PIN2, HW_CONFIG::MUX::PIN3
};

// Network configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip;

// SCPI Parser
SCPI_Parser scpi_parser;

// FreeRTOS Handles
SemaphoreHandle_t xSerialSemaphore;
QueueHandle_t xSampleQueue;    // Sampling data queue
SemaphoreHandle_t xSampleMutex;  // Sampling control mutex

// Server instance
EthernetServer server(5025);

// Analog sampling configuration
volatile bool sampling_active = false;
volatile uint16_t current_max_samples = SYS_CONFIG::DEFAULT_SAMPLES;
volatile uint16_t* samples = nullptr;      // Dynamic sample buffer
volatile uint16_t sample_count = 0;        // Current sample count

// Current selection state
volatile uint8_t current_input = SYS_CONFIG::INVALID_CHANNEL;
volatile uint8_t current_load = SYS_CONFIG::INVALID_CHANNEL;
volatile uint8_t current_drive = SYS_CONFIG::INVALID_CHANNEL;
volatile bool device_present = false;  // Device presence status

// Timer handle
HardwareTimer *timer;

// SPI and I2C instances
SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for W5500
TwoWire Wire1(HW_CONFIG::I2C::I2C1_SDA, HW_CONFIG::I2C::I2C1_SCL);  // I2C1 for TCA6424A
TwoWire Wire2(HW_CONFIG::I2C::I2C2_SDA, HW_CONFIG::I2C::I2C2_SCL);  // I2C2 for MCP4725
MCP3201 adc(&SPI);               // Create MCP3201 instance using SPI1
TCA6424A tcaDrive(HW_CONFIG::I2C_ADDR::TCA6424A_DRIVE);       // Drive control using I2C1
TCA6424A tcaLoad(HW_CONFIG::I2C_ADDR::TCA6424A_LOAD);        // Load selector using I2C1
OneWire oneWire(HW_CONFIG::ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// DAC configuration
MCP4725 dac(HW_CONFIG::I2C_ADDR::MCP4725, &Wire2);    // Create MCP4725 instance using I2C2

// Function prototypes
bool startSampling(uint8_t channel);
void stopSampling();
bool selectInput(uint8_t input);
bool selectLoad(uint8_t load);
bool selectDrive(uint8_t drive);
bool setLogicalChannel(uint16_t logical_channel);
void TaskAnalogRead(void *pvParameters);
void TaskNetwork(void *pvParameters);
uint16_t readADC();
void timerCallback();
void measureTemp(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface);
void TaskDeviceCheck(void *pvParameters);

// Sampling data structure
struct SampleData {
    uint16_t value;
    uint16_t index;
};

// Read ADC value
uint16_t readADC() {
    // Select the correct chip
    uint8_t chip = current_input / SYS_CONFIG::ADC_CHANNELS_PER_CHIP;
    uint8_t channel = current_input % SYS_CONFIG::ADC_CHANNELS_PER_CHIP;
    
    // Set chip select low for the selected ADC
    digitalWrite(chipSelectPins[chip], LOW);
    
    // Read from ADC
    uint16_t value = adc.read(channel);
    
    // Set chip select high
    digitalWrite(chipSelectPins[chip], HIGH);
    
    return value;
}

// Start sampling on specified channel
bool startSampling(uint8_t channel) {
    if (sampling_active) return false;  // Already sampling
    
    // Configure channel
    if (!selectInput(channel)) return false;
    
    // Allocate sample buffer
    if (samples != nullptr) {
        free((void*)samples);
    }
    samples = (uint16_t*)malloc(sizeof(uint16_t) * current_max_samples);
    if (samples == nullptr) return false;
    
    // Initialize sampling
    sample_count = 0;
    sampling_active = true;
    
    // Turn on measurement LED
    digitalWrite(HW_CONFIG::MEASURE_LED, LOW);  // LED on (active low)
    
    // Start timer
    timer->resume();
    
    return true;
}

// Stop sampling
void stopSampling() {
    if (!sampling_active) return;
    
    // Stop timer
    timer->pause();
    
    // Turn off measurement LED
    digitalWrite(HW_CONFIG::MEASURE_LED, HIGH);  // LED off
    
    // Clean up
    sampling_active = false;
    if (samples != nullptr) {
        free((void*)samples);
        samples = nullptr;
    }
    sample_count = 0;
}

// Logical to physical channel mapping
bool setLogicalChannel(uint16_t logical_channel) {
    if (logical_channel >= SYS_CONFIG::NUM_LOGICAL_CHANNELS) return false;
    
    // Get load and input channel from mapping table
    uint8_t loadNum = channelLoadMap[logical_channel];
    uint8_t inputNum = channelInputMap[logical_channel];
    uint8_t driveNum = channelDriveMap[logical_channel];
    
    // Select drive first
    if (!selectDrive(driveNum)) return false;
    delay(3);  // Wait for drive to stabilize
    
    // Select load next
    if (!selectLoad(loadNum)) return false;
    delay(3);  // Wait for load to stabilize
    
    // Select input channel last
    if (!selectInput(inputNum)) return false;
    delay(1);  // Wait for channel to establish
    
    return true;
}

bool selectInput(uint8_t input) {
    if (input >= SYS_CONFIG::NUM_INPUTS) return false;
    
    // Get chip number and line number from mapping table
    uint8_t chipNum = inputChannelMap[input][0];
    uint8_t lineNum = inputChannelMap[input][1];
    
    // Set chip select pins according to chipNum (one-hot encoding)
    for (uint8_t i = 0; i < SYS_CONFIG::NUM_CHIPS; i++) {
        digitalWrite(chipSelectPins[i], (i == chipNum) ? HIGH : LOW);
    }
    
    // Set line select pins according to lineNum
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(muxSelectPins[i], (lineNum >> i) & 0x01);
    }
    
    current_input = input;
    return true;
}

bool selectLoad(uint8_t load) {
    if (load >= SYS_CONFIG::NUM_LOADS) return false;
    
    // Clear all pins first
    for (uint8_t pin = 0; pin < 24; pin++) {
        tcaLoad.writePin(pin, false);
    }
    
    // Set the selected load pin
    tcaLoad.writePin(loadPins[load], true);
    current_load = load;
    return true;
}

bool selectDrive(uint8_t drive) {
    if (drive >= SYS_CONFIG::NUM_DRIVES) return false;
    
    // Clear all pins first
    for (uint8_t pin = 0; pin < 24; pin++) {
        tcaDrive.writePin(pin, false);
    }
    
    // Set the selected drive pin
    tcaDrive.writePin(drivePins[drive], true);
    current_drive = drive;
    return true;
}

// SCPI command handlers
void identify(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    interface.println(F("ATS2411 Test System"));
}

void getChannelInfo(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    interface.println(F("Channel Configuration:"));
    interface.println(F("Input Channels: 20"));
}

void getSampleConfig(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    interface.println(F("Sampling Configuration:"));
    interface.print(F("Sample Rate: ")); 
    interface.println(F("10 kHz"));
    interface.print(F("Sample Count: ")); 
    interface.println(String(sample_count));
}

void measure(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    if (parameters.Size() < 1) {
        interface.println(F("ERR: Missing channel parameter"));
        resetToDefault();  // Reset to default state on error
        return;
    }

    int channel = String(parameters[0]).toInt();
    uint16_t points = 1;  // Default to single point
    
    // Check if sample points are specified
    if (parameters.Size() > 1) {
        points = String(parameters[1]).toInt();
        // Validate sample points range
        if (points < SYS_CONFIG::MIN_SAMPLES || points > SYS_CONFIG::MAX_SAMPLES) {
            interface.println(F("ERR: Sample points must be between 1000 and 10000"));
            resetToDefault();  // Reset to default state on error
            return;
        }
    }
    
    // Validate channel
    if (channel < 0 || channel >= SYS_CONFIG::NUM_INPUTS) {
        interface.println(F("ERR: Invalid channel"));
        resetToDefault();  // Reset to default state on error
        return;
    }

    // Configure test channel and start measurement
    if (!startSampling(channel)) {
        interface.println(F("ERR: Failed to start sampling"));
        resetToDefault();  // Reset to default state on error
        return;
    }

    // If single point
    if (points == 1) {
        uint16_t sample;
        if (xQueueReceive(xSampleQueue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            interface.println(String(sample));
        } else {
            interface.println(F("ERR: Timeout waiting for sample"));
        }
        stopSampling();  // stopSampling calls resetToDefault
        return;
    }

    // Multi-point sampling
    for (uint16_t i = 0; i < points; i++) {
        uint16_t sample;
        if (xQueueReceive(xSampleQueue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            interface.println(String(sample));
        } else {
            interface.println(F("ERR: Timeout waiting for sample"));
            stopSampling();  // stopSampling calls resetToDefault
            return;
        }
    }
    
    stopSampling();  // stopSampling calls resetToDefault
}

void measureTemp(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    if (commands.Size() == 1) {
        // Read temperature from DS18B20
        sensors.requestTemperatures();
        float temp = sensors.getTempCByIndex(0);
        interface.println(String(temp, 2));
    } else {
        interface.println(F("ERR: Invalid parameter"));
    }
}

void getSystemStatus(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    String response;
    
    // System identification
    response += "Device: ATS2411\n";
    
    // Device presence status
    response += "DUT Online: " + String(device_present ? "Yes" : "No") + "\n";
    
    // ADC Configuration
    response += "ADC Status: " + String(sampling_active ? "Sampling" : "Stopped") + "\n";
    response += "Sample Rate: " + String(10000) + " Hz\n";
    
    // Current channel configuration
    response += "Current Input: " + String(current_input) + "\n";
    response += "Current Load: " + String(current_load) + "\n";
    response += "Current Drive: " + String(current_drive) + "\n";
    response += "Logical Channel: " + String(current_input) + "\n";
    
    // Network configuration
    response += "IP Address: " + String(Ethernet.localIP()[0]) + "." +
                               String(Ethernet.localIP()[1]) + "." +
                               String(Ethernet.localIP()[2]) + "." +
                               String(Ethernet.localIP()[3]) + "\n";
    
    interface.println(response);
}

void startCalibration(SCPI_Commands commands, SCPI_Parameters parameters, Stream& interface) {
    // Calculate voltage step
    float voltageStep = (float)(SYS_CONFIG::CAL_END_MV - SYS_CONFIG::CAL_START_MV) / SYS_CONFIG::CAL_STEPS;
    
    // Select calibration input channel
    if (!selectInput(SYS_CONFIG::CAL_CHANNEL)) {
        interface.println(F("ERR: Failed to select calibration channel"));
        return;
    }
    
    // Header information
    interface.println(F("Calibration Data:"));
    interface.println(F("Format: <step>,<dac_mv>,<adc_value>"));
    
    // Perform calibration steps
    for (int step = 0; step < SYS_CONFIG::CAL_STEPS; step++) {
        // Calculate target voltage in mV
        float targetVoltage = SYS_CONFIG::CAL_START_MV + (step * voltageStep);
        
        // Convert mV to DAC value (12-bit)
        uint16_t dacValue = (uint16_t)((targetVoltage / 5000.0) * 4095);
        
        // Set DAC output
        dac.writeDAC(dacValue);
        delay(10);  // Allow voltage to settle
        
        // Read ADC value
        uint16_t adcValue = readADC();
        
        // Output data
        interface.print(String(step));
        interface.print(F(","));
        interface.print(String(targetVoltage, 2));
        interface.print(F(","));
        interface.println(String(adcValue));
        
        // Small delay between steps
        delay(50);
    }
    
    // Reset DAC to 0V
    dac.writeDAC(0);
    interface.println(F("Calibration complete"));
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize pins
    // Reset and chip select pins initialization
    pinMode(HW_CONFIG::W5500_RST, OUTPUT);
    pinMode(HW_CONFIG::W5500_CS, OUTPUT);
    pinMode(HW_CONFIG::MCP3201_CS, OUTPUT);
    pinMode(HW_CONFIG::TCA6424_RST, OUTPUT);
    pinMode(HW_CONFIG::DEVICE_PRESENT, INPUT_PULLUP);
    pinMode(HW_CONFIG::MEASURE_LED, OUTPUT);
    digitalWrite(HW_CONFIG::MEASURE_LED, HIGH);  // LED off by default
    
    // Channel select pins initialization
    for(uint8_t i = 0; i < 3; i++) {
        pinMode(chipSelectPins[i], OUTPUT);
        digitalWrite(chipSelectPins[i], HIGH);  // Default: no chip selected
    }
    
    pinMode(HW_CONFIG::MUX::PIN1, OUTPUT);
    pinMode(HW_CONFIG::MUX::PIN2, OUTPUT);
    pinMode(HW_CONFIG::MUX::PIN3, OUTPUT);
    digitalWrite(HW_CONFIG::MUX::PIN1, LOW);
    digitalWrite(HW_CONFIG::MUX::PIN2, LOW);
    digitalWrite(HW_CONFIG::MUX::PIN3, LOW);
    
    // Load resistance selection pins initialization
    pinMode(HW_CONFIG::LOAD_RES::PIN1, OUTPUT);
    pinMode(HW_CONFIG::LOAD_RES::PIN2, OUTPUT);
    digitalWrite(HW_CONFIG::LOAD_RES::PIN1, HIGH);  // Default: 22.5Ω
    digitalWrite(HW_CONFIG::LOAD_RES::PIN2, HIGH);
    
    // Reset signal initialization
    digitalWrite(HW_CONFIG::W5500_RST, HIGH);
    digitalWrite(HW_CONFIG::TCA6424_RST, HIGH);
    digitalWrite(HW_CONFIG::W5500_CS, HIGH);
    digitalWrite(HW_CONFIG::MCP3201_CS, HIGH);
    
    // Reset TCA6424A
    digitalWrite(HW_CONFIG::TCA6424_RST, LOW);
    delay(1);
    digitalWrite(HW_CONFIG::TCA6424_RST, HIGH);
    delay(1);
    
    // Initialize I2C
    Wire1.begin();  // I2C1 for TCA6424A
    Wire2.begin();  // I2C2 for MCP4725
    
    // Initialize SPI
    SPI.begin();  // SPI1 for W5500
    SPI_2.begin();  // SPI2 for ADC
    
    // Initialize TCA6424A
    for(int i = 0; i < 24; i++) {
        tcaDrive.setPinDirection(i, false);  // Set all pins as outputs
        tcaLoad.setPinDirection(i, false);   // Set all pins as outputs
        tcaDrive.writePin(i, false);         // Set all pins low
        tcaLoad.writePin(i, false);          // Set all pins low
    }
    
    // Initialize temperature sensor
    sensors.begin();
    
    // Initialize W5500
    digitalWrite(HW_CONFIG::W5500_RST, LOW);
    delay(1);
    digitalWrite(HW_CONFIG::W5500_RST, HIGH);
    delay(1);
    
    // Initialize Ethernet
    Ethernet.init(HW_CONFIG::W5500_CS);
    ip = IPAddress(192, 168, 1, 177);
    Ethernet.begin(mac, ip);
    
    // Initialize Timer
    timer = new HardwareTimer(TIM1);
    timer->setOverflow(SYS_CONFIG::TIMER_PERIOD, MICROSEC_FORMAT);
    timer->attachInterrupt(timerCallback);
    timer->pause();
    
    // Initialize SCPI parser
    scpi_parser.RegisterCommand(F("*IDN?"), &identify);
    scpi_parser.RegisterCommand(F("SYSTem:CHANnel?"), &getChannelInfo);
    scpi_parser.RegisterCommand(F("SYSTem:SAMPle?"), &getSampleConfig);
    scpi_parser.RegisterCommand(F("MEASure"), &measure);
    scpi_parser.RegisterCommand(F("MEASure:TEMPerature"), &measureTemp);
    scpi_parser.RegisterCommand(F("SYSTem:STATus?"), &getSystemStatus);
    scpi_parser.RegisterCommand(F("CALibration:STARt"), &startCalibration);
    
    // Create FreeRTOS tasks
    xSampleQueue = xQueueCreate(1, sizeof(SampleData));
    xSampleMutex = xSemaphoreCreateMutex();
    xTaskCreate(TaskAnalogRead, "AnalogRead", 128, NULL, 2, NULL);
    xTaskCreate(TaskNetwork, "Network", 256, NULL, 1, NULL);
    xTaskCreate(TaskDeviceCheck, "DeviceCheck", 64, NULL, 1, NULL);
}

void loop() {
    // Empty. Things are done in Tasks.
}

void TaskAnalogRead(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        SampleData data;
        if (xQueueReceive(xSampleQueue, &data, portMAX_DELAY) == pdTRUE) {
            // Process sampling data
            samples[data.index] = data.value;
            sample_count++;
            if (sample_count >= current_max_samples) {
                stopSampling();
            }
        }
    }
}

void TaskNetwork(void *pvParameters) {
    (void) pvParameters;
    
    for (;;) {
        // Check if there is a client
        EthernetClient client = server.available();
        if (client) {
            while (client.connected()) {
                if (client.available()) {
                    String message = client.readStringUntil('\n');
                    if (message.length() > 0) {
                        scpi_parser.ProcessInput(client, "\n");
                    }
                }
            }
            client.stop();
        }
        vTaskDelay(1);
    }
}

void TaskDeviceCheck(void *pvParameters) {
    pinMode(HW_CONFIG::DEVICE_PRESENT, INPUT_PULLUP);
    
    for (;;) {
        device_present = !digitalRead(HW_CONFIG::DEVICE_PRESENT);  // Active low input
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
    }
}

// Timer callback function
void timerCallback() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t sample = readADC();
    xQueueSendFromISR(xSampleQueue, &sample, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Configure complete test channel
bool configureTestChannel(uint16_t logical_channel) {
    if (logical_channel >= SYS_CONFIG::NUM_LOGICAL_CHANNELS) return false;
    
    // Get mapping relationship
    uint8_t driveNum = channelDriveMap[logical_channel];
    uint8_t loadNum = channelLoadMap[logical_channel];
    uint8_t inputNum = channelInputMap[logical_channel];
    
    // Configure in order: drive -> load -> input channel
    if (!selectDrive(driveNum)) return false;
    delay(3);  // Wait for drive to stabilize
    
    if (!selectLoad(loadNum)) return false;
    delay(3);  // Wait for load to stabilize
    
    if (!selectInput(inputNum)) return false;
    delay(1);  // Wait for channel to establish
    
    return true;
}

// Configure complete test channel (without drive enable)
bool configureTestChannelNoDrive(uint16_t logical_channel) {
    if (logical_channel >= SYS_CONFIG::NUM_LOGICAL_CHANNELS) return false;
    
    // Get mapping relationship
    uint8_t driveNum = channelDriveMap[logical_channel];
    uint8_t loadNum = channelLoadMap[logical_channel];
    uint8_t inputNum = channelInputMap[logical_channel];
    LoadResistance resistance = channelResistanceMap[logical_channel];
    
    // Disable drive first
    tcaDrive.writePin(drivePins[driveNum], false);
    
    // Set load resistance
    if (!setLoadResistance(resistance)) return false;
    
    // Select load
    if (!selectLoad(loadNum)) return false;
    delay(3);  // Wait for load to stabilize
    
    // Select input channel
    if (!selectInput(inputNum)) return false;
    delay(1);  // Wait for channel to establish
    
    current_drive_num = driveNum;  // Save current drive number for later use
    return true;
}

// Reset all control signals to default state
void resetToDefault() {
    // Disable all drives
    for(int i = 0; i < SYS_CONFIG::NUM_DRIVES; i++) {
        tcaDrive.writePin(drivePins[i], false);
    }
    
    // Disable all loads
    for(int i = 0; i < SYS_CONFIG::NUM_LOADS; i++) {
        tcaLoad.writePin(loadPins[i], false);
    }
    
    // Reset channel select signals
    for(uint8_t i = 0; i < 3; i++) {
        digitalWrite(chipSelectPins[i], HIGH);  // Default: no chip selected
    }
    digitalWrite(HW_CONFIG::MUX::PIN1, LOW);
    digitalWrite(HW_CONFIG::MUX::PIN2, LOW);
    digitalWrite(HW_CONFIG::MUX::PIN3, LOW);
    
    // Set default load resistance (22.5Ω)
    digitalWrite(HW_CONFIG::LOAD_RES::PIN1, HIGH);
    digitalWrite(HW_CONFIG::LOAD_RES::PIN2, HIGH);
    
    // Reset ADC chip select
    digitalWrite(HW_CONFIG::MCP3201_CS, HIGH);
}

// Initialize load resistance selection pins
void initLoadResistance() {
    pinMode(HW_CONFIG::LOAD_RES::PIN1, OUTPUT);
    pinMode(HW_CONFIG::LOAD_RES::PIN2, OUTPUT);
    digitalWrite(HW_CONFIG::LOAD_RES::PIN1, HIGH);  // Default: 22.5Ω
    digitalWrite(HW_CONFIG::LOAD_RES::PIN2, HIGH);
}

// Set load resistance
bool setLoadResistance(LoadResistance resistance) {
    switch(resistance) {
        case OHM_22_5:
            digitalWrite(HW_CONFIG::LOAD_RES::PIN1, HIGH);
            digitalWrite(HW_CONFIG::LOAD_RES::PIN2, HIGH);
            break;
        case OHM_20_0:
            digitalWrite(HW_CONFIG::LOAD_RES::PIN1, LOW);
            digitalWrite(HW_CONFIG::LOAD_RES::PIN2, HIGH);
            break;
        case OHM_18_5:
            digitalWrite(HW_CONFIG::LOAD_RES::PIN1, HIGH);
            digitalWrite(HW_CONFIG::LOAD_RES::PIN2, LOW);
            break;
        default:
            return false;
    }
    delay(1);  // Wait for resistance to stabilize
    return true;
}
