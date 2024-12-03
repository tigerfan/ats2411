#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ethernet2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>
#include <Vrekrer_scpi_parser.h>
#include <TCA6424A.h>
#include <MCP4725.h>

// Pin definitions
#define TEMP_SENSOR PA0  // Internal temperature sensor
#define MCP3201_CS_PIN PB12  // CS pin for MCP3201
#define ONE_WIRE_PIN PA15  // One wire pin

// 优化全局变量定义
static SCPI_Parser scpi_parser;
static MCP3201 adc(MCP3201_CS_PIN);  // 使用MCP3201类
static TCA6424A inputExpander(0x23);  // I2C address for input expander
static TCA6424A loadExpander(0x22);   // I2C address for load expander
static OneWire oneWire(ONE_WIRE_PIN);
static DallasTemperature sensors(&oneWire);

// 网络配置
static uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
static IPAddress ip(192, 168, 1, 16);
static EthernetServer server(5025);

// 任务句柄
static TaskHandle_t xHandleDigitalRead = NULL;
static TaskHandle_t xHandleSampling = NULL;
static TaskHandle_t xHandleNetwork = NULL;

// 采样缓冲区
static uint16_t samplingBuffer[10000];  // 最大支持 10000 点采样
static volatile size_t samplingCount = 0;
static volatile bool isSampling = false;

// MCP3201 ADC类定义
class MCP3201 {
public:
    MCP3201(int cs_pin) : _cs_pin(cs_pin) {
        pinMode(_cs_pin, OUTPUT);
        digitalWrite(_cs_pin, HIGH);
    }
    
    uint16_t read() {
        uint16_t value;
        
        digitalWrite(_cs_pin, LOW);
        delayMicroseconds(1);
        
        uint8_t msb = SPI.transfer(0x00);
        uint8_t lsb = SPI.transfer(0x00);
        
        digitalWrite(_cs_pin, HIGH);
        
        value = ((msb & 0x1F) << 8) | lsb;
        value >>= 1;  // 13-bit to 12-bit conversion
        
        return value;
    }
    
private:
    int _cs_pin;
};

// 温度测量函数
static float getInternalTemp() {
    int16_t raw = analogRead(TEMP_SENSOR);
    return ((1.43 - ((3.3/4096) * raw)) / 0.0043) + 25;
}

static float getExternalTemp() {
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    return (temp == DEVICE_DISCONNECTED_C) ? 999.9f : temp;
}

// SCPI命令处理函数
static void handleMeasure(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    if(parameters.Size() < 1) {
        interface.println("ERR: Channel required");
        return;
    }
    
    int ch = parameters.First().toInt();
    int sampleCount = parameters.Size() > 1 ? constrain(parameters.Last().toInt(), 1000, 10000) : 1000;
    
    if(ch < 1 || ch > 18) {
        interface.println("ERR: Invalid channel");
        return;
    }
    
    // 选择通道
    inputExpander.writePort(0, 1 << (ch-1));
    delay(1);
    
    if(sampleCount > 1) {
        uint32_t sum = 0;
        for(int i = 0; i < sampleCount; i++) {
            sum += adc.read();
            delay(1);
        }
        interface.printf("%d,%lu,AVG\n", ch, sum/sampleCount);
    } else {
        interface.printf("%d,%d,RAW\n", ch, adc.read());
    }
    
    inputExpander.writePort(0, 0);  // 关闭所有通道
}

static void handleTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    interface.printf("%.1f,%.1f\n", getInternalTemp(), getExternalTemp());
}

// FreeRTOS 任务
static void TaskDigitalRead(void *pvParameters) {
    (void)pvParameters;
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void TaskSampling(void *pvParameters) {
    (void)pvParameters;
    for(;;) {
        if(isSampling && samplingCount < 10000) {
            samplingBuffer[samplingCount++] = adc.read();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void TaskNetworkCommunication(void *pvParameters) {
    (void)pvParameters;
    for(;;) {
        EthernetClient client = server.available();
        if(client) {
            while(client.connected()) {
                if(client.available()) {
                    String cmd = client.readStringUntil('\n');
                    scpi_parser.ProcessInput(client, cmd.c_str());
                }
            }
            client.stop();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    // 初始化外设
    SPI.begin();
    Wire.begin();
    sensors.begin();
    
    // 初始化扩展器
    inputExpander.initialize();
    loadExpander.initialize();
    
    // 配置网络
    Ethernet.begin(mac, ip);
    server.begin();
    
    // 注册SCPI命令
    scpi_parser.RegisterCommand(F("MEASure?"), &handleMeasure);
    scpi_parser.RegisterCommand(F("TEMPerature?"), &handleTemperature);
    
    // 创建任务
    xTaskCreate(TaskDigitalRead, "DigitalRead", 128, NULL, 1, &xHandleDigitalRead);
    xTaskCreate(TaskSampling, "Sampling", 128, NULL, 2, &xHandleSampling);
    xTaskCreate(TaskNetworkCommunication, "Network", 256, NULL, 1, &xHandleNetwork);
    
    vTaskStartScheduler();
}

void loop() {
    // 不会执行到这里
}
