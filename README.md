# ATS2411 自动测试系统

基于STM32F103的自动测试系统，支持多通道电压采集和负载控制。

## 硬件配置

### 主控制器
- MCU: STM32F103CB (BluePill)
- 时钟: 72MHz
- Flash: 128KB
- RAM: 20KB

### 外设配置
1. ADC (MCP3201)
   - 接口：SPI2
   - 引脚：
     - CS: PA4
     - MOSI: PB15
     - MISO: PB14
     - SCK: PB13

2. 负载控制 (TCA6424A)
   - 接口：I2C1
   - 地址：0x22
   - 引脚：
     - SCL: PB6
     - SDA: PB7
     - RST: PB3

3. 驱动控制 (TCA6424A)
   - 接口：I2C1
   - 地址：0x23
   - 引脚：同上

4. 网络接口 (W5500)
   - 接口：SPI1
   - 引脚：
     - CS: PB12
     - RST: PA12
     - MOSI: PA7
     - MISO: PA6
     - SCK: PA5

5. 温度传感器 (DS18B20)
   - 接口：单总线
   - 引脚：PA10

### 通道配置
- 20个测试通道
- 3种负载阻值：
  - 22.5Ω (通道0-4, 15-19)
  - 20.0Ω (通道5-9)
  - 18.5Ω (通道10-14)

## 软件功能

### 采样功能
- 采样率：10kHz
- 采样点数：1000-10000可配置
- ADC分辨率：12位

### SCPI命令集
1. `*IDN?`
   - 功能：获取设备标识信息
   - 返回：设备名称、版本号等

2. `MEASure <channel>[,<points>]`
   - 功能：测量指定通道的电压
   - 参数：
     - channel：通道号(0-19)
     - points：采样点数(1000-10000，可选)
   - 返回：测量结果（单位：ADC码值）

3. `MEASure:TEMPerature`
   - 功能：测量当前温度
   - 返回：温度值（单位：摄氏度）

4. `SYSTem:CHANnel?`
   - 功能：查询通道配置信息
   - 返回：通道数量和配置

5. `SYSTem:SAMPle?`
   - 功能：查询采样配置信息
   - 返回：采样率、点数范围等

6. `SYSTem:STATus?`
   - 功能：查询系统状态
   - 返回：各个模块的工作状态

7. `CALibration:STARt`
   - 功能：启动校准过程
   - 返回：校准结果

## 测量流程
1. 选择测试通道
2. 根据通道自动配置对应的负载阻值
3. 开启驱动（延迟10ms）
4. 进行采样
5. 关闭驱动（提前10ms）
6. 恢复所有控制信号到默认状态

## 安全特性
1. 驱动和负载互锁：同时只能开启一个驱动和一个负载
2. 采样保护：驱动开启前后有延时保护
3. 错误处理：任何错误发生时自动恢复到安全状态
4. 上电初始化：确保所有控制信号处于安全状态

## 开发环境
- 平台：PlatformIO
- 框架：Arduino
- 依赖库：
  - STM32duino FreeRTOS
  - Ethernet2
  - OneWire
  - DallasTemperature
  - MCP_ADC
  - MCP4725
  - I2Cdevlib-Core
  - I2Cdevlib-TCA6424A
  - Vrekrer SCPI parser

## 注意事项
1. 首次上电时，确保所有外设连接正确
2. 测量时注意遵循正确的通道选择顺序
3. 避免频繁切换负载，以延长继电器寿命
4. 定期检查温度，避免系统过热