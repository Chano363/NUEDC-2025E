#include "gyro_uart.h"
#include "ti_msp_dl_config.h"
#include "bsp_init.h"
#include "../Lib/delay.h"
#include "../Lib/math_utils.h"

#define ICM_ADDR0 0x68U
#define ICM_ADDR1 0x69U
#define WHO_AM_I 0x75U
#define GYRO_Z1 0x29U
#define PWR_MGMT0 0x4EU
#define ACCEL_CONFIG0 0x50U
#define GYRO_CONFIG0 0x4FU
#define GYRO_SCALE (2000.0f / 32768.0f)
#define BIAS_SAMPLES 300U
#define DEADBAND_DPS 0.20f

/*
 * 地猛星开发板 PA23/VREF+ 板载 1uF + 100nF 对地电容，不适合作 SCL。
 * 原串口陀螺仪已停用，复用其无板载电容的 PA8=SCL、PA9=SDA。
 */
#define SCL_PORT gyro_PORT
#define SDA_PORT gyro_PORT
#define SCL_PIN gyro_ICM_SCL_PIN
#define SDA_PIN gyro_ICM_SDA_PIN
#define SCL_IOMUX gyro_ICM_SCL_IOMUX
#define SDA_IOMUX gyro_ICM_SDA_IOMUX

static uint8_t g_addr;
static float g_angle, g_bias;
static uint32_t g_last_tick;
static GyroData_t g_data;
static GyroStatus_t g_status;

/*
 * 使用内部上拉时上升沿明显慢于外部 4.7k 上拉，因此软件 I2C
 * 暂时降至较低速率，优先保证 ICM42688 能可靠识别电平。
 */
static void dly(void) { for (volatile uint32_t i=0;i<120U;i++) __NOP(); }
/*
 * 软件 I2C 开漏实现：
 * 输出锁存器始终保持为 0，enableOutput() 拉低总线，
 * disableOutput() 释放总线，由内部/外部上拉产生高电平。
 */
static void scl0(void) { DL_GPIO_enableOutput(SCL_PORT, SCL_PIN); }
static void scl1(void) { DL_GPIO_disableOutput(SCL_PORT, SCL_PIN); }
static void sda0(void) { DL_GPIO_enableOutput(SDA_PORT, SDA_PIN); }
static void sda1(void) { DL_GPIO_disableOutput(SDA_PORT, SDA_PIN); }
static bool sda_read(void) { return DL_GPIO_readPins(SDA_PORT,SDA_PIN) != 0U; }
static void i2c_stop(void);

static void i2c_gpio_init(void)
{
    DL_GPIO_initDigitalInputFeatures(SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 后续开启输出时只能拉低，绝不推挽输出高电平。 */
    DL_GPIO_clearPins(SCL_PORT, SCL_PIN);
    DL_GPIO_clearPins(SDA_PORT, SDA_PIN);
    scl1();
    sda1();
}

static void i2c_bus_recover(void)
{
    /* 若从机停在未完成的读周期，发送 9 个时钟并产生 STOP。 */
    sda1();
    scl1();
    dly();
    for (uint8_t i = 0U; i < 9U; i++) {
        scl0();
        dly();
        scl1();
        dly();
    }
    i2c_stop();
}

static void i2c_start(void) { sda1(); scl1(); dly(); sda0(); dly(); scl0(); dly(); }
static void i2c_stop(void) { sda0(); dly(); scl1(); dly(); sda1(); dly(); }

static bool i2c_write(uint8_t v)
{
    for (uint8_t i=0;i<8U;i++) {
        if (v & 0x80U) sda1(); else sda0();
        dly(); scl1(); dly(); scl0(); dly(); v <<= 1;
    }
    sda1(); dly(); scl1(); dly();
    bool ack = !sda_read();
    scl0(); dly();
    return ack;
}

static uint8_t i2c_read(bool ack)
{
    uint8_t v=0U; sda1();
    for (uint8_t i=0;i<8U;i++) {
        v <<= 1; scl1(); dly(); if (sda_read()) v++; scl0(); dly();
    }
    if (ack) sda0(); else sda1();
    scl1(); dly(); scl0(); sda1();
    return v;
}

static bool write_reg(uint8_t reg,uint8_t val)
{
    i2c_start(); bool ok=i2c_write(g_addr<<1);
    ok=i2c_write(reg)&&ok; ok=i2c_write(val)&&ok; i2c_stop();
    g_status.tx_count++; if(!ok)g_status.error_count++; return ok;
}

static bool read_regs(uint8_t reg,uint8_t *buf,uint8_t n)
{
    i2c_start(); bool ok=i2c_write(g_addr<<1);
    ok=i2c_write(reg)&&ok; i2c_start();
    ok=i2c_write((g_addr<<1)|1U)&&ok;
    for(uint8_t i=0;i<n;i++) buf[i]=i2c_read(i+1U<n);
    i2c_stop(); g_status.tx_count++; g_status.rx_count++;
    if(!ok)g_status.error_count++; return ok;
}

static bool read_z(float *dps)
{
    uint8_t b[2]; if(!read_regs(GYRO_Z1,b,2U))return false;
    *dps=(float)(int16_t)(((uint16_t)b[0]<<8)|b[1])*GYRO_SCALE; return true;
}

void Gyro_CalibrateBias(void)
{
    float sum=0.0f; uint16_t n=0U;
    for(uint16_t i=0;i<BIAS_SAMPLES;i++){float x;if(read_z(&x)){sum+=x;n++;}delay_ms(2U);}
    if(n){g_bias=sum/(float)n;g_angle=0.0f;g_last_tick=BSP_GetTick();}
    g_status.bias_dps=g_bias;
}

void Gyro_Init(void)
{
    g_status=(GyroStatus_t){0}; g_data=(GyroData_t){0}; g_angle=0.0f;g_bias=0.0f;g_last_tick=BSP_GetTick();
    i2c_gpio_init();
    i2c_bus_recover();
    /* 按例程要求，电源稳定后再进行第一次 WHO_AM_I 访问。 */
    delay_ms(20U);
    bool found=false;
    for(uint8_t k=0;k<10U&&!found;k++){g_addr=ICM_ADDR0;uint8_t w;if(read_regs(WHO_AM_I,&w,1U)&&w==0x47U){g_status.who_am_i=w;found=true;}else{g_addr=ICM_ADDR1;if(read_regs(WHO_AM_I,&w,1U)&&w==0x47U){g_status.who_am_i=w;found=true;}}delay_ms(10U);}
    if(!found)return;
    bool ok=write_reg(PWR_MGMT0,0x00U);delay_ms(2U);ok=write_reg(ACCEL_CONFIG0,0x06U)&&ok;ok=write_reg(GYRO_CONFIG0,0x06U)&&ok;ok=write_reg(PWR_MGMT0,0x0FU)&&ok;delay_ms(20U);
    g_status.address=g_addr;g_status.initialized=ok;if(ok)Gyro_CalibrateBias();
}

bool Gyro_GetData(GyroData_t *out)
{
    float dps; if(!out)return false; bool ok=false;
    for(uint8_t i=0;i<3U&&!ok;i++)ok=read_z(&dps);
    if(!ok)return false;
    if(AbsFloat(dps-g_bias)<1.0f)g_bias+=0.002f*(dps-g_bias);
    g_status.bias_dps=g_bias;dps-=g_bias;if(AbsFloat(dps)<DEADBAND_DPS)dps=0.0f;
    uint32_t now=BSP_GetTick(),dt=now-g_last_tick;if(dt>100U)dt=100U;g_last_tick=now;
    g_angle+=dps*(float)dt*0.001f;while(g_angle>=180.0f)g_angle-=360.0f;while(g_angle<-180.0f)g_angle+=360.0f;
    g_data.angle_deg=g_angle;g_data.dps=dps;g_data.frame_count++;g_data.valid=true;*out=g_data;return true;
}

GyroStatus_t Gyro_GetStatus(void){return g_status;}
void Gyro_ProcessByte(uint8_t byte){(void)byte;}
