#include "drv_pwm.h"
#include "ti_msp_dl_config.h"

/* ══════════════════════════════════════════════════════════════
   MSPM0 PWM 踩坑：边沿对齐模式下 CC 值反直觉！
     CC=0   → ≈100% 占空比（危险！）
     CC=200 → 0% 占空比（=period 值）
   本驱动层做取反映射，对外暴露直觉语义：
     0 = 0%, 200 = 100%
   ══════════════════════════════════════════════════════════════ */
#define PWM_PERIOD 200

void Drv_PWM_Init(void)
{
    /* SysConfig 已配置 CC=200 并 startTimer=DL_TIMER_START */
    DL_TimerG_startCounter(PWM_MOTOR_L_INST);
    DL_TimerA_startCounter(PWM_MOTOR_R_INST);
}

/**
 * @brief 设置 PWM 占空比
 * @param timer_inst  定时器实例
 * @param cc_idx      CC 通道索引
 * @param duty        占空比 0~period（0=停, period=满速）
 */
void Drv_PWM_SetDuty(GPTIMER_Regs *timer_inst, uint8_t cc_idx, uint16_t duty)
{
    /* 取反映射：对外 duty=0 → CC=period(真0%), duty=period → CC=0(真100%) */
    uint16_t cc = PWM_PERIOD - duty;
    if (cc > PWM_PERIOD) cc = 0;          /* duty > period 时的保护 */
    DL_Timer_setCaptureCompareValue(timer_inst, cc, cc_idx);
}

void Drv_PWM_Start(GPTIMER_Regs *timer_inst)
{
    DL_Timer_startCounter(timer_inst);
}

void Drv_PWM_Stop(GPTIMER_Regs *timer_inst)
{
    DL_Timer_stopCounter(timer_inst);
}
