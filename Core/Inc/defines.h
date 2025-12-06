#ifndef DEFINES_H
#define DEFINES_H

// ========== Constants ==========
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#ifndef PI
  #define PI 3.14159265358979323f
#endif
#define PIx2 6.283185307179586f
#define PIo3 1.047197551196598f
#define sqrt3_1o 0.5773502691896258f
#define PI_3o 0.9549296585513721f

#define U1_TIMER LL_HRTIM_TIMER_B
#define V1_TIMER LL_HRTIM_TIMER_A
#define W1_TIMER LL_HRTIM_TIMER_E
#define U2_TIMER LL_HRTIM_TIMER_F
#define V2_TIMER LL_HRTIM_TIMER_D
#define W2_TIMER LL_HRTIM_TIMER_C

#define GATE_DRIVER_RESET_US 1U
#define deadTime 640

// #define USE_EMRAX_MOTOR
#define USE_AMK_MOTOR

#ifdef USE_EMRAX_MOTOR
#define N_STEP_ENCODER 8192
#define N_POLES 10
#define Kt 0.94f
#define J 0.02521f
#define MAX_SPEED 5500/60.0f*1e-6f*N_STEP_ENCODER
#endif
#ifdef USE_AMK_MOTOR
//#define N_STEP_ENCODER 262144
#define N_STEP_ENCODER 65536 // waveform appears to be 17-bit, first bit is too noisy so 16-bit
#define N_POLES 5
#define Kt 0.26f
#define J 2.74e-4f
#define MAX_SPEED 20000.0f/60.0f*1e-6f*N_STEP_ENCODER
#endif

// FOC constants
#define MAX_CMD_D 0.5f
#define MIN_CMD_D -0.5f
#define MAX_CMD_Q 1.3f
#define MIN_CMD_Q -1.3f

// CANBus receive IDs
#define CAN_CMD_ID 0x201708UL
#define CAN_CFG_ID 0x201608UL
// CANBus send IDs
#define CAN_STAT1_ID 0x801302UL
#define CAN_STAT2_ID 0x801402UL
#define CAN_STAT3_ID 0x801502UL
#define CAN_DEBUG_ID 0x100008UL


// ========== Macros ==========
#define DELAY_US(us) \
  do { \
    uint32_t clk_cycle_start = DWT->CYCCNT;\
    uint32_t clk_cycles = (SystemCoreClock / 1000000U) * us;\
    while ((DWT->CYCCNT - clk_cycle_start) < clk_cycles);\
  } while (0)

#endif // DEFINES_H
