#include <stdint.h>
#include <math.h>


// CORE REGISTERS (TM4C/MSP432E4 family style) //
// Direct register control (no DriverLib) //


// --- CLOCK GATING REGISTERS ---
#define SYSCTL_RCGCGPIO_R  (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCI2C_R   (*((volatile uint32_t *)0x400FE620))
#define SYSCTL_RCGCPWM_R   (*((volatile uint32_t *)0x400FE640))
#define SYSCTL_PRGPIO_R    (*((volatile uint32_t *)0x400FEA08))

// --- GPIO PORT F (Servo PWM on PF2, LED on PF3) ---
#define GPIO_PORTF_DIR_R   (*((volatile uint32_t *)0x4005D400))
#define GPIO_PORTF_AFSEL_R (*((volatile uint32_t *)0x4005D420))
#define GPIO_PORTF_DEN_R   (*((volatile uint32_t *)0x4005D51C))
#define GPIO_PORTF_PCTL_R  (*((volatile uint32_t *)0x4005D52C))
#define GPIO_PORTF_DATA_R  (*((volatile uint32_t *)0x4005D3FC))

// --- GPIO PORT G (I2C: PG0=SCL, PG1=SDA) ---
#define GPIO_PORTG_AFSEL_R (*((volatile uint32_t *)0x4005E420))
#define GPIO_PORTG_DEN_R   (*((volatile uint32_t *)0x4005E51C))
#define GPIO_PORTG_PCTL_R  (*((volatile uint32_t *)0x4005E52C))
#define GPIO_PORTG_ODR_R   (*((volatile uint32_t *)0x4005E50C))

// --- I2C1 MODULE REGISTERS ---
#define I2C1_MSA_R         (*((volatile uint32_t *)0x40021000))
#define I2C1_MCS_R         (*((volatile uint32_t *)0x40021004))
#define I2C1_MDR_R         (*((volatile uint32_t *)0x40021008))
#define I2C1_MTPR_R        (*((volatile uint32_t *)0x4002100C))
#define I2C1_MCR_R         (*((volatile uint32_t *)0x40021020))

// --- PWM0 MODULE REGISTERS (Generator 1 on PF2) ---
#define PWM0_ENABLE_R      (*((volatile uint32_t *)0x40028008))
#define PWM0_1_CTL_R       (*((volatile uint32_t *)0x40028080))
#define PWM0_1_LOAD_R      (*((volatile uint32_t *)0x40028090))
#define PWM0_1_CMPA_R      (*((volatile uint32_t *)0x40028098))
#define PWM0_1_GENA_R      (*((volatile uint32_t *)0x400280A0))
#define PWM0_CC_R          (*((volatile uint32_t *)0x40028FC8))


// CONFIG //

#define MPU_ADDR 0x68
#define PI 3.14159265f

// Servo PWM (known-good)
// 50Hz, PWM clock 250kHz => 4us/tick => 20ms = 5000 ticks
#define SERVO_PERIOD_TICKS 5000U
#define SERVO_MIN_TICKS    200U   // ~0.8ms (wider)
#define SERVO_MAX_TICKS    600U   // ~2.4ms (wider)

// LED PF3
#define LED_PIN_MASK       0x08

// Behavior tuning
#define LOOP_MS            60

// Move ONLY when error exceeds this threshold
#define DEADZONE_DEG       12.0f

// "How much correction per degree of slouch"
#define SERVO_GAIN         2.0f

// Limit correction magnitude
#define MAX_SERVO_DELTA    40     // servo in [90-40, 90+40]

// Optional: only update if angle changes enough
#define MIN_ANGLE_STEP     2


// GLOBALS //

static float currentPitch = 0.0f;
static float neutralPitch = 0.0f;
static int   lastServoAngle = 90;


// HELPERS
static int clamp_i(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void DelayCycles(volatile uint32_t cycles) {
    while (cycles--) { __asm("nop"); }
}

// blocking delay (no SysTick dependency)
static void DelayMs(uint32_t ms) {
    while (ms--) DelayCycles(120000/3);
}

static void LED_On(void)  { GPIO_PORTF_DATA_R |= LED_PIN_MASK; }
static void LED_Off(void) { GPIO_PORTF_DATA_R &= ~LED_PIN_MASK; }


// PROTOTYPES //
void InitPeripherals(void);
void I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t data);
void I2C_ReadSequential(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t count);
void ReadIMU(void);
void SetServoAngle(int angle);


// MAIN //
int main(void) {
    InitPeripherals();
    LED_Off();

    // Optional servo proof sweep (leave it in)
    SetServoAngle(90);  DelayMs(400);
    SetServoAngle(30);  DelayMs(600);
    SetServoAngle(150); DelayMs(600);
    SetServoAngle(90);  DelayMs(600);

    // Wake MPU6050
    I2C_Write(MPU_ADDR, 0x6B, 0x00);
    DelayMs(50);

   
    // CALIBRATION (hold upright + still) //

    SetServoAngle(90);
    LED_Off();
    DelayMs(2000);

    // toss first readings
    for (int i = 0; i < 30; i++) { ReadIMU(); DelayMs(10); }

    // robust average
    float sum = 0.0f;
    for (int i = 0; i < 150; i++) {
        ReadIMU();
        sum += currentPitch;
        DelayMs(10);
    }
    neutralPitch = sum / 150.0f;

    lastServoAngle = 90;


    // CONTROL LOOP (THRESHOLD + PROPORTIONAL) //
    while (1) {

        ReadIMU();

        float error = currentPitch - neutralPitch;
        float absErr = fabsf(error);

        if (absErr < DEADZONE_DEG) {
            // Good posture: hold center, LED off
            LED_Off();

            if (lastServoAngle != 90) {
                SetServoAngle(90);
                lastServoAngle = 90;
            }
        } else {
            // Bad posture: proportional correction, LED on
            LED_On();

            int delta = (int)(SERVO_GAIN * error);
            delta = clamp_i(delta, -MAX_SERVO_DELTA, MAX_SERVO_DELTA);

            int target = 90 + delta;
            target = clamp_i(target, 45, 135);

            int diff = target - lastServoAngle;
            if (diff < 0) diff = -diff;

            if (diff >= MIN_ANGLE_STEP) {
                SetServoAngle(target);
                lastServoAngle = target;
            }
        }

        DelayMs(LOOP_MS);
    }
}

// IMU read (Accel Y/Z -> pitch) //
// pitch = atan2(ay, az) //
void ReadIMU(void) {
    uint8_t buffer[6];
    I2C_ReadSequential(MPU_ADDR, 0x3B, buffer, 6);

    int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);

    currentPitch = (atan2f((float)ay, (float)az) * 180.0f) / PI;
}


// Servo (PWM0 Generator 1 on PF2) //
void SetServoAngle(int angle) {
    angle = clamp_i(angle, 0, 180);

    uint32_t high_ticks =
        SERVO_MIN_TICKS +
        ((uint32_t)angle * (SERVO_MAX_TICKS - SERVO_MIN_TICKS)) / 180U;

    PWM0_1_CMPA_R = (SERVO_PERIOD_TICKS - high_ticks - 1U);
}

// I2C low-level // 
void I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    I2C1_MSA_R = (devAddr << 1);
    I2C1_MDR_R = regAddr;
    I2C1_MCS_R = 0x03;
    while (I2C1_MCS_R & 0x01);

    I2C1_MDR_R = data;
    I2C1_MCS_R = 0x05;
    while (I2C1_MCS_R & 0x01);
}

void I2C_ReadSequential(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t count) {
    I2C1_MSA_R = (devAddr << 1);
    I2C1_MDR_R = regAddr;
    I2C1_MCS_R = 0x03;
    while (I2C1_MCS_R & 0x01);

    I2C1_MSA_R = (devAddr << 1) | 0x01;

    for (uint8_t i = 0; i < count; i++) {
        if (i == 0 && count == 1)       I2C1_MCS_R = 0x07;
        else if (i == 0)                I2C1_MCS_R = 0x0B;
        else if (i == (count - 1))      I2C1_MCS_R = 0x05;
        else                            I2C1_MCS_R = 0x09;

        while (I2C1_MCS_R & 0x01);
        data[i] = (uint8_t)I2C1_MDR_R;
    }
}

// Peripheral init: PF2 PWM, PF3 GPIO out, PG0/PG1 I2C //
void InitPeripherals(void) {
    SYSCTL_RCGCGPIO_R |= 0x0060; // F + G
    SYSCTL_RCGCI2C_R  |= 0x0002; // I2C1
    SYSCTL_RCGCPWM_R  |= 0x0001; // PWM0

    while ((SYSCTL_PRGPIO_R & 0x0060) == 0) {;}

    // PF2 -> PWM (mux code 6)
    GPIO_PORTF_AFSEL_R |= 0x04;
    GPIO_PORTF_PCTL_R   = (GPIO_PORTF_PCTL_R & 0xFFFFF0FF) | 0x00000600;
    GPIO_PORTF_DEN_R   |= 0x04;

    // PF3 -> LED output
    GPIO_PORTF_AFSEL_R &= ~LED_PIN_MASK;
    GPIO_PORTF_DIR_R   |= LED_PIN_MASK;
    GPIO_PORTF_DEN_R   |= LED_PIN_MASK;
    LED_Off();

    // PG0/PG1 -> I2C1 (mux code 2), SDA open-drain
    GPIO_PORTG_AFSEL_R |= 0x03;
    GPIO_PORTG_ODR_R   |= 0x02;
    GPIO_PORTG_PCTL_R   = (GPIO_PORTG_PCTL_R & 0xFFFFFF00) | 0x00000022;
    GPIO_PORTG_DEN_R   |= 0x03;

    // I2C1 master enable
    I2C1_MCR_R  = 0x10;
    I2C1_MTPR_R = 0x07;

    // PWM clock divider: /64 -> 250kHz
    PWM0_CC_R     = 0x0105;
    PWM0_1_CTL_R  = 0;
    PWM0_1_GENA_R = 0x0000008C;
    PWM0_1_LOAD_R = SERVO_PERIOD_TICKS - 1U;
    PWM0_1_CMPA_R = SERVO_PERIOD_TICKS - 375U;
    PWM0_1_CTL_R |= 1;
    PWM0_ENABLE_R |= 0x04;
}