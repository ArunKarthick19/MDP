/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c for Lab 4 (for Ver D STM32F board)
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2022 STMicroelectronics.
	  * All rights reserved.
	  *
	  * This software is licensed under terms that can be found in the LICENSE file
	  * in the root directory of this software component.
	  * If no LICENSE file comes with this software, it is provided AS-IS.
	  *
	  ******************************************************************************
	  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "oled.h"
	#include <string.h>
	//#include "ICM20948.h"
	//#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
	#include "oled.h"
	//#include "MPU6050.h"
	#include <math.h>
	float motor_D_speed_factor = 0.83; //0.0769
	float motor_A_speed_factor = 0.90;  // ADD THIS LINE - start with same as D, then tune

	// Add a global to reset heading correction
	static float heading_err = 0.0f;
	static uint32_t steer_t_prev = 0;

	static int32_t fwdt2_total_counts = 0;   // accumulated across both fwdt2 runs
	static int32_t fwdt2_last_counts = 0;    // last individual fwdt2 distance


	// ===== HC-SR04 Ultrasonic Sensor =====
	static volatile uint32_t us_ic_rise = 0;       // rising edge capture value
	static volatile uint32_t us_ic_fall = 0;       // falling edge capture value
	static volatile uint8_t  us_capture_done = 0;  // 1 = measurement ready
	static volatile uint8_t  us_edge_state = 0;    // 0 = waiting for rise, 1 = waiting for fall
	static float us_distance_cm = 0.0f;            // latest distance in cm
	static float gz_filtered = 0.0f;
	static uint8_t fwdt2_mode = 0;  // 1 = fwdt2 active, send capture1 on obstacle

	// Add near your other globals
	static uint8_t us_obstacle_confirm_count = 0;
	#define US_OBSTACLE_CONFIRM_NEEDED 3  // need 3 consecutive close readings
	// Add near your other globals
	static uint8_t seq_active = 0;      // 1 = sequence running
	static uint8_t seq_step = 0;        // current step
	static uint8_t seq_type = 0;        // 0 = RIGHT, 1 = LEFT

	// ===== Ultrasonic continuous mode =====
	static uint32_t us_last_trigger_ms = 0;
#define US_TRIGGER_INTERVAL_MS  60
#define US_OBSTACLE_THRESHOLD_CM 40.0f //fwdt2()
#define IR_VALID_MIN_CM          10.0f
#define IR_CLEARED_THRESHOLD_CM  40.0f   // obstacle beside if < 40, cleared if > 40
#define IR_VALID_MAX_CM          80.0f   // above 80 = garbage

	static uint8_t obstacle_count = 0;  // 0 = waiting for obs1, 1 = waiting for obs2

	static volatile uint8_t us_obstacle_stopped = 0;  // 1 = we paused due to obstacle
	static float us_live_distance_cm = -1.0f;          // latest valid reading for OLED

	// Save movement state when obstacle pauses us
	static int32_t us_saved_target_counts = 0;
	static int     us_saved_direction = 0;

	uint16_t ADC_VAL[2];

	// For 1ms-2ms pulse width (standard servo)
	int32_t servo_revl = 980;  // tune this value for revl tightness
	int32_t servo_left2 = 1000;     // 1.0ms = right (0°) //1010
	int32_t servo_leftt2 = 1050;

	int32_t servo_straight = 1500;  // 1.5ms = center (90°)
	int32_t servo_right = 2325;//2050
	int32_t servo_rightt2 = 2130;//2050
	int32_t servo_revr = 2110;// 2.0ms = left (180°)
	int32_t servo_current = 1500;    // Current servo position
	int16_t servo_angle = 90;      // Current servo angle (0-180 degrees)

	int mode = 0;              // Tracks current mode (0-5)
	int motor_direction = 0;   // 0=forward, 1=reverse
	int motor_running = 0;     // NEW: 0=stopped, 1=running

	//automatic iteration
	uint32_t last_mode_tick = 0;
	const uint32_t MODE_INTERVAL_MS = 600;  // 1 second

	// ===== UART3 command receiver (newline-terminated) =====
	static uint8_t  uart3_rx_ch;
	static char     uart3_line[64];
	static uint32_t uart3_idx = 0;
	static volatile uint8_t uart3_line_ready = 0;


	// Distance control variables
	#define WHEEL_DIAMETER_CM 6.8436  // Measure your actual wheel diameter!
	#define COUNTS_PER_ROTATION 260 // From your encoder specs
	#define CM_PER_COUNT (3.14159 * WHEEL_DIAMETER_CM / COUNTS_PER_ROTATION)


	int32_t distance_target_counts = 0;  // Target distance in encoder counts
	int32_t distance_start_position = 0; // Starting position
	int distance_mode = 0;                // 0=idle, 1=moving

	// -------- ICM-20948 minimal accel+gyro test (I2C) --------
	#define ICM20948_ADDR_7BIT   0x68                 // AD0 low -> 0x68 (common)
	#define ICM20948_ADDR        (ICM20948_ADDR_7BIT << 1) // HAL wants 8-bit addr

	#define REG_BANK_SEL         0x7F
	#define WHO_AM_I             0x00
	#define PWR_MGMT_1           0x06

	#define ACCEL_XOUT_H         0x2D
	#define GYRO_XOUT_H          0x33

//IR range
#define IR_CLEARED_THRESHOLD_CM  40.0f   // tune: when IR reads > this, obstacle is gone
#define IR_VALID_MIN_CM          3.0f    // ignore garbage readings below this

	static float ir_read_pa2(void) {
	    return 27.0f / (((float)ADC_VAL[0] * 3.3f / 4095.0f) - 0.1f) - 2.0f;
	}
	static float ir_read_pa3(void) {
	    return 27.0f / (((float)ADC_VAL[1] * 3.3f / 4095.0f) - 0.1f) - 2.0f;
	}

	// ===== Turn control using ICM-20948 gyro Z =====
	static float gyro_z_bias_dps = 0.0f;   // bias (deg/s)
	static float yaw_deg = 0.0f;           // integrated angle (deg)
	static uint32_t yaw_t_prev_ms = 0;

	static float absf(float x){ return (x < 0) ? -x : x; }

	// Forward declaration (put near the top, before icm_get_gz_dps)
	static uint8_t icm_read_accel_gyro(int16_t *ax, int16_t *ay, int16_t *az,
	                                   int16_t *gx, int16_t *gy, int16_t *gz);

	static float icm_get_gz_dps(void)
	{
	    int16_t ax, ay, az, gx, gy, gz;
	    if (!icm_read_accel_gyro(&ax,&ay,&az,&gx,&gy,&gz)) return 0.0f;

	    // Common assumption: gyro full-scale = ±250 dps => 131 LSB/dps
	    // If you configure a different range later, update this constant.
	    return ((float)gz / 131.0f);
	}
	void drive_both(int pwm_A, int pwm_D, uint8_t dir)
	{
	    if (dir == 0) {
	        Motor_reverse(pwm_A);
	        MotorD_reverse(pwm_D);
	    } else {
	        Motor_forward(pwm_A);
	        MotorD_forward(pwm_D);
	    }
	}

	static void gyro_calibrate_bias(void)
	{
	    const int N = 300;
	    float sum = 0.0f;

	    for (int i = 0; i < N; i++) {
	        sum += icm_get_gz_dps();
	        HAL_Delay(5);
	    }
	    gyro_z_bias_dps = sum / (float)N;
	}


	static volatile uint8_t oled_page = 1;  // 1..8
	static volatile uint8_t imu_ready = 0;

	static HAL_StatusTypeDef icm_write_u8(uint8_t reg, uint8_t val)
	{
	    return HAL_I2C_Mem_Write(&hi2c2, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
	}

	static HAL_StatusTypeDef icm_read(uint8_t reg, uint8_t *buf, uint16_t len)
	{
	    return HAL_I2C_Mem_Read(&hi2c2, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
	}

	static void icm_select_bank(uint8_t bank) // bank: 0..3
	{
	    // Datasheet uses USER_BANK in REG_BANK_SEL; common libs set bank in bits [5:4]
	    icm_write_u8(REG_BANK_SEL, (bank << 4));
	}

	static uint8_t icm_whoami(void)
	{
	    uint8_t v = 0x00;
	    icm_select_bank(0);
	    if (icm_read(WHO_AM_I, &v, 1) != HAL_OK) return 0x00;
	    return v;
	}

	static uint8_t icm_init_minimal(void)
	{
	    icm_select_bank(0);

	    // Wake from sleep: write 0x01 to PWR_MGMT_1 (common init sequence)
	    // (Clock select + sleep cleared)
	    if (icm_write_u8(PWR_MGMT_1, 0x01) != HAL_OK) return 0;

	    HAL_Delay(50);

	    // Verify sensor ID
	    uint8_t id = icm_whoami();
	    if (id != 0xEA) return 0;

	    return 1;
	}

	static void OLED_ShowGyro(void)
	{
	    int16_t ax, ay, az, gx, gy, gz;

	    if (!imu_ready) {
	        OLED_ShowString(0, 0, "ICM20948 FAIL");
	        OLED_Refresh_Gram();
	        return;
	    }

	    if (!icm_read_accel_gyro(&ax,&ay,&az,&gx,&gy,&gz)) {
	        OLED_ShowString(0, 0, "IMU READ ERR");
	        OLED_Refresh_Gram();
	        return;
	    }

	    // Convert to deg/s if using ±250 dps range (131 LSB/dps)
	    int gx_dps = (int)((float)gx / 131.0f);
	    int gy_dps = (int)((float)gy / 131.0f);
	    int gz_dps = (int)((float)gz / 131.0f);

	    char l1[21], l2[21], l3[21];

	    snprintf(l1, sizeof(l1), "GX:%4d  GY:%4d", gx_dps, gy_dps);
	    snprintf(l2, sizeof(l2), "GZ:%4d dps", gz_dps);
	    snprintf(l3, sizeof(l3), "rawZ:%6d", gz);

	    OLED_Clear();
	    OLED_ShowString(0, 0, "Gyro (dps)");
	    OLED_ShowString(0, 20, l1);
	    OLED_ShowString(0, 40, l2);
	    // optional extra line:
	    // OLED_ShowString(0, 50, l3);
	    OLED_Refresh_Gram();
	}


	static int16_t be16_to_i16(uint8_t hi, uint8_t lo)
	{
	    return (int16_t)((hi << 8) | lo);
	}

	static uint8_t icm_read_accel_gyro(int16_t *ax, int16_t *ay, int16_t *az,
	                                   int16_t *gx, int16_t *gy, int16_t *gz)
	{
	    uint8_t buf[12];
	    icm_select_bank(0);

	    // Read accel(6) + gyro(6) in two reads to keep it simple & reliable
	    if (icm_read(ACCEL_XOUT_H, buf, 6) != HAL_OK) return 0;
	    *ax = be16_to_i16(buf[0], buf[1]);
	    *ay = be16_to_i16(buf[2], buf[3]);
	    *az = be16_to_i16(buf[4], buf[5]);

	    if (icm_read(GYRO_XOUT_H, buf, 6) != HAL_OK) return 0;
	    *gx = be16_to_i16(buf[0], buf[1]);
	    *gy = be16_to_i16(buf[2], buf[3]);
	    *gz = be16_to_i16(buf[4], buf[5]);

	    return 1;
	}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
	void fwd(int distance_cm);  // ADD THIS LINE
	void rev(int distance_cm);  // ADD THIS

	int check_distance_complete(void);  // ADD THIS LINE
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	uint32_t counter = 0;    // Timer 2 counter
	int16_t count = 0;       // Convert counter to signed value
	int16_t no_of_tick = 50; // number of tick used in SysTick to calculate speed, in msec
	int16_t speed =0;        // speed in term of number of edges detected per Systick
	int16_t rpm = 0;         // speed in rpm number of count/sec * 60 sec  divide by 260 count per round
	int start=0;             // use to start stop the motor
	int32_t pwmVal=0;       //pwm value to control motor speed
	int32_t pwmVal_raw=0;   //pwm value before clamping - for debugging
	int16_t pwmMax = (7200-200); // Maximum PWM value = 7200 keep the maximum value to 7000
	int16_t pwmMin = 250;    // offset value to compensate for deadzone
	int err; // status for checking return

	char buf[20];      //buffer for displaying values on OLED display
	int encoder_A = 0; // encoders reading of Drive A (from complement of TIM2->CNT)
	int encoder_D = 0; // encoders reading of Drive D (from TIM5->CNT)

	int16_t position = 0;     // position of the motor (1 rotation = 260 count)
	extern int16_t oldpos;    // // see SysTick_Handler in stm32f4xx_it.c
	int16_t angle = 0;        // angle of rotation, in degree resolution = 360 degree/260 tick
	int16_t target_angle = 0; // target angle of rotation,
	int16_t position_target;  // target position
	int16_t direction;        // motor direction 0 or 1
	int16_t error;            // error between target and actual
	int32_t error_area = 0;   // area under error - to calculate I for PI implementation
	int32_t error_old, error_change, error_rate; // to calculate D for PID control
	int32_t millisOld, millisNow, dt; // to calculate I and D for PID control

	int16_t Kp = 0;  // Initial PID values
	int16_t Kd = 0;
	float_t Ki = 0;

#define PWM_ARR 7199   // same as TIM1->ARR / htim1.Init.Period


	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	    // --- Ultrasonic sensor on TIM3 CH3 ---
	    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	    {
	        if (us_edge_state == 0)
	        {
	            us_ic_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            us_edge_state = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
	        }
	        else
	        {
	            us_ic_fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	            us_edge_state = 0;
	            us_capture_done = 1;
	            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	        }
	        return;
	    }

	    // --- Original encoder code ---
	    counter = __HAL_TIM_GET_COUNTER(htim);
	    count = (int16_t)counter;
	    position = count/2;
	    angle = count/2;
	}



	volatile uint8_t do_left_turn = 0;
	//buttoncode
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	    if (GPIO_Pin == USER_PB_Pin)
	    {
	        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);

	        // simple debounce guard (optional but helpful)
	        static uint32_t last_ms = 0;
	        uint32_t now = HAL_GetTick();
	        if (now - last_ms < 200) return;
	        last_ms = now;

	        do_left_turn = 1;   // request a left turn (handled in main loop)
	    }
	}

	// For the 2nd pass (case 3): if IR reads garbage, drive until we find the obstacle, then clear it
	void fwd_until_ir_clear_v2(uint8_t sensor) {
	    us_obstacle_stopped = 0;
	    TIM2->CNT = 0;
	    TIM5->CNT = 0;
	    distance_start_position = 0;
	    position = 0;
	    motor_running = 0;
	    motor_direction = 0;
	    Servo_SetPWM(servo_straight);
	    distance_mode = 0;

	    int32_t cruise_A = (int32_t)(5500 * motor_A_speed_factor);
	    int32_t cruise_D = (int32_t)(5500 * motor_D_speed_factor);

	    char dbg[40];
	    snprintf(dbg, sizeof(dbg), "IR_V2 s=%d\r\n", sensor);
	    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);

	    float ir_now = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();

	    // Phase 1: If we already see obstacle (valid close reading), skip to phase 2
	    if (ir_now >= IR_VALID_MIN_CM && ir_now <= IR_CLEARED_THRESHOLD_CM) {
	        snprintf(dbg, sizeof(dbg), "V2_ALREADY %.1f\r\n", ir_now);
	        HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	    } else {
	        // Drive forward until IR sees the obstacle (valid close reading)
	        drive_both(cruise_A, cruise_D, 0);
	        uint32_t timeout = HAL_GetTick();
	        uint8_t found = 0;
	        while (HAL_GetTick() - timeout < 10000) {
	            HCSR04_ContinuousUpdate();
	            float ir = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();
	            if (ir >= IR_VALID_MIN_CM && ir <= IR_CLEARED_THRESHOLD_CM) {
	                found = 1;
	                snprintf(dbg, sizeof(dbg), "V2_SAW %.1f\r\n", ir);
	                HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	                break;
	            }
	            drive_both(cruise_A, cruise_D, 0);
	            HAL_Delay(5);
	        }

	        if (!found) {
	            Motor_brake();
	            HAL_UART_Transmit(&huart3, (uint8_t*)"V2_NOTFOUND\r\n", 13, 50);
	            return;
	        }
	    }

	    // Phase 2: Keep driving until obstacle clears (reads far/garbage = past it)
	    drive_both(cruise_A, cruise_D, 0);
	    uint32_t timeout = HAL_GetTick();
	    while (HAL_GetTick() - timeout < 15000) {
	        HCSR04_ContinuousUpdate();
	        float ir = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();
	        if (ir > IR_CLEARED_THRESHOLD_CM && ir < IR_VALID_MAX_CM) {
	            snprintf(dbg, sizeof(dbg), "V2_CLR %.1f\r\n", ir);
	            HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	            break;
	        }
	        drive_both(cruise_A, cruise_D, 0);
	        HAL_Delay(10);
	    }

	    Motor_brake();
	    motor_running = 0;
	    distance_mode = 0;
	    HAL_UART_Transmit(&huart3, (uint8_t*)"V2_DONE\r\n", 9, 50);
	}
	// Drive forward (encoder-corrected via main loop) until IR sensor clears obstacle
	// sensor: 0 = PA2 (left), 1 = PA3 (right)
	void fwd_until_ir_clear(uint8_t sensor) {
	    us_obstacle_stopped = 0;
	    TIM2->CNT = 0;
	    TIM5->CNT = 0;
	    distance_start_position = 0;
	    position = 0;
	    motor_running = 0;
	    motor_direction = 0;
	    Servo_SetPWM(servo_straight);
	    distance_mode = 0;

	    int32_t cruise_A = (int32_t)(5300 * motor_A_speed_factor);
	    int32_t cruise_D = (int32_t)(5300 * motor_D_speed_factor);

	    char dbg[40];
	    snprintf(dbg, sizeof(dbg), "IR_WAIT s=%d\r\n", sensor);
	    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);

	    float ir_now = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();

	    // Garbage reading = obstacle already behind, skip entirely
	    if (ir_now < IR_VALID_MIN_CM || ir_now > IR_VALID_MAX_CM) {
	        snprintf(dbg, sizeof(dbg), "IR_SKIP %.1f\r\n", ir_now);
	        HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	        return;
	    }

	    // Already seeing obstacle (valid close reading)
	    if (ir_now <= IR_CLEARED_THRESHOLD_CM) {
	        snprintf(dbg, sizeof(dbg), "IR_ALREADY %.1f\r\n", ir_now);
	        HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	    } else {
	        // Phase 1: Drive until IR SEES the obstacle
	        drive_both(cruise_A, cruise_D, 0);
	        uint32_t timeout = HAL_GetTick();
	        uint8_t found = 0;
	        while (HAL_GetTick() - timeout < 8000) {
	            HCSR04_ContinuousUpdate();
	            float ir = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();
	            if (ir >= IR_VALID_MIN_CM && ir <= IR_CLEARED_THRESHOLD_CM) {
	                found = 1;
	                snprintf(dbg, sizeof(dbg), "IR_SAW %.1f\r\n", ir);
	                HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	                break;
	            }
	            drive_both(cruise_A, cruise_D, 0);
	            HAL_Delay(10);
	        }

	        if (!found) {
	            Motor_brake();
	            HAL_UART_Transmit(&huart3, (uint8_t*)"IR_NOTFOUND\r\n", 13, 50);
	            return;
	        }
	    }

	    // Phase 2: Keep driving until obstacle clears (valid far reading)
	    drive_both(cruise_A, cruise_D, 0);
	    uint32_t timeout = HAL_GetTick();
	    while (HAL_GetTick() - timeout < 15000) {
	        HCSR04_ContinuousUpdate();
	        float ir = (sensor == 0) ? ir_read_pa2() : ir_read_pa3();
	        if (ir > IR_CLEARED_THRESHOLD_CM && ir < IR_VALID_MAX_CM) {
	            snprintf(dbg, sizeof(dbg), "IR_CLR %.1f\r\n", ir);
	            HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	            break;
	        }
	        drive_both(cruise_A, cruise_D, 0);
	        HAL_Delay(10);
	    }

	    Motor_brake();
	    motor_running = 0;
	    distance_mode = 0;
	    HAL_UART_Transmit(&huart3, (uint8_t*)"IR_DONE\r\n", 9, 50);
	}


	void MotorDrive_enable(void) {
		  //Enable PWM through TIM4-CH1/CH4 to drive the DC motor - Rev D board
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // on Motor drive A interface
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // on Motor drive A interface
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // on Motor drive D interface
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // on Motor drive D interface
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//cout
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//cout

	}

	void Motor_forward_forreverse(int pwmVal){
	    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmVal);

	    // Motor D: forward (swap CH3/CH4)
	    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmVal);
	    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
	}

	void Motor_reverse_forreverse(int  pwmVal){
		 // move robot forward send the values to serial port for display
		 Motor_direction(0); //reverse
		 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmVal); // output PWM waveform to drive motor A
		 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,pwmVal); // output PWM waveform to drive motor D - for debugging
		  //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmVal);


	}
	void MotorC_direction(uint8_t forward){
	  if(forward){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0); // IN1=0
	  } else {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0); // IN2=0
	  }
	}
	void MotorC_stop(void){
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_ARR);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWM_ARR);
	}

	void MotorC_forward(int pwm){
	  // choose which channel means "forward" for your wiring
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	}

	void MotorC_reverse(int pwm){
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);
	}

	void MotorD_forward(int pwm){
	  // Motor D forward
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);  // IN1=0
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm); // IN2=pwm
	}

	void MotorD_reverse(int pwm){
	  // Motor D reverse
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm); // IN1=pwm
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);   // IN2=0
	}
//testing
//	void Servo_SetPWM(int32_t pwm_value) {
//	    // Clamp PWM value to safe range
//	    if (pwm_value < 30) pwm_value = 30;
//	    if (pwm_value > 150) pwm_value = 150;
//
//	    // Set PWM directly on TIM12 Channel 1 (like working code)
//	    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm_value);
//
//	    servo_current = pwm_value;
//	}
	void Servo_SetPWM(int32_t us)
	{
	    if (us < 600) us = 600;
	    if (us > 2400) us = 2400;

	    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, us);
	    servo_current = us;
	}
	static float get_gz_filtered_dps(void)
	{
	    float gz = icm_get_gz_dps() - gyro_z_bias_dps;
	    gz_filtered = 0.8f * gz_filtered + 0.2f * gz;
	    return gz_filtered;
	}

	static float gyro_settle_integrate(uint32_t settle_ms)
	{
	    float extra_deg = 0.0f;
	    uint32_t t_prev = HAL_GetTick();
	    uint32_t start = t_prev;

	    while (HAL_GetTick() - start < settle_ms)
	    {
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - t_prev) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        t_prev = now;

	        float gz = icm_get_gz_dps() - gyro_z_bias_dps;
	        extra_deg += gz * dt_s;

	        //HAL_Delay(2);
	        while (HAL_GetTick() == now) {}
	    }
	    return extra_deg;
	}

	void Servo_Init(void) {
	    // Start PWM on TIM12 Channel 1
	    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	    // Set initial position to straight
	    Servo_SetPWM(servo_straight);
	}
	void Motor_stop(void) {
			//Set both IN1 and IN2 pins = '1'
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
			  // C
			  MotorC_stop();

//			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
//			  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	}

	// ACTIVE BRAKE (crisp stop)
	void Motor_brake(void)
	{
	    // Read current direction, apply opposite briefly
	    if (motor_direction == 0) {
	        // Was going forward, pulse reverse
	        Motor_forward(5000);   // your "forward" is actually reverse in your wiring
	        MotorD_forward(5000);
	    } else {
	        // Was going reverse, pulse forward
	        Motor_reverse(5000);
	        MotorD_reverse(5000);
	    }
	    HAL_Delay(60);    // short reverse pulse — tune 40-80ms

	    // Now coast to zero
	    Motor_stop();
	}

	void Motor_brake_turn(void)
	{
	    // Read current direction, apply opposite briefly
	    if (motor_direction == 0) {
	        // Was going forward, pulse reverse
	        Motor_forward(4400);   // your "forward" is actually reverse in your wiring
	        MotorD_forward(4400);
	    } else {
	        // Was going reverse, pulse forward
	        Motor_reverse(4400);
	        MotorD_reverse(4400);
	    }
	    HAL_Delay(60);    // short reverse pulse — tune 40-80ms

	    // Now coast to zero
	    Motor_stop();
	}

	void Motor_direction(uint8_t forward) {
		if (forward){// move forward
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0); // set IN1 to maximum PWM (7199) for '1'
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0); // set IN1 to maximum PWM (7199) for '1' for Drive D
		  }
		else { // reverse
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN2 to maximum PWM (7199) for '1'
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0); // set IN2 to maximum PWM (7199) for '1' for Drive D
		}
	}

	void Motor_reverse(int  pwmVal){
		 // rotate motor in clockwise forward send the values to serial port for display
		 Motor_direction(1); //forward
		 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwmVal); // output PWM waveform to drive motor A
		 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,pwmVal); // output PWM waveform to drive motor D - for debugging
		 //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmVal);  // C IN2 pwm

//		sprintf(buf, "PWM = %4dF ", pwmVal);
//		OLED_ShowString(0, 20, buf);
		//OLED_Refresh_Gram();
	}

	void Motor_forward(int  pwmVal){
		 // move robot forward send the values to serial port for display
		 Motor_direction(0); //reverse
		 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmVal); // output PWM waveform to drive motor A
		 __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,pwmVal); // output PWM waveform to drive motor D - for debugging
		  //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmVal);

//		sprintf(buf, "PWM = %4dR ", pwmVal);
//		OLED_ShowString(0, 20, buf);
		//OLED_Refresh_Gram();
	}
	static void OLED_ShowIR(void)
	{
	    float ir6 = 27.0f / (((float)ADC_VAL[0] * 3.3f / 4095.0f) - 0.1f) - 2.0f;
	    float ir7 = 27.0f / (((float)ADC_VAL[1] * 3.3f / 4095.0f) - 0.1f) - 2.0f;
	    float total_cm = fwdt2_total_counts * CM_PER_COUNT / 2.325f;

	    char line1[21];
	    char line2[21];
	    char line3[21];

	    snprintf(line1, sizeof(line1), "RIR: %6.1f cm  ", ir6);
	    snprintf(line2, sizeof(line2), "LIR: %6.1f cm  ", ir7);
	    //snprintf(line3, sizeof(line3), "FD:%.0f US:%.0f", total_cm, us_live_distance_cm);
	    snprintf(line3, sizeof(line3), "US : %.1f cm", us_live_distance_cm);

	    // No OLED_Clear() — just overwrite in place
	    OLED_ShowString(0, 0,  (uint8_t*)line1);
	    OLED_ShowString(0, 20, (uint8_t*)line2);
	    OLED_ShowString(0, 40, (uint8_t*)line3);
	    OLED_Refresh_Gram();
	}
	static void gyro_rebias_quick(void)
	{
	    const int N = 100;
	    float sum = 0.0f;

	    for (int i = 0; i < N; i++) {
	        sum += icm_get_gz_dps();
	        HAL_Delay(5);
	    }

	    gyro_z_bias_dps = sum / (float)N;
	    gz_filtered = 0.0f;
	}

	void avoid_right(void)
	{
	    Motor_stop();
	    HAL_Delay(50);
	    gyro_rebias_quick();

	    yaw_deg = 0.0f;
	    yaw_t_prev_ms = HAL_GetTick();

	    int32_t cruise_pwm_A = (int32_t)(5400 * motor_A_speed_factor);
	    int32_t cruise_pwm_D = (int32_t)(5400 * motor_D_speed_factor);

	    // Phase 1: Steer right 45 deg
	    Servo_SetPWM(servo_right);
	    while (absf(yaw_deg) < 40.0f)
	    {
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;
	        yaw_deg += (icm_get_gz_dps() - gyro_z_bias_dps) * dt_s;
	        drive_both(cruise_pwm_A, cruise_pwm_D, 0);
	        while (HAL_GetTick() == now) {}
	    }

	    // Phase 2: Steer left back to 0 (straight)
	    Servo_SetPWM(servo_left2);
	    while (absf(yaw_deg) > 2.0f)
	    {
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;
	        yaw_deg += (icm_get_gz_dps() - gyro_z_bias_dps) * dt_s;
	        drive_both(cruise_pwm_A, cruise_pwm_D, 0);
	        while (HAL_GetTick() == now) {}
	    }

	    // Phase 3: Drive straight past the obstacle
	    Servo_SetPWM(servo_straight);
	    uint32_t straight_start = HAL_GetTick();
	    while (HAL_GetTick() - straight_start < 800)  // ~800ms straight, tune this
	    {
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;
	        yaw_deg += (icm_get_gz_dps() - gyro_z_bias_dps) * dt_s;
	        drive_both(cruise_pwm_A, cruise_pwm_D, 0);
	        while (HAL_GetTick() == now) {}
	    }

	    // Phase 4: Steer left 45 deg
	    float phase4_yaw_start = yaw_deg;
	    Servo_SetPWM(servo_left2);
	    uint32_t p4_timeout = HAL_GetTick();
	    while (absf(yaw_deg - phase4_yaw_start) < 40.0f)
	    {
	        if (HAL_GetTick() - p4_timeout > 3000) break;  // safety timeout
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;
	        yaw_deg += (icm_get_gz_dps() - gyro_z_bias_dps) * dt_s;
	        drive_both(cruise_pwm_A, cruise_pwm_D, 0);
	        while (HAL_GetTick() == now) {}
	    }

	    // Phase 5: Steer right back to 0
	    Servo_SetPWM(servo_right);
	    uint32_t p5_timeout = HAL_GetTick();
	    while (absf(yaw_deg) > 2.0f)
	    {
	        if (HAL_GetTick() - p5_timeout > 3000) break;  // safety timeout
	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;
	        yaw_deg += (icm_get_gz_dps() - gyro_z_bias_dps) * dt_s;
	        drive_both(cruise_pwm_A, cruise_pwm_D, 0);
	        while (HAL_GetTick() == now) {}
	    }

	    Motor_brake_turn();
	    Servo_SetPWM(servo_straight);
	    HAL_Delay(50);

	    HAL_UART_Transmit(&huart3, (uint8_t*)"OK AVOID\r\n", 10, 50);
	}
	void turn_left_gyro_t2(float target_deg)
	{
	    //target_deg = 90;
	    //Motor_stop();
	    //HAL_Delay(200);
	    //gyro_calibrate_bias();
	    gz_filtered = 0.0f;
	    yaw_deg = 0.0f;
	    yaw_t_prev_ms = HAL_GetTick();

	    Servo_SetPWM(servo_leftt2);

	    const int fast_outer = 4800, fast_inner = 2250;
const int med_outer  = 4800, med_inner  = 2250;
const int slow_outer = 4800,  slow_inner = 2250;

	    uint32_t last_disp = 0;
	    uint32_t timeout_start = HAL_GetTick();

	    while (absf(yaw_deg) < target_deg)
	    {
	        if (HAL_GetTick() - timeout_start > 6000) break;

	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;

	        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
	        yaw_deg += gz_dps * dt_s;

	        float remaining = target_deg - absf(yaw_deg);

	        int o_pwm, i_pwm;
	        if (remaining > 25.0f) {
	            o_pwm = fast_outer; i_pwm = fast_inner;
	        } else if (remaining > 10.0f) {
	            o_pwm = med_outer;  i_pwm = med_inner;
	        } else {
	            o_pwm = slow_outer; i_pwm = slow_inner;
	        }

	        drive_both(i_pwm, o_pwm, 0);

	        if (now - last_disp > 100) {
	            last_disp = now;
	            char l1[21], l2[21];
	            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
	            snprintf(l2, sizeof(l2), "rem:%3d", (int)remaining);
	            OLED_ShowString(0, 0,  "TURN LEFT       ");
	            OLED_ShowString(0, 20, l1);
	            OLED_ShowString(0, 40, l2);
	            OLED_Refresh_Gram();
	        }
	        while (HAL_GetTick() == now) {}
	    }

	    Motor_brake_turn();

	    float settle_extra = gyro_settle_integrate(200);
	    float total_turned = absf(yaw_deg) + absf(settle_extra);

	    Servo_SetPWM(servo_straight);
	    HAL_Delay(100);

	    char dbg[40];
	    snprintf(dbg, sizeof(dbg), "TURNL actual:%.1f\r\n", total_turned);
	    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	}

	void turn_left_gyro(float target_deg)
	{
	    //target_deg = 90;
	    //Motor_stop();
	    //HAL_Delay(200);
	    //gyro_calibrate_bias();
	    gz_filtered = 0.0f;
	    yaw_deg = 0.0f;
	    yaw_t_prev_ms = HAL_GetTick();

	    Servo_SetPWM(servo_left2);

	    const int fast_outer = 4800, fast_inner = 2250;
const int med_outer  = 4650, med_inner  = 2250;
const int slow_outer = 4500,  slow_inner = 2250;

	    uint32_t last_disp = 0;
	    uint32_t timeout_start = HAL_GetTick();

	    while (absf(yaw_deg) < target_deg)
	    {
	        if (HAL_GetTick() - timeout_start > 6000) break;

	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;

	        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
	        yaw_deg += gz_dps * dt_s;

	        float remaining = target_deg - absf(yaw_deg);

	        int o_pwm, i_pwm;
	        if (remaining > 25.0f) {
	            o_pwm = fast_outer; i_pwm = fast_inner;
	        } else if (remaining > 10.0f) {
	            o_pwm = med_outer;  i_pwm = med_inner;
	        } else {
	            o_pwm = slow_outer; i_pwm = slow_inner;
	        }

	        drive_both(i_pwm, o_pwm, 0);

	        if (now - last_disp > 100) {
	            last_disp = now;
	            char l1[21], l2[21];
	            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
	            snprintf(l2, sizeof(l2), "rem:%3d", (int)remaining);
	            OLED_ShowString(0, 0,  "TURN LEFT       ");
	            OLED_ShowString(0, 20, l1);
	            OLED_ShowString(0, 40, l2);
	            OLED_Refresh_Gram();
	        }
	        while (HAL_GetTick() == now) {}
	    }

	    Motor_brake_turn();

	    float settle_extra = gyro_settle_integrate(200);
	    float total_turned = absf(yaw_deg) + absf(settle_extra);

	    Servo_SetPWM(servo_straight);
	    HAL_Delay(100);

	    char dbg[40];
	    snprintf(dbg, sizeof(dbg), "TURNL actual:%.1f\r\n", total_turned);
	    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	}

	void turn_right_gyro(float target_deg)
	{
	    //target_deg = 90;
//	    Motor_stop();
//	    HAL_Delay(200);
//	    gyro_calibrate_bias();
	    gz_filtered = 0.0f;
	    yaw_deg = 0.0f;
	    yaw_t_prev_ms = HAL_GetTick();

	    Servo_SetPWM(servo_right);

	    const int fast_outer = 4800, fast_inner = 2250;
const int med_outer  = 4650, med_inner  = 2250;
const int slow_outer = 4500,  slow_inner = 2250;


	    uint32_t last_disp = 0;
	    uint32_t timeout_start = HAL_GetTick();

	    while (absf(yaw_deg) < target_deg)
	    {
	        if (HAL_GetTick() - timeout_start > 6000) break;

	        uint32_t now = HAL_GetTick();
	        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
	        if (dt_s < 0.001f) dt_s = 0.001f;
	        if (dt_s > 0.05f)  dt_s = 0.05f;
	        yaw_t_prev_ms = now;

	        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
	        yaw_deg += gz_dps * dt_s;

	        float remaining = target_deg - absf(yaw_deg);

	        int o_pwm, i_pwm;
	        if (remaining > 25.0f) {
	            o_pwm = fast_outer; i_pwm = fast_inner;
	        } else if (remaining > 10.0f) {
	            o_pwm = med_outer;  i_pwm = med_inner;
	        } else {
	            o_pwm = slow_outer; i_pwm = slow_inner;
	        }

	        drive_both(o_pwm, i_pwm, 0);

	        if (now - last_disp > 100) {
	            last_disp = now;
	            char l1[21];
	            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
	            OLED_ShowString(0, 0,  "TURN RIGHT      ");
	            OLED_ShowString(0, 20, l1);
	            OLED_Refresh_Gram();
	        }
	        while (HAL_GetTick() == now) {}
	    }

	    Motor_brake_turn();

	    float settle_extra = gyro_settle_integrate(200);
	    float total_turned = absf(yaw_deg) + absf(settle_extra);

	    Servo_SetPWM(servo_straight);
	    HAL_Delay(100);

	    char dbg[40];
	    snprintf(dbg, sizeof(dbg), "TURNR actual:%.1f\r\n", total_turned);
	    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
	}

	void turn_right_gyro_t2(float target_deg)
		{
		    //target_deg = 90;
	//	    Motor_stop();
	//	    HAL_Delay(200);
	//	    gyro_calibrate_bias();
		    gz_filtered = 0.0f;
		    yaw_deg = 0.0f;
		    yaw_t_prev_ms = HAL_GetTick();

		    Servo_SetPWM(servo_rightt2);

		    const int fast_outer = 4800, fast_inner = 2250;
	const int med_outer  = 4800, med_inner  = 2250;
	const int slow_outer = 4800,  slow_inner = 2250;

		    uint32_t last_disp = 0;
		    uint32_t timeout_start = HAL_GetTick();

		    while (absf(yaw_deg) < target_deg)
		    {
		        if (HAL_GetTick() - timeout_start > 8000) break;

		        uint32_t now = HAL_GetTick();
		        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
		        if (dt_s < 0.001f) dt_s = 0.001f;
		        if (dt_s > 0.05f)  dt_s = 0.05f;
		        yaw_t_prev_ms = now;

		        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
		        yaw_deg += gz_dps * dt_s;

		        float remaining = target_deg - absf(yaw_deg);

		        int o_pwm, i_pwm;
		        if (remaining > 25.0f) {
		            o_pwm = fast_outer; i_pwm = fast_inner;
		        } else if (remaining > 10.0f) {
		            o_pwm = med_outer;  i_pwm = med_inner;
		        } else {
		            o_pwm = slow_outer; i_pwm = slow_inner;
		        }

		        drive_both(o_pwm, i_pwm, 0);

		        if (now - last_disp > 100) {
		            last_disp = now;
		            char l1[21];
		            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
		            OLED_ShowString(0, 0,  "TURN RIGHT      ");
		            OLED_ShowString(0, 20, l1);
		            OLED_Refresh_Gram();
		        }
		        while (HAL_GetTick() == now) {}
		    }

		    Motor_brake_turn();

		    float settle_extra = gyro_settle_integrate(200);
		    float total_turned = absf(yaw_deg) + absf(settle_extra);

		    Servo_SetPWM(servo_straight);
		    HAL_Delay(100);

		    char dbg[40];
		    snprintf(dbg, sizeof(dbg), "TURNR actual:%.1f\r\n", total_turned);
		    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
		}


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	    if (huart->Instance == USART3)
	    {
	    	//HAL_UART_Transmit(&huart3, (uint8_t*)"*", 1, 10);// only for debugging, ignore

	        char c = (char)uart3_rx_ch;

	        if (!uart3_line_ready)
	        {
	            if (c == '\n' || c == '\r')
	            {
	                if (uart3_idx > 0) {
	                    uart3_line[uart3_idx] = '\0';
	                    uart3_line_ready = 1;
	                }
	                uart3_idx = 0;
	            }
	            else
	            {
	                if (uart3_idx < sizeof(uart3_line) - 1) {
	                    uart3_line[uart3_idx++] = c;
	                } else {
	                    // overflow -> reset
	                    uart3_idx = 0;
	                }
	            }
	        }

	        // re-arm RX
	        HAL_UART_Receive_IT(&huart3, &uart3_rx_ch, 1);
	    }
	}





	int16_t PID_Control(){
		  //Control Loop
		  if (abs(error)>2){ //more than 2 degree difference
			  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); //Buzzer
			  //angle = (int)(position*360/265);  // supposed to be 260 ticks/rev, measured = 265
			  //error = target_angle - angle;

			sprintf(buf, "error = %5d, ", (int)error);
			HAL_UART_Transmit(&huart2, buf, 14, HAL_MAX_DELAY); // Send through BT @9600
			sprintf(buf, " Kp = %5d, ", Kp);
			HAL_UART_Transmit(&huart2, buf, 14, HAL_MAX_DELAY); // Send through BT @9600

//			if (error > 0)
//			  direction = 0; //forward
//			else
//			  direction = 1; //reverse direction

			millisNow = HAL_GetTick();
			dt = (millisNow - millisOld); // time elapsed in millisecond
			millisOld = millisNow; // store the current time for next round

			error_area = error_area + error*dt; // area under error for Ki

			error_change = error - error_old; // change in error
			error_old = error; //store the error for next round
			error_rate = (error_change)*1000/dt; // for Kd - dt in millsecond

//			pwmVal = (int)(error*Kp + error_area*Ki/10000);  // PI, D is below
//			pwmVal_raw = (int)(error*Kp); // for debugging
			pwmVal = (int)(abs(error)*Kp + abs(error_area)*Ki/10000);  // PI, D is below
			pwmVal_raw = (int)(abs(error)*Kp); // for debugging


			// Adjust pwmVal to account for deadzone
			if (pwmVal>0)
				pwmVal = pwmVal+pwmMin;
			if (pwmVal<0)
				pwmVal = pwmVal-pwmMin;

			if (pwmVal > pwmMax)  // Clamp the PWM to its maximum positive value
			   pwmVal = pwmMax;


			pwmVal = (int)(pwmVal + error_rate*Kd);  // + D which is subtration

			if (pwmVal < -pwmMax)  // Clamp the PWM to its minimum negative value
			   pwmVal = -pwmMax;

			sprintf(buf, " Praw = %6d, ", pwmVal_raw);
			HAL_UART_Transmit(&huart2, buf, 17, HAL_MAX_DELAY); // Send through BT @9600

			sprintf(buf, " Pwm = %6d ", pwmVal);
			HAL_UART_Transmit(&huart2, buf, 17, HAL_MAX_DELAY); // Send through BT @9600

			buf[0] = '\r';
			buf[1] = '\n';  // move to next line on serial port
			HAL_UART_Transmit(&huart2, buf, 2, HAL_MAX_DELAY); // Send through USB port

			return(pwmVal);
			} // if loop
	}

	/* USER CODE BEGIN 0 */
	// ... existing code ...
	// ===== HC-SR04 Ultrasonic Functions =====

	// ===== HC-SR04 Ultrasonic Functions =====

	void HCSR04_Init(void)
	{
	    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	}

	void HCSR04_Trigger(void)
	{
	    us_capture_done = 0;
	    us_edge_state = 0;
	    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

	    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	    for (volatile int i = 0; i < 720; i++) { __NOP(); }
	    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	}

	float HCSR04_ProcessCapture(void)
	{
	    if (!us_capture_done) return -1.0f;

	    uint32_t pulse_us;
	    if (us_ic_fall >= us_ic_rise)
	        pulse_us = us_ic_fall - us_ic_rise;
	    else
	        pulse_us = (65535 - us_ic_rise) + us_ic_fall + 1;

	    float dist = (float)pulse_us * 0.0343f / 2.0f;

	    us_capture_done = 0;

	    if (dist < 2.0f || dist > 400.0f)
	        return -1.0f;

	    return dist;
	}

	// Call this every loop iteration — handles trigger, read, OLED, and obstacle logic
	void HCSR04_ContinuousUpdate(void)
	{
	    uint32_t now = HAL_GetTick();

	    // --- Trigger at regular intervals ---
	    if (now - us_last_trigger_ms >= US_TRIGGER_INTERVAL_MS)
	    {
	        // First, process the previous capture (if any)
	        float dist = HCSR04_ProcessCapture();
	        if (dist > 0)
	        {
	            us_live_distance_cm = dist;
	        }

	        // Trigger next measurement
	        HCSR04_Trigger();
	        us_last_trigger_ms = now;
	    }

	    // --- Obstacle detection during movement ---
	    if (us_live_distance_cm > 0)
	    {
	        if (distance_mode == 1 && motor_direction == 0 && !us_obstacle_stopped)
	        {
	            if (us_live_distance_cm < US_OBSTACLE_THRESHOLD_CM)
	            {
	                us_obstacle_confirm_count++;
	                if (us_obstacle_confirm_count >= US_OBSTACLE_CONFIRM_NEEDED)
	                {
	                    us_obstacle_confirm_count = 0;
	                    Motor_brake();
	                    motor_running = 0;
	                    us_obstacle_stopped = 1;

	                    if (fwdt2_mode)
	                    {
	                        fwdt2_mode = 0;
	                        distance_mode = 0;

	                        int32_t encA = abs((int16_t)TIM2->CNT);
	                        int32_t encD = abs((int16_t)TIM5->CNT);
	                        fwdt2_last_counts = (encA + encD) / 2;
	                        fwdt2_total_counts += fwdt2_last_counts;

	                        float last_cm = fwdt2_last_counts * CM_PER_COUNT / 2.325f;
	                        float total_cm = fwdt2_total_counts * CM_PER_COUNT / 2.325f;

	                        char msg[50];
	                        snprintf(msg, sizeof(msg), "FWD2D:%.1f T:%.1f\r\n", last_cm, total_cm);
	                        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 50);

	                        obstacle_count++;
	                        if (obstacle_count == 1)
	                            HAL_UART_Transmit(&huart3, (uint8_t*)"capture1\r\n", 10, 50);
	                        else
	                            HAL_UART_Transmit(&huart3, (uint8_t*)"capture2\r\n", 10, 50);
	                    }
	                    else
	                    {
	                        char msg[40];
	                        snprintf(msg, sizeof(msg), "OBSTACLE:%.1f\r\n", us_live_distance_cm);
	                        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
	                    }
	                }
	            }
	            else
	            {
	                us_obstacle_confirm_count = 0;
	            }
	        }

	        // Check if obstacle has cleared — resume movement
	        if (us_obstacle_stopped && distance_mode == 1)
	        {
	            if (us_live_distance_cm >= US_OBSTACLE_THRESHOLD_CM + 10.0f)
	            {
	                us_obstacle_stopped = 0;
	                motor_running = 1;
	                error_area = 0;
	                error_old = 0;

	                HAL_UART_Transmit(&huart3, (uint8_t*)"RESUMED\r\n", 9, 10);
	            }
	        }
	    }


	}
	void fwdt2(void) {
		if (obstacle_count == 0) {
		        fwdt2_total_counts = 0;  // fresh run
		    }
	    TIM2->CNT = 0;
	    TIM5->CNT = 0;
	    position = 0;
	    distance_start_position = 0;
	    distance_target_counts = 99999;

	    motor_running = 1;
	    motor_direction = 0;
	    Servo_SetPWM(servo_straight);
	    distance_mode = 1;
	    fwdt2_mode = 1;

	    error = 0;
	    error_old = 0;
	    error_area = 0;
	    err = 0;

	    heading_err = 0.0f;          // <-- add here
	    steer_t_prev = HAL_GetTick(); // <-- add here
	}
	void fwd(int distance_cm) {
	    // Convert cm to encoder counts
		 distance_target_counts = (int32_t)((distance_cm / CM_PER_COUNT) * 2.325);


	    // Reset encoders to zero
	    TIM2->CNT = 0;
	    TIM5->CNT = 0;

	    // Store starting position
	    distance_start_position = 0;
	    position = 0;

	    // Set to forward direction, straight
	    motor_running = 1;
	    motor_direction = 0;  // 0 = forward
	    Servo_SetPWM(servo_straight);

	    // Enable distance mode
	    distance_mode = 1;

	    // Reset error tracking
	    error = 0;
	    error_old = 0;
	    error_area = 0;
	    err = 0;

	    sprintf(buf, "Moving %dcm\r\n", distance_cm);
	    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    heading_err = 0.0f;          // <-- add here
	    steer_t_prev = HAL_GetTick(); // <-- add here
	}


	void rev(int distance_cm) {
	    distance_target_counts = (int32_t)((distance_cm / CM_PER_COUNT) * 2.325);

	    TIM2->CNT = 0;
	    TIM5->CNT = 0;

	    distance_start_position = 0;
	    position = 0;

	    motor_running = 1;
	    motor_direction = 1;  // 1 = reverse
	    Servo_SetPWM(servo_straight);

	    distance_mode = 1;

	    error = 0;
	    error_old = 0;
	    error_area = 0;
	    err = 0;

	    sprintf(buf, "Reverse %dcm\r\n", distance_cm);
	    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	}


	void turn_left_gyro_final(float target_deg)
		{
			target_deg = 90;
		    Motor_stop();
		    HAL_Delay(200);
		    gyro_calibrate_bias();
		    gz_filtered = 0.0f;
		    yaw_deg = 0.0f;
		    yaw_t_prev_ms = HAL_GetTick();

		    Servo_SetPWM(servo_left2);

		    // --- speed tiers ---
		    const int fast_outer = 4900, fast_inner = 2250;
	const int med_outer  = 4700, med_inner  = 2250;
	const int slow_outer = 4500,  slow_inner = 2250;




		    uint32_t last_disp = 0;
		    uint32_t timeout_start = HAL_GetTick();

		    while (absf(yaw_deg) < target_deg)
		    {
		        if (HAL_GetTick() - timeout_start > 6000) break;

		        uint32_t now = HAL_GetTick();
		        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
		        if (dt_s < 0.001f) dt_s = 0.001f;
		        if (dt_s > 0.05f)  dt_s = 0.05f;
		        yaw_t_prev_ms = now;

		        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
		        yaw_deg += gz_dps * dt_s;

		        float remaining = target_deg - absf(yaw_deg);

		        int o_pwm, i_pwm;
		        if (remaining > 25.0f) {
		            o_pwm = fast_outer; i_pwm = fast_inner;
		        } else if (remaining > 10.0f) {
		            o_pwm = med_outer;  i_pwm = med_inner;
		        } else {
		            o_pwm = slow_outer; i_pwm = slow_inner;
		        }

		        // Left turn: A=inner, D=outer
		        drive_both(i_pwm, o_pwm, 0);

		        if (now - last_disp > 100) {
		            last_disp = now;
		            char l1[21], l2[21];
		            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
		            snprintf(l2, sizeof(l2), "rem:%3d", (int)remaining);
		            OLED_Clear();
		            OLED_ShowString(0, 0, "TURN LEFT");
		            OLED_ShowString(0, 20, l1);
		            OLED_ShowString(0, 40, l2);
		            OLED_Refresh_Gram();
		        }
		        //HAL_Delay(2);
		        while (HAL_GetTick() == now) {}
		    }

		    Motor_brake();

		    // Settle: measure how much we coast after stopping
		    float settle_extra = gyro_settle_integrate(200);
		    float total_turned = absf(yaw_deg) + absf(settle_extra);

		    Servo_SetPWM(servo_straight);
		    HAL_Delay(100);

		    // Debug: send actual turn to RPi
		    char dbg[40];
		    snprintf(dbg, sizeof(dbg), "TURNL actual:%.1f\r\n", total_turned);
		    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
		}

	void turn_right_gyro_final(float target_deg)
		{
			target_deg = 90;
		    Motor_stop();
		    HAL_Delay(200);
		    gyro_calibrate_bias();
		    gz_filtered = 0.0f;
		    yaw_deg = 0.0f;
		    yaw_t_prev_ms = HAL_GetTick();

		    Servo_SetPWM(servo_right);

		    const int fast_outer = 4900, fast_inner = 2250;
	const int med_outer  = 4700, med_inner  = 2250;
	const int slow_outer = 4500,  slow_inner = 2250;


		    uint32_t last_disp = 0;
		    uint32_t timeout_start = HAL_GetTick();

		    while (absf(yaw_deg) < target_deg)
		    {
		        if (HAL_GetTick() - timeout_start > 6000) break;

		        uint32_t now = HAL_GetTick();
		        float dt_s = (now - yaw_t_prev_ms) / 1000.0f;
		        if (dt_s < 0.001f) dt_s = 0.001f;
		        if (dt_s > 0.05f)  dt_s = 0.05f;
		        yaw_t_prev_ms = now;

		        float gz_dps = icm_get_gz_dps() - gyro_z_bias_dps;
		        yaw_deg += gz_dps * dt_s;

		        float remaining = target_deg - absf(yaw_deg);

		        int o_pwm, i_pwm;
		        if (remaining > 25.0f) {
		            o_pwm = fast_outer; i_pwm = fast_inner;
		        } else if (remaining > 10.0f) {
		            o_pwm = med_outer;  i_pwm = med_inner;
		        } else {
		            o_pwm = slow_outer; i_pwm = slow_inner;
		        }

		        // Right turn: A=outer, D=inner
		        drive_both(o_pwm, i_pwm, 0);

		        if (now - last_disp > 100) {
		            last_disp = now;
		            char l1[21];
		            snprintf(l1, sizeof(l1), "Yaw:%3d/%3d", (int)absf(yaw_deg), (int)target_deg);
		            OLED_Clear();
		            OLED_ShowString(0, 0, "TURN RIGHT");
		            OLED_ShowString(0, 20, l1);
		            OLED_Refresh_Gram();
		        }
		        //HAL_Delay(2);
		        while (HAL_GetTick() == now) {}
		    }

		    Motor_brake();

		    float settle_extra = gyro_settle_integrate(200);
		    float total_turned = absf(yaw_deg) + absf(settle_extra);

		    Servo_SetPWM(servo_straight);
		    HAL_Delay(100);

		    char dbg[40];
		    snprintf(dbg, sizeof(dbg), "TURNR actual:%.1f\r\n", total_turned);
		    HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);
		}

	int check_distance_complete() {
	    if (distance_mode == 0) return 1;
	    int32_t distance_traveled = abs(position - distance_start_position);
	    if (distance_traveled >= distance_target_counts) {
	        Motor_brake();
	        distance_mode = 0;
	        motor_running = 0;

	        // Skip buzzer during sequence
	        if (!seq_active) {
	            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	            HAL_Delay(200);
	            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	        }

	        sprintf(buf, "C0mp1et3! %d counts\r\n", distance_traveled);
	        HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	        HAL_UART_Transmit(&huart3, (uint8_t*)"OK D\r\n", 6, HAL_MAX_DELAY);
	        return 1;
	    }
	    return 0;
	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  uint8_t *oled_buf; // buffer to store value to be display on OLED
	  uint8_t i, status; // status for checking return

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 2);
  /* USER CODE BEGIN 2 */


	  MotorDrive_enable(); // enable PWM needed to drive MotroDrive A and D
	  Servo_Init();

	  HCSR04_Init(); // Initialize HC-SR04 ultrasonic sensor

	  // start TIM2/TIM5-Encoder to read Motor rotation in interrupt mode
	  // Hall sensors produce 13 ticks/counts per turn, gear ratio = 20
	  // 260 count per rotation of output (wheel)
	  // 360 degree = 260 ticks/counts
	  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); // Motor Drive A
	  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL); // Motor Drive D
	  rpm = (int)((1000/no_of_tick) * 60/260 * 1/dt);  // For calculating motor rpm - by multiplying it with speed value

	  OLED_Init();
	  OLED_ShowString(10, 5, "MDP SC2079"); // show message on OLED display at line 5)
	  OLED_ShowString(40, 30, "GROUP 25"); // show message on OLED display at line 30)
	  oled_buf = "Task 2 lol"; // anther way to show message through buffer
	  OLED_ShowString(10,50, oled_buf); //another message at line 50
	  imu_ready = icm_init_minimal();
	  gyro_calibrate_bias();
	  HAL_UART_Receive_IT(&huart3, &uart3_rx_ch, 1);


	  uint8_t sbuf[] = "SC2104\n\r";  // send to serial port
	  HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through Serial Port @115200
	  HAL_UART_Transmit(&huart2, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through BT @9600

	  OLED_Refresh_Gram();
	  HAL_Delay(3000); // pause for 3 second to show message
	  OLED_Clear(); // get display ready


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  start = 0;
	  angle = 0;
	  target_angle = 1000; // rotate 1000 degree
	  error = target_angle - angle;
	  error_old = 0;
	  error_area = 0;

	  // motor drive here
	  OLED_Clear();
	  //OLED_ShowString(0, 0, "Target: ");
	  //OLED_ShowString(0, 10, "Rotated: ");
	  //OLED_ShowString(0, 30, "RPM = ");
	  //sprintf(buf, "%4d", target_angle);//Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulse per revolution
	  //OLED_ShowString(60, 0, buf);

	  //OLED_ShowString(15, 40, "Press User"); // show message on OLED display at line 40)
	  //OLED_ShowString(0, 50, "button to stop"); // show message on OLED display at line 50)
	  OLED_Refresh_Gram();

	  Kp = 5;   // range: 1 to 10
	  Ki = 3;   // range 0 to 3
	  Kd = 1;   // range: 0 to 3

	  if (target_angle > 0)  // Determine rotation direction)
		 direction = 0;
	  else
		 direction = 1;

	  start = 1; // do a step response upon reset and power up
	  MotorDrive_enable(); // enable PWM needed to drive MotroDrive A and D
	  millisOld = HAL_GetTick(); // get time value before starting - for PID



	  while (1){
//		  if (start==0){ // reset and wait for the User PB to be pressed
//			  Motor_stop();
//			  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10); // LED
//			  OLED_ShowString(15, 40, "Press User"); // show message on OLED display at line 40)
//			  OLED_ShowString(0, 50, "button to start"); // show message on OLED display at line 50)
//			  OLED_Refresh_Gram();
//			  err = 0;// for checking whether error has settle down near to zero
//			  angle = 0;
//			  error_old = 0;
//			  error_area = 0;
//			  error = target_angle - angle;
//			  }
//		  while (start==0){ //wait for the User PB to be pressed
//			  HAL_Delay(500);
//			  millisOld = HAL_GetTick(); // get time value before starting - for PID
//			  }

		  uint32_t now = HAL_GetTick();

//		  if (now - last_mode_tick >= MODE_INTERVAL_MS)
//		  {
//		      last_mode_tick = now;
//
//		      mode++;
//		      if (mode > 18)
//		          mode = 0;
//
//		      Apply_Mode(mode);
//		  }


		  while (1) {
			  HCSR04_ContinuousUpdate();
			  OLED_ShowIR();

			    // Debug: Send heartbeat every 2 seconds
//			    static uint32_t last_heartbeat = 0;
//			    uint32_t now = HAL_GetTick();
//
//			    if (now - last_heartbeat > 2000) {
//			        last_heartbeat = now;
//			        HAL_UART_Transmit(&huart3, (uint8_t*)"beforeif\r\n", 11, HAL_MAX_DELAY);
//			    }

//			    static uint8_t forced_once = 0;
//			    if (!forced_once) {
//			        forced_once = 1;
//
//			        strcpy(uart3_line, "OLED:FORCED TEST");
//			        //uart3_line_ready = 1;
//			    }

			    //UART3_Poll_Line();

			  if (uart3_line_ready)
			  {

			      uart3_line_ready = 0;


			      HAL_UART_Transmit(&huart3, (uint8_t*)"CMD=[", 5, 10);
			      HAL_UART_Transmit(&huart3, (uint8_t*)uart3_line, strlen(uart3_line), 10);
			      HAL_UART_Transmit(&huart3, (uint8_t*)"]\r\n", 3, 10);

			      // Example command: OLED:Hello world
			      if (strncmp(uart3_line, "OLED:", 5) == 0)
			      {
			          const char *msg = uart3_line + 5;

			          // Debug: Send what we're about to display
			          HAL_UART_Transmit(&huart3, (uint8_t*)"111111\r\n", 11, HAL_MAX_DELAY);
			          char debug[80];
			          HAL_UART_Transmit(&huart3, (uint8_t*)"22222\r\n", 11, HAL_MAX_DELAY);
			          snprintf(debug, sizeof(debug), "DEBUG: Displaying '%s'\r\n", msg);
			          HAL_UART_Transmit(&huart3, (uint8_t*)"33333\r\n", 11, HAL_MAX_DELAY);
			          HAL_UART_Transmit(&huart3, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);
			          HAL_UART_Transmit(&huart3, (uint8_t*)"44444\r\n", 11, HAL_MAX_DELAY);

			          // Try to display on OLED
			          OLED_Clear();
			          OLED_ShowString(0, 0, "RPi says:");
			          OLED_ShowString(0, 20, (char*)msg);
			          OLED_Refresh_Gram();

			          // Send acknowledgment
			          const char *ack = "OK OLED\r\n";
			          HAL_UART_Transmit(&huart3, (uint8_t*)ack, strlen(ack), HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "TURNL:", 6) == 0)
			      {
			          int deg = atoi(uart3_line + 6);
			          turn_left_gyro((float)deg);
			          HAL_UART_Transmit(&huart3, (uint8_t*)"OK TURNL\r\n", 10, HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "TURNR:", 6) == 0)
			      {
			          int deg = atoi(uart3_line + 6);
			          turn_right_gyro((float)deg);
			          HAL_UART_Transmit(&huart3, (uint8_t*)"OK TURNR\r\n", 10, HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "FWD:", 4) == 0)
			      {
			          int cm = atoi(uart3_line + 4);  // parse the number after "FWD:"
			          fwd(cm);
			          //HAL_UART_Transmit(&huart3, (uint8_t*)"OK FWD\r\n", 8, HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "REV:", 4) == 0)
			      {
			          int cm = atoi(uart3_line + 4);
			          rev(cm);
			          //HAL_UART_Transmit(&huart3, (uint8_t*)"OK REV\r\n", 8, HAL_MAX_DELAY);
			      }
//			      else if (strncmp(uart3_line, "REVL:", 5) == 0)
//			      {
//			          int cm = atoi(uart3_line + 5);
//			          revl(cm);
//			          HAL_UART_Transmit(&huart3, (uint8_t*)"OK REVL\r\n", 9, HAL_MAX_DELAY);
//			      }
//			      else if (strncmp(uart3_line, "REVR:", 5) == 0)
//			      {
//			          int cm = atoi(uart3_line + 5);
//			          revr(cm);
//			          HAL_UART_Transmit(&huart3, (uint8_t*)"OK REVR\r\n", 9, HAL_MAX_DELAY);
//			      }
			      else if (strncmp(uart3_line, "DIST", 4) == 0)
			      {
			          char dist_buf[32];
			          if (us_live_distance_cm > 0)
			              snprintf(dist_buf, sizeof(dist_buf), "DIST:%.1f\r\n", us_live_distance_cm);
			          else
			              snprintf(dist_buf, sizeof(dist_buf), "DIST:ERR\r\n");
			          HAL_UART_Transmit(&huart3, (uint8_t*)dist_buf, strlen(dist_buf), HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "FWDT2", 5) == 0)
			      {
			          fwdt2();
			          //HAL_UART_Transmit(&huart3, (uint8_t*)"CAPTURE1\r\n", 10, HAL_MAX_DELAY);
			      }
			      else if (strncmp(uart3_line, "RIGHT2", 6) == 0)
			      {
			          seq_type = 2;
			          seq_step = 0;
			          seq_active = 1;
			      }
			      else if (strncmp(uart3_line, "LEFT2", 5) == 0)
			      {
			          seq_type = 3;
			          seq_step = 0;
			          seq_active = 1;
			      }
			      else if (strncmp(uart3_line, "RIGHT", 5) == 0)
			      {
			          seq_type = 0;
			          seq_step = 0;
			          seq_active = 1;
			      }
			      else if (strncmp(uart3_line, "LEFT", 4) == 0)
			      {
			          seq_type = 1;
			          seq_step = 0;
			          seq_active = 1;
			      }
			      else
			      {
			          HAL_UART_Transmit(&huart3, (uint8_t*)"ERR Unknown cmd\r\n", 17, HAL_MAX_DELAY);
			      }
			  }

			  if (do_left_turn) {
			      do_left_turn = 0;
			      fwd(50);

			  }

//			  do_left_turn = 1;
//
//			  static uint32_t last_oled_ms = 0;
//			  uint32_t now_ms = HAL_GetTick();
//
//			  if (now_ms - last_oled_ms > 200) {   // update 5 times/sec
//			      last_oled_ms = now_ms;
//			      OLED_ShowGyro();
//			  }


//			  if (do_left_turn) {
//			      do_left_turn = 0;
//
//			      motor_running = 0;
//			      Motor_stop();
//
//			      OLED_Clear();
//			      OLED_ShowString(0, 0, "Turning right");
//			      OLED_ShowString(0, 20, "Target: 90 deg");
//			      OLED_Refresh_Gram();
//
//			      //turn_left_gyro(90.0f);
//			      turn_right_gyro(90.0f);
//
//			      OLED_Clear();
//			      OLED_ShowString(0, 0, "Turn done");
//			      OLED_Refresh_Gram();
//
//			      motor_running = 1;
//			  }

		      // Check if distance movement is complete
		      if (distance_mode == 1) {
		          check_distance_complete();
		      }

		      // --- Sequence state machine ---
		      // --- Sequence state machine ---
		      if (seq_active && motor_running == 0 && distance_mode == 0)
		      {
		          if (seq_type == 0)  // RIGHT (obstacle 1, small 10x10)
		          {
		              switch (seq_step) {
		                  case 0: turn_right_gyro(50.0f); seq_step++; break;
		                  case 1: turn_left_gyro(50.0f);  seq_step++; break;
		                  case 2: fwd(12); seq_step++; break;
		                  case 3: turn_left_gyro(50.0f);  seq_step++; break;
		                  case 4: turn_right_gyro(50.0); seq_step++; break;
		                  case 5: fwdt2(); seq_step++; break;
		                  case 6:
		                      seq_active = 0;
		                      //HAL_UART_Transmit(&huart3, (uint8_t*)"capture2\r\n", 10, 50);
		                      break;
		              }
		          }

		          else if (seq_type == 1)  // LEFT (obstacle 1, small 10x10)
		          {
		              switch (seq_step) {
		                  case 0: turn_left_gyro(50.0f);  seq_step++; break;
		                  case 1: turn_right_gyro(50.0f); seq_step++; break;
		                  case 2: fwd(12); seq_step++; break;
		                  case 3: turn_right_gyro(50.0f); seq_step++; break;
		                  case 4: turn_left_gyro(50.0f);  seq_step++; break;
		                  case 5: fwdt2(); seq_step++; break;
		                  case 6:
		                      seq_active = 0;
		                      //HAL_UART_Transmit(&huart3, (uint8_t*)"capture2\r\n", 9, 50);
		                      break;
		              }
		          }
		          else if (seq_type == 2)  // RIGHT2
		          {
		              char dbg[60];
		              float rir = ir_read_pa2();
		              float lir = ir_read_pa3();
		              snprintf(dbg, sizeof(dbg), "R2 s=%d RIR=%.1f LIR=%.1f mr=%d dm=%d uo=%d\r\n",
		                       seq_step, rir, lir, motor_running, distance_mode, us_obstacle_stopped);
		              HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);

		              switch (seq_step) {
		                  case 0: turn_right_gyro(90.0f);       seq_step++; break;
		                  case 1: fwd_until_ir_clear(1);        seq_step++; break;
		                  case 2: turn_left_gyro_t2(178.0f);    seq_step++; break;
		                  case 3: fwd_until_ir_clear_v2(1);     seq_step++; break;
		                  case 4: turn_left_gyro(85.0f);        seq_step++; break;
		                  case 5: {
		                      int drive_back_cm = (int)(fwdt2_total_counts * CM_PER_COUNT / 2.325f);
		                      drive_back_cm - 50;
		                      fwd(drive_back_cm);
		                      seq_step++;
		                      break;
		                  }
		                  case 6: turn_left_gyro(90.0f);            seq_step++; break;
						  case 7: turn_right_gyro(90.0f);            seq_step++; break;
						  case 8: fwdt2();            seq_step++; break;
		                  case 9:
		                      seq_active = 0;
		                      HAL_UART_Transmit(&huart3, (uint8_t*)"OK RIGHT2\r\n", 11, 50);
		                      break;
		              }
		          }

		          else if (seq_type == 3)  // LEFT2 (obstacle 2, IR-guided, mirror)
		          {

		        	  char dbg[60];
							  float rir = ir_read_pa2();
							  float lir = ir_read_pa3();
							  snprintf(dbg, sizeof(dbg), "L2 s=%d RIR=%.1f LIR=%.1f mr=%d dm=%d uo=%d\r\n",
									   seq_step, rir, lir, motor_running, distance_mode, us_obstacle_stopped);
							  HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), 50);

							  switch (seq_step) {
							      case 0: turn_left_gyro(90.0f);            seq_step++; break;
							      case 1: fwd_until_ir_clear(0);            seq_step++; break;
							      case 2: turn_right_gyro_t2(178.0f);       seq_step++; break;
							      case 3: fwd_until_ir_clear_v2(0);         seq_step++; break;
							      case 4: turn_right_gyro(85.0f);           seq_step++; break;
							      case 5: {
							          int drive_back_cm = (int)(fwdt2_total_counts * CM_PER_COUNT / 2.325f);
							          drive_back_cm - 50;
							          fwd(drive_back_cm);
							          seq_step++;
							          break;
							      }
							      case 6: turn_right_gyro(90.0f);            seq_step++; break;
							      case 7: turn_left_gyro(90.0f);            seq_step++; break;
							      case 8: fwdt2();            seq_step++; break;

							      case 9:
							          seq_active = 0;
							          HAL_UART_Transmit(&huart3, (uint8_t*)"OK LEFT2\r\n", 10, 50);
							          break;
							  }
		          }
		      }

		      // Original mode cycling code (disabled when in distance mode)
		      if (distance_mode == 0) {
		          uint32_t now = HAL_GetTick();
		          if (now - last_mode_tick >= MODE_INTERVAL_MS) {
		              last_mode_tick = now;
		              mode++;
//		              if (mode > 18) mode = 0;
//		              Apply_Mode(mode);
		          }
		      }
		  pwmVal = PID_Control(); // call the PID control loop calculation
		  //pwmVal = 600;          // this will overwrite PID control above
		  //error = 5;              // to overwrite control loop checking


		  int32_t pwm = pwmVal;
		  if (pwm < 0) pwm = -pwm;        // abs()
		  if (pwm > pwmMax) pwm = pwmMax; // clamp

		  // Calculate separate speeds for C and D
		  // Calculate separate speeds for A and D
		  // Calculate separate speeds for A and D
//		  int32_t pwm_A = (int32_t)(pwm * motor_A_speed_factor);
//		  int32_t pwm_D = (int32_t)(pwm * motor_D_speed_factor);
//
//		  if (pwm_A > pwmMax) pwm_A = pwmMax;
//		  if (pwm_D > pwmMax) pwm_D = pwmMax;
//
//		  // Direction controlled ONLY by button (direction_state)
//		  if (direction_state == 0) {
//		      // Forward direction
//		      Motor_forward(pwm_A);    // Motor A
//		      MotorD_forward(pwm_D);   // Motor D
//		  } else {
//		      // Reverse direction
//		      Motor_reverse(pwm_A);    // Motor A
//		      MotorD_reverse(pwm_D);   // Motor D
//		  }

		  // Calculate separate speeds for A and D
		  int32_t pwm_A = (int32_t)(pwm * motor_A_speed_factor);
		  int32_t pwm_D = (int32_t)(pwm * motor_D_speed_factor);

		  if (pwm_A > pwmMax) pwm_A = pwmMax;
		  if (pwm_D > pwmMax) pwm_D = pwmMax;

		  // Motors ALWAYS spin forward (ignore direction_state for motors)
		  if (motor_running == 0) {
			  Motor_stop();  // Mode 0: STOP
		  } else {
		      if (motor_direction == 0) {
		    	  Motor_reverse(pwm_A);
				  MotorD_reverse(pwm_D);
		      }
//		      else if (motor_direction == 2) {
//		      		          Motor_reverse(pwm_A);
//		      		        MotorD_forward(pwm_D);
//		      		      } //crab turn
		      else {
		    	  Motor_forward(pwm_A);
				  MotorD_forward(pwm_D);
		      }
		  }
//
		  if (abs(error) <= 3){ // error is not more than 3 deg - assume steady state
			  err++; // to keep track how long it has reached steady state
			  angle = (int)(position*360/260);  //calculate the angle
			  error = target_angle - angle; // calculate the error
			  }

		  //serial_uart(); // send the various data to the serial port for display

		  if (err > 5 && !seq_active) {
		  	          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		  	          HAL_Delay(500);
		  	          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		  	          err = 0;
		  	      }

		      // Update OLED display
//		  const char* mode_names[] = {
//				  "STOP+STRAIGHT ",
//				  "STOP+RIGHT    ",
//				  "STOP+LEFT     ",
//				  "FWD+STRAIGHT  ",
//				  "FWD+LEFT      ",
//				  "FWD+RIGHT     ",
//				  "REV+STRAIGHT  ",
//				  "REV+LEFT      ",
//				  "REV+RIGHT     "};
//		  const char* mode_names[] = {
//				  	  	  "0 ",
//						  "1 ",
//						  "2 ",
//						  "3 ",
//						  "4 ",
//						  "5 ",
//						  "6 ",
//						  "7 ",
//						  "8 ",
//						  "9 ",
//						  "10 ",
//						  "11 ",
//						  "12 ",
//						  "13 ",
//						  "14 ",
//						  "15 ",
//						  "16 ",
//						  "17 ",
//						  "18 "
//
//		  				  	  	  };
//		  sprintf(buf, "Mode: %d", mode);
//		  OLED_ShowString(0, 20, buf);
//
//		  OLED_ShowString(0, 30, mode_names[mode]);
//
//		  OLED_Refresh_Gram();

//accel gyro  reading
//		  if (oled_page == 4) {
//		      OLED_Clear();
//
//		      OLED_ShowString(0, 0, "Display 4/8");
//
//		      if (!imu_ready) {
//		          OLED_ShowString(0, 20, "ICM20948 FAIL");
//		          OLED_Refresh_Gram();
//		      } else {
//		          int16_t ax, ay, az, gx, gy, gz;
//		          if (icm_read_accel_gyro(&ax,&ay,&az,&gx,&gy,&gz)) {
//
//		              // For lab observation:
//		              // Accel raw around 16384 (1g) when axis aligned with gravity
//		              // Gyro should be near 0 dps; show single digit by converting
//		              float gz_dps = (float)gz / 131.0f; // 131 LSB/dps for ±250 dps (common)
//		              if (gz_dps < 0) gz_dps = -gz_dps;
//
//		              char line1[21], line2[21];
//		              snprintf(line1, sizeof(line1), "AZ(raw): %d", (int)az);
//		              snprintf(line2, sizeof(line2), "GZ(dps): %d", (int)(gz_dps + 0.5f));
//
//		              OLED_ShowString(0, 20, line1);
//		              OLED_ShowString(0, 40, line2);
//		              OLED_Refresh_Gram();
//		          } else {
//		              OLED_ShowString(0, 20, "IMU READ ERR");
//		              OLED_Refresh_Gram();
//		          }
//		      }
//
//		      HAL_Delay(100);
//		  }

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		} // while

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 71;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET__Pin SDIN_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED3_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
	/*
	void OLED_show(void *argument, int y, int x) // display message on OLED panel
	{
		//uint8_t hello[20]="Hello World";
		OLED_Init();
		OLED_Display_On();
	//	OLED_ShowString(10,10,argument);
		OLED_ShowString(y, x, argument);
		OLED_Refresh_Gram();
	}
	*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
