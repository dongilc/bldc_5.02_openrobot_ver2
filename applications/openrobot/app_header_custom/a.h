// This file is autogenerated by VESC Tool

#ifndef A_H_
#define A_H_

// VESC ID
#ifndef APPCONF_CONTROLLER_ID
#define APPCONF_CONTROLLER_ID 76
#endif

// Timeout
#ifndef APPCONF_TIMEOUT_MSEC
#define APPCONF_TIMEOUT_MSEC 1000
#endif

// Timeout Brake Current
#ifndef APPCONF_TIMEOUT_BRAKE_CURRENT
#define APPCONF_TIMEOUT_BRAKE_CURRENT 0
#endif

// Can Status Message Mode
#ifndef APPCONF_SEND_CAN_STATUS
#define APPCONF_SEND_CAN_STATUS 1
#endif

// Can Status Rate
#ifndef APPCONF_SEND_CAN_STATUS_RATE_HZ
#define APPCONF_SEND_CAN_STATUS_RATE_HZ 1000
#endif

// CAN Baud Rate
#ifndef APPCONF_CAN_BAUD_RATE
#define APPCONF_CAN_BAUD_RATE 3
#endif

// Pairing Done
#ifndef APPCONF_PAIRING_DONE
#define APPCONF_PAIRING_DONE 0
#endif

// Enable Permanent UART
#ifndef APPCONF_PERMANENT_UART_ENABLED
#define APPCONF_PERMANENT_UART_ENABLED 1
#endif

// Shutdown Mode
#ifndef APPCONF_SHUTDOWN_MODE
#define APPCONF_SHUTDOWN_MODE 7
#endif

// CAN Mode
#ifndef APPCONF_CAN_MODE
#define APPCONF_CAN_MODE 0
#endif

// UAVCAN ESC Index
#ifndef APPCONF_UAVCAN_ESC_INDEX
#define APPCONF_UAVCAN_ESC_INDEX 0
#endif

// UAVCAN Raw Throttle Mode
#ifndef APPCONF_UAVCAN_RAW_MODE
#define APPCONF_UAVCAN_RAW_MODE 0
#endif

// APP to Use
#ifndef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE 8
#endif

// Control Type
#ifndef APPCONF_PPM_CTRL_TYPE
#define APPCONF_PPM_CTRL_TYPE 0
#endif

// PID Max ERPM
#ifndef APPCONF_PPM_PID_MAX_ERPM
#define APPCONF_PPM_PID_MAX_ERPM 15000
#endif

// Input Deadband
#ifndef APPCONF_PPM_HYST
#define APPCONF_PPM_HYST 0.15
#endif

// Pulselength Start
#ifndef APPCONF_PPM_PULSE_START
#define APPCONF_PPM_PULSE_START 1
#endif

// Pulselength End
#ifndef APPCONF_PPM_PULSE_END
#define APPCONF_PPM_PULSE_END 2
#endif

// Pulselength Center
#ifndef APPCONF_PPM_PULSE_CENTER
#define APPCONF_PPM_PULSE_CENTER 1.5
#endif

// Median Filter
#ifndef APPCONF_PPM_MEDIAN_FILTER
#define APPCONF_PPM_MEDIAN_FILTER 1
#endif

// Safe Start
#ifndef APPCONF_PPM_SAFE_START
#define APPCONF_PPM_SAFE_START 1
#endif

// Throttle Expo
#ifndef APPCONF_PPM_THROTTLE_EXP
#define APPCONF_PPM_THROTTLE_EXP 0
#endif

// Throttle Expo Brake
#ifndef APPCONF_PPM_THROTTLE_EXP_BRAKE
#define APPCONF_PPM_THROTTLE_EXP_BRAKE 0
#endif

// Throttle Expo Mode
#ifndef APPCONF_PPM_THROTTLE_EXP_MODE
#define APPCONF_PPM_THROTTLE_EXP_MODE 2
#endif

// Positive Ramping Time
#ifndef APPCONF_PPM_RAMP_TIME_POS
#define APPCONF_PPM_RAMP_TIME_POS 0.4
#endif

// Negative Ramping Time
#ifndef APPCONF_PPM_RAMP_TIME_NEG
#define APPCONF_PPM_RAMP_TIME_NEG 0.2
#endif

// Multiple VESCs Over CAN
#ifndef APPCONF_PPM_MULTI_ESC
#define APPCONF_PPM_MULTI_ESC 1
#endif

// Traction Control
#ifndef APPCONF_PPM_TC
#define APPCONF_PPM_TC 0
#endif

// TC Max ERPM Difference
#ifndef APPCONF_PPM_TC_MAX_DIFF
#define APPCONF_PPM_TC_MAX_DIFF 3000
#endif

// Max ERPM for direction switch
#ifndef APPCONF_PPM_MAX_ERPM_FOR_DIR
#define APPCONF_PPM_MAX_ERPM_FOR_DIR 4000
#endif

// Smart Reverse Max Duty Cycle
#ifndef APPCONF_PPM_SMART_REV_MAX_DUTY
#define APPCONF_PPM_SMART_REV_MAX_DUTY 0.07
#endif

// Smart Reverse Ramp Time
#ifndef APPCONF_PPM_SMART_REV_RAMP_TIME
#define APPCONF_PPM_SMART_REV_RAMP_TIME 3
#endif

// Control Type
#ifndef APPCONF_ADC_CTRL_TYPE
#define APPCONF_ADC_CTRL_TYPE 0
#endif

// Input Deadband
#ifndef APPCONF_ADC_HYST
#define APPCONF_ADC_HYST 0.15
#endif

// ADC1 Min Voltage
#ifndef APPCONF_ADC_VOLTAGE_START
#define APPCONF_ADC_VOLTAGE_START 0.9
#endif

// ADC1 Max Voltage
#ifndef APPCONF_ADC_VOLTAGE_END
#define APPCONF_ADC_VOLTAGE_END 3
#endif

// ADC1 Center Voltage
#ifndef APPCONF_ADC_VOLTAGE_CENTER
#define APPCONF_ADC_VOLTAGE_CENTER 2
#endif

// ADC2 Min Voltage
#ifndef APPCONF_ADC_VOLTAGE2_START
#define APPCONF_ADC_VOLTAGE2_START 0.9
#endif

// ADC2 Max Voltage
#ifndef APPCONF_ADC_VOLTAGE2_END
#define APPCONF_ADC_VOLTAGE2_END 3
#endif

// Use Filter
#ifndef APPCONF_ADC_USE_FILTER
#define APPCONF_ADC_USE_FILTER 1
#endif

// Safe Start
#ifndef APPCONF_ADC_SAFE_START
#define APPCONF_ADC_SAFE_START 1
#endif

// Invert Cruise Control Button
#ifndef APPCONF_ADC_CC_BUTTON_INVERTED
#define APPCONF_ADC_CC_BUTTON_INVERTED 0
#endif

// Invert Reverse Button
#ifndef APPCONF_ADC_REV_BUTTON_INVERTED
#define APPCONF_ADC_REV_BUTTON_INVERTED 0
#endif

// Invert ADC1 Voltage
#ifndef APPCONF_ADC_VOLTAGE_INVERTED
#define APPCONF_ADC_VOLTAGE_INVERTED 0
#endif

// Invert ADC2 Voltage
#ifndef APPCONF_ADC_VOLTAGE2_INVERTED
#define APPCONF_ADC_VOLTAGE2_INVERTED 0
#endif

// Throttle Expo
#ifndef APPCONF_ADC_THROTTLE_EXP
#define APPCONF_ADC_THROTTLE_EXP 0
#endif

// Throttle Expo Brake
#ifndef APPCONF_ADC_THROTTLE_EXP_BRAKE
#define APPCONF_ADC_THROTTLE_EXP_BRAKE 0
#endif

// Throttle Expo Mode
#ifndef APPCONF_ADC_THROTTLE_EXP_MODE
#define APPCONF_ADC_THROTTLE_EXP_MODE 2
#endif

// Positive Ramping Time
#ifndef APPCONF_ADC_RAMP_TIME_POS
#define APPCONF_ADC_RAMP_TIME_POS 0.3
#endif

// Negative Ramping Time
#ifndef APPCONF_ADC_RAMP_TIME_NEG
#define APPCONF_ADC_RAMP_TIME_NEG 0.1
#endif

// Multiple VESCs Over CAN
#ifndef APPCONF_ADC_MULTI_ESC
#define APPCONF_ADC_MULTI_ESC 1
#endif

// Traction Control
#ifndef APPCONF_ADC_TC
#define APPCONF_ADC_TC 0
#endif

// TC Max ERPM Difference
#ifndef APPCONF_ADC_TC_MAX_DIFF
#define APPCONF_ADC_TC_MAX_DIFF 3000
#endif

// Update Rate
#ifndef APPCONF_ADC_UPDATE_RATE_HZ
#define APPCONF_ADC_UPDATE_RATE_HZ 500
#endif

// Baudrate
#ifndef APPCONF_UART_BAUDRATE
#define APPCONF_UART_BAUDRATE 115200
#endif

// Control Type
#ifndef APPCONF_CHUK_CTRL_TYPE
#define APPCONF_CHUK_CTRL_TYPE 1
#endif

// Input Deadband
#ifndef APPCONF_CHUK_HYST
#define APPCONF_CHUK_HYST 0.15
#endif

// Positive Ramping Time
#ifndef APPCONF_CHUK_RAMP_TIME_POS
#define APPCONF_CHUK_RAMP_TIME_POS 0.4
#endif

// Negative Ramping Time
#ifndef APPCONF_CHUK_RAMP_TIME_NEG
#define APPCONF_CHUK_RAMP_TIME_NEG 0.2
#endif

// ERPM Per Second Cruise Control
#ifndef APPCONF_STICK_ERPM_PER_S_IN_CC
#define APPCONF_STICK_ERPM_PER_S_IN_CC 3000
#endif

// Throttle Expo
#ifndef APPCONF_CHUK_THROTTLE_EXP
#define APPCONF_CHUK_THROTTLE_EXP 0
#endif

// Throttle Expo Brake
#ifndef APPCONF_CHUK_THROTTLE_EXP_BRAKE
#define APPCONF_CHUK_THROTTLE_EXP_BRAKE 0
#endif

// Throttle Expo Mode
#ifndef APPCONF_CHUK_THROTTLE_EXP_MODE
#define APPCONF_CHUK_THROTTLE_EXP_MODE 2
#endif

// Multiple VESCs Over CAN
#ifndef APPCONF_CHUK_MULTI_ESC
#define APPCONF_CHUK_MULTI_ESC 1
#endif

// Traction Control
#ifndef APPCONF_CHUK_TC
#define APPCONF_CHUK_TC 0
#endif

// TC Max ERPM Difference
#ifndef APPCONF_CHUK_TC_MAX_DIFF
#define APPCONF_CHUK_TC_MAX_DIFF 3000
#endif

// Use Smart Reverse
#ifndef APPCONF_CHUK_USE_SMART_REV
#define APPCONF_CHUK_USE_SMART_REV 1
#endif

// Smart Reverse Max Duty Cycle
#ifndef APPCONF_CHUK_SMART_REV_MAX_DUTY
#define APPCONF_CHUK_SMART_REV_MAX_DUTY 0.07
#endif

// Smart Reverse Ramp Time
#ifndef APPCONF_CHUK_SMART_REV_RAMP_TIME
#define APPCONF_CHUK_SMART_REV_RAMP_TIME 3
#endif

// Speed
#ifndef APPCONF_NRF_SPEED
#define APPCONF_NRF_SPEED 1
#endif

// TX Power
#ifndef APPCONF_NRF_POWER
#define APPCONF_NRF_POWER 3
#endif

// CRC
#ifndef APPCONF_NRF_CRC
#define APPCONF_NRF_CRC 1
#endif

// Retry Delay
#ifndef APPCONF_NRF_RETR_DELAY
#define APPCONF_NRF_RETR_DELAY 0
#endif

// Retries
#ifndef APPCONF_NRF_RETRIES
#define APPCONF_NRF_RETRIES 3
#endif

// Radio Channel
#ifndef APPCONF_NRF_CHANNEL
#define APPCONF_NRF_CHANNEL 76
#endif

// Address 0
#ifndef APPCONF_NRF_ADDR_B0
#define APPCONF_NRF_ADDR_B0 198
#endif

// Address 1
#ifndef APPCONF_NRF_ADDR_B1
#define APPCONF_NRF_ADDR_B1 199
#endif

// Address 2
#ifndef APPCONF_NRF_ADDR_B2
#define APPCONF_NRF_ADDR_B2 0
#endif

// Send ACK
#ifndef APPCONF_NRF_SEND_CRC_ACK
#define APPCONF_NRF_SEND_CRC_ACK 1
#endif

// P
#ifndef APPCONF_BALANCE_KP
#define APPCONF_BALANCE_KP 0
#endif

// I
#ifndef APPCONF_BALANCE_KI
#define APPCONF_BALANCE_KI 0
#endif

// D
#ifndef APPCONF_BALANCE_KD
#define APPCONF_BALANCE_KD 0
#endif

// Loop Hertz
#ifndef APPCONF_BALANCE_HERTZ
#define APPCONF_BALANCE_HERTZ 1000
#endif

// Pitch Axis Fault Cutoff
#ifndef APPCONF_BALANCE_FAULT_PITCH
#define APPCONF_BALANCE_FAULT_PITCH 20
#endif

// Roll Axis Fault Cutoff
#ifndef APPCONF_BALANCE_FAULT_ROLL
#define APPCONF_BALANCE_FAULT_ROLL 45
#endif

// Duty Cycle Fault Cutoff
#ifndef APPCONF_BALANCE_FAULT_DUTY
#define APPCONF_BALANCE_FAULT_DUTY 0.9
#endif

// ADC1 Switch Voltage
#ifndef APPCONF_BALANCE_FAULT_ADC1
#define APPCONF_BALANCE_FAULT_ADC1 0
#endif

// ADC2 Switch Voltage
#ifndef APPCONF_BALANCE_FAULT_ADC2
#define APPCONF_BALANCE_FAULT_ADC2 0
#endif

// Pitch Fault Delay
#ifndef APPCONF_BALANCE_FAULT_DELAY_PITCH
#define APPCONF_BALANCE_FAULT_DELAY_PITCH 0
#endif

// Roll Fault Delay
#ifndef APPCONF_BALANCE_FAULT_DELAY_ROLL
#define APPCONF_BALANCE_FAULT_DELAY_ROLL 0
#endif

// Duty Fault Delay
#ifndef APPCONF_BALANCE_FAULT_DELAY_DUTY
#define APPCONF_BALANCE_FAULT_DELAY_DUTY 0
#endif

// Half Switch Fault Delay
#ifndef APPCONF_BALANCE_FAULT_DELAY_SWITCH_HALF
#define APPCONF_BALANCE_FAULT_DELAY_SWITCH_HALF 0
#endif

// Full Switch Fault Delay
#ifndef APPCONF_BALANCE_FAULT_DELAY_SWITCH_FULL
#define APPCONF_BALANCE_FAULT_DELAY_SWITCH_FULL 0
#endif

// ADC Half State Fault ERPM
#ifndef APPCONF_BALANCE_FAULT_ADC_HALF_ERPM
#define APPCONF_BALANCE_FAULT_ADC_HALF_ERPM 1000
#endif

// Tiltback Angle
#ifndef APPCONF_BALANCE_TILTBACK_ANGLE
#define APPCONF_BALANCE_TILTBACK_ANGLE 15
#endif

// Tiltback Speed
#ifndef APPCONF_BALANCE_TILTBACK_SPEED
#define APPCONF_BALANCE_TILTBACK_SPEED 5
#endif

// Duty Cycle Tiltback
#ifndef APPCONF_BALANCE_TILTBACK_DUTY
#define APPCONF_BALANCE_TILTBACK_DUTY 0.75
#endif

// High Voltage Tiltback
#ifndef APPCONF_BALANCE_TILTBACK_HIGH_V
#define APPCONF_BALANCE_TILTBACK_HIGH_V 200
#endif

// Low Voltage Tiltback
#ifndef APPCONF_BALANCE_TILTBACK_LOW_V
#define APPCONF_BALANCE_TILTBACK_LOW_V 0
#endif

// Constant Tiltback
#ifndef APPCONF_BALANCE_TILTBACK_CONSTANT
#define APPCONF_BALANCE_TILTBACK_CONSTANT 0
#endif

// Constant Tiltback ERPM
#ifndef APPCONF_BALANCE_TILTBACK_CONSTANT_ERPM
#define APPCONF_BALANCE_TILTBACK_CONSTANT_ERPM 500
#endif

// Startup Pitch Axis Angle Tolerance
#ifndef APPCONF_BALANCE_STARTUP_PITCH_TOLERANCE
#define APPCONF_BALANCE_STARTUP_PITCH_TOLERANCE 20
#endif

// Startup Roll Axis Angle Tolerance
#ifndef APPCONF_BALANCE_STARTUP_ROLL_TOLERANCE
#define APPCONF_BALANCE_STARTUP_ROLL_TOLERANCE 8
#endif

// Startup Centering Speed
#ifndef APPCONF_BALANCE_STARTUP_SPEED
#define APPCONF_BALANCE_STARTUP_SPEED 30
#endif

// Deadzone
#ifndef APPCONF_BALANCE_DEADZONE
#define APPCONF_BALANCE_DEADZONE 0
#endif

// Current Boost
#ifndef APPCONF_BALANCE_CURRENT_BOOST
#define APPCONF_BALANCE_CURRENT_BOOST 0
#endif

// Multiple VESCs Over CAN
#ifndef APPCONF_BALANCE_MULTI_ESC
#define APPCONF_BALANCE_MULTI_ESC 0
#endif

// Yaw P
#ifndef APPCONF_BALANCE_YAW_KP
#define APPCONF_BALANCE_YAW_KP 0
#endif

// Yaw I
#ifndef APPCONF_BALANCE_YAW_KI
#define APPCONF_BALANCE_YAW_KI 0
#endif

// Yaw D
#ifndef APPCONF_BALANCE_YAW_KD
#define APPCONF_BALANCE_YAW_KD 0
#endif

// Roll Steer KP
#ifndef APPCONF_BALANCE_ROLL_STEER_KP
#define APPCONF_BALANCE_ROLL_STEER_KP 0
#endif

// Roll Steer ERPM KP
#ifndef APPCONF_BALANCE_ROLL_STEER_ERPM_KP
#define APPCONF_BALANCE_ROLL_STEER_ERPM_KP 0
#endif

// Brake Current
#ifndef APPCONF_BALANCE_BRAKE_CURRENT
#define APPCONF_BALANCE_BRAKE_CURRENT 0
#endif

// Yaw Current Clamp
#ifndef APPCONF_BALANCE_YAW_CURRENT_CLAMP
#define APPCONF_BALANCE_YAW_CURRENT_CLAMP 0
#endif

// Setpoint Pitch Low Pass Filter
#ifndef APPCONF_BALANCE_SETPOINT_PITCH_FILTER
#define APPCONF_BALANCE_SETPOINT_PITCH_FILTER 0
#endif

// Setpoint Target Low Pass Filter
#ifndef APPCONF_BALANCE_SETPOINT_TARGET_FILTER
#define APPCONF_BALANCE_SETPOINT_TARGET_FILTER 1
#endif

// Setpoint Filter Clamp
#ifndef APPCONF_BALANCE_SETPOINT_FILTER_CLAMP
#define APPCONF_BALANCE_SETPOINT_FILTER_CLAMP 8
#endif

// D term PT1 Filter
#ifndef APPCONF_BALANCE_KD_PT1_FREQUENCY
#define APPCONF_BALANCE_KD_PT1_FREQUENCY 0
#endif

// Control Type
#ifndef APPCONF_PAS_CTRL_TYPE
#define APPCONF_PAS_CTRL_TYPE 0
#endif

// Sensor Type
#ifndef APPCONF_PAS_SENSOR_TYPE
#define APPCONF_PAS_SENSOR_TYPE 0
#endif

// PAS Max Current
#ifndef APPCONF_PAS_CURRENT_SCALING
#define APPCONF_PAS_CURRENT_SCALING 0.1
#endif

// Pedal RPM Start
#ifndef APPCONF_PAS_PEDAL_RPM_START
#define APPCONF_PAS_PEDAL_RPM_START 10
#endif

// Pedal RPM End
#ifndef APPCONF_PAS_PEDAL_RPM_END
#define APPCONF_PAS_PEDAL_RPM_END 180
#endif

// Invert Pedal Direction
#ifndef APPCONF_PAS_INVERT_PEDAL_DIRECTION
#define APPCONF_PAS_INVERT_PEDAL_DIRECTION 0
#endif

// Sensor Magnets
#ifndef APPCONF_PAS_MAGNETS
#define APPCONF_PAS_MAGNETS 24
#endif

// Use Filter
#ifndef APPCONF_PAS_USE_FILTER
#define APPCONF_PAS_USE_FILTER 1
#endif

// Positive Ramping Time
#ifndef APPCONF_PAS_RAMP_TIME_POS
#define APPCONF_PAS_RAMP_TIME_POS 0.6
#endif

// Negative Ramping Time
#ifndef APPCONF_PAS_RAMP_TIME_NEG
#define APPCONF_PAS_RAMP_TIME_NEG 0.3
#endif

// Update Rate
#ifndef APPCONF_PAS_UPDATE_RATE_HZ
#define APPCONF_PAS_UPDATE_RATE_HZ 500
#endif

// IMU Type
#ifndef APPCONF_IMU_TYPE
#define APPCONF_IMU_TYPE 1
#endif

// IMU AHRS Mode
#ifndef APPCONF_IMU_AHRS_MODE
#define APPCONF_IMU_AHRS_MODE 0
#endif

// Sample Rate
#ifndef APPCONF_IMU_SAMPLE_RATE_HZ
#define APPCONF_IMU_SAMPLE_RATE_HZ 200
#endif

// Accelerometer Confidence Decay
#ifndef APPCONF_IMU_ACCEL_CONFIDENCE_DECAY
#define APPCONF_IMU_ACCEL_CONFIDENCE_DECAY 1
#endif

// Mahony KP
#ifndef APPCONF_IMU_MAHONY_KP
#define APPCONF_IMU_MAHONY_KP 0.3
#endif

// Mahony KI
#ifndef APPCONF_IMU_MAHONY_KI
#define APPCONF_IMU_MAHONY_KI 0
#endif

// Madgwick Beta
#ifndef APPCONF_IMU_MADGWICK_BETA
#define APPCONF_IMU_MADGWICK_BETA 0.1
#endif

// Imu Rotation Roll
#ifndef APPCONF_IMU_ROT_ROLL
#define APPCONF_IMU_ROT_ROLL 0
#endif

// Imu Rotation Pitch
#ifndef APPCONF_IMU_ROT_PITCH
#define APPCONF_IMU_ROT_PITCH 0
#endif

// Imu Rotation Yaw
#ifndef APPCONF_IMU_ROT_YAW
#define APPCONF_IMU_ROT_YAW 0
#endif

// Accel Offset X
#ifndef APPCONF_IMU_A_OFFSET_0
#define APPCONF_IMU_A_OFFSET_0 0
#endif

// Accel Offset Y
#ifndef APPCONF_IMU_A_OFFSET_1
#define APPCONF_IMU_A_OFFSET_1 0
#endif

// Accel Offset Z
#ifndef APPCONF_IMU_A_OFFSET_2
#define APPCONF_IMU_A_OFFSET_2 0
#endif

// Gyro Offset X
#ifndef APPCONF_IMU_G_OFFSET_0
#define APPCONF_IMU_G_OFFSET_0 0
#endif

// Gyro Offset Y
#ifndef APPCONF_IMU_G_OFFSET_1
#define APPCONF_IMU_G_OFFSET_1 0
#endif

// Gyro Offset Z
#ifndef APPCONF_IMU_G_OFFSET_2
#define APPCONF_IMU_G_OFFSET_2 0
#endif

// Gyro Offset Comp X
#ifndef APPCONF_IMU_G_OFFSET_COMP_FACT_0
#define APPCONF_IMU_G_OFFSET_COMP_FACT_0 0
#endif

// Gyro Offset Comp Y
#ifndef APPCONF_IMU_G_OFFSET_COMP_FACT_1
#define APPCONF_IMU_G_OFFSET_COMP_FACT_1 0
#endif

// Gyro Offset Comp Z
#ifndef APPCONF_IMU_G_OFFSET_COMP_FACT_2
#define APPCONF_IMU_G_OFFSET_COMP_FACT_2 0
#endif

// Gyro Offset Comp Clamp
#ifndef APPCONF_IMU_G_OFFSET_COMP_CLAMP
#define APPCONF_IMU_G_OFFSET_COMP_CLAMP 5
#endif

// A_H_
#endif

