#ifndef __BNO_08x_I2C_H
#define __BNO_08x_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

// Used to eneble or disable functions needed for rough telescope position
// Uncomment lne below for fully featured driver

#define USE_FOR_TELESCOPE

// Enable disable extra features
//#define USE_ERROR_REPORT
//#define USE_COUNTER_REPORT
//#define USE_REORIENT //Tare

// For telescope use we need to be able to calibrate and change the orientation
#ifdef USE_FOR_TELESCOPE
	#define ACCELEROMETER									0X01
	#define GYROSCOPE_CALIBRATED					0X02
	#define MAGNETIC_FIELD_CALIBRATED			0X03
	#define ROTATION_VECTOR								0X05
	#define GAME_ROTATION_VECTOR					0X08
	#define USE_REORIENT // Used to set X to North, Y to East and Z down
#else
	//Features that can be used. Play with the conditions above to activate them
	#define RAW_ACCELEROMETER							0X14
	#define ACCELEROMETER									0X01
	#define LINEAR_ACCELERATION						0X04
	#define GRAVITY												0X06
	#define RAW_GYROSCOPE									0X15
	#define GYROSCOPE_CALIBRATED					0X02
	#define GYROSCOPE_UNCALIBRATED				0X07
	#define RAW_MAGNETOMETER							0X16
	#define MAGNETIC_FIELD_CALIBRATED			0X03
	#define MAGNETIC_FIELD_UNCALIBRATED		0X0F
	#define ROTATION_VECTOR								0X05
	#define GAME_ROTATION_VECTOR					0X08
	#define GEOMAGNETIC_ROTATION_VECTOR		0X09
	#define PRESSURE											0X0A
	#define AMBIENT_LIGHT									0X0B
	#define HUMIDITY											0X0C
	#define PROXIMITY											0X0D
	#define TEMPERATURE										0X0E
	#define RESERVED											0X17
	#define TAP_DETECTOR									0X10
	#define STEP_DETECTOR									0X18
	#define STEP_COUNTER									0X11
	#define SIGNIFICANT_MOTION						0X12
	#define STABILITY_CLASSIFIER					0X13
	#define SHAKE_DETECTOR								0X19
	#define FLIP_DETECTOR									0X1A
	#define PICKUP_DETECTOR								0X1B
	#define STABILITY_DETECTOR						0X1C
	#define PERSONAL_ACTIVITY_CLASSIFIER	0X1E
	#define SLEEP_DETECTOR								0X1F
	#define TILT_DETECTOR									0X20
	#define POCKET_DETECTOR								0X21
	#define CIRCLE_DETECTOR								0X22
	#define HEART_RATE_MONITOR						0X23
	#define ARVR_STABILIZED_RV						0X28
	#define ARVR_STABILIZED_GRV						0X29
	#define GYRO_INTEGRATED_RV						0X2A
	#define IZRO_MOTION_REQUEST						0X2B
	#define RAW_OPTICAL_FLOW							0X2C
	#define DEAD_RECKONING_POSE						0X2D
	#define WHEEL_ENCODER									0X2E
#endif

// Address
#define BNO_W_ADDR											0x96
#define BNO_R_ADDR											0x97
#define ADDR_BNO_DFU_W_ADDR							0x52
#define ADDR_BNO_DFU_R_ADDR							0x53

// Delays
#define RESET_DELAY											200
#define PORT_TIMEOUT										500
#define Q_FLIP													0.70710678118654752440084436210485
#define _180_DIV_PI											57.295779515f // = 180 / PI

// Array sizes
#define SEQUENCE_SIZE										6
#define HEADER_SIZE											4
#define RX_PACKET_SIZE									284 // For BNO086 276 for BNO080
#define TX_PACKET_SIZE									21
	
// Channels
#define CHANNEL_COMMAND									0
#define CHANNEL_EXECUTABLE							1
#define CHANNEL_CONTROL									2
#define CHANNEL_REPORTS									3
#define CHANNEL_WAKE_REPORTS						4
#define CHANNEL_GYRO																5

// Reports
#define REPORT_UNSOLICITED_RESPONSE			0x00
#define REPORT_UNSOLICITED_RESPONSE1		0x01

#define REPORT_SENSOR_FLUSH_REQUEST			0xF0
#define REPORT_SENSOR_FLUSH_RESPONSE		0xEF

#define REPORT_COMMAND_REQUEST					0xF2
#define REPORT_COMMAND_RESPONSE					0xF1

#define REPORT_FRS_READ_REQUEST					0xF4
#define REPORT_FRS_READ_RESPONSE				0xF3

#define REPORT_FRS_WRITE_REQUEST				0xF7
#define REPORT_FRS_WRITE_DATA_REQUEST		0xF6
#define REPORT_FRS_WRITE_RESPONSE				0xF5

#define REPORT_PRODUCT_ID_REQUEST				0xF9
#define REPORT_PRODUCT_ID_RESPONSE			0xF8


#define REPORT_TIMESTAMP_REBASE					0xFA
#define REPORT_BASE_TIMESTAMP_REF				0xFB

#define REPORT_GET_FEATURE_REQUEST			0xFE
#define REPORT_GET_FEATURE_RESPONSE			0xFC

#define REPORT_SET_FEATURE_COMMAND			0xFD

// Command and Subcommand values
#define COMMAND_ERRORS									1
#define COMMAND_COUNTER									2

#define COMMAND_TARE										3
#define COMMAND_TARE_NOW								0
#define COMMAND_TARE_PERSIST						1
#define COMMAND_TARE_REORIENT						2

#define COMMAND_INITIALIZE							4
#define COMMAND_INITIALIZE_RESET				1
#define COMMAND_INITIALIZE_ON						2
#define COMMAND_INITIALIZE_SLEEP				3

#define COMMAND_SAVE_DCD								6

#define COMMAND_ME_CALIBRATE						7
#define COMMAND_ME_CALIBRATE_CONFIG			0
#define COMMAND_ME_CALIBRATE_GET				1

#define COMMAND_DCD_PERIOD_SAVE					9

#define COMMAND_OSCILLATOR							0x0A
#define COMMAND_CLEAR_DCD_RESET					0x0B
#define COMMAND_TURNTABLE_CAL						0x0C

#define COMMAND_BOOTLOADER							0x0D
#define COMMAND_BOOTLOADER_MODE_REQ			0x00
#define COMMAND_BOOTLOADER_STATUS_REQ		0x01

#define COMMAND_INTERACTIVE_CAL_REQ			0x0E
#define COMMAND_WHEEL_REQ								0x0F

#define COMMAND_UNSOLICITED_INITIALIZE	0x84

#define CALIBRATE_ACCEL									0
#define CALIBRATE_GYRO									1
#define CALIBRATE_MAG										2
#define CALIBRATE_PLANAR_ACCEL					3
#define CALIBRATE_ON_TABLE							4
#define CALIBRATE_ACCEL_GYRO_MAG				5
#define CALIBRATE_ALL										6

#define CALIBRATE_STOP									5

// FRS Record Ids
#define STATIC_CALIBRATION_AGM                   0x7979
#define NOMINAL_CALIBRATION                      0x4D4D
#define STATIC_CALIBRATION_SRA                   0x8A8A
#define NOMINAL_CALIBRATION_SRA                  0x4E4E
#define DYNAMIC_CALIBRATION                      0x1F1F
#define ME_POWER_MGMT                            0xD3E2
#define SYSTEM_ORIENTATION                       0x2D3E
#define ACCEL_ORIENTATION                        0x2D41
#define SCREEN_ACCEL_ORIENTATION                 0x2D43
#define GYROSCOPE_ORIENTATION                    0x2D46
#define MAGNETOMETER_ORIENTATION                 0x2D4C
#define ARVR_STABILIZATION_RV                    0x3E2D
#define ARVR_STABILIZATION_GRV                   0x3E2E
#define TAP_DETECT_CONFIG                        0xC269
#define SIG_MOTION_DETECT_CONFIG                 0xC274
#define SHAKE_DETECT_CONFIG                      0x7D7D
#define MAX_FUSION_PERIOD                        0xD7D7
#define SERIAL_NUMBER                            0x4B4B
#define ES_PRESSURE_CAL                          0x39AF
#define ES_TEMPERATURE_CAL                       0x4D20
#define ES_HUMIDITY_CAL                          0x1AC9
#define ES_AMBIENT_LIGHT_CAL                     0x39B1
#define ES_PROXIMITY_CAL                         0x4DA2
#define ALS_CAL                                  0xD401
#define PROXIMITY_SENSOR_CAL                     0xD402
#define PICKUP_DETECTOR_CONFIG                   0x1B2A
#define FLIP_DETECTOR_CONFIG                     0xFC94
#define STABILITY_DETECTOR_CONFIG                0xED85
#define ACTIVITY_TRACKER_CONFIG                  0xED88
#define SLEEP_DETECTOR_CONFIG                    0xED87
#define TILT_DETECTOR_CONFIG                     0xED89
#define POCKET_DETECTOR_CONFIG                   0xEF27
#define CIRCLE_DETECTOR_CONFIG                   0xEE51
#define USER_RECORD                              0x74B4
#define ME_TIME_SOURCE_SELECT                    0xD403
#define UART_FORMAT                              0xA1A1
#define GYRO_INTEGRATED_RV_CONFIG                0xA1A2
#define DR_IMU_CONFIG                            0xDED2
#define DR_VEL_EST_CONFIG                        0xDED3
#define DR_SYNC_CONFIG                           0xDED4
#define DR_QUAL_CONFIG                           0xDED5
#define DR_CAL_CONFIG                            0xDED6
#define DR_LIGHT_REC_CONFIG                      0xDED8
#define DR_FUSION_CONFIG                         0xDED9
#define DR_OF_CONFIG                             0xDEDA
#define DR_WHEEL_CONFIG                          0xDEDB
#define DR_CAL                                   0xDEDC
#define DR_WHEEL_SELECT                          0xDEDF
#define FRS_ID_META_RAW_ACCELEROMETER            0xE301
#define FRS_ID_META_ACCELEROMETER                0xE302
#define FRS_ID_META_LINEAR_ACCELERATION          0xE303
#define FRS_ID_META_GRAVITY                      0xE304
#define FRS_ID_META_RAW_GYROSCOPE                0xE305
#define FRS_ID_META_GYROSCOPE_CALIBRATED         0xE306
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED       0xE307
#define FRS_ID_META_RAW_MAGNETOMETER             0xE308
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED    0xE309
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED  0xE30A
#define FRS_ID_META_ROTATION_VECTOR              0xE30B
#define FRS_ID_META_GAME_ROTATION_VECTOR         0xE30C
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR  0xE30D
#define FRS_ID_META_PRESSURE                     0xE30E
#define FRS_ID_META_AMBIENT_LIGHT                0xE30F
#define FRS_ID_META_HUMIDITY                     0xE310
#define FRS_ID_META_PROXIMITY                    0xE311
#define FRS_ID_META_TEMPERATURE                  0xE312
#define FRS_ID_META_TAP_DETECTOR                 0xE313
#define FRS_ID_META_STEP_DETECTOR                0xE314
#define FRS_ID_META_STEP_COUNTER                 0xE315
#define FRS_ID_META_SIGNIFICANT_MOTION           0xE316
#define FRS_ID_META_STABILITY_CLASSIFIER         0xE317
#define FRS_ID_META_SHAKE_DETECTOR               0xE318
#define FRS_ID_META_FLIP_DETECTOR                0xE319
#define FRS_ID_META_PICKUP_DETECTOR              0xE31A
#define FRS_ID_META_STABILITY_DETECTOR           0xE31B
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER 0xE31C
#define FRS_ID_META_SLEEP_DETECTOR               0xE31D
#define FRS_ID_META_TILT_DETECTOR                0xE31E
#define FRS_ID_META_POCKET_DETECTOR              0xE31F
#define FRS_ID_META_CIRCLE_DETECTOR              0xE320
#define FRS_ID_META_HEART_RATE_MONITOR           0xE321
#define FRS_ID_META_ARVR_STABILIZED_RV           0xE322
#define FRS_ID_META_ARVR_STABILIZED_GRV          0xE323
#define FRS_ID_META_GYRO_INTEGRATED_RV           0xE324
#define FRS_ID_META_RAW_OPTICAL_FLOW             0xE326
//Record IDs from SH-2 figure 28
//These are used to read and set various configuration options
#define FRS_RECORDID_SERIAL_NUMBER								0x4B4B
#define FRS_RECORDID_SYSTEM_ORIENTATION						0x2D3E

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED									0
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE		1
#define FRS_WRITE_STATUS_BUSY											2
#define FRS_WRITE_STATUS_WRITE_COMPLETED					3
#define FRS_WRITE_STATUS_READY										4
#define FRS_WRITE_STATUS_FAILED										5
#define FRS_WRITE_STATUS_NOT_READY								6 // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH						7
#define FRS_WRITE_STATUS_RECORD_VALID							8
#define FRS_WRITE_STATUS_INVALID_RECORD						9
#define FRS_WRITE_STATUS_DEVICE_ERROR							10
#define FRS_WRITE_STATUS_READ_ONLY								11

// Status values
#define FRS_READ_STATUS_NO_ERROR                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE           1
#define FRS_READ_STATUS_BUSY                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE             4
#define FRS_READ_STATUS_RECORD_EMPTY                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED 7
#define FRS_READ_STATUS_DEVICE_ERROR                    8

// The below values are pre-calculated using SCALE_Q(n) = (1.0f / (1 << n))
#define SCALE_Q4			0.06250000000000000000
#define SCALE_Q7			0.00781250000000000000
#define SCALE_Q8			0.00390625000000000000
#define SCALE_Q9			0.00195312500000000000
#define SCALE_Q10			0.00097656250000000000
#define SCALE_Q12			0.00024414062500000000
#define SCALE_Q14			0.00006103515625000000
#define SCALE_Q17			0.00000762939453125000
#define SCALE_Q20			0.00000095367431640625
#define SCALE_Q25			0.00000002980232238770
#define SCALE_Q30			0.00000000093132257462
#define SCALE_TO_Q14	16384.0

// General Enums
typedef enum {
	NA = 0,
	PowerOnReset,
	InternalSystemReset,
	WatchdogTimeout,
	ExternalReset,
	Other,
} BNO_ResetCause_t;

typedef enum {
	InternalOscillator = 0,
	ExternalCrystal,
	ExternalClock,
	OscillatorError
} BNO_Oscillator_t;

typedef enum {
	ResetToBootloader = 0, // Reset to bootloader Mode
	UpgradeApp, //1 – Upgrade Application Mode; upgrade the application image in flash.
	ValidateImage, //2 – Validate Image Mode; validate an application image without updating the flash
	LauchApp, //3 – Launch Application; launch the application image in flash
	BootInvalid
} BNO_BootMode_t;

typedef enum {
	MOTION_INTENT_UNKNOWN = 0, // FME_MOBILE_MOTION_INTENT_UNKNOWN – this is the initial state assumed by the sensor hub
	INTENT_STATIONARY_WITHOUT_VIBRATION, //
	MOTION_INTENT_STATIONARY_WITH_VIBRATION, //
	MOTION_INTENT_IN_MOTION, //
	MOTION_INTENT_IN_MOTION_ACCELERATING
} BNO_MotionIntent_t;

// General structures
typedef struct __attribute__((packed)) {
	BNO_ResetCause_t	rst_Cause;	// report 0xF8 byte 1 Reset Cause
	uint8_t						sw_Maj;			// report 0xF8 byte 2 SW Version Major
	uint8_t						sw_Min;			// report 0xF8 byte 3 SW Version Minor
	uint32_t					sw_PN;			// report 0xF8 byte 4-7 SW Part Number
	uint32_t					sw_BN;			// report 0xF8 byte 8-11 SW Build Number
	uint16_t					sw_VP;			// report 0xF8 byte 12-13 SW Version Patch
} BNO_productID_t;

typedef struct __attribute__((packed)){
	uint8_t status;
	uint16_t wordOffset;
} BNO_FrsWriteResp_t;

typedef struct __attribute__((packed)){
	uint8_t len_status;
	uint16_t wordOffset;
	uint32_t data0;
	uint32_t data1;
	uint16_t frsType;
} BNO_FrsReadResp_t;

#define RESPONSE_VALUES	11
typedef struct __attribute__((packed)) {
	uint8_t seq;
	uint8_t command;
	uint8_t commandSeq;
	uint8_t respSeq;
	uint8_t r[RESPONSE_VALUES];
} BNO_CommandResp_t;

#define COMMAND_PARAMS	9
typedef struct __attribute__((packed)) {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS];
} BNO_CommandReq_t;

#ifdef USE_ERROR_REPORT
typedef struct __attribute__((packed)) {
	uint8_t	Severity;	// Severity – the severity of the error currently being reported
	uint8_t	SeqNo;		// Error sequence number. A monotonically incrementing uin8_t that counts all the errors generated for the reported severity. It may rollover.
	uint8_t	Source;		// Error source. 0 – reserved, 1 – MotionEngine, 2 – MotionHub, 3 – SensorHub, 4 – Chip level executable, 5-254 reserved. 255 – no error to report.
	uint8_t	Error;		// Error. See library API
	uint8_t	Module;		// Error module. See library API
	uint8_t	Code;			// Error code. See library API
} BNO_Error_t;
#endif

#ifdef USE_COUNTER_REPORT
typedef struct __attribute__((packed)) {
	uint32_t offered;   // [events]
	uint32_t accepted;  // [events]
	uint32_t on;        // [events]
	uint32_t attempted;	// [events]
} BNO_Counts_t;
#endif

#ifdef USE_REORIENT
typedef enum {
	TARE_Z = 4,
	TARE_ALL = 7,
} BNO_TareAxis_t;

typedef enum {
	RotationVector = 0,
	GamingRotationVector = 1,
	GeomagneticRotationVector = 2,
	GyroIntegratedRotationVector = 3,
	ArVrStabilizedRotationVector = 4,
	ArVrStabilizedGameRotationVector =5,
} BNO_TareRV_t;

#endif
typedef struct __attribute__((packed)) {
	uint8_t	Status;					// Status (0 – success. Non Zero – Operation failed)
	uint8_t	AccCalEnable;		// Accel Cal Enable (1 – enabled, 0 – disabled)
	uint8_t	GyroCalEnable;	// Gyro Cal Enable (1 – enabled, 0 – disabled)
	uint8_t	MagCalEnable;		// Mag Cal Enable (1 – enabled, 0 – disabled)
	uint8_t	PlanCalEnable;	// Planar Accel Cal Enable (1 – enabled, 0 – disabled)
	uint8_t	OnTableEnable;	// On Table Cal Enable (1 – enabled, 0 – disabled)
} BNO_calibrationStat_t;

typedef struct __attribute__((packed)) {
	uint8_t OperationMode;
	uint8_t reserved;
	uint32_t Status;
	uint32_t Errors;
} BNO_Boot_t;

typedef struct __attribute__((packed)) {
	uint8_t sensorID; // sensor id
	uint8_t flags; // FEAT_... values
	uint16_t changeSensitivity;
	uint32_t reportInterval_uS;
	uint32_t batchInterval_uS;
	uint32_t sensorSpecific;
} BNO_Feature_t;

// Sensor values 

typedef struct BNO_RollPitchYaw {
	float Roll;
	float Pitch;
	float Yaw;
} BNO_RollPitchYaw_t;

#ifdef RAW_ACCELEROMETER
// Raw Accelerometer
typedef struct __attribute__((packed)) {
	// Units are ADC counts
	int16_t X; // [ADC counts]
	int16_t Y; // [ADC counts]
	int16_t Z; // [ADC counts]
	uint32_t TimeStamp; // [uS]
} BNO_RawAccelerometer_t;
#endif

#if defined(ACCELEROMETER) || defined(LINEAR_ACCELERATION) || defined(GRAVITY)
// Accelerometer
typedef struct __attribute__((packed)) {
	float X;
	float Y;
	float Z;
} BNO_Accelerometer_t;
#endif

#ifdef RAW_GYROSCOPE
// Raw Gyroscope
typedef struct __attribute__((packed)) {
	// Units are ADC counts
	int16_t X; // [ADC Counts]
	int16_t Y; // [ADC Counts]
	int16_t Z; // [ADC Counts]
	int16_t Temperature; // [ADC Counts]
	uint32_t TimeStamp; // [uS]
} BNO_RawGyroscope_t;
#endif

#ifdef GYROSCOPE_CALIBRATED
// Gyroscope
typedef struct __attribute__((packed)) {
	// Units are rad/s
	float X;
	float Y;
	float Z;
} BNO_Gyroscope_t;
#endif

#ifdef GYROSCOPE_UNCALIBRATED
// Uncalibrated Gyroscope
typedef struct __attribute__((packed)) {
	// Units are rad/s
	float X; // [rad/s]
	float Y; // [rad/s]
	float Z; // [rad/s]
	float BiasX; // [rad/s]
	float BiasY; // [rad/s]
	float BiasZ; // [rad/s]
} BNO_GyroscopeUncalibrated_t;
#endif

#ifdef RAW_MAGNETOMETER
// Raw Magnetometer
typedef struct __attribute__((packed)) {
	// Units are ADC counts
	int16_t X; // [ADC Counts]
	int16_t Y; // [ADC Counts]
	int16_t Z; // [ADC Counts]
	uint32_t TimeStamp; // [uS]
} BNO_RawMagnetometer_t;
#endif

#ifdef MAGNETIC_FIELD_CALIBRATED
// Magnetic Field
typedef struct __attribute__((packed)) {
	// Units are uTesla
	float X; // [uTesla]
	float Y; // [uTesla]
	float Z; // [uTesla]
} BNO_MagneticField_t;
#endif

#ifdef MAGNETIC_FIELD_UNCALIBRATED
// Uncalibrated Magnetic Field
typedef struct __attribute__((packed)) {
	// Units are uTesla
	float X; // [uTesla]
	float Y; // [uTesla]
	float Z; // [uTesla]
	float BiasX; // [uTesla]
	float BiasY; // [uTesla]
	float BiasZ; // [uTesla]
} BNO_MagneticFieldUncalibrated_t;
#endif

#if defined(ROTATION_VECTOR) || defined(GEOMAGNETIC_ROTATION_VECTOR) || defined(ARVR_STABILIZED_RV)
// Rotation Vector with Accuracy
typedef struct __attribute__((packed)) {
	float I; // Quaternion component i
	float J; // Quaternion component j
	float K; // Quaternion component k
	float Real; // Quaternion component, real
	float Accuracy; // Accuracy estimate [radians]
} BNO_RotationVectorWAcc_t;
#endif

#if defined(GAME_ROTATION_VECTOR) || defined(ARVR_STABILIZED_GRV)
// Rotation Vector
typedef struct __attribute__((packed)) {
	float I; // Quaternion component i
	float J; // Quaternion component j
	float K; // Quaternion component k
	float Real; // Quaternion component real
} BNO_RotationVector_t;
#endif

#ifdef TAP_DETECTOR
typedef enum {
	TAPDET_X			= 1,
	TAPDET_X_POS	= 2,
	TAPDET_Y			= 4,
	TAPDET_Y_POS	= 8,
	TAPDET_Z			= 16,
	TAPDET_Z_POS	= 32,
	TAPDET_DOUBLE = 64,
} BNO_Tap_t;

#endif

#ifdef STEP_COUNTER
typedef struct __attribute__((packed)) {
    uint32_t Latency; // Step counter latency [uS].
    uint16_t Steps; // Steps counted.
} BNO_StepCounter_t;
#endif

#ifdef STABILITY_CLASSIFIER
typedef enum {
	STABILITY_CLASSIFIER_UNKNOWN = 0,
	STABILITY_CLASSIFIER_ON_TABLE,
	STABILITY_CLASSIFIER_STATIONARY,
	STABILITY_CLASSIFIER_STABLE,
	STABILITY_CLASSIFIER_MOTION,
} BNO_Stability_t;
#endif

#ifdef SHAKE_DETECTOR
typedef enum {
	SHAKE_X = 1,
	SHAKE_Y = 2,
	SHAKE_Z = 4,
} BNO_Shake_t;
#endif

#ifdef PICKUP_DETECTOR
typedef enum {
	PICKUP_LEVEL_TO_NOT_LEVEL = 1,
	PICKUP_STOP_WITHIN_REGION = 2,
} BNO_Pickup_t;
#endif

#ifdef STABILITY_DETECTOR
typedef enum {
	STABILITY_ENTERED = 1,
	STABILITY_EXITED	= 2,
} BNO_StabilityDetector_t;
#endif

#ifdef PERSONAL_ACTIVITY_CLASSIFIER
typedef enum {
	PAC_UNKNOWN = 0,
	PAC_IN_VEHICLE,
	PAC_ON_BICYCLE,
	PAC_ON_FOOT,
	PAC_STILL,
	PAC_TILTING,
	PAC_WALKING,
	PAC_RUNNING,
} BNO_PAC_t;
// Personal Activity Classifier
typedef struct BNO_PersonalActivityClassifier {
	uint8_t Page;
	uint8_t LastPage;
	BNO_PAC_t MostLikelyState;
	uint8_t Confidence[10];
} BNO_PersonalActivityClassifier_t;
#endif

#ifdef GYRO_INTEGRATED_RV
// Gyro Integrated Rotation Vector
typedef struct BNO_GyroIntegratedRV {
	float I;   // Quaternion component i
	float J;   // Quaternion component j
	float K;   // Quaternion component k
	float Real;// Quaternion component real
	float AngleVelX; // Angular velocity about x [rad/s]
	float AngleVelY; // Angular velocity about y [rad/s]
	float AngleVelZ; // Angular velocity about z [rad/s]
} BNO_GyroIntegratedRV_t;
#endif

#ifdef IZRO_MOTION_REQUEST
// Interactive ZRO Motion Intent. See the SH-2 Reference Manual, 6.4.13
typedef enum {
	IZRO_MI_UNKNOWN = 0,
	IZRO_MI_STATIONARY_NO_VIBRATION,
	IZRO_MI_STATIONARY_WITH_VIBRATION,
	IZRO_MI_IN_MOTION,
	IZRO_MI_ACCELERATING,
} BNO_IZroMotionIntent_t;

typedef enum {
	IZRO_MR_NO_REQUEST = 0,
	IZRO_MR_STAY_STATIONARY,
	IZRO_MR_STATIONARY_NON_URGENT,
	IZRO_MR_STATIONARY_URGENT,
} BNO_IZroMotionRequest_t;

typedef struct BNO_IZroRequest {
	BNO_IZroMotionIntent_t Intent;
	BNO_IZroMotionRequest_t Request;
} BNO_IZroRequest_t;
#endif

#ifdef RAW_OPTICAL_FLOW
typedef struct BNO_RawOptFlow {
	uint32_t TimeStamp;
	int16_t Dt;
	int16_t Dx;
	int16_t Dy;
	int16_t Iq;
	uint8_t ResX;
	uint8_t ResY;
	uint8_t Shutter;
	uint8_t FrameMax;
	uint8_t FrameAvg;
	uint8_t FrameMin;
	uint8_t LaserOn;
} BNO_RawOptFlow_t;
#endif

#ifdef DEAD_RECKONING_POSE
typedef struct BNO_DeadReckoningPose {
	uint32_t TimeStamp;
	float LinPosX;
	float LinPosY;
	float LinPosZ;
	float I;
	float J;
	float K;
	float Real;
	float LinVelX;
	float LinVelY;
	float LinVelZ;
	float AngleVelX;
	float AngleVelY;
	float AngleVelZ;
} BNO_DeadReckoningPose_t;
#endif

#ifdef WHEEL_ENCODER
typedef struct BNO_WheelEncoder {
	uint32_t TimeStamp;
	uint8_t WheelIndex;
	uint8_t DataType;
	uint16_t Data;
} BNO_WheelEncoder_t;
#endif

typedef struct BNO_SensorValue {
	uint8_t sensorId; // Which sensor produced this event.
	uint8_t sequence; // The sequence number increments once for each report sent. Gaps in the sequence numbers indicate missing or dropped reports.
	uint8_t status; // bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy 0 - Unreliable 1 - Accuracy low 2 - Accuracy medium 3 - Accuracy high
	uint64_t timestamp;  // [uS]
	uint32_t delay; /// [uS] value is delay * 2^exponent (see status)
	union {
		#ifdef RAW_ACCELEROMETER
			BNO_RawAccelerometer_t RawAccelerometer;
		#endif
		#ifdef ACCELEROMETER
			BNO_Accelerometer_t Accelerometer;
		#endif
		#ifdef LINEAR_ACCELERATION
			BNO_Accelerometer_t LinearAcceleration;
		#endif
		#ifdef GRAVITY
			BNO_Accelerometer_t Gravity;
		#endif
		#ifdef RAW_GYROSCOPE
			BNO_RawGyroscope_t RawGyroscope;
		#endif
		#ifdef GYROSCOPE_CALIBRATED
			BNO_Gyroscope_t Gyroscope;
		#endif
		#ifdef GYROSCOPE_UNCALIBRATED
			BNO_GyroscopeUncalibrated_t GyroscopeUncal;
		#endif
		#ifdef RAW_MAGNETOMETER
			BNO_RawMagnetometer_t RawMagnetometer;
		#endif
		#ifdef MAGNETIC_FIELD_CALIBRATED
			BNO_MagneticField_t MagneticField;
		#endif
		#ifdef MAGNETIC_FIELD_UNCALIBRATED
			BNO_MagneticFieldUncalibrated_t MagneticFieldUncal;
		#endif
		#ifdef ROTATION_VECTOR
			BNO_RotationVectorWAcc_t RotationVector;
		#endif
		#ifdef GAME_ROTATION_VECTOR
			BNO_RotationVector_t GameRotationVector;
		#endif
		#ifdef GEOMAGNETIC_ROTATION_VECTOR
			BNO_RotationVectorWAcc_t GeoMagRotationVector;
		#endif
		#ifdef PRESSURE
			float Pressure; // Atmospheric Pressure.  [hectopascals]
		#endif
		#ifdef AMBIENT_LIGHT
			float AmbientLight; // Ambient Light.  [lux]
		#endif
		#ifdef HUMIDITY
			float Humidity; // Relative Humidity.  [percent]
		#endif
		#ifdef PROXIMITY
			float Proximity; // Proximity.  [cm]
		#endif
		#ifdef TEMPERATURE
			float Temperature; // Temperature.  [C]
		#endif
		#ifdef RESERVED
			float Reserved;  // Reserved
		#endif
		#ifdef TAP_DETECTOR
			BNO_Tap_t TapDetectorFlag;
		#endif
		#ifdef STEP_DETECTOR
			uint32_t StepDetectorLatency; // Step detect latency [uS]
		#endif
		#ifdef STEP_COUNTER
			BNO_StepCounter_t StepCounter;
		#endif
		#ifdef SIGNIFICANT_MOTION
			uint16_t SignificantMotion;
		#endif
		#ifdef STABILITY_CLASSIFIER
			BNO_Stability_t StabilityClassifier;
		#endif
		#ifdef SHAKE_DETECTOR
			BNO_Shake_t ShakeDetector;
		#endif
		#ifdef FLIP_DETECTOR
			uint16_t FlipDetector;
		#endif
		#ifdef PICKUP_DETECTOR
			BNO_Pickup_t PickupDetector;
		#endif
		#ifdef STABILITY_DETECTOR
			BNO_StabilityDetector_t StabilityDetector;
		#endif
		#ifdef PERSONAL_ACTIVITY_CLASSIFIER
			BNO_PersonalActivityClassifier_t PersonalActivityClassifier;
		#endif
		#ifdef SLEEP_DETECTOR
			uint8_t SleepDetector;
		#endif
		#ifdef TILT_DETECTOR
			uint16_t TiltDetector;
		#endif
		#ifdef POCKET_DETECTOR
			uint16_t PocketDetector;
		#endif
		#ifdef CIRCLE_DETECTOR
			uint16_t CircleDetector;
		#endif
		#ifdef HEART_RATE_MONITOR
			uint16_t HeartRateMonitor; // Heart rate in beats per minute.
		#endif
		#ifdef ARVR_STABILIZED_RV
			BNO_RotationVectorWAcc_t ArVrStabilizedRV;
		#endif
		#ifdef ARVR_STABILIZED_GRV
			BNO_RotationVector_t ArVrStabilizedGRV;
		#endif
		#ifdef GYRO_INTEGRATED_RV
			BNO_GyroIntegratedRV_t GyroIntegratedRV;
		#endif
		#ifdef IZRO_MOTION_REQUEST
			BNO_IZroRequest_t IzroRequest;
		#endif
		#ifdef RAW_OPTICAL_FLOW
			BNO_RawOptFlow_t RawOptFlow;
		#endif
		#ifdef DEAD_RECKONING_POSE
			BNO_DeadReckoningPose_t DeadReckoningPose;
		#endif
		#ifdef WHEEL_ENCODER
			BNO_WheelEncoder_t WheelEncoder;
		#endif
	} SenVal;
} BNO_SensorValue_t;

extern BNO_SensorValue_t sensorData;
extern BNO_RollPitchYaw_t rpy;
// Initialize the sesnor
// During reset or power-on sequence, the bootloader first checks the status of the BOOTN pin.
// If the pin is pulled low during reset or poweron, the BNO08X will enter the bootloader mode.
// If the BOOTN pin is pulled high, then the bootloader starts the application
HAL_StatusTypeDef BNO_Init(void);
// Check if we have unexpected reset
uint8_t isResetOccurred(void);
// Get the sensor that has new data
uint8_t BNO_getSensorEventID(void);
// Soft reset the sensor
uint8_t BNO_Reset(void);
// Turn sensor ON
HAL_StatusTypeDef BNO_On(void);
// When sleep command is issued all sensors that are configured as always on or wake (see 1.3.5.1) will continue to operate all, other sensors will be disabled
HAL_StatusTypeDef BNO_Sleep(void);
// Returns the product id data structure
BNO_productID_t BNO_getProductID(void);
HAL_StatusTypeDef BNO_writeFRS(const uint16_t length, const uint16_t frsType);
HAL_StatusTypeDef BNO_readFRS(const uint16_t frsType);
#ifdef USE_ERROR_REPORT
// See 6.4.1 Report Errors (0x01)
HAL_StatusTypeDef BNO_getErrors(void);
#endif

#ifdef USE_COUNTER_REPORT
// See 6.4.2 Counter Commands (0x02)
HAL_StatusTypeDef BNO_getCounter(const uint8_t sensorID);
HAL_StatusTypeDef BNO_clearCounter(const uint8_t sensorID);
#endif

#ifdef USE_REORIENT
// See 6.4.3.1 Tare Now (0x00)
HAL_StatusTypeDef BNO_TareNow(const BNO_TareAxis_t axis, const BNO_TareRV_t vector);
//This command instructs SH-2 to persist the results of the last tare operation to flash for use at the next system restart.
HAL_StatusTypeDef BNO_TarePerist(void);
// This command instructs SH-2 to set the current run-time sensor reorientation.
// The rotation vector is a signed, 16-bit 2’s-complement fixed point number with a Q-point of 14.
HAL_StatusTypeDef BNO_TareSetReorientation(const double X, const double Y, const double Z, const double W);
// Clears the previously applied tare operation.
HAL_StatusTypeDef BNO_ClearTare(void);
#endif

// !!! ToDo check why is not responding
// The sensor hub responds to the Initialize command with an Initialize Response. In the case
// where the sensor hub reinitializes itself, this response is unsolicited. An unsolicited response is
// also generated after startup
HAL_StatusTypeDef BNO_Initialize(void);
// Save Dynamic Calibration Data (DCD) to flash
HAL_StatusTypeDef BNO_saveCalibration(void);
// This command is sent by the host to configure the ME calibration of the accelerometer, gyro and
// magnetometer giving the host the ability to control when calibration is performed.
HAL_StatusTypeDef BNO_condigureCalibration(const uint8_t sensors);
// This command is sent by the host to request the enable/disable state of the accelerometer, gyro and magnetometer calibration routines
HAL_StatusTypeDef BNO_enableCalibration(void);
// The Configure Periodic DCD Save command configures the automatic saving of DCD. There is
// no response to this command. This command does not inhibit the Save DCD command.
// 0 Enable, 1 Disable
HAL_StatusTypeDef BNO_configurePeriodicDcdSave(const uint8_t enableStatus);
// The Get Oscillator Type command is used to get information about the oscillator type used in the clock system of the SH-2
BNO_Oscillator_t BNO_getOscillatorType(void);
// This command performs an atomic clearDCD (from RAM) and system reset
HAL_StatusTypeDef BNO_clearDcdReset(void);
// See 6.4.10 Simple Calibration Commands (0x0C)
HAL_StatusTypeDef BNO_simpleCalibration(const uint32_t usInterval, const uint16_t calibrationTimeMs);
// !!! ToDo check why is not responding
// The bootloader operating mode request is used to request various operating modes of the FSP200 bootloader
HAL_StatusTypeDef BNO_setBootMode(const BNO_BootMode_t mode);
// !!! ToDo check why is not responding
// Request product ID information about the FSP200 bootloader
BNO_BootMode_t BNO_getBootMode(void);
// The interactive calibration feature requires that the sensor hub be told of the device’s intended motion.
HAL_StatusTypeDef BNO_interactiveCalibration(const BNO_MotionIntent_t intent);
#ifdef WHEEL_ENCODER
// Provide a single sample of wheel encoder data. No response is sent for this command.
HAL_StatusTypeDef BNO_WheelRequest(const uint8_t wheelIndex, const uint32_t timeStampUs, const int16_t wheelData, const uint8_t dataType);
#endif

// This is sent from the host to the hub to trigger a flush of outstanding data from a given sensor
HAL_StatusTypeDef BNO_forceFlush(const uint8_t sensorID);

#ifdef USE_FOR_TELESCOPE
// Sets -X North, Y East Z down
HAL_StatusTypeDef setTelescopeOrientation(void);
// This command is sent by the host to request the enable/disable state of the accelerometer, gyro
// and magnetometer calibration routines.
HAL_StatusTypeDef BNO_getCalibrationStatus(void);

HAL_StatusTypeDef BNO_isCalibrationComplete(void);

HAL_StatusTypeDef BNO_calibrateHighAccuracyAndReset(void);
#endif

HAL_StatusTypeDef BNO_dataAvailable(void);
// Enable features an set report time in mili seconds
BNO_Feature_t BNO_getFeature(const uint8_t sensorID);
HAL_StatusTypeDef BNO_setFeature(const uint8_t reportID, const uint32_t microsBetweenReports, const uint32_t specificConfig);	
	
BNO_RollPitchYaw_t BNO_getRollPitchYaw(void);


#ifdef RAW_ACCELEROMETER
// Raw uncalibrated accelerometer data (ADC units), used for testing
BNO_RawAccelerometer_t getRawAccelerometer(void);
#endif
#ifdef ACCELEROMETER
// Calibrated Acceleration (m/s2). Acceleration of the device including gravity
BNO_Accelerometer_t getaccelerometer(void);
#endif
#ifdef LINEAR_ACCELERATION
// Linear acceleration (m/s2). Acceleration of the device with gravity removed
BNO_Accelerometer_t getLinearAcceleration(void);
#endif
#ifdef GRAVITY
// Gravity (m/s2)
BNO_Accelerometer_t getGravity(void);
#endif
#ifdef RAW_GYROSCOPE
// Raw uncalibrated gyroscope (ADC units). Data direct from the gyroscope, used for testing
BNO_RawGyroscope_t getRawGyroscope(void);
#endif
#ifdef GYROSCOPE_CALIBRATED
// Calibrated gyroscope (rad/s). The angular velocity of the device
BNO_Gyroscope_t getGyroscope(void);
#endif
#ifdef GYROSCOPE_UNCALIBRATED
// Uncalibrated gyroscope (rad/s). Angular velocity without bias compensation.
BNO_GyroscopeUncalibrated_t getGyroscopeUncal(void);
#endif
#ifdef RAW_MAGNETOMETER
// Raw magnetic field measurement (in ADC units). Direct data from the magnetometer, used for testing
BNO_RawMagnetometer_t getRawMagnetometer(void);
#endif
#ifdef MAGNETIC_FIELD_CALIBRATED
// Magnetic field calibrated (in µTesla). The fully calibrated magnetic field measurement.
BNO_MagneticField_t getMagneticField(void);
#endif
#ifdef MAGNETIC_FIELD_UNCALIBRATED
// Magnetic field uncalibrated (in µTesla). The magnetic field measurement without hard-iron offset applied
BNO_MagneticFieldUncalibrated_t getMagneticFieldUncal(void);
#endif
#ifdef ROTATION_VECTOR
// THE ROTATION VECTOR PROVIDES an orientation output that is expressed as a quaternion referenced to magnetic north
// and gravity. It is produced by fusing the outputs of the accelerometer, gyroscope and magnetometer. The rotation
// vector is the most accurate orientation estimate available. The magnetometer provides correction in yaw to
// reduce drift and the gyroscope enables the most responsive performance.
BNO_RotationVectorWAcc_t getRotationVector(void);
#endif
#ifdef GAME_ROTATION_VECTOR
// THE GAME ROTATION VECTOR is an orientation output that is expressed as a quaternion with no specific reference for
// heading, while roll and pitch are referenced against gravity. It is produced by fusing the outputs of the
// accelerometer and the gyroscope (i.e. no magnetometer). The game rotation vector does not use the
// magnetometer to correct the gyroscopes drift in yaw. This is a deliberate omission (as specified by Google) to
// allow gaming applications to use a smoother representation of the orientation without the jumps that an
// instantaneous correction provided by a magnetic field update could provide.
BNO_RotationVector_t getGameRotationVector(void);
#endif
#ifdef GEOMAGNETIC_ROTATION_VECTOR
// THE GEOMAGNETIC ROTATION VECTOR is an orientation output that is expressed as a quaternion referenced to magnetic
// north and gravity. It is produced by fusing the outputs of the accelerometer and magnetometer. The gyroscope is
// specifically excluded in order to produce a rotation vector output using less power than is required to produce the
// rotation vector of section 2.2.4. The consequences of removing the gyroscope are:
// • Less responsive output since the highly dynamic outputs of the gyroscope are not used
// • More errors in the presence of varying magnetic fields
BNO_RotationVectorWAcc_t getGeoMagRotationVector(void);
#endif
#ifdef PRESSURE
// The pressures sensor reports atmospheric pressure. The units are hectopascals. 
float getPressure(void);
#endif
#ifdef AMBIENT_LIGHT
// The ambient light sensor reports the measures the amount of light entering the sensor. The units are lux. 
float getAmbientLight(void);
#endif
#ifdef HUMIDITY
// The humidity sensor reports relative humidity in the ambient air. The units are percent.
float getHumidity(void);
#endif
#ifdef PROXIMITY
// The proximity sensor reports distance from the device. The units are centimeters.
float getProximity(void);
#endif
#ifdef TEMPERATURE
// The temperature sensor reports ambient temperature. The units are °C. 
float getTemperature(void);
#endif
#ifdef RESERVED
// 
float getReserved(void);
#endif
#ifdef TAP_DETECTOR
// The tap detector reports single and double taps. 
BNO_Tap_t getTapDetectorFlag(void);
#endif
#ifdef STEP_DETECTOR
// The step detector reports steps detected. Each report indicates a single step.
uint32_t getStepDetectorLatency(void);
#endif
#ifdef STEP_COUNTER
// The step counter reports steps counted. The units are steps.
BNO_StepCounter_t getStepCounter(void);
#endif
#ifdef SIGNIFICANT_MOTION
// The significant motion detector sends a single report when it detects significant motion. It
// automatically turns itself off after sending its single report. 
uint16_t getSignificantMotion(void);
#endif
#ifdef STABILITY_CLASSIFIER
// The stability classifier sensor reports the type of stability detected.
BNO_Stability_t getStabilityClassifier(void);
#endif
#ifdef SHAKE_DETECTOR
// The shake detector sends a report each time it detects a shake. 
BNO_Shake_t getShakeDetector(void);
#endif
#ifdef FLIP_DETECTOR
// The flip detector sends a report each time it detects a flip.
uint16_t getFlipDetector(void);
#endif
#ifdef PICKUP_DETECTOR
// The pickup detector sends a report each time it detects a pickup.
BNO_Pickup_t getPickupDetector(void);
#endif
#ifdef STABILITY_DETECTOR
// The stability detector sends a report each time it detects entry in or exit from a stable state.
BNO_StabilityDetector_t getStabilityDetector(void);
#endif
#ifdef PERSONAL_ACTIVITY_CLASSIFIER
// See 6.5.36 Personal Activity Classifier (0x1E)
BNO_PersonalActivityClassifier_t getPersonalActivityClassifier(void);
#endif
#ifdef SLEEP_DETECTOR
// The sleep detector sends a report each time it detects a change in sleep state.
uint8_t getSleepDetector(void);
#endif
#ifdef TILT_DETECTOR
// The tilt detector sends a report each time it detects a tilt.
uint16_t getTiltDetector(void);
#endif
#ifdef POCKET_DETECTOR
// The pocket detector sends a report each time it detects entry in or exit from a pocket state.
uint16_t getPocketDetector(void);
#endif
#ifdef CIRCLE_DETECTOR
// The circle detector sends a report each time it detects a double circle gesture. 
uint16_t getCircleDetector(void);
#endif
#ifdef HEART_RATE_MONITOR
// The heart rate monitor measures the user’s heart rate reported in beats per minute (bpm)
uint16_t getHeartRateMonitor(void);
#endif
#ifdef ARVR_STABILIZED_RV
// AR/VR STABILIZED ROTATION VECTOR
// Estimates of the magnetic field and the roll/pitch of the device can create a potential correction in the rotation
// vector produced. For applications (typically augmented or virtual reality applications) where a sudden jump can be
// disturbing, the output is adjusted to prevent these jumps in a manner that takes account of the velocity of the
// sensor system. This process is called AR/VR stabilization. An FRS (Flash Record System – see Figure 1-31)
// record is provided to allow configuration of this feature
BNO_RotationVectorWAcc_t BNO_getArvrStabilizedRV(void);
#endif
#ifdef ARVR_STABILIZED_GRV
// AR/VR STABILIZED GAME ROTATION VECTOR
// While the magnetometer is removed from the calculation of the game rotation vector, the accelerometer itself can
// create a potential correction in the rotation vector produced (i.e. the estimate of gravity changes). For applications
// (typically augmented or virtual reality applications) where a sudden jump can be disturbing, the output is adjusted
// to prevent these jumps in a manner that takes account of the velocity of the sensor system. This process is called
// AR/VR stabilization. An FRS (Flash Record System – see Figure 1-31) record is provided to allow configuration of this feature.
BNO_RotationVector_t BNO_getArVrStabilizedGRV(void);
#endif
#ifdef GYRO_INTEGRATED_RV
// GYRO ROTATION VECTOR
// Head tracking systems within a virtual reality headset require low latency processing of motion. To facilitate this,
// the BNO08X can provide a rotation vector at rates up to 1kHz. The gyro rotation Vector provides an alternate
// orientation to the standard rotation vector. Compared to the standard rotation vector the gyro rotation vector has
// an optimized processing path and correction style (correction is the adjustments made to the output based on
// more accurate estimates of gravity, mag field, angular velocity) that is suitable for head tracking applications.
BNO_GyroIntegratedRV_t BNO_getGyroIntegratedRV(void);
#endif
#ifdef IZRO_MOTION_REQUEST
BNO_IZroRequest_t getIzroRequest(void);
#endif
#ifdef RAW_OPTICAL_FLOW
// The Optical Flow report provides raw optical flow sensor data.
BNO_RawOptFlow_t getRawOptFlow(void);
#endif
#ifdef DEAD_RECKONING_POSE
// The Dead Reckoning Pose report provides the linear position, angular position, linear velocity,
// and angular velocity as determined by the Dead Reckoning system.
BNO_DeadReckoningPose_t getDeadReckoningPose(void);
#endif
#ifdef WHEEL_ENCODER
BNO_WheelEncoder_t getWheelEncoder(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BNO_08x_I2C_H */
