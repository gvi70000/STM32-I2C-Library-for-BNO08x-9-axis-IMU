#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "BNO_08x_I2C.h"
#include "i2c.h"
#include "gpio.h"
#include "tim.h"

//Global Variables

// The interrupt variable
volatile uint8_t BNO_Ready = 0;
//A monotonically incrementing uint8_t that rolls over. It is used to detect missing commands and to synchronize responses
static uint8_t commandSequenceNumber = 0;
// Similar with  the above var
static uint8_t cmdSeqNo = 0;
// The data array for read and write operations
static uint8_t bufferIO[RX_PACKET_SIZE];
// 6 SHTP channels. Each channel has its own sequence number
static uint8_t sequenceNumber[SEQUENCE_SIZE];
// Variable that hold the reset status of the sensor
static uint8_t resetOccurred = 0;

static uint8_t saveDcdStatus = 1;

// Structure to hold the product ID
static BNO_productID_t myID = {0};
#ifdef USE_ERROR_REPORT
static BNO_Error_t errors  = {0};
#endif
#ifdef USE_COUNTER_REPORT
static BNO_Counts_t counts  = {0};
#endif
#ifdef USE_ERROR_REPORT
static BNO_Error_t errors  = {0};
#endif
// Structure to hold the product calibration status data
static BNO_calibrationStat_t calibrationStatus = {0};
// Structure to hold the product calibration status data
static BNO_FrsWriteResp_t frsWriteResponse = {0};
static BNO_FrsReadResp_t frsReadResponse = {0};
static BNO_Oscillator_t oscillatorType = {0};
static BNO_Boot_t bootLoader = {0};

#ifdef USE_FOR_TELESCOPE
static BNO_CommandReq_t cmdRequest = {0};
static BNO_CommandResp_t cmdResponse = {0};
static BNO_Feature_t sensorFeartures = {0};
BNO_RollPitchYaw_t rpy = {0};
#endif

BNO_SensorValue_t sensorData = {0};


// Wait for an interrupt to occur or timeout in 200ms
static uint8_t waitInt(void) {
	uint32_t timeOut = HAL_GetTick() + RESET_DELAY;
	while(timeOut > HAL_GetTick()) {
		if(BNO_Ready) {
			BNO_Ready = 0;
			return 1;
		}
	}
	return 0;	
}

// Timer 2 set to get microseconds
static void start_timer(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();
	// Prescale to get 1 count per uS
	uint16_t prescaler = 274;
	htim2.Instance = TIM2;
	htim2.Init.Period = 0xFFFFFFFF;
	htim2.Init.Prescaler = prescaler;
	htim2.Init.ClockDivision = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start(&htim2);
}

// Get microseconds
static uint32_t timeNowUs(void) {
	return __HAL_TIM_GET_COUNTER(&htim2);
}

// Delay microseconds
static void delay_us(uint32_t t) {
	uint32_t now = timeNowUs();
	uint32_t start = now;
	while ((now - start) < t) {
		now = timeNowUs();
	}
}

#ifdef USE_FOR_TELESCOPE
// get 1/sqrt(x)
static float invSqrt(const float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// Update structures quatIJKR and angles
static void quaternionUpdate(const BNO_RotationVectorWAcc_t *inRotVector) {
	double norm = invSqrt(inRotVector->I * inRotVector->I + inRotVector->J * inRotVector->J + inRotVector->K * inRotVector->K + inRotVector->Real * inRotVector->Real);    // normalize quaternion
	
	double q1 = inRotVector->I * norm; //x
	double q2 = inRotVector->J * norm; //y
	double q3 = inRotVector->K * norm; //z
	double q4 = inRotVector->Real * norm; //w

	rpy.Pitch = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	rpy.Roll  = -asinf(2.0f * (q2*q4 - q1*q3));
	rpy.Yaw   = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	rpy.Pitch *= _180_DIV_PI;
	rpy.Roll  *= _180_DIV_PI;
	rpy.Yaw   *= _180_DIV_PI;
	
	if(rpy.Yaw >= 0.0)
		rpy.Yaw = 360.0 - rpy.Yaw;
	else	
		rpy.Yaw = -rpy.Yaw;
	
	if(rpy.Pitch >= 0.0)
		rpy.Pitch = 180.0 - rpy.Pitch;
	else
		rpy.Pitch = -(rpy.Pitch + 180.f);
}
#endif

// Sets to buffrIO first 21 bytes to 0
static void resetHeader(const uint8_t id) {
	memset(bufferIO, 0, TX_PACKET_SIZE);
	bufferIO[4] = id;
}

// Send a packet data to BNO
static HAL_StatusTypeDef sendPacket(const uint8_t channelNumber) {
	// dataLength includes the SHTP_HEADER_SIZE
	uint8_t dataLength = 0;
	bufferIO[2] = channelNumber;
	bufferIO[3] = sequenceNumber[channelNumber]++;
	if(bufferIO[2] == CHANNEL_EXECUTABLE) {
		dataLength = 5;
	} else {
		switch(bufferIO[4]) {
			case REPORT_SENSOR_FLUSH_REQUEST:
			case REPORT_GET_FEATURE_REQUEST:
			case REPORT_PRODUCT_ID_REQUEST:
				dataLength = 6;
			break;			
			case REPORT_FRS_READ_REQUEST:
				dataLength = 12;
			break;
			case COMMAND_ME_CALIBRATE:
			case COMMAND_TARE:
			case COMMAND_SAVE_DCD:
			case REPORT_COMMAND_REQUEST:
			case REPORT_FRS_WRITE_REQUEST:
				dataLength = 16;
			break;
			case REPORT_SET_FEATURE_COMMAND:
				dataLength = 21;
			break;
		}
	}
	bufferIO[0] = dataLength & 0xFF;
	bufferIO[1] = (dataLength >> 8) & 0x7F;
	// Send packet to IMU
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, BNO_W_ADDR, bufferIO, dataLength, PORT_TIMEOUT);
	delay_us(RESET_DELAY); // Delay 100 microsecs before next I2C
	return ret;
}

// Get a data packet from BNO
static HAL_StatusTypeDef receivePacket(void) {
	// Reset Interrupt status
	BNO_Ready = 0;
	memset(bufferIO, 0, TX_PACKET_SIZE);
	//Ask for 4 bytes to find out how much data we need to read
	if(HAL_I2C_Master_Receive(&hi2c1, BNO_R_ADDR, bufferIO, HEADER_SIZE, PORT_TIMEOUT) != HAL_OK) {
		return HAL_ERROR;
	}
	// Calculate the number of data bytes in packet
	uint16_t rxPacketLength = *(uint16_t *)&bufferIO;//getPacketLenghth();
	// No data received ?
	if(!rxPacketLength) {
		return HAL_ERROR;
	}
	if(rxPacketLength > RX_PACKET_SIZE) {
		return HAL_ERROR;
	}
	// Wait 100us
	delay_us(RESET_DELAY); // Delay 100 microsecs before next I2C
	if(HAL_I2C_Master_Receive(&hi2c1, BNO_R_ADDR, bufferIO, rxPacketLength, PORT_TIMEOUT) != HAL_OK) {
		return HAL_ERROR;
	}
	delay_us(RESET_DELAY); // Delay 100 microsecs before next I2C
	return HAL_OK;
}

// Send a command on exe channel
static HAL_StatusTypeDef sendExecutable(const uint8_t cmd) {
	resetHeader(cmd);
	return sendPacket(CHANNEL_EXECUTABLE);
}
// Compute a sensor value from bufferIO
//bufferIO - 0..3 Header
//bufferIO - 4..8 Time stamp
//bufferIO - 9 Which sensor produced this event
//bufferIO - 10 Sequence number increments once for each report sent. Gaps in the sequence numbers indicate missing or dropped reports.
//bufferIO - 11 Status bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy
static void getSensorValue(void) {
	//Calculate the number of data bytes in this packet
	//int16_t dataLength = *(uint16_t *)&bufferIO - 4;
	sensorData.sensorId = bufferIO[9];
	sensorData.timestamp = *(uint32_t *)&bufferIO[5];
	#ifdef GYRO_INTEGRATED_RV
	if(sensorData.sensorId != GYRO_INTEGRATED_RV) {
		sensorData.sequence = bufferIO[10];
		sensorData.status = bufferIO[11] & 0x03; //Get status bits
	} else {
		sensorData.sequence = 0;
		sensorData.status = 0; //Get status bits
	}
	#else
		sensorData.sequence = bufferIO[10];
		sensorData.status = bufferIO[11] & 0x03; //Get status bits
	#endif


	switch(sensorData.sensorId)	{
		#ifdef RAW_ACCELEROMETER
		case RAW_ACCELEROMETER:
			sensorData.SenVal.RawAccelerometer.X = *(int16_t *)&bufferIO[13];
			sensorData.SenVal.RawAccelerometer.Y = *(int16_t *)&bufferIO[15];
			sensorData.SenVal.RawAccelerometer.Z = *(int16_t *)&bufferIO[17];
			sensorData.SenVal.RawAccelerometer.TimeStamp = *(uint32_t *)&bufferIO[21];
		break;
		#endif
		#ifdef ACCELEROMETER
		case ACCELEROMETER:
			sensorData.SenVal.Accelerometer.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.Accelerometer.Y = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.Accelerometer.Z = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
		break;
		#endif
		#ifdef LINEAR_ACCELERATION
		case LINEAR_ACCELERATION:
			sensorData.SenVal.LinearAcceleration.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.LinearAcceleration.Y = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.LinearAcceleration.Z = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
		break;
		#endif
		#ifdef GRAVITY
		case GRAVITY:
			sensorData.SenVal.Gravity.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.Gravity.Y = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
			sensorData.SenVal.Gravity.Z = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
		break;
		#endif
		#ifdef RAW_GYROSCOPE
		case RAW_GYROSCOPE:
			sensorData.SenVal.RawGyroscope.X = *(int16_t *)&bufferIO[13];
			sensorData.SenVal.RawGyroscope.Y = *(int16_t *)&bufferIO[15];
			sensorData.SenVal.RawGyroscope.Z = *(int16_t *)&bufferIO[17];
			sensorData.SenVal.RawGyroscope.Temperature = *(int16_t *)&bufferIO[19];
			sensorData.SenVal.RawGyroscope.TimeStamp = *(uint32_t *)&bufferIO[21];
		break;
		#endif
		#ifdef GYROSCOPE_CALIBRATED
		case GYROSCOPE_CALIBRATED:
			sensorData.SenVal.Gyroscope.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q9;
			sensorData.SenVal.Gyroscope.Y = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q9;
			sensorData.SenVal.Gyroscope.Z = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q9;
		break;
		#endif
		#ifdef GYROSCOPE_UNCALIBRATED
		case GYROSCOPE_UNCALIBRATED:
			sensorData.SenVal.GyroscopeUncal.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q9;
			sensorData.SenVal.GyroscopeUncal.Y = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q9;
			sensorData.SenVal.GyroscopeUncal.Z = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q9;
			sensorData.SenVal.GyroscopeUncal.BiasX = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q9;
			sensorData.SenVal.GyroscopeUncal.BiasY = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q9;
			sensorData.SenVal.GyroscopeUncal.BiasZ = (float)(*(int16_t *)&bufferIO[23]) * SCALE_Q9;
		break;
		#endif
		#ifdef RAW_MAGNETOMETER
		case RAW_MAGNETOMETER:
			sensorData.SenVal.RawMagnetometer.X = *(int16_t *)&bufferIO[13];
			sensorData.SenVal.RawMagnetometer.Y = *(int16_t *)&bufferIO[15];
			sensorData.SenVal.RawMagnetometer.Z = *(int16_t *)&bufferIO[17];
			sensorData.SenVal.RawMagnetometer.TimeStamp = *(uint32_t *)&bufferIO[21];
		break;
		#endif
		#ifdef MAGNETIC_FIELD_CALIBRATED
		case MAGNETIC_FIELD_CALIBRATED:
			sensorData.SenVal.MagneticField.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q4;
			sensorData.SenVal.MagneticField.Y = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q4;
			sensorData.SenVal.MagneticField.Z = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q4;
		break;
		#endif
		#ifdef MAGNETIC_FIELD_UNCALIBRATED
		case MAGNETIC_FIELD_UNCALIBRATED:
			sensorData.SenVal.MagneticFieldUncal.X = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q4;
			sensorData.SenVal.MagneticFieldUncal.Y = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q4;
			sensorData.SenVal.MagneticFieldUncal.Z = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q4;
			sensorData.SenVal.MagneticFieldUncal.BiasX = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q4;
			sensorData.SenVal.MagneticFieldUncal.BiasY = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q4;
			sensorData.SenVal.MagneticFieldUncal.BiasZ = (float)(*(int16_t *)&bufferIO[23]) * SCALE_Q4;
		break;
		#endif
		#ifdef ROTATION_VECTOR
		case ROTATION_VECTOR:
			#ifdef USE_FOR_TELESCOPE
			sensorData.SenVal.RotationVector.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.Accuracy = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q14;
			// Update Euler 
			//quaternionUpdate(&sensorData.SenVal.RotationVector);
			#else
			sensorData.SenVal.RotationVector.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
			sensorData.SenVal.RotationVector.Accuracy = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q14;
			#endif
		break;
		#endif
		#ifdef GAME_ROTATION_VECTOR
			case GAME_ROTATION_VECTOR:
			sensorData.SenVal.GameRotationVector.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.GameRotationVector.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.GameRotationVector.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.GameRotationVector.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
		break;
		#endif
		#ifdef GEOMAGNETIC_ROTATION_VECTOR
		case GEOMAGNETIC_ROTATION_VECTOR:
			sensorData.SenVal.GeoMagRotationVector.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.GeoMagRotationVector.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.GeoMagRotationVector.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.GeoMagRotationVector.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
			sensorData.SenVal.GeoMagRotationVector.Accuracy = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q14;
		break;
		#endif
		#ifdef PRESSURE
		case PRESSURE:
			sensorData.SenVal.Pressure = (float)(*(int32_t *)&bufferIO[13]) * SCALE_Q20;
		break;
		#endif
		#ifdef AMBIENT_LIGHT
		case AMBIENT_LIGHT:
			sensorData.SenVal.AmbientLight = (float)(*(int32_t *)&bufferIO[13]) * SCALE_Q8;
		break;
		#endif
		#ifdef HUMIDITY
		case HUMIDITY:
			sensorData.SenVal.Humidity = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q8;
		break;
		#endif
		#ifdef PROXIMITY
		case PROXIMITY:
			sensorData.SenVal.Proximity = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q4;
		break;
		#endif
		#ifdef TEMPERATURE
		case TEMPERATURE:
			sensorData.SenVal.Temperature = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q7;
		break;
		#endif
		#ifdef RESERVED
		case RESERVED:
			sensorData.SenVal.Reserved = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q7;
		break;
		#endif
		#ifdef TAP_DETECTOR
		case TAP_DETECTOR:
			sensorData.SenVal.TapDetectorFlag = bufferIO[13];
		break;
		#endif
		#ifdef STEP_DETECTOR
		case STEP_DETECTOR:
			sensorData.SenVal.StepDetectorLatency = *(uint32_t *)&bufferIO[13];
		break;
		#endif
		#ifdef STEP_COUNTER
		case STEP_COUNTER:
			sensorData.SenVal.StepCounter.Latency = *(uint32_t *)&bufferIO[13];
			sensorData.SenVal.StepCounter.Steps = *(uint32_t *)&bufferIO[17];
		break;
		#endif
		#ifdef SIGNIFICANT_MOTION
		case SIGNIFICANT_MOTION:
			sensorData.SenVal.SignificantMotion = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef STABILITY_CLASSIFIER
		case STABILITY_CLASSIFIER: 
			sensorData.SenVal.StabilityClassifier = bufferIO[13];
		break;
		#endif
		#ifdef SHAKE_DETECTOR
		case SHAKE_DETECTOR:
			sensorData.SenVal.ShakeDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef FLIP_DETECTOR
		case FLIP_DETECTOR:
			sensorData.SenVal.FlipDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef PICKUP_DETECTOR
		case PICKUP_DETECTOR:
			sensorData.SenVal.PickupDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef STABILITY_DETECTOR
		case STABILITY_DETECTOR:
			sensorData.SenVal.StabilityDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef PERSONAL_ACTIVITY_CLASSIFIER
		case PERSONAL_ACTIVITY_CLASSIFIER:
			sensorData.SenVal.PersonalActivityClassifier.Page = bufferIO[13] & 0x7F;
			sensorData.SenVal.PersonalActivityClassifier.LastPage = ((bufferIO[13] & 0x80) != 0);
			sensorData.SenVal.PersonalActivityClassifier.MostLikelyState = bufferIO[14];
			// ToDo remove for loop, use pointer
			for (int n = 0; n < 10; n++) {
				sensorData.SenVal.PersonalActivityClassifier.Confidence[n] = bufferIO[15+n];
			}
		break;
		#endif
		#ifdef SLEEP_DETECTOR
		case SLEEP_DETECTOR:
			sensorData.SenVal.SleepDetector = bufferIO[13];
		break;
		#endif
		#ifdef TILT_DETECTOR
		case TILT_DETECTOR:
			sensorData.SenVal.TiltDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef POCKET_DETECTOR
		case POCKET_DETECTOR:
			sensorData.SenVal.PocketDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef CIRCLE_DETECTOR
		case CIRCLE_DETECTOR:
			sensorData.SenVal.CircleDetector = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef HEART_RATE_MONITOR
		case HEART_RATE_MONITOR:
			sensorData.SenVal.HeartRateMonitor = *(uint16_t *)&bufferIO[13];
		break;
		#endif
		#ifdef ARVR_STABILIZED_RV
		case ARVR_STABILIZED_RV:
			sensorData.SenVal.ArVrStabilizedRV.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedRV.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedRV.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedRV.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedRV.Accuracy = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q12;
		break;
		#endif
		#ifdef ARVR_STABILIZED_GRV
		case ARVR_STABILIZED_GRV:
			sensorData.SenVal.ArVrStabilizedGRV.I = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedGRV.J = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedGRV.K = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q14;
			sensorData.SenVal.ArVrStabilizedGRV.Real = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q14;
		break;
		#endif
		#ifdef GYRO_INTEGRATED_RV
		case GYRO_INTEGRATED_RV:
			sensorData.SenVal.GyroIntegratedRV.I = (float)(*(int16_t *)&bufferIO[9]) * SCALE_Q14;
			sensorData.SenVal.GyroIntegratedRV.J = (float)(*(int16_t *)&bufferIO[11]) * SCALE_Q14;
			sensorData.SenVal.GyroIntegratedRV.J = (float)(*(int16_t *)&bufferIO[13]) * SCALE_Q14;
			sensorData.SenVal.GyroIntegratedRV.Real = (float)(*(int16_t *)&bufferIO[15]) * SCALE_Q14;
			sensorData.SenVal.GyroIntegratedRV.AngleVelX = (float)(*(int16_t *)&bufferIO[17]) * SCALE_Q10;
			sensorData.SenVal.GyroIntegratedRV.AngleVelY = (float)(*(int16_t *)&bufferIO[19]) * SCALE_Q10;
			sensorData.SenVal.GyroIntegratedRV.AngleVelZ = (float)(*(int16_t *)&bufferIO[21]) * SCALE_Q10;
		break;
		#endif
		#ifdef IZRO_MOTION_REQUEST
		case IZRO_MOTION_REQUEST:
			sensorData.SenVal.IzroRequest.Intent = (BNO_IZroMotionIntent_t)bufferIO[13];
			sensorData.SenVal.IzroRequest.Request = (BNO_IZroMotionRequest_t)bufferIO[14];
		break;
		#endif
		#ifdef RAW_OPTICAL_FLOW
		case RAW_OPTICAL_FLOW:
			sensorData.SenVal.RawOptFlow.Dx = *(int16_t *)&bufferIO[13];
			sensorData.SenVal.RawOptFlow.Dy = *(int16_t *)&bufferIO[15];
			sensorData.SenVal.RawOptFlow.Iq = *(int16_t *)&bufferIO[17];
			sensorData.SenVal.RawOptFlow.ResX = bufferIO[19];
			sensorData.SenVal.RawOptFlow.ResY = bufferIO[20];
			sensorData.SenVal.RawOptFlow.Shutter = bufferIO[21];
			sensorData.SenVal.RawOptFlow.FrameMax = bufferIO[22];
			sensorData.SenVal.RawOptFlow.FrameAvg = bufferIO[23];
			sensorData.SenVal.RawOptFlow.FrameMin = bufferIO[24];
			sensorData.SenVal.RawOptFlow.LaserOn = bufferIO[25];
			sensorData.SenVal.RawOptFlow.Dt = *(int16_t *)&bufferIO[27];
			sensorData.SenVal.RawOptFlow.TimeStamp = *(int32_t *)&bufferIO[29];
		break;
		#endif
		#ifdef DEAD_RECKONING_POSE
		case DEAD_RECKONING_POSE:
			sensorData.SenVal.DeadReckoningPose.TimeStamp = *(int32_t *)&bufferIO[13];
			sensorData.SenVal.DeadReckoningPose.LinPosX = (float)(*(int32_t *)&bufferIO[17]) * SCALE_Q17;
			sensorData.SenVal.DeadReckoningPose.LinPosY = (float)(*(int32_t *)&bufferIO[21]) * SCALE_Q17;
			sensorData.SenVal.DeadReckoningPose.LinPosZ = (float)(*(int32_t *)&bufferIO[25]) * SCALE_Q17;

			sensorData.SenVal.DeadReckoningPose.I = (float)(*(int32_t *)&bufferIO[29]) * SCALE_Q30;
			sensorData.SenVal.DeadReckoningPose.J = (float)(*(int32_t *)&bufferIO[33]) * SCALE_Q30;
			sensorData.SenVal.DeadReckoningPose.K = (float)(*(int32_t *)&bufferIO[37]) * SCALE_Q30;
			sensorData.SenVal.DeadReckoningPose.Real = (float)(*(int32_t *)&bufferIO[41]) * SCALE_Q30;

			sensorData.SenVal.DeadReckoningPose.LinVelX = (float)(*(int32_t *)&bufferIO[45]) * SCALE_Q25;
			sensorData.SenVal.DeadReckoningPose.LinVelY = (float)(*(int32_t *)&bufferIO[49]) * SCALE_Q25;
			sensorData.SenVal.DeadReckoningPose.LinVelZ = (float)(*(int32_t *)&bufferIO[53]) * SCALE_Q25;

			sensorData.SenVal.DeadReckoningPose.AngleVelX = (float)(*(int32_t *)&bufferIO[57]) * SCALE_Q25;
			sensorData.SenVal.DeadReckoningPose.AngleVelY = (float)(*(int32_t *)&bufferIO[61]) * SCALE_Q25;
			sensorData.SenVal.DeadReckoningPose.AngleVelZ = (float)(*(int32_t *)&bufferIO[65]) * SCALE_Q25;
		break;
		#endif
		#ifdef WHEEL_ENCODER
		case WHEEL_ENCODER:
			sensorData.SenVal.WheelEncoder.TimeStamp = *(int32_t *)&bufferIO[13];
			sensorData.SenVal.WheelEncoder.WheelIndex = bufferIO[17];
			sensorData.SenVal.WheelEncoder.DataType = bufferIO[18];
			sensorData.SenVal.WheelEncoder.Data = *(int16_t *)&bufferIO[19];
		break;
		#endif
	}
}

// Process a command response
static HAL_StatusTypeDef processCommandResponse(void) {
	// Reset complete
	switch(cmdResponse.command) {
		#ifdef USE_ERROR_REPORT
		case COMMAND_ERRORS: // 0x01 – report all errors in the error queue
			errors = *(BNO_Error_t *)&bufferIO[9];
			return HAL_OK;
		break;
		#endif
		#ifdef USE_COUNTER_REPORT
		case COMMAND_COUNTER: // 0x02 – Counter command
			counts = *(BNO_Counts_t *)&bufferIO[9];
			return HAL_OK;
		break;
		#endif
		case COMMAND_UNSOLICITED_INITIALIZE: // 0x84 – Initialize (unsolicited)
		case COMMAND_INITIALIZE: // 0x04 – Initialize
			if (!bufferIO[9]) {
				resetOccurred = 1;
				return HAL_OK;
			}
		break;
		case COMMAND_SAVE_DCD: // 0x06 – Save DCD
			saveDcdStatus = bufferIO[9];
			return HAL_OK;
		break;	
		case COMMAND_ME_CALIBRATE: // 0x07 – Configure ME Calibration
			calibrationStatus = *(BNO_calibrationStat_t *)&bufferIO[9];
			return HAL_OK;
		break;
		case COMMAND_OSCILLATOR: // 0x0A – Get Oscillator Type Command
			oscillatorType = bufferIO[9];
			return HAL_OK;
		break;	
		case COMMAND_TURNTABLE_CAL: // 0x0C – Turntable Calibration
			calibrationStatus.Status = bufferIO[9];
			return HAL_OK;
		break;	
		case COMMAND_BOOTLOADER:// 0x0D – Bootloader command
			bootLoader = *(BNO_Boot_t *)&bufferIO[10];
			return HAL_OK;
		break;		
	}
	return HAL_ERROR;
}

// Process a response
static HAL_StatusTypeDef processResponse(void) {
	switch(bufferIO[4]) { 
		case REPORT_UNSOLICITED_RESPONSE: // 0x00
			if(bufferIO[2] == CHANNEL_COMMAND) return HAL_OK;
		break;
		case REPORT_UNSOLICITED_RESPONSE1: // 0x01
			if(bufferIO[2] == CHANNEL_EXECUTABLE) return HAL_OK;
		break;	
		case REPORT_COMMAND_RESPONSE: // 0xF1
			cmdResponse = *(BNO_CommandResp_t *)&bufferIO[5];
			return processCommandResponse();
		break;	
		case REPORT_FRS_READ_RESPONSE: // 0xF3
			frsReadResponse = *(BNO_FrsReadResp_t *)&bufferIO[5];
			return HAL_OK;
		break;		
		case REPORT_FRS_WRITE_RESPONSE: // 0xF5
			frsWriteResponse = *(BNO_FrsWriteResp_t *)&bufferIO[5];
			return HAL_OK;
		break;				
		case REPORT_PRODUCT_ID_RESPONSE: // 0xF8
			myID = *(BNO_productID_t *)&bufferIO[5];
			return HAL_OK;
		break;
		case REPORT_BASE_TIMESTAMP_REF: // 0xFB
			if(bufferIO[2] == CHANNEL_REPORTS) {
				getSensorValue();
				return HAL_OK;
			}
			return HAL_ERROR;
		break;
		case REPORT_GET_FEATURE_RESPONSE: // 0xFC
			sensorFeartures = *(BNO_Feature_t *)&bufferIO[5];
			return HAL_OK;
		break;
		case REPORT_SENSOR_FLUSH_RESPONSE: // 0xEF
			if(bufferIO[2] == CHANNEL_REPORTS) {
				// Not using them, so....
				return HAL_OK;
			}
			return HAL_ERROR;
		break;
	}
	return HAL_ERROR;
}

// Wait to receive a packet from BNO
static HAL_StatusTypeDef waitForPacket(void) {
	if(waitInt()) {
		return receivePacket();
	}
	return HAL_ERROR;
}

// Wait for a response from sensor
static HAL_StatusTypeDef waitForCommandResponse(void) {
	uint8_t sendChannel = CHANNEL_CONTROL; 
	uint8_t receiveChannel = CHANNEL_CONTROL;
	uint8_t expectedResponse = REPORT_COMMAND_RESPONSE;
	switch(bufferIO[4]) {
		case REPORT_PRODUCT_ID_REQUEST:
			expectedResponse = REPORT_PRODUCT_ID_RESPONSE;
		break;
		case REPORT_SENSOR_FLUSH_REQUEST:
			receiveChannel = CHANNEL_REPORTS;
			expectedResponse = REPORT_SENSOR_FLUSH_RESPONSE;
		break;
		case REPORT_GET_FEATURE_REQUEST:
			expectedResponse = REPORT_GET_FEATURE_RESPONSE;
		break;
		case REPORT_FRS_WRITE_REQUEST:
			expectedResponse = REPORT_FRS_WRITE_RESPONSE;
		break;
		case REPORT_FRS_READ_REQUEST:
			expectedResponse = REPORT_FRS_READ_RESPONSE;
		break;
	}
	if(sendPacket(sendChannel) == HAL_OK) {
		uint8_t retry = 5;
		while(retry) {
			if(waitForPacket() == HAL_OK) {
				if((bufferIO[2] == receiveChannel)  && (bufferIO[4] == expectedResponse)) {
					return processResponse(); // Found correct packet!
				}
			}
			retry--;
		}
	}
	return HAL_ERROR;
}

// Gets the sensor SW information
static HAL_StatusTypeDef getID(void) {
	resetHeader(REPORT_PRODUCT_ID_REQUEST);
	return waitForCommandResponse();
}

//--------------------------------------------------------------------------------------------------------------------------
// Initialize the sesnor
// During reset or power-on sequence, the bootloader first checks the status of the BOOTN pin.
// If the pin is pulled low during reset or poweron, the BNO08X will enter the bootloader mode.
// If the BOOTN pin is pulled high, then the bootloader starts the application
HAL_StatusTypeDef BNO_Init(void) {
	BNO_BOOT_On; // Off only for DFU mode 0x52 I2C Address
	// Set reset pin low
	BNO_RST_Off;
	HAL_Delay(RESET_DELAY);
	// Start us timer
	start_timer();
	// Enable interrupt BNO_Ready
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	// Delay for RESET_DELAY_US to ensure reset takes effect
	BNO_RST_On;
	HAL_Delay(RESET_DELAY);
	// If we got the initiat packet we make a soft reset
	//if(waitForPacket()) {
		if(processResponse() == HAL_OK) {
			// Wait for intterupt
			if(waitInt()) {
				if(BNO_Reset() == HAL_OK) {
					// Finally, we want to interrogate the device about its model and version.
					BNO_On();
					return getID();
				}
			}
		}
	//}
	return HAL_ERROR;
}

// Check if we have unexpected reset
uint8_t isResetOccurred(void) {
	if(resetOccurred) {
		resetOccurred = 0;
		return 1;
	}
	return resetOccurred;
}
// Get the sensor that has new data
uint8_t BNO_getSensorEventID(void) {
	return sensorData.sensorId;
}

// Soft reset the sensor
HAL_StatusTypeDef BNO_Reset(void) {
	if(sendExecutable(COMMAND_INITIALIZE_RESET) != HAL_OK) { // Write 1 byte to chan EXE
		return HAL_ERROR;
	}
	HAL_Delay(700); // 700 millisecs for reboot
	// 2 packet to be ignored after reset
	if(waitForCommandResponse() == HAL_OK) {
		if(resetOccurred) {
			resetOccurred = 0;
		} else {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

// Turn sensor ON
HAL_StatusTypeDef BNO_On(void) {
	return sendExecutable(COMMAND_INITIALIZE_ON);
}

// When sleep command is issued all sensors that are configured as always on or wake (see 1.3.5.1) will continue to operate all, other sensors will be disabled
HAL_StatusTypeDef BNO_Sleep(void) {
	return sendExecutable(COMMAND_INITIALIZE_SLEEP);
}

// Returns the product id data structure see 6.3.1 and 6.3.2
BNO_productID_t BNO_getProductID(void) {
	return myID;
}

// Length in 32-bit words of the record to be written. If the length is set to 0 then the record is erased
HAL_StatusTypeDef BNO_writeFRS(const uint16_t length, const uint16_t frsType) {
	// FRS Write Request (0xF7)
	resetHeader(REPORT_FRS_WRITE_REQUEST);
	//bufferIO[5] = 0; // Reserved
	*(uint16_t *)&bufferIO[6] = length;
	*(uint16_t *)&bufferIO[8] = frsType;
	// No response for this
	if(waitForCommandResponse() == HAL_OK) { 
		//!!! ToDo analize the response and do something with it
		uint8_t sendMoreData = 0;
		uint8_t completed = 0;
		// Status is bufferIO[5] see 6.3.5 FRS Write Response (0xF5)
		switch(bufferIO[5]) {
			case FRS_WRITE_STATUS_RECEIVED:
			case FRS_WRITE_STATUS_READY:
				sendMoreData = 1;
			break;
			case FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE:
			case FRS_WRITE_STATUS_BUSY:
			case FRS_WRITE_STATUS_FAILED:
			case FRS_WRITE_STATUS_NOT_READY:
			case FRS_WRITE_STATUS_INVALID_LENGTH:
			case FRS_WRITE_STATUS_INVALID_RECORD:
			case FRS_WRITE_STATUS_DEVICE_ERROR:
			case FRS_WRITE_STATUS_READ_ONLY:
				completed = 1;
			break;
			case FRS_WRITE_STATUS_WRITE_COMPLETED:
				// Successful completion
				completed = 1;
			break;
			case FRS_WRITE_STATUS_RECORD_VALID:

			break;
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}

// See 6.3.6 FRS Read Request (0xF4) 6.3.7 FRS Read Response (0xF3)
HAL_StatusTypeDef BNO_readFRS(const uint16_t frsType) {
	resetHeader(REPORT_FRS_READ_REQUEST);
	//bufferIO[5] = 0; // Reserved
	//bufferIO[6] = 0; // Read from start
	*(uint16_t *)&bufferIO[8] = frsType;
	//bufferIO[10] = 0; // Read all avail data
	if(waitForCommandResponse() == HAL_OK) { 
		//!!! ToDo analize the response and do something with it
		// Get status portion of len_status field
		uint8_t status = bufferIO[5] & 0x0F;
		// Get Datalen portion of len_status field
		uint8_t Datalen = ((bufferIO[5] >> 4) & 0x0F);
		
		switch(status) { 
			// Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
			case FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE:
			case FRS_READ_STATUS_BUSY:
			case FRS_READ_STATUS_OFFSET_OUT_OF_RANGE:
			case FRS_READ_STATUS_DEVICE_ERROR:
				// Operation failed
				return HAL_ERROR;
			break;
			case FRS_READ_STATUS_RECORD_EMPTY: // Empty record
				return HAL_OK;
			break;
			// If read is done...complete the operation
			case FRS_READ_STATUS_READ_RECORD_COMPLETED:
			case FRS_READ_STATUS_READ_BLOCK_COMPLETED:
			case FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED:
				return HAL_OK;
			break; 
		}
	}
	return HAL_ERROR;
}

#ifdef USE_ERROR_REPORT
HAL_StatusTypeDef BNO_getErrors(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_ERRORS; // 0x01 – report all errors in the error queue
	//bufferIO[7] = 0; // he severity of errors to report. Errors of this severity and higher will be reported. 0 – highest priority
	if(waitForCommandResponse() == HAL_OK) {
		// Error source. 0 – reserved, 1 – MotionEngine, 2 – MotionHub, 3 – SensorHub, 4 – Chip level executable, 5-254 reserved. 255 – no error to report.
		if(errors.Source == 0xFF) {
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}
#endif

#ifdef USE_COUNTER_REPORT
HAL_StatusTypeDef BNO_getCounter(const uint8_t sensorID) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_COUNTER; // 0x02 – Counter command
	//bufferIO[7] = 0; // 0x00 – Sub-command: get counts
	bufferIO[8] = sensorID;
	if(waitForCommandResponse() == HAL_OK) {
		// Error source. 0 – reserved, 1 – MotionEngine, 2 – MotionHub, 3 – SensorHub, 4 – Chip level executable, 5-254 reserved. 255 – no error to report.
		if(errors.Source == 0xFF) {
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef BNO_clearCounter(const uint8_t sensorID) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_COUNTER; // 0x02 – Counter command
	bufferIO[7] = 1; // 0x01 – Sub-command: clear counts
	bufferIO[8] = sensorID;
	return sendPacket(CHANNEL_CONTROL);
}
#endif

#ifdef USE_REORIENT
// See 6.4.3.1 Tare Now (0x00)
HAL_StatusTypeDef BNO_TareNow(const BNO_TareAxis_t axis, const BNO_TareRV_t vector) {
 	// from SH-2 6.4.3 Tare (0x03)
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_TARE; // 0x03 – Tare command
	//bufferIO[7] = 0; // 0x00 – Subcommand: Perform Tare now
	// Bitmap of axes to tare: Bit 0= X, Bit 1= Y, Bit 2= Z 7 all
	bufferIO[8] = axis;
	//Which rotation vector to use as the basis for Tare adjustment.
	//Rotation Vector to use as basis for tare.
	//0: Rotation Vector
	//1: Gaming Rotation Vector
	//2: Geomagnetic Rotation Vector
	//3: Gyro-Integrated Rotation Vector
	//4: ARVR-Stabilized Rotation Vector
	//5: ARVR-Stabilized Game Rotation Vector
	bufferIO[9] = vector; // 
	return sendPacket(CHANNEL_CONTROL);
}

//This command instructs SH-2 to persist the results of the last tare operation to flash for use at the next system restart.
HAL_StatusTypeDef BNO_TarePerist(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_TARE; // 0x03 – Tare command
	bufferIO[7] = COMMAND_TARE_PERSIST; // 0x01 – Persist Tare
	return sendPacket(CHANNEL_CONTROL);
}

// This command instructs SH-2 to set the current run-time sensor reorientation.
// The rotation vector is a signed, 16-bit 2’s-complement fixed point number with a Q-point of 14.
HAL_StatusTypeDef BNO_TareSetReorientation(const double X, const double Y, const double Z, const double W) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_TARE; // 0x03 – Tare command
	bufferIO[7] = COMMAND_TARE_REORIENT; // 0x02 – Set Reorientation
	*(int16_t *)&bufferIO[8] = (int16_t)(X * SCALE_TO_Q14);
	*(int16_t *)&bufferIO[10] = (int16_t)(Y * SCALE_TO_Q14);
	*(int16_t *)&bufferIO[12] = (int16_t)(Z * SCALE_TO_Q14);
	*(int16_t *)&bufferIO[14] = (int16_t)(W * SCALE_TO_Q14);
	return sendPacket(CHANNEL_CONTROL);
}

// Clears the previously applied tare operation.
HAL_StatusTypeDef BNO_ClearTare(void){
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_TARE; // 0x03 – Tare command
	bufferIO[7] = COMMAND_TARE_REORIENT; // 0x02 – Set Reorientation
	return sendPacket(CHANNEL_CONTROL);
}

#endif


// The sensor hub responds to the Initialize command with an Initialize Response. In the case
// where the sensor hub reinitializes itself, this response is unsolicited. An unsolicited response is
// also generated after startup
HAL_StatusTypeDef BNO_Initialize(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_INITIALIZE; // 0x04 – Initialize command
	bufferIO[7] = COMMAND_INITIALIZE_RESET; //1 Reinitialize the entire sensor hub.
	return waitForCommandResponse();
}

// Save Dynamic Calibration Data (DCD) to flash
HAL_StatusTypeDef BNO_saveCalibration(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_SAVE_DCD; // 0x06 – Save DCD Command
	saveDcdStatus = 1; // Set it as non 0
	if(waitForCommandResponse() == HAL_OK) {
		if(!saveDcdStatus) return HAL_OK;
	}
	return HAL_ERROR;
}

// This command is sent by the host to configure the ME calibration of the accelerometer, gyro and
// magnetometer giving the host the ability to control when calibration is performed.
HAL_StatusTypeDef BNO_condigureCalibration(const uint8_t sensors) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_ME_CALIBRATE; // 0x07 – ME Calibration Command
	//bufferIO[10] = 0; // 0x00 – Subcommand: Configure ME Calibration
	// Make the internal calStatus variable non-zero (fail)
	calibrationStatus.Status = 1;
	switch(sensors) {
		case CALIBRATE_ACCEL:
			bufferIO[7] = 1; // Accel Cal Enable (1 – enabled, 0 – disabled)
		break;
		case CALIBRATE_GYRO:
			bufferIO[8] = 1; // Gyro Cal Enable (1 – enabled, 0 – disabled)
		break;
		case CALIBRATE_MAG:
			bufferIO[9] = 1; // Mag Cal Enable (1 – enabled, 0 – disabled)
		break;
		case CALIBRATE_PLANAR_ACCEL:
			bufferIO[11] = 1; // Planar Accel Cal Enable (1 – enabled, 0 – disabled)
		break;
		case CALIBRATE_ON_TABLE:
			bufferIO[12] = 1; // On Table Cal Enable (1 – enabled, 0 – disabled)
		break;
		case CALIBRATE_ACCEL_GYRO_MAG:
			bufferIO[7] = bufferIO[8] = bufferIO[9] = 1;
		break;
		case CALIBRATE_ALL:
			bufferIO[7] = bufferIO[8] = bufferIO[9] = bufferIO[11] = bufferIO[12] = 1;
		break;
	}
	bufferIO[13] = ((sensors & 0x60) >> 5);
	if(waitForCommandResponse() == HAL_OK) {
		if(!calibrationStatus.Status) {
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

// This command is sent by the host to request the enable/disable state of the accelerometer, gyro and magnetometer calibration routines
HAL_StatusTypeDef BNO_enableCalibration(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_ME_CALIBRATE;
	bufferIO[10] = COMMAND_ME_CALIBRATE_GET;
	return waitForCommandResponse();
}

// The Configure Periodic DCD Save command configures the automatic saving of DCD. There is
// no response to this command. This command does not inhibit the Save DCD command.
// 0 Enable, 1 Disable
HAL_StatusTypeDef BNO_configurePeriodicDcdSave(const uint8_t enableStatus) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_DCD_PERIOD_SAVE; // 0x09 Configure Periodic DCD Save
	bufferIO[7] = enableStatus;
	return sendPacket(CHANNEL_CONTROL);
}


// The Get Oscillator Type command is used to get information about the oscillator type used in the clock system of the SH-2
BNO_Oscillator_t BNO_getOscillatorType(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_OSCILLATOR;
	if(waitForCommandResponse() == HAL_OK) {
		if(oscillatorType > ExternalClock) return OscillatorError;
	}
	return oscillatorType;
}

// This command performs an atomic clearDCD (from RAM) and system reset
HAL_StatusTypeDef BNO_clearDcdReset(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_CLEAR_DCD_RESET;
	if(waitForCommandResponse() == HAL_OK) {
		if(resetOccurred) {
			resetOccurred = 0;
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

// ToDo Check functionality Not getting a reponce to start
// See 6.4.10 Simple Calibration Commands (0x0C)
HAL_StatusTypeDef BNO_simpleCalibration(const uint32_t usInterval, const uint16_t calibrationTimeMs) {
	if(BNO_condigureCalibration(CALIBRATE_ON_TABLE) == HAL_OK) {
		resetHeader(REPORT_COMMAND_REQUEST);
		bufferIO[5] = cmdSeqNo++;
		bufferIO[6] = COMMAND_TURNTABLE_CAL; // 0x0C – Turntable Calibration
		//bufferIO[7] = 0x00; // 0x00 – Start Calibration
		*(uint32_t *)&bufferIO[8] = usInterval;
		if(waitForCommandResponse() == HAL_OK) {//CHANNEL_CONTROL, REPORT_COMMAND_RESPONSE
			if(!calibrationStatus.Status) {
				uint32_t endCalibrationTime = HAL_GetTick() + calibrationTimeMs;
				while(HAL_GetTick() < endCalibrationTime) {} // Just wait
				// Stop the calibration and get response	
				resetHeader(REPORT_COMMAND_REQUEST);
				bufferIO[5] = cmdSeqNo++;
				bufferIO[6] = COMMAND_TURNTABLE_CAL; // 0x0C – Turntable Calibration
				bufferIO[7] = 0x01; // 0x01 – Finish Calibration
				if(waitForCommandResponse() == HAL_OK) {	
					if(!calibrationStatus.Status) {
						return HAL_OK;
					}
				}
			}
		}
	}
	return HAL_ERROR;
}

// ToDo Check functionality Not getting a reponce to start
// The bootloader operating mode request is used to request various operating modes of the FSP200 bootloader
HAL_StatusTypeDef BNO_setBootMode(const BNO_BootMode_t mode) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_BOOTLOADER; // 0x0D – Bootloader command
	bufferIO[7] = COMMAND_BOOTLOADER_MODE_REQ; // 0x00 – Sub-command: Bootloader Operating Mode Request
	bufferIO[8] = mode; // Bootloader Operating Mode ID
	return sendPacket(CHANNEL_CONTROL);
}

// ToDo Check functionality Not getting a reponce to start
// Request product ID information about the FSP200 bootloader
BNO_BootMode_t BNO_getBootMode(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_BOOTLOADER; // 0x0D – Bootloader command
	bufferIO[7] = COMMAND_BOOTLOADER_STATUS_REQ; // 0x01 – Sub-command: Bootloader Status Request
	if(waitForCommandResponse() == HAL_OK) {
		return bootLoader.OperationMode;
	}
	return BootInvalid;
}

// The interactive calibration feature requires that the sensor hub be told of the device’s intended motion.
HAL_StatusTypeDef BNO_interactiveCalibration(const BNO_MotionIntent_t intent) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_INTERACTIVE_CAL_REQ; // 0x0E – Interactive Calibration command
	bufferIO[7] = intent; // Motion intent
	return sendPacket(CHANNEL_CONTROL);
}

#ifdef WHEEL_ENCODER
// Provide a single sample of wheel encoder data. No response is sent for this command.
HAL_StatusTypeDef BNO_WheelRequest(const uint8_t wheelIndex, const uint32_t timeStampUs, const int16_t wheelData, const uint8_t dataType) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_WHEEL_REQ; // 0x0F – Wheel Request
	bufferIO[7] = wheelIndex; // Wheel Index (0 = Left Wheel, 1 = Right Wheel)
	*(uint32_t *)&bufferIO[8] = timeStampUs; // The timestamp is a 32-bit unsigned timestamp in microseconds.
	*(int16_t *)&bufferIO[12] = wheelData; // The timestamp is a 32-bit unsigned timestamp in microseconds.
	bufferIO[14] = dataType; // Data Type (0 = Position, 1 = Velocity)
	return sendPacket(CHANNEL_CONTROL);
}
#endif

// This is sent from the host to the hub to trigger a flush of outstanding data from a given sensor
HAL_StatusTypeDef  BNO_forceFlush(const uint8_t sensorID) {
	resetHeader(REPORT_SENSOR_FLUSH_REQUEST);
	bufferIO[5] = sensorID;
	return waitForCommandResponse();
}

// This command is sent by the host to request the enable/disable state of the accelerometer, gyro
// and magnetometer calibration routines.
HAL_StatusTypeDef  BNO_getCalibrationStatus(void) {
	resetHeader(REPORT_COMMAND_REQUEST);
	bufferIO[5] = cmdSeqNo++;
	bufferIO[6] = COMMAND_ME_CALIBRATE;
	bufferIO[10] = 0x01; // 0x01 – Subcommand: Get ME Calibration
	return waitForCommandResponse();
}

// Check if calibration is complete
HAL_StatusTypeDef BNO_isCalibrationComplete(void) {
	if(!calibrationStatus.Status) {
		return HAL_OK;
	}
	return HAL_ERROR;
}

#ifdef USE_FOR_TELESCOPE

HAL_StatusTypeDef setTelescopeOrientation(void) {
	if(BNO_TareNow(TARE_ALL, RotationVector) == HAL_OK){
		// -X North, Y East Z down
		if(BNO_TareSetReorientation(0.0, 0.0, 0.0, Q_FLIP) == HAL_OK){
			// Save new orientation and reset
			if(BNO_TarePerist() == HAL_OK){
				return BNO_Reset();
			}
		}
	}
	return HAL_ERROR;
}

// Start the calibration for 20s or until accuracy is 3
HAL_StatusTypeDef BNO_calibrateHighAccuracyAndReset(void) {
	const uint16_t calibrationTime = 26000;
	if(BNO_condigureCalibration(CALIBRATE_ACCEL_GYRO_MAG) == HAL_OK) {
		if(BNO_setFeature(MAGNETIC_FIELD_CALIBRATED, 100000, 0) == HAL_OK) {
			if(BNO_setFeature(GAME_ROTATION_VECTOR, 100000, 0) == HAL_OK) {
				uint32_t startTime = HAL_GetTick() + calibrationTime;
				uint8_t magA,grvA = 0;
				while(HAL_GetTick() < startTime) {
					if((BNO_dataAvailable() == HAL_OK) && sensorData.sensorId){
						if(sensorData.sensorId == MAGNETIC_FIELD_CALIBRATED){
							magA = sensorData.status;
							//printf("MagA=%d\r\n", magA);
						}
						if(sensorData.sensorId == GAME_ROTATION_VECTOR){
							grvA = sensorData.status;
							//printf("GrvA=%d\r\n", grvA);
						}
						sensorData.sensorId = 0;
					}
					// If we have maximum accuracy we stop
					if((magA == 3) && (grvA == 3))
						break;
				}
				if(BNO_saveCalibration() == HAL_OK){
					return BNO_Reset();
				}
			}
		}
	}
	return HAL_ERROR;
}
#endif
// Check if we have new data
HAL_StatusTypeDef BNO_dataAvailable(void) {
	if(waitForPacket() == HAL_OK) {
		return processResponse();
	}
	return HAL_ERROR;
}

// Enable features an set report time in mili seconds
BNO_Feature_t BNO_getFeature(const uint8_t sensorID) {
	resetHeader(REPORT_GET_FEATURE_REQUEST);
	BNO_Feature_t ret = {0};
	bufferIO[5] = sensorID; // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	if(waitForCommandResponse() == HAL_OK) {
		ret = sensorFeartures;
	}
	return ret;
}

// Enable features an set report time in mili seconds
HAL_StatusTypeDef BNO_setFeature(const uint8_t sensorID, const uint32_t microsBetweenReports, const uint32_t specificConfig) {
	resetHeader(REPORT_SET_FEATURE_COMMAND); // Set feature command. Reference page 55
//	sensorFeartures.sensorID = sensorID;
//	sensorFeartures.flags = 0;
//	sensorFeartures.changeSensitivity = 0;
//	sensorFeartures.reportInterval_uS = microsBetweenReports;
//	sensorFeartures.batchInterval_uS = 0;
//	sensorFeartures.sensorSpecific = specificConfig;
//	*(BNO_Feature_t *)&bufferIO[5] = sensorFeartures;
	bufferIO[5] = sensorID; // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	//bufferIO[6] = 0; // Feature flags
	//*(uint16_t *)&bufferIO[7] = 0; // Change sensitivity [absolute | relative] 
	*(uint32_t *)&bufferIO[9] = microsBetweenReports; // Report interval (LSB) in microseconds
	//*(uint32_t *)&bufferIO[13] = 0; // Batch Interval
	*(uint32_t *)&bufferIO[17] = specificConfig; // Sensor-specific config
	//Transmit packet on channel 2, 17 bytes
	return sendPacket(CHANNEL_CONTROL);
}

//
BNO_RollPitchYaw_t BNO_getRollPitchYaw(void) {
	return rpy;
}

#ifdef RAW_ACCELEROMETER
BNO_RawAccelerometer_t getRawAccelerometer(void) {
	return sensorData.SenVal.RawAccelerometer;
}
#endif
#ifdef ACCELEROMETER
BNO_Accelerometer_t getaccelerometer(void) {
	return sensorData.SenVal.Accelerometer;
}
#endif
#ifdef LINEAR_ACCELERATION
BNO_Accelerometer_t getLinearAcceleration(void) {
	return sensorData.SenVal.LinearAcceleration;
}
#endif
#ifdef GRAVITY
BNO_Accelerometer_t getGravity(void) {
	return sensorData.SenVal.Gravity;
}
#endif
#ifdef RAW_GYROSCOPE
BNO_RawGyroscope_t getRawGyroscope(void) {
	return sensorData.SenVal.RawGyroscope;
}
#endif
#ifdef GYROSCOPE_CALIBRATED
BNO_Gyroscope_t getGyroscope(void) {
	return sensorData.SenVal.Gyroscope;
}
#endif
#ifdef GYROSCOPE_UNCALIBRATED
BNO_GyroscopeUncalibrated_t getGyroscopeUncal(void) {
	return sensorData.SenVal.GyroscopeUncal;
}
#endif
#ifdef RAW_MAGNETOMETER
BNO_RawMagnetometer_t getRawMagnetometer(void) {
	return sensorData.SenVal.RawMagnetometer;
}
#endif
#ifdef MAGNETIC_FIELD_CALIBRATED
BNO_MagneticField_t getMagneticField(void) {
	return sensorData.SenVal.MagneticField;
}
#endif
#ifdef MAGNETIC_FIELD_UNCALIBRATED
BNO_MagneticFieldUncalibrated_t getMagneticFieldUncal(void) {
	return sensorData.SenVal.MagneticFieldUncal;
}
#endif
#ifdef ROTATION_VECTOR
BNO_RotationVectorWAcc_t getRotationVector(void) {
	return sensorData.SenVal.RotationVector;
}
#endif
#ifdef GAME_ROTATION_VECTOR
BNO_RotationVector_t getGameRotationVector(void) {
	return sensorData.SenVal.GameRotationVector;
}
#endif
#ifdef GEOMAGNETIC_ROTATION_VECTOR
BNO_RotationVectorWAcc_t getGeoMagRotationVector(void) {
	return sensorData.SenVal.GeoMagRotationVector;
}
#endif
#ifdef PRESSURE
float getPressure(void) {
	return sensorData.SenVal.Pressure; // Atmospheric Pressure.  [hectopascals]
}
#endif
#ifdef AMBIENT_LIGHT
float getAmbientLight(void) {
	return sensorData.SenVal.AmbientLight; // Ambient Light.  [lux]
}
#endif
#ifdef HUMIDITY
float getHumidity(void) {
	return sensorData.SenVal.Humidity; // Relative Humidity.  [percent]
}
#endif
#ifdef PROXIMITY
float getProximity(void) {
	return sensorData.SenVal.Proximity; // Proximity.  [cm]
}
#endif
#ifdef TEMPERATURE
float getTemperature(void) {
	return sensorData.SenVal.Temperature; // Temperature.  [C]
}
#endif
#ifdef RESERVED
float getReserved(void) {
	return sensorData.SenVal.Reserved;  // Reserved
}
#endif
#ifdef TAP_DETECTOR
BNO_Tap_t getTapDetectorFlag(void) {
	return sensorData.SenVal.TapDetectorFlag;
}
#endif
#ifdef STEP_DETECTOR
uint32_t getStepDetectorLatency(void) {
	return sensorData.SenVal.StepDetectorLatency; // Step detect latency [uS]
}
#endif
#ifdef STEP_COUNTER
BNO_StepCounter_t getStepCounter(void) {
	return sensorData.SenVal.StepCounter;
}
#endif
#ifdef SIGNIFICANT_MOTION
uint16_t getSignificantMotion(void) {
	return sensorData.SenVal.SignificantMotion;
}
#endif
#ifdef STABILITY_CLASSIFIER
BNO_Stability_t getStabilityClassifier(void) {
	return sensorData.SenVal.StabilityClassifier;
}
#endif
#ifdef SHAKE_DETECTOR
BNO_Shake_t getShakeDetector(void) {
	return sensorData.SenVal.ShakeDetector;
}
#endif
#ifdef FLIP_DETECTOR
uint16_t getFlipDetector(void) {
	return sensorData.SenVal.FlipDetector;
}
#endif
#ifdef PICKUP_DETECTOR
BNO_Pickup_t getPickupDetector(void) {
	return sensorData.SenVal.PickupDetector;
}
#endif
#ifdef STABILITY_DETECTOR
BNO_StabilityDetector_t getStabilityDetector(void) {
	return sensorData.SenVal.StabilityDetector;
}
#endif
#ifdef PERSONAL_ACTIVITY_CLASSIFIER
BNO_PersonalActivityClassifier_t getPersonalActivityClassifier(void) {
	return sensorData.SenVal.PersonalActivityClassifier;
}
#endif
#ifdef SLEEP_DETECTOR
uint8_t getSleepDetector(void) {
	return sensorData.SenVal.SleepDetector;
}
#endif
#ifdef TILT_DETECTOR
uint16_t getTiltDetector(void) {
	return sensorData.SenVal.TiltDetector;
}
#endif
#ifdef POCKET_DETECTOR
uint16_t getPocketDetector(void) {
	return sensorData.SenVal.PocketDetector;
}
#endif
#ifdef CIRCLE_DETECTOR
uint16_t getCircleDetector(void) {
	return sensorData.SenVal.CircleDetector;
}
#endif
#ifdef HEART_RATE_MONITOR
uint16_t getHeartRateMonitor(void) {
	return sensorData.SenVal.HeartRateMonitor; // Heart rate in beats per minute.
}
#endif
#ifdef ARVR_STABILIZED_RV
BNO_RotationVectorWAcc_t BNO_getArvrStabilizedRV(void) {
	return sensorData.SenVal.ArVrStabilizedRV;
}
#endif
#ifdef ARVR_STABILIZED_GRV
BNO_RotationVector_t BNO_getArVrStabilizedGRV(void) {
	return sensorData.SenVal.ArVrStabilizedGRV;
}
#endif
#ifdef GYRO_INTEGRATED_RV
BNO_GyroIntegratedRV_t BNO_getGyroIntegratedRV(void) {
	return sensorData.SenVal.GyroIntegratedRV;
}
#endif
#ifdef IZRO_MOTION_REQUEST
BNO_IZroRequest_t getIzroRequest(void) {
	return sensorData.SenVal.IzroRequest;
}
#endif
#ifdef RAW_OPTICAL_FLOW
BNO_RawOptFlow_t getRawOptFlow(void) {
	return sensorData.SenVal.RawOptFlow;
}
#endif
#ifdef DEAD_RECKONING_POSE
BNO_DeadReckoningPose_t getDeadReckoningPose(void) {
	return sensorData.SenVal.DeadReckoningPose;
}
#endif
#ifdef WHEEL_ENCODER
BNO_WheelEncoder_t getWheelEncoder(void) {
	return sensorData.SenVal.WheelEncoder;
}
#endif