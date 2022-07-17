#ifndef MPU925_H_
#define MPU925_H_

#include "main.h"
#include "MPU9250_Config.h"

typedef enum GyroRange_ {
  GYRO_RANGE_250DPS = 0,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
  ACCEL_RANGE_2G = 0,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
  DLPF_BANDWIDTH_184HZ = 0,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
  LP_ACCEL_ODR_0_24HZ = 0,
  LP_ACCEL_ODR_0_49HZ,
  LP_ACCEL_ODR_0_98HZ,
  LP_ACCEL_ODR_1_95HZ,
  LP_ACCEL_ODR_3_91HZ,
  LP_ACCEL_ODR_7_81HZ,
  LP_ACCEL_ODR_15_63HZ,
  LP_ACCEL_ODR_31_25HZ,
  LP_ACCEL_ODR_62_50HZ,
  LP_ACCEL_ODR_125HZ,
  LP_ACCEL_ODR_250HZ,
  LP_ACCEL_ODR_500HZ
} SampleRateDivider;

uint8_t MPU9250_Init(SPI_HandleTypeDef *hspi);
/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(SPI_HandleTypeDef *hspi, int16_t* AccData, int16_t* MagData, int16_t* GyroData);

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SPI_HandleTypeDef *hspi, SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(SPI_HandleTypeDef *hspi, DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(SPI_HandleTypeDef *hspi, GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(SPI_HandleTypeDef *hspi, AccelRange range);

#endif /* MPU925_H_ */
