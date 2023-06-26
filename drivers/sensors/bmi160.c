/****************************************************************************
 * drivers/sensors/bmi160.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/bmi160.h>

#include <math.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <sys/param.h>

#if defined(CONFIG_SENSORS_BMI160)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID               0xd1

/* I2C  Address
 *
 * NOTE: If SDO pin is pulled to VDDIO, use 0x69
 */

#ifdef CONFIG_BMI160_I2C_ADDR_68
#define BMI160_I2C_ADDR     0x68
#else
#define BMI160_I2C_ADDR     0x69
#endif

#define BMI160_I2C_FREQ     400000
#define BMI160_SPI_FREQ     BMI160_SPI_MAXFREQUENCY

#define BMI160_DEFAULT_INTERVAL 10000  /* Default conversion interval. */

#define BMI160_CHIP_ID          (0x00) /* Chip ID */
#define BMI160_ERROR            (0x02) /* Error register */
#define BMI160_PMU_STAT         (0x03) /* Current power mode */
#define BMI160_DATA_0           (0x04) /* MAG X  7:0 (LSB) */
#define BMI160_DATA_1           (0x05) /* MAG X 15:8 (MSB) */
#define BMI160_DATA_2           (0x06) /* MAG Y  7:0 (LSB) */
#define BMI160_DATA_3           (0x07) /* MAG Y 15:8 (MSB) */
#define BMI160_DATA_4           (0x08) /* MAG Z  7:0 (LSB) */
#define BMI160_DATA_5           (0x09) /* MAG Z 15:8 (MSB) */
#define BMI160_DATA_6           (0x0A) /* RHALL  7:0 (LSB) */
#define BMI160_DATA_7           (0x0B) /* RHALL 15:8 (MSB) */
#define BMI160_DATA_8           (0x0C) /* GYR X  7:0 (LSB) */
#define BMI160_DATA_9           (0x0D) /* GYR X 15:8 (MSB) */
#define BMI160_DATA_10          (0x0E) /* GYR Y  7:0 (LSB) */
#define BMI160_DATA_11          (0x0F) /* GYR Y 15:8 (MSB) */
#define BMI160_DATA_12          (0x10) /* GYR Z  7:0 (LSB) */
#define BMI160_DATA_13          (0x11) /* GYR Z 15:8 (MSB) */
#define BMI160_DATA_14          (0x12) /* ACC X  7:0 (LSB) */
#define BMI160_DATA_15          (0x13) /* ACC X 15:8 (MSB) */
#define BMI160_DATA_16          (0x14) /* ACC Y  7:0 (LSB) */
#define BMI160_DATA_17          (0x15) /* ACC Y 15:8 (MSB) */
#define BMI160_DATA_18          (0x16) /* ACC Z  7:0 (LSB) */
#define BMI160_DATA_19          (0x17) /* ACC Z 15:8 (MSB) */
#define BMI160_SENSORTIME_0     (0x18) /* Sensor time 0 */
#define BMI160_SENSORTIME_1     (0x19) /* Sensor time 1 */
#define BMI160_SENSORTIME_2     (0x1A) /* Sensor time 2 */
#define BMI160_STAT             (0x1B) /* Status register */
#define BMI160_INTR_STAT_0      (0x1C) /* Interrupt status */
#define BMI160_INTR_STAT_1      (0x1D)
#define BMI160_INTR_STAT_2      (0x1E)
#define BMI160_INTR_STAT_3      (0x1F)
#define BMI160_TEMPERATURE_0    (0x20) /* Temperature */
#define BMI160_TEMPERATURE_1    (0x21)
#define BMI160_FIFO_LENGTH_0    (0x22) /* FIFO length */
#define BMI160_FIFO_LENGTH_1    (0x23)
#define BMI160_FIFO_DATA        (0x24)
#define BMI160_ACCEL_CONFIG     (0x40) /* ACCEL config for ODR, bandwidth and undersampling */
#define BMI160_ACCEL_RANGE      (0x41) /* ACCEL range */
#define BMI160_GYRO_CONFIG      (0x42) /* GYRO config for ODR and bandwidth */
#define BMI160_GYRO_RANGE       (0x43) /* GYRO range */
#define BMI160_MAG_CONFIG       (0x44) /* MAG config for ODR */
#define BMI160_FIFO_DOWN        (0x45) /* GYRO and ACCEL downsampling rates for FIFO */
#define BMI160_FIFO_CONFIG_0    (0x46) /* FIFO config */
#define BMI160_FIFO_CONFIG_1    (0x47)
#define BMI160_MAG_IF_0         (0x4B) /* MAG interface */
#define BMI160_MAG_IF_1         (0x4C)
#define BMI160_MAG_IF_2         (0x4D)
#define BMI160_MAG_IF_3         (0x4E)
#define BMI160_MAG_IF_4         (0x4F)
#define BMI160_INTR_ENABLE_0    (0x50) /* Interrupt enable */
#define BMI160_INTR_ENABLE_1    (0x51)
#define BMI160_INTR_ENABLE_2    (0x52)
#define BMI160_INTR_OUT_CTRL    (0x53)
#define BMI160_INTR_LATCH       (0x54) /* Latch duration */
#define BMI160_INTR_MAP_0       (0x55) /* Map interrupt */
#define BMI160_INTR_MAP_1       (0x56)
#define BMI160_INTR_MAP_2       (0x57)
#define BMI160_INTR_DATA_0      (0x58) /* Data source */
#define BMI160_INTR_DATA_1      (0x59)
#define BMI160_INTR_LOWHIGH_0   (0x5A) /* Threshold interrupt */
#define BMI160_INTR_LOWHIGH_1   (0x5B)
#define BMI160_INTR_LOWHIGH_2   (0x5C)
#define BMI160_INTR_LOWHIGH_3   (0x5D)
#define BMI160_INTR_LOWHIGH_4   (0x5E)
#define BMI160_INTR_MOTION_0    (0x5F)
#define BMI160_INTR_MOTION_1    (0x60)
#define BMI160_INTR_MOTION_2    (0x61)
#define BMI160_INTR_MOTION_3    (0x62)
#define BMI160_INTR_TAP_0       (0x63)
#define BMI160_INTR_TAP_1       (0x64)
#define BMI160_INTR_ORIENT_0    (0x65)
#define BMI160_INTR_ORIENT_1    (0x66)
#define BMI160_INTR_FLAT_0      (0x67)
#define BMI160_INTR_FLAT_1      (0x68)
#define BMI160_FOC_CONFIG       (0x69) /* Fast offset configuration */
#define BMI160_CONFIG           (0x6A) /* Miscellaneous configuration */
#define BMI160_IF_CONFIG        (0x6B) /* Serial interface configuration */
#define BMI160_PMU_TRIGGER      (0x6C) /* GYRO power mode trigger */
#define BMI160_SELF_TEST        (0x6D) /* Self test */
#define BMI160_NV_CONFIG        (0x70) /* SPI/I2C selection */
#define BMI160_OFFSET_0         (0x71) /* ACCEL and GYRO offset */
#define BMI160_OFFSET_1         (0x72)
#define BMI160_OFFSET_2         (0x73)
#define BMI160_OFFSET_3         (0x74)
#define BMI160_OFFSET_4         (0x75)
#define BMI160_OFFSET_5         (0x76)
#define BMI160_OFFSET_6         (0x77)
#define BMI160_STEP_COUNT_0     (0x78) /* Step counter interrupt */
#define BMI160_STEP_COUNT_1     (0x79)
#define BMI160_STEP_CONFIG_0    (0x7A) /* Step counter configuration */
#define BMI160_STEP_CONFIG_1    (0x7B)
#define BMI160_CMD              (0x7e) /* Command register */

/* Register 0x40 - ACCEL_CONFIG accel bandwidth */

#define ACCEL_OSR4_AVG1   (0 << 4)
#define ACCEL_OSR2_AVG2   (1 << 4)
#define ACCEL_NORMAL_AVG4 (2 << 4)
#define ACCEL_CIC_AVG8    (3 << 4)
#define ACCEL_RES_AVG2    (4 << 4)
#define ACCEL_RES_AVG4    (5 << 4)
#define ACCEL_RES_AVG8    (6 << 4)
#define ACCEL_RES_AVG16   (7 << 4)
#define ACCEL_RES_AVG32   (8 << 4)
#define ACCEL_RES_AVG64   (9 << 4)
#define ACCEL_RES_AVG128  (10 << 4)

#define ACCEL_ODR_0_78HZ      (0x01)
#define ACCEL_ODR_1_56HZ      (0x02)
#define ACCEL_ODR_3_12HZ      (0x03)
#define ACCEL_ODR_6_25HZ      (0x04)
#define ACCEL_ODR_12_5HZ      (0x05)
#define ACCEL_ODR_25HZ        (0x06)
#define ACCEL_ODR_50HZ        (0x07)
#define ACCEL_ODR_100HZ       (0x08)
#define ACCEL_ODR_200HZ       (0x09)
#define ACCEL_ODR_400HZ       (0x0A)
#define ACCEL_ODR_800HZ       (0x0B)
#define ACCEL_ODR_1600HZ      (0x0C)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE   (0x00 << 4)
#define GYRO_OSR2_MODE   (0x01 << 4)
#define GYRO_NORMAL_MODE (0x02 << 4)
#define GYRO_CIC_MODE    (0x03 << 4)

#define GYRO_ODR_25HZ         (0x06)
#define GYRO_ODR_50HZ         (0x07)
#define GYRO_ODR_100HZ        (0x08)
#define GYRO_ODR_200HZ        (0x09)
#define GYRO_ODR_400HZ        (0x0A)
#define GYRO_ODR_800HZ        (0x0B)
#define GYRO_ODR_1600HZ       (0x0C)
#define GYRO_ODR_3200HZ       (0x0D)

/* Register 0x7b STEP_CONFIG_1 */

#define STEP_CNT_EN           (1 << 3)

/* Register 0x7e - CMD */

#define ACCEL_PM_SUSPEND      (0X10)
#define ACCEL_PM_NORMAL       (0x11)
#define ACCEL_PM_LOWPOWER     (0X12)
#define GYRO_PM_SUSPEND       (0x14)
#define GYRO_PM_NORMAL        (0x15)
#define GYRO_PM_FASTSTARTUP   (0x17)
#define MAG_PM_SUSPEND        (0x18)
#define MAG_PM_NORMAL         (0x19)
#define MAG_PM_LOWPOWER       (0x1A)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Sensor ODR */

struct bmi160_odr_s
{
  uint8_t regval;    /* the data of register */
  unsigned long odr; /* the unit is us */
};

struct accel_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/* Device struct */

struct bmi160_dev_s
{
  /* sensor_lowerhalf_s must be in the first line. */

  struct sensor_lowerhalf_s lower;      /* Lower half sensor driver. */

  struct work_s work;                   /* Interrupt handler worker. */
  unsigned long interval;               /* Sensor acquisition interval. */

#if defined(CONFIG_SENSORS_BMI160_I2C)  
  FAR struct i2c_master_s *i2c;         /* I2C interface */
#else /* CONFIG_SENSORS_BMI160_SPI */
  FAR struct spi_dev_s *spi;            /* SPI interface */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv,
                              uint8_t regaddr);
static void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t *regval, int len);

/* Sensor handle functions */

static void bmi160_accel_enable(FAR struct bmi160_dev_s *priv, bool enable);
static void bmi160_gyro_enable(FAR struct bmi160_dev_s *priv, bool enable);

/* Sensor ops functions */

static int bmi160_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us);
static int bmi160_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR unsigned long *period_us);
static int bmi160_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int bmi160_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable);

/* Sensor poll functions */

static void bmi160_accel_worker(FAR void *arg);
static void bmi160_gyro_worker(FAR void *arg);

static int bmi160_checkid(FAR struct bmi160_dev_s *priv);
static int bmi160_findodr(unsigned long time,
                          FAR const struct bmi160_odr_s *odr_s,
                          int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmi160_accel_ops =
{
  .activate     = bmi160_accel_activate,      /* Enable/disable sensor. */
  .set_interval = bmi160_set_accel_interval,  /* Set output data period. */
};

static const struct sensor_ops_s g_bmi160_gyro_ops =
{
  .activate     = bmi160_gyro_activate,      /* Enable/disable sensor. */
  .set_interval = bmi160_set_gyro_interval,  /* Set output data period. */
};

static const struct bmi160_odr_s g_bmi160_gyro_odr[] =
{
  { GYRO_ODR_25HZ,  40000 }, /* Sampling interval is 40ms. */
  { GYRO_ODR_50HZ,  20000 }, /* Sampling interval is 20ms. */
  { GYRO_ODR_100HZ, 10000 }, /* Sampling interval is 10ms. */
  { GYRO_ODR_200HZ,  5000 }, /* Sampling interval is 5ms. */
  { GYRO_ODR_400HZ,  2500 }, /* Sampling interval is 2.5ms. */
  { GYRO_ODR_800HZ,  1250 }, /* Sampling interval is 1.25ms. */
  { GYRO_ODR_1600HZ,  625 }, /* Sampling interval is 0.625ms. */
  { GYRO_ODR_3200HZ,  312 }, /* Sampling interval is 0.3125ms. */
};

static const struct bmi160_odr_s g_bmi160_accel_odr[] =
{
  { BMI160_ACCEL_ODR_0_78HZ, 1282000 }, /* Sampling interval is 1282.0ms. */
  { BMI160_ACCEL_ODR_1_56HZ,  641000 }, /* Sampling interval is 641.0ms. */
  { BMI160_ACCEL_ODR_3_12HZ,  320500 }, /* Sampling interval is 320.5ms. */
  { BMI160_ACCEL_ODR_6_25HZ,  160000 }, /* Sampling interval is 160.0ms. */
  { BMI160_ACCEL_ODR_12_5HZ,   80000 }, /* Sampling interval is 80.0ms. */
  { BMI160_ACCEL_ODR_25HZ,     40000 }, /* Sampling interval is 40.0ms. */
  { BMI160_ACCEL_ODR_50HZ,     20000 }, /* Sampling interval is 20.0ms. */
  { BMI160_ACCEL_ODR_100HZ,    10000 }, /* Sampling interval is 10.0ms. */
  { BMI160_ACCEL_ODR_200HZ,     5000 }, /* Sampling interval is 5.0ms. */
  { BMI160_ACCEL_ODR_400HZ,     2500 }, /* Sampling interval is 2.5ms. */
  { BMI160_ACCEL_ODR_800HZ,     1250 }, /* Sampling interval is 1.25ms. */
  { BMI160_ACCEL_ODR_1600HZ,     625 }, /* Sampling interval is 0.625ms. */
};

/****************************************************************************
 * Name: bmi160_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SPI
static inline void bmi160_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMI160 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, BMI160_SPI_FREQ);
}
#endif

/****************************************************************************
 * Name: bmi160_getreg8
 *
 * Description:
 *   Read from an 8-bit BMI160 register
 *
 ****************************************************************************/

static uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = BMI160_I2C_FREQ;
  msg[0].addr      = BMI160_I2C_ADDR;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = BMI160_I2C_FREQ;
  msg[1].addr      = BMI160_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

/****************************************************************************
 * Name: bmi160_putreg8
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

static void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = BMI160_I2C_FREQ;
  msg[0].addr      = BMI160_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi160_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

static void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                           uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = BMI160_I2C_FREQ;
  msg[0].addr      = BMI160_I2C_ADDR;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = BMI160_I2C_FREQ;
  msg[1].addr      = BMI160_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi160_configspi(priv->spi);

  /* Select the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the BMI160 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi160_checkid
 *
 * Description:
 *   Read and verify the BMI160 chip ID
 *
 ****************************************************************************/

static int bmi160_checkid(FAR struct bmi160_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = bmi160_getreg8(priv, BMI160_CHIP_ID);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmi160_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters:
 *   time  - Desired interval.
 *   odr_s - Array of sensor output data rate.
 *   len   - Array length.
 *
 * Returned Value:
 *   Index of the best fit ODR.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_findodr(unsigned long time,
                          FAR const struct bmi160_odr_s *odr_s,
                          int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      if (time == odr_s[i].odr)
        {
          return i;
        }
    }

  return i - 1;
}

/****************************************************************************
 * Name: bmi160_accel_enable
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   priv   - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_accel_enable(FAR struct bmi160_dev_s *priv, bool enable)
{
  int idx;

  if (enable)
    {
      /* Set accel as normal mode. */

      bmi160_putreg8(priv, BMI160_CMD, ACCEL_PM_NORMAL);
      usleep(30000);

      idx = bmi160_findodr(priv->interval, g_bmi160_accel_odr,
                           nitems(g_bmi160_accel_odr));
      bmi160_putreg8(priv, BMI160_ACCEL_CONFIG,
                     ACCEL_NORMAL_AVG4 | g_bmi160_accel_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi160_accel_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      /* Set suspend mode to sensors. */

      work_cancel(HPWORK, &priv->work);
      bmi160_putreg8(priv, BMI160_CMD, ACCEL_PM_SUSPEND);
    }
}

/****************************************************************************
 * Name: bmi160_gyro_enable
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   priv   - The instance of lower half sensor driver
 *   enable - true(enable) and false(disable)
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_gyro_enable(FAR struct bmi160_dev_s *priv,
                               bool enable)
{
  int idx;

  if (enable)
    {
      /* Set gyro as normal mode. */

      bmi160_putreg8(priv, BMI160_CMD, GYRO_PM_NORMAL);
      usleep(30000);

      idx = bmi160_findodr(priv->interval, g_bmi160_gyro_odr,
                           nitems(g_bmi160_gyro_odr));
      bmi160_putreg8(priv, BMI160_GYRO_CONFIG,
                    GYRO_NORMAL_MODE | g_bmi160_gyro_odr[idx].regval);

      work_queue(HPWORK, &priv->work,
                 bmi160_gyro_worker, priv,
                 priv->interval / USEC_PER_TICK);
    }
  else
    {
      work_cancel(HPWORK, &priv->work);

      /* Set suspend mode to sensors. */

      bmi160_putreg8(priv, BMI160_CMD, GYRO_PM_SUSPEND);
    }
}

/****************************************************************************
 * Name: bmi160_set_accel_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_set_accel_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR unsigned long *period_us)
{
  FAR struct bmi160_dev_s *priv = (FAR struct bmi160_dev_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi160_findodr(*period_us, g_bmi160_accel_odr,
                       nitems(g_bmi160_accel_odr));
  bmi160_putreg8(priv, BMI160_ACCEL_CONFIG,
                 ACCEL_NORMAL_AVG4 | g_bmi160_accel_odr[num].regval);

  priv->interval = g_bmi160_accel_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi160_set_gyro_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us > max_delay it will be truncated to max_delay and if
 *   *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *                by lower half driver.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_set_gyro_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR unsigned long *period_us)
{
  FAR struct bmi160_dev_s *priv = (FAR struct bmi160_dev_s *)lower;
  int num;

  /* Sanity check. */

  if (NULL == priv || NULL == period_us)
    {
      return -EINVAL;
    }

  num = bmi160_findodr(*period_us, g_bmi160_gyro_odr,
                       nitems(g_bmi160_gyro_odr));
  bmi160_putreg8(priv, BMI160_GYRO_CONFIG,
                 GYRO_NORMAL_MODE | g_bmi160_gyro_odr[num].regval);

  priv->interval = g_bmi160_gyro_odr[num].odr;
  *period_us = priv->interval;
  return OK;
}

/****************************************************************************
 * Name: bmi160_gyro_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_gyro_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                bool enable)
{
  FAR struct bmi160_dev_s *priv = (FAR struct bmi160_dev_s *)lower;

  bmi160_gyro_enable(priv, enable);

  return OK;
}

/****************************************************************************
 * Name: bmi160_accel_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int bmi160_accel_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct bmi160_dev_s *priv = (FAR struct bmi160_dev_s *)lower;

  bmi160_accel_enable(priv, enable);

  return OK;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: bmi160_accel_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_accel_worker(FAR void *arg)
{
  FAR struct bmi160_dev_s *priv = arg;
  struct sensor_accel accel;
  struct accel_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi160_accel_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi160_getregs(priv, BMI160_DATA_14, (FAR uint8_t *)&p, 6);
  accel.x = p.x;
  accel.y = p.y;
  accel.z = p.z;

  bmi160_getregs(priv, BMI160_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  accel.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
}

/****************************************************************************
 * Name: bmi160_gyro_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void bmi160_gyro_worker(FAR void *arg)
{
  FAR struct bmi160_dev_s *priv = arg;
  struct sensor_gyro gyro;
  struct gyro_t p;
  uint32_t time;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work,
             bmi160_accel_worker, priv,
             priv->interval / USEC_PER_TICK);

  bmi160_getregs(priv, BMI160_DATA_8, (FAR uint8_t *)&p, 6);
  gyro.x = p.x;
  gyro.y = p.y;
  gyro.z = p.z;

  bmi160_getregs(priv, BMI160_SENSORTIME_0, (FAR uint8_t *)&time, 3);

  /* Adjust sensing time into 24 bit */

  time >>= 8;
  gyro.timestamp = time;

  priv->lower.push_event(priv->lower.priv, &gyro, sizeof(gyro));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_register_accel
 *
 * Description:
 *   Register the BMI160 accel sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
static int bmi160_register_accel(int devno,
                                 FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
static int bmi160_register_accel(int devno,
                                 FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi160_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the STK31850 device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* config accelerometer */

#ifdef CONFIG_SENSORS_BMI160_I2C
  priv->i2c = dev;
#else /* CONFIG_SENSORS_BMI160_SPI */
  priv->spi = dev;
#endif

  priv->lower.ops = &g_bmi160_accel_ops;
  priv->lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->lower.uncalibrated = true;
  priv->interval = BMI160_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi160_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi160_putreg8(priv, BMI160_PMU_TRIGGER, 0);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register accel driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_register_gyro
 *
 * Description:
 *   Register the BMI160 gyro sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
static int bmi160_register_gyro(int devno,
                                FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
static int bmi160_register_gyro(int devno,
                                FAR struct spi_dev_s *dev)
#endif
{
  FAR struct bmi160_dev_s *priv;
  int ret ;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* config gyroscope */

#ifdef CONFIG_SENSORS_BMI160_I2C
  priv->i2c = dev;
#else /* CONFIG_SENSORS_BMI160_SPI */
  priv->spi = dev;
#endif

  priv->lower.ops = &g_bmi160_gyro_ops;
  priv->lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->lower.uncalibrated = true;
  priv->interval = BMI160_DEFAULT_INTERVAL;
  priv->lower.nbuffer = 1;

  /* Read and verify the deviceid */

  ret = bmi160_checkid(priv);
  if (ret < 0)
    {
      snerr("Wrong Device ID!\n");
      kmm_free(priv);
      return ret;
    }

  /* set sensor power mode */

  bmi160_putreg8(priv, BMI160_PMU_TRIGGER, 0);

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register gyro driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 accel and gyro sensor.
 *
 * Input Parameters:
 *   devno   - Sensor device number.
 *   config  - Interrupt fuctions.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
int bmi160_register(int devno, FAR struct i2c_master_s *dev)
#else /* CONFIG_BMI160_SPI */
int bmi160_register(int devno, FAR struct spi_dev_s *dev)
#endif
{
  int ret;

  ret = bmi160_register_accel(devno, dev);
  DEBUGASSERT(ret >= 0);

  ret = bmi160_register_gyro(devno, dev);
  DEBUGASSERT(ret >= 0);

  sninfo("BMI160 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_SENSORS_BMI160 */
