/***************************************************************************//**
 *   @file   ad7091r5.h
 *   @brief  Header file for ad7091r5 Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef SRC_AD7091R5_H_
#define SRC_AD7091R5_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include "i2c.h"
#include "gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* AD7091r5 registers */
#define AD7091R_REG_RESULT		0
#define AD7091R_REG_CHANNEL		1
#define AD7091R_REG_CONF		2
#define AD7091R_REG_ALERT		3
#define AD7091R_REG_CH_LOW_LIMIT(ch)	((ch) * 3 + 4)
#define AD7091R_REG_CH_HIGH_LIMIT(ch)	((ch) * 3 + 5)
#define AD7091R_REG_CH_HYSTERESIS(ch)	((ch) * 3 + 6)

/* AD7091R_REG_RESULT */
#define REG_RESULT_CH_ID(x)		(((x) >> 13) & 0x3)
#define REG_RESULT_CONV_RESULT(x)	((x) & 0xfff)

/* AD7091R_REG_CONF */
#define REG_CONF_GPO0_ALERT		BIT(4)
#define REG_CONF_GPO0_BUSY		BIT(5)
#define REG_CONF_AUTO			BIT(8)
#define REG_CONF_CMD			BIT(10)
#define REG_CONF_GPO0_DRIVE_TYPE	BIT(15)

#define REG_CONF_SLEEP_MODE_MASK	(BIT(0) | BIT(1))
#define REG_CONF_GPO1_MASK		BIT(2)
#define REG_CONF_GPO0_MASK		BIT(3)
#define REG_CONF_GPO2_MASK		BIT(14)
#define REG_CONF_MODE_MASK		(REG_CONF_AUTO | REG_CONF_CMD)
#define REG_CONF_GPO0_MODE_MASK		(REG_CONF_GPO0_DRIVE_TYPE | REG_CONF_GPO0_BUSY | REG_CONF_GPO0_ALERT)


/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum ad7091r_mode
 * @brief Converter supported modes
 */
enum ad7091r_mode {
	/** Sample mode, conversion started by CONVST */
	AD7091R_MODE_SAMPLE,
	/** Command mode, conversion starts on the first pos edge of SCL */
	AD7091R_MODE_COMMAND,
	/** Sample mode, convert continuously */
	AD7091R_MODE_AUTOCYCLE,
};

/**
 * @enum ad7091r_sleep_mode
 * @brief Converter supported sleep modes
 */
enum ad7091r_sleep_mode {
	/** Default operation:
	 * Sleep mode Off, Internal reference Off */
	AD7091R_SLEEP_MODE_0,
	/** Sleep mode Off, Internal reference On */
	AD7091R_SLEEP_MODE_1,
	/** Sleep mode On, Internal reference Off */
	AD7091R_SLEEP_MODE_2,
	/** Sleep mode On, Internal reference On */
	AD7091R_SLEEP_MODE_3,
};

/**
 * @enum ad7091r_port
 * @brief Converter general purpose outputs
 */
enum ad7091r_port {
	/** GPO0 */
	AD7091R_GPO0,
	/** GPO1 */
	AD7091R_GPO1,
	/** GPO2 */
	AD7091R_GPO2,
};

/**
 * @enum ad7091r_gp0_mode
 * @brief Port 0 configuration
 */
enum ad7091r_gpo0_mode {
	/** GPO0 is output port */
	AD7091R_GPO0_ENABLED,
	/** GPO0 is Alert indicator */
	AD7091R_GPO0_ALERT,
	/** GPO0 is busy indicator, device is converting */
	AD7091R_GPO0_BUSY,
};

/**
 * @struct ad7091r5_init_param
 * @brief  Structure containing the init parameters needed by the ad7091r5 device
 */
struct ad7091r5_init_param {
	/* I2C */
	i2c_init_param		*i2c_init;
	/** RESET GPIO initialization structure. */
	struct gpio_init_param	*gpio_resetn;
};

/**
 * @struct ad7091r5_dev
 * @brief  Structure representing an ad7091r5 device
 */
struct ad7091r5_dev {
	/* I2C descriptor */
	struct i2c_desc 	*i2c_desc;
	/** RESET GPIO handler. */
	struct gpio_desc	*gpio_resetn;
	/** Device mode */
	enum ad7091r_mode 	mode;
	/** GP0 mode */
	enum ad7091r_gpo0_mode	gpo0_mode;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
int32_t ad7091r5_init(struct ad7091r5_dev **device,
		      struct ad7091r5_init_param *init_param);

/* Remove the device and release resources. */
int32_t ad7091r5_remove(struct ad7091r5_dev *dev);

/* Set device mode */
int32_t ad7091r_set_mode(struct ad7091r5_dev *dev,
			 enum ad7091r_mode mode);

/* Set device sleep mode */
int32_t ad7091r_sleep_mode(struct ad7091r5_dev *dev,
			   enum ad7091r_sleep_mode mode);

/* Set device set port value */
int32_t ad7091r_set_port(struct ad7091r5_dev *dev,
			 enum ad7091r_port port, bool value);

/* Set device set GPO0 mode */
int32_t ad7091r_set_gpo0_mode(struct ad7091r5_dev *dev,
			      enum ad7091r_gpo0_mode mode, bool is_cmos);

/* Select device channel. */
int32_t ad7091r_set_channel(struct ad7091r5_dev *dev, uint8_t channel);

/* Read one sample. */
int32_t ad7091r_read_one(struct ad7091r5_dev *dev, uint8_t channel,
			 uint16_t *read_val);

#endif /* SRC_AD7091R5_H_ */
