/***************************************************************************//**
 *   @file   iio_axi_dac.c
 *   @brief  Implementation of iio_axi_dac
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "iio_axi_dac.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "platform_drivers.h"
#include "xml.h"
#include "util.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Init and create iio_axi_dac.
 * @param iio_axi_adc - pointer to iio_axi_dac.
 * @param init - init parameters.
 * @return SUCCESS in case of success or negative value otherwise.
 */
ssize_t iio_axi_dac_init(struct iio_axi_dac **iio_axi_dac,
		struct iio_axi_dac_init_par *init)
{
	*iio_axi_dac = calloc(1, sizeof(struct iio_axi_dac));
	if (!(*iio_axi_dac))
		return FAILURE;
	(*iio_axi_dac)->dac = init->dac;
	(*iio_axi_dac)->dmac = init->dmac;
	(*iio_axi_dac)->dac_ddr_base = init->dac_ddr_base;
	(*iio_axi_dac)->dcache_flush = init->dcache_flush;

	return SUCCESS;
}

/**
 * @brief Free the resources allocated by iio_axi_dac_init().
 * @param iio_axi_dac - pointer to iio_axi_adc.
 * @return SUCCESS in case of success or negative value otherwise.
 */
ssize_t iio_axi_dac_remove(struct iio_axi_dac *iio_axi_dac)
{
	free(iio_axi_dac);

	return SUCCESS;
}

/**
 * get_dds_calibscale
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_voltage_calibscale(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel)
{
	int32_t val, val2;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_get_calib_scale(tinyiiod_dac->dac, channel->ch_num,
			&val, &val2);
	int32_t i = 0;
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		ret = (ssize_t) snprintf(buf, len, "-");
		i++;
	}
	ret = i + (ssize_t) snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val,
				     labs(val2));

	return ret;
}

/**
 * get_dds_calibphase
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_voltage_calibphase(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel)
{
	int32_t val, val2;
	int32_t i = 0;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_get_calib_phase(tinyiiod_dac->dac, channel->ch_num,
			&val, &val2);
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		i++;
	}
	return i + snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val, labs(val2));
}

/**
 * get_dds_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_voltage_sampling_frequency(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	/* This function doesn't have an equivalent function in axi_dac_core,
	 * and it should be implemented there first */

	return -ENOENT;
}

/**
 * get_dds_altvoltage_phase
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_altvoltage_phase(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	uint32_t phase;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_get_phase(tinyiiod_dac->dac, channel->ch_num, &phase);
	if (ret < 0)
		return ret;

	return snprintf(buf, len, "%"PRIu32"", phase);
}

/**
 * get_dds_altvoltage_scale
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_altvoltage_scale(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	int32_t scale;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_get_scale(tinyiiod_dac->dac, channel->ch_num, &scale);
	if (ret < 0)
		return ret;

	return snprintf(buf, len, "%"PRIi32".%.6"PRIi32"", (scale / 1000000),
			(scale % 1000000));
}

/**
 * get_dds_altvoltage_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_altvoltage_frequency(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	uint32_t freq;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_get_frequency(tinyiiod_dac->dac, channel->ch_num, &freq);
	if (ret < 0)
		return ret;

	return snprintf(buf, len, "%"PRIi32"", freq);
}

/**
 * get_dds_altvoltage_raw
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_altvoltage_raw(void *device, char *buf, size_t len,
				      const struct iio_ch_info *channel)
{
	/* This function doesn't have an equivalent function in axi_dac_core,
	 * and it should be implemented there first */

	return -ENOENT;
}

/**
 * get_dds_altvoltage_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
static ssize_t get_altvoltage_sampling_frequency(void *device, char *buf,
		size_t len,
		const struct iio_ch_info *channel)
{
	/* This function doesn't have an equivalent function in axi_dac_core,
	 * and it should be implemented there first */

	return -ENOENT;
}

/**
 * set_dds_calibscale
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_voltage_calibscale(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel)
{
	float calib= strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_set_calib_scale(tinyiiod_dac->dac, channel->ch_num, val, val2);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_calibphase
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_voltage_calibphase(void *device, char *buf, size_t len,
				  const struct iio_ch_info *channel)
{
	float calib = strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_set_calib_phase(tinyiiod_dac->dac, channel->ch_num, val, val2);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_voltage_sampling_frequency(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	/* This function doesn't have an equivalent function in axi_dac_core,
	 * and it should be implemented there first */

	return -ENOENT;
}

/**
 * set_dds_altvoltage_phase
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_altvoltage_phase(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	uint32_t phase = srt_to_uint32(buf);
	struct iio_axi_dac * iiod_dac = (struct iio_axi_dac *)device;
	ssize_t ret = axi_dac_dds_set_phase(iiod_dac->dac, channel->ch_num, phase);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_altvoltage_scale
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_altvoltage_scale(void *device, char *buf, size_t len,
					const struct iio_ch_info *channel)
{
	float fscale = strtof(buf, NULL);
	int32_t scale = fscale * 1000000;
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_set_scale(tinyiiod_dac->dac, channel->ch_num, scale);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_altvoltage_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_altvoltage_frequency(void *device, char *buf, size_t len,
		const struct iio_ch_info *channel)
{
	uint32_t freq = srt_to_uint32(buf);
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret = axi_dac_dds_set_frequency(tinyiiod_dac->dac, channel->ch_num, freq);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_altvoltage_raw
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_altvoltage_raw(void *device, char *buf, size_t len,
				      const struct iio_ch_info *channel)
{
	uint32_t dds_mode = srt_to_uint32(buf);
	struct iio_axi_dac* tinyiiod_dac = (struct iio_axi_dac*)device;
	ssize_t ret;
	ret = axi_dac_set_datasel(tinyiiod_dac->dac, -1, dds_mode ? AXI_DAC_DATA_SEL_DDS : AXI_DAC_DATA_SEL_DMA);
	if (ret < 0)
		return ret;

	return len;
}

/**
 * set_dds_altvoltage_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
static ssize_t set_altvoltage_sampling_frequency(void *device, char *buf,
		size_t len,
		const struct iio_ch_info *channel)
{
	/* This function doesn't have an equivalent function in axi_dac_core,
	 * and it should be implemented there first */

	return -ENOENT;
}

static struct iio_attribute iio_attr_voltage_calibphase = {
	.name = "calibphase",
	.show = get_voltage_calibphase,
	.store = set_voltage_calibphase,
};

static struct iio_attribute iio_attr_voltage_calibscale = {
	.name = "calibscale",
	.show = get_voltage_calibscale,
	.store = set_voltage_calibscale,
};

static struct iio_attribute iio_attr_voltage_sampling_frequency = {
	.name = "sampling_frequency",
	.show = get_voltage_sampling_frequency,
	.store = set_voltage_sampling_frequency,
};

static struct iio_attribute iio_attr_altvoltage_raw = {
	.name = "raw",
	.show = get_altvoltage_raw,
	.store = set_altvoltage_raw,
};
static struct iio_attribute iio_attr_altvoltage_phase = {
	.name = "phase",
	.show = get_altvoltage_phase,
	.store = set_altvoltage_phase,
};
static struct iio_attribute iio_attr_altvoltage_frequency = {
	.name = "frequency",
	.show = get_altvoltage_frequency,
	.store = set_altvoltage_frequency,
};
static struct iio_attribute iio_attr_altvoltage_scale = {
	.name = "scale",
	.show = get_altvoltage_scale,
	.store = set_altvoltage_scale,
};
static struct iio_attribute iio_attr_altvoltage_sampling_frequency = {
	.name = "sampling_frequency",
	.show = get_altvoltage_sampling_frequency,
	.store = set_altvoltage_sampling_frequency,
};

static struct iio_attribute *iio_voltage_attributes[] = {
	&iio_attr_voltage_calibscale,
	&iio_attr_voltage_calibphase,
	&iio_attr_voltage_sampling_frequency,
	NULL,
};
static struct iio_attribute *iio_altvoltage_attributes[] = {
	&iio_attr_altvoltage_raw,
	&iio_attr_altvoltage_phase,
	&iio_attr_altvoltage_scale,
	&iio_attr_altvoltage_frequency,
	&iio_attr_altvoltage_sampling_frequency,
	NULL,
};
static struct iio_channel iio_channel_voltage0 = {
	.name = "voltage0",
	.attributes = iio_voltage_attributes,
};
static struct iio_channel iio_channel_voltage1 = {
	.name = "voltage1",
	.attributes = iio_voltage_attributes,
};
static struct iio_channel iio_channel_voltage2 = {
	.name = "voltage2",
	.attributes = iio_voltage_attributes,
};
static struct iio_channel iio_channel_voltage3 = {
	.name = "voltage3",
	.attributes = iio_voltage_attributes,
};

static struct iio_channel iio_channel_altvoltage0 = {
	.name = "altvoltage0",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage1 = {
	.name = "altvoltage1",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage2 = {
	.name = "altvoltage2",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage3 = {
	.name = "altvoltage3",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage4 = {
	.name = "altvoltage4",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage5 = {
	.name = "altvoltage5",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage6 = {
	.name = "altvoltage6",
	.attributes = iio_altvoltage_attributes,
};
static struct iio_channel iio_channel_altvoltage7 = {
	.name = "altvoltage7",
	.attributes = iio_altvoltage_attributes,
};

static struct iio_channel *iio_dac_channels[] = {
	&iio_channel_voltage0,
	&iio_channel_voltage1,
	&iio_channel_voltage2,
	&iio_channel_voltage3,
	&iio_channel_altvoltage0,
	&iio_channel_altvoltage1,
	&iio_channel_altvoltage2,
	&iio_channel_altvoltage3,
	&iio_channel_altvoltage4,
	&iio_channel_altvoltage5,
	&iio_channel_altvoltage6,
	&iio_channel_altvoltage7,
	NULL,
};

/**
 * Create iio_device
 * @param *device name
 * @return iio_device or NULL, in case of failure
 */
struct iio_device *iio_axi_dac_create_device(const char *device_name)
{
	struct iio_device *iio_dac_device;

	iio_dac_device = calloc(1, sizeof(struct iio_device));
	if (!iio_dac_device)
		return NULL;
	iio_dac_device->name = device_name;
	iio_dac_device->channels = iio_dac_channels;
	iio_dac_device->attributes = NULL; /* no device attribute */

	return iio_dac_device;
}

/**
 * Transfer data from RAM to DAC
 * @param *device name
 * @param *buff
 * @param bytes_count
 * @return bytes_count
 */
ssize_t iio_axi_dac_transfer_mem_to_dev(struct axi_dmac	*tx_dmac,
				uint32_t dac_ddr_baseaddr, size_t bytes_count)
{
	tx_dmac->flags = DMA_CYCLIC;
	ssize_t ret = axi_dmac_transfer(tx_dmac, dac_ddr_baseaddr,
					bytes_count);
	if(ret < 0)
		return ret;

	return bytes_count;
}

/**
 * Write data to RAM
 * @param *device name
 * @param *buff
 * @param *offset in memory, used if some data have been already written
 * @param bytes_count
 * @return bytes_count
 */
ssize_t iio_axi_dac_write_dev(struct iio_axi_dac *iiod_dac, const char *buf,
		      size_t offset,  size_t bytes_count)
{
	ssize_t ret = axi_dac_set_datasel(iiod_dac->dac, -1, AXI_DAC_DATA_SEL_DMA);
	if(ret < 0)
		return ret;

	ret = axi_dac_set_buff(iiod_dac->dac, iiod_dac->dac_ddr_base + offset,
			       (uint16_t *)buf,
			       bytes_count);
	if(ret < 0)
		return ret;

	if(iiod_dac->dcache_flush)
		iiod_dac->dcache_flush();

	return bytes_count;
}

enum ch_type{
	CH_VOLTGE,
	CH_ALTVOLTGE,
};

/**
 * @brief Fill device with channels
 * @param device.
 * @param ch_no.
 * @param ch_t - channel type.
 * @return SUCCESS in case of success or negative value otherwise.
 */
static ssize_t iio_axi_dac_channel_xml(struct xml_node *device, uint8_t ch_no, enum ch_type ch_t)
{
	char *ch_id[] = {"voltage", "altvoltage"};
	char *ch_name[] = {"_I_F", "_Q_F"};
	struct iio_attribute **iio_attributes;
	struct xml_node *attribute, *channel;
	struct xml_attribute *att = NULL;
	char buff[256];
	uint8_t i, j;
	ssize_t ret;

	for (i = 0; i < ch_no; i++) {
		ret = xml_create_node(&channel, "channel");
		if (ret < 0)
			return ret;
		ret = sprintf(buff, "%s%d", ch_id[ch_t], i);
		if (ret < 0)
			return ret;
		ret = xml_create_attribute(&att, "id", buff);
		if (ret < 0)
			return ret;
		ret = xml_add_attribute(channel, att);
		if (ret < 0)
			return ret;
		ret = xml_create_attribute(&att, "type", "output");
		if (ret < 0)
			return ret;
		ret = xml_add_attribute(channel, att);
		if (ret < 0)
			return ret;

		if (ch_t == CH_VOLTGE) {
			ret = xml_create_node(&attribute, "scan-element");
			if (ret < 0)
				return ret;
			ret = sprintf(buff, "%d", i);
			if (ret < 0)
				return ret;
			ret = xml_create_attribute(&att, "index", buff);
			if (ret < 0)
				return ret;
			ret = xml_add_attribute(attribute, att);
			if (ret < 0)
				return ret;
			ret = xml_create_attribute(&att, "format", "le:S16/16&gt;&gt;0");
			if (ret < 0)
				return ret;
			ret = xml_add_attribute(attribute, att);
			if (ret < 0)
				return ret;
			ret = xml_add_node(channel, attribute);
			if (ret < 0)
				return ret;
		}
		else
		{
			/* CH_ALTVOLTGE */
			ret = sprintf(buff, "TX%d%s%d", (i / 4) + 1, ch_name[(i % 4) / 2], (i % 2) + 1);
			if (ret < 0)
				return ret;
			ret = xml_create_attribute(&att, "name", buff);
			if (ret < 0)
				return ret;
			ret = xml_add_attribute(channel, att);
			if (ret < 0)
				return ret;
		}
		iio_attributes = (ch_t == CH_VOLTGE) ? iio_voltage_attributes : iio_altvoltage_attributes;

		for (j = 0; iio_attributes[j] != NULL; j++) {
			ret = xml_create_node(&attribute, "attribute");
			if (ret < 0)
				return ret;
			ret = xml_create_attribute(&att, "name",
					iio_attributes[j]->name);
			if (ret < 0)
				return ret;
			ret = xml_add_attribute(attribute, att);
			if (ret < 0)
				return ret;
			ret = sprintf(buff, "out_%s%d_%s", ch_id[ch_t], i, iio_attributes[j]->name);
			if (ret < 0)
				return ret;
			ret = xml_create_attribute(&att, "filename", buff);
			if (ret < 0)
				return ret;
			ret = xml_add_attribute(attribute, att);
			if (ret < 0)
				return ret;
			ret = xml_add_node(channel, attribute);
			if (ret < 0)
				return ret;
		}
		ret = xml_add_node(device, channel);
		if (ret < 0)
			return ret;
	}

	return SUCCESS;
}

/**
 * @brief Get an axi_adc xml.
 * @param xml - xml string.
 * @param device_name.
 * @param ch_no.
 * @return SUCCESS in case of success or negative value otherwise.
 */
ssize_t iio_axi_dac_get_xml(char** xml, const char *device_name, uint8_t ch_no)
{
	struct xml_node *device;
	struct xml_attribute *att;
	struct xml_document *document = NULL;
	ssize_t ret;

	ret = xml_create_node(&device, "device");
	if (ret < 0)
		goto error;
	ret = xml_create_attribute(&att, "id", device_name);
	if (ret < 0)
		goto error;
	ret = xml_add_attribute(device, att);
	if (ret < 0)
		goto error;
	ret = xml_create_attribute(&att, "name", device_name);
	if (ret < 0)
		goto error;
	ret = xml_add_attribute(device, att);
	if (ret < 0)
		goto error;
	ret = iio_axi_dac_channel_xml(device, ch_no, CH_VOLTGE);
	if (ret < 0)
		goto error;
	ret = iio_axi_dac_channel_xml(device, ch_no * 2, CH_ALTVOLTGE);
	if (ret < 0)
		goto error;
	ret = xml_create_document(&document, device);
	if (ret < 0) {
		if (document)
			xml_delete_document(document);
		goto error;
	}
	*xml = document->buff;

error:
	if (device)
		xml_delete_node(device);

	return ret;
}
