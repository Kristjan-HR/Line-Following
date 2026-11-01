/*
 * This file includes code from the TCS34725 driver,
 * Copyright (c) 2024 Marcel
 * Original driver is MIT‑licensed; see LICENSE-DRIVER
 */

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>

#include <ctime>
#include <cstring>
#include <algorithm>
#include <stdexcept>

#include <iostream>

#include "TCS34725.hpp"



TCS34725::TCS34725(int smbus, std::uint8_t address) : smbus(0), address(address), integration_time(0xFFU), gain(TCS34725::GAIN_1X)
{
	char name[20];
	snprintf(name, sizeof(name) - 1, "/dev/i2c-%d", smbus);
	this->smbus = open(name, O_RDWR);
	if (this->smbus < 0) {
		// Exceptions are usually avoided or forbidden in embedded code.
		// We like to avoid some ideoms from embedded programming
		// to realise the RAII principle.
		throw std::runtime_error("SMBus does not exist");
	}

	if (ioctl(this->smbus, I2C_SLAVE, this->address & 0x7f) < 0) {
		throw std::runtime_error("TCS34725::Address not found");
	}

	std::uint8_t id = i2c_smbus_read_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::SENSORID);
	if (auto it = std::find(ids.cbegin(), ids.cend(), id); it == ids.cend()) {
		std::cerr << "Got id " << int{id} << std::endl;
		throw std::runtime_error("TCS34725: Wrong ID");
	}
	this->set_integration_time(this->integration_time);
	this->set_gain(this->gain);
	this->enable();
}

TCS34725::~TCS34725()
{
	this->disable();
	close(this->smbus);
}


inline void TCS34725::delay(void)
{
	static struct timespec t{ 0, (256 - this->integration_time) * 2400000 };
	nanosleep(&t, nullptr);
}

inline bool TCS34725::valid(void)
{
	std::uint8_t status = i2c_smbus_read_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::STATUS);
	return (0U == (status & 0x01U)) ? false : true;
}


void TCS34725::enable(void)
{
	static const constexpr struct timespec threems = {0, 3000000};

	std::uint8_t enabled = i2c_smbus_read_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ENABLE);
	i2c_smbus_write_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ENABLE,
			enabled | TCS34725::ENABLE_PON);
	nanosleep(&threems, nullptr);
	i2c_smbus_write_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ENABLE,
			enabled | TCS34725::ENABLE_PON | TCS34725::ENABLE_AEN);
}

void TCS34725::disable(void)
{
	std::uint8_t enabled = i2c_smbus_read_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ENABLE);
	enabled &= ~(TCS34725::ENABLE_PON | TCS34725::ENABLE_AEN);
	i2c_smbus_write_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ENABLE, enabled);
}

void TCS34725::set_integration_time_cycles(std::uint8_t it)
{
	this->integration_time = it;
	i2c_smbus_write_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::ATIME,
			this->integration_time);
}

void TCS34725::set_integration_time(float ms)
{
	if (255 * 2.4 < ms) {
		throw std::runtime_error("TCS34725: integration time too long");
	}
	std::uint8_t cycles = static_cast<std::uint8_t>(256 - ms / 2.4);
	this->set_integration_time_cycles(cycles);
}

void TCS34725::set_gain(TCS34725::gain_t gain)
{
	this->gain = gain;
	i2c_smbus_write_byte_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::CONTROL, this->gain);
}


void TCS34725::get_raw(std::array<std::uint16_t, 4> &result)
{
	while (!this->valid()) {
		this->delay();
	}
	result[0] = i2c_smbus_read_word_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::RDATA);
	result[1] = i2c_smbus_read_word_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::GDATA);
	result[2] = i2c_smbus_read_word_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::BDATA);
	result[3] = i2c_smbus_read_word_data(this->smbus,
			TCS34725::COMMAND_BIT | TCS34725::CDATA);
}

void TCS34725::get_rgb(float& r, float& g, float& b)
{
	std::array<std::uint16_t, 4> color;
	this->get_raw(color);
	if (color[3] == 0) {
		r = g = b = 0;
		return;
	}
	r = static_cast<float>(color[0]) / static_cast<float>(color[3]);
	g = static_cast<float>(color[1]) / static_cast<float>(color[3]);
	b = static_cast<float>(color[2]) / static_cast<float>(color[3]);
}
