/*
 * This file includes code from the TCS34725 driver,
 * Copyright (c) 2024 Marcel
 * Original driver is MIT‑licensed; see LICENSE-DRIVER
 */

#ifndef TCS34725_INCLUDED
#define TCS34725_INCLUDED

#include <cstdint>

#include <array>

class TCS34725
{
	static constexpr std::uint8_t ADDRESS = 0x29U;
	static const constexpr std::array<std::uint8_t, 3> ids{ 0x4DU, 0x10U, 0x44U };

	static constexpr std::uint8_t COMMAND_BIT = 0x80U;

	static constexpr std::uint8_t ENABLE      = 0x00U;

	static constexpr std::uint8_t ATIME       = 0x01U;
	static constexpr std::uint8_t AILT        = 0x04U;
	static constexpr std::uint8_t AIHT        = 0x06U;
	static constexpr std::uint8_t APERS       = 0x0CU;
	static constexpr std::uint8_t CONTROL     = 0x0FU;
	static constexpr std::uint8_t SENSORID    = 0x12U;
	static constexpr std::uint8_t STATUS      = 0x13U;

	static constexpr std::uint8_t CDATA       = 0x14U;
	static constexpr std::uint8_t RDATA       = 0x16U; 
	static constexpr std::uint8_t GDATA       = 0x18U;
	static constexpr std::uint8_t BDATA       = 0x1AU;

	static constexpr std::uint8_t ENABLE_PON  = 0x01U;
	static constexpr std::uint8_t ENABLE_AEN  = 0x02U;
	static constexpr std::uint8_t ENABLE_WEN  = 0x08U;
	static constexpr std::uint8_t ENABLE_AIEN = 0x02U;

public:
	enum gain_t : std::uint8_t {
		GAIN_1X  = 0x00U,
		GAIN_4X  = 0x01U,
		GAIN_16X = 0x02U,
		GAIN_60X = 0x03U,
	};

private:
	int smbus;
	std::uint8_t address;
	std::uint8_t integration_time;
	gain_t gain;

public:
	explicit TCS34725(int smbus = 1, std::uint8_t address = TCS34725::ADDRESS);
	TCS34725(const TCS34725&) = delete;
	~TCS34725();

	void enable(void);
	void disable(void);
	void set_integration_time_cycles(std::uint8_t);
	void set_integration_time(float);
	void set_gain(gain_t);
	void get_raw(std::array<std::uint16_t, 4>&);
	void get_rgb(float&, float&, float&);
private:
	inline void delay(void);
	inline bool valid(void);
};

#endif
