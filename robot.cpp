#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <fcntl.h>
#include <iostream>
#include <lgpio.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "TCS34725.hpp"

constexpr int CHIP = 0;
constexpr int STOP = 17;
constexpr int LF = 16, LR = 12, RF = 13, RR = 19;
constexpr int SPI_CHANNEL = 0;

constexpr double VREF = 3.3;
constexpr double STOP_DISTANCE_CM = 20.0;
constexpr double PWM_HZ = 1000.0;
constexpr double MOTOR_POWER = 0.30;
constexpr double STARTUP_POWER = 0.20;
constexpr int BLACK_CLEAR_THRESHOLD = 100;

constexpr auto LOOP_INTERVAL = std::chrono::milliseconds(10);

enum class Wheel { Left, Right };

int spi_fd = -1;

int read_adc(int channel) {
  unsigned char tx[3] = {1, static_cast<unsigned char>((8 + channel) << 4), 0};
  unsigned char rx[3] = {};

  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<unsigned long>(tx);
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx);
  transfer.len = sizeof(tx);
  transfer.speed_hz = 1'000'000;
  transfer.bits_per_word = 8;

  if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0)
    return -1;

  return ((rx[1] & 3) << 8) | rx[2];
}

double voltage_to_distance(double voltage) {
  constexpr double volts[] = {0.530, 0.813, 1.320, 1.731, 1.961, 2.201};
  constexpr double cm[] = {80.0, 60.0, 40.0, 30.0, 25.0, 20.0};

  if (voltage <= volts[0])
    return cm[0];
  if (voltage >= volts[5])
    return cm[5];

  for (int i = 0; i < 5; ++i) {
    if (voltage <= volts[i + 1]) {
      return cm[i] + (voltage - volts[i]) * (cm[i + 1] - cm[i]) /
                         (volts[i + 1] - volts[i]);
    }
  }

  return cm[5];
}

void motor(int chip, int forward_pin, int reverse_pin, double power) {
  power = std::clamp(power, -0.5, 0.5);

  if (power > 0.0) {
    lgTxPwm(chip, reverse_pin, PWM_HZ, 0, 0, 0);
    lgTxPwm(chip, forward_pin, PWM_HZ, power * 100.0, 0, 0);
  } else if (power < 0.0) {
    lgTxPwm(chip, forward_pin, PWM_HZ, 0, 0, 0);
    lgTxPwm(chip, reverse_pin, PWM_HZ, -power * 100.0, 0, 0);
  } else {
    lgTxPwm(chip, forward_pin, PWM_HZ, 0, 0, 0);
    lgTxPwm(chip, reverse_pin, PWM_HZ, 0, 0, 0);
  }
}

void stop_motors(int chip) {
  motor(chip, LF, LR, 0.0);
  motor(chip, RF, RR, 0.0);
}

void run_wheel(int chip, Wheel wheel, double power) {
  if (wheel == Wheel::Left) {
    motor(chip, LF, LR, power);
    motor(chip, RF, RR, 0.0);
  } else {
    motor(chip, LF, LR, 0.0);
    motor(chip, RF, RR, power);
  }
}

int main() {
  int chip = lgGpiochipOpen(CHIP);
  if (chip < 0) {
    std::cerr << "Could not open GPIO chip\n";
    return 1;
  }

  if (lgGpioClaimInput(chip, LG_SET_PULL_UP, STOP) < 0) {
    std::cerr << "Could not configure stop input\n";
    lgGpiochipClose(chip);
    return 1;
  }

  for (int pin : {LF, LR, RF, RR}) {
    if (lgGpioClaimOutput(chip, 0, pin, 0) < 0) {
      std::cerr << "Could not configure motor GPIO\n";
      stop_motors(chip);
      lgGpiochipClose(chip);
      return 1;
    }
  }

  spi_fd = open("/dev/spidev0.0", O_RDWR);
  if (spi_fd < 0) {
    std::cerr << "Could not open SPI device\n";
    stop_motors(chip);
    lgGpiochipClose(chip);
    return 1;
  }

  unsigned int spi_speed = 1'000'000;
  unsigned char spi_mode = 0, spi_bits = 8;
  ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
  ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
  ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

  try {
    TCS34725 sensor;
    sensor.set_integration_time(2.4F * 8);
    sensor.set_gain(TCS34725::GAIN_1X);

    Wheel active_wheel = Wheel::Left;
    bool line_found = false;
    bool previous_black = false;
    double distance_cm = 80.0;
    int loop_count = 0;

    while (true) {
      std::array<std::uint16_t, 4> color{};
      sensor.get_raw(color);
      int colour_clear = color[3];

      if (lgGpioRead(chip, STOP) == 1)
        break;

      if (loop_count++ % 4 == 0) {
        int adc_value = read_adc(SPI_CHANNEL);
        if (adc_value < 0) {
          std::cerr << "ADC read failed; stopping\n";
          break;
        }

        distance_cm = voltage_to_distance(adc_value * VREF / 1023.0);
      }

      bool black = colour_clear < BLACK_CLEAR_THRESHOLD;

      if (!line_found) {
        if (black) {
          line_found = true;
          previous_black = true;
          stop_motors(chip);
        } else if (distance_cm > STOP_DISTANCE_CM) {
          motor(chip, LF, LR, STARTUP_POWER);
          motor(chip, RF, RR, -STARTUP_POWER);
        } else {
          stop_motors(chip);
        }
      } else {
        if (previous_black && !black) {
          active_wheel =
              active_wheel == Wheel::Left ? Wheel::Right : Wheel::Left;
        }

        previous_black = black;

        if (distance_cm > STOP_DISTANCE_CM)
          run_wheel(chip, active_wheel, MOTOR_POWER);
        else
          stop_motors(chip);
      }

      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(LOOP_INTERVAL)
              .count()));
    }
  } catch (const std::exception &e) {
    std::cerr << "Colour sensor error: " << e.what() << '\n';
    stop_motors(chip);
    close(spi_fd);
    lgGpiochipClose(chip);
    return 1;
  }

  stop_motors(chip);
  close(spi_fd);
  lgGpiochipClose(chip);
  return 0;
}
