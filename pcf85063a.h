#ifndef PCF85063A_H
#define PCF85063A_H

#include <stdint.h>
#include <time.h>

#include "stm32l1xx_i2c.h"

/**
 * @brief  PCF85063A Temperature Sensor I2C Interface pins
 */
#define PCF85063A_I2C			 (I2C1)
#define PCF85063A_I2C_CLK		 (RCC_APB1Periph_I2C1)
#define PCF85063A_I2C_SCL_PIN	 (GPIO_Pin_6) 			/* PB.06 */
#define PCF85063A_I2C_SCL_SOURCE (GPIO_PinSource6)
#define PCF85063A_I2C_SDA_PIN	 (GPIO_Pin_7) 			/* PB.07 */
#define PCF85063A_I2C_GPIO_PORT	 (GPIOB)				/* GPIOB */
#define PCF85063A_I2C_GPIO_CLK	 (RCC_AHBPeriph_GPIOB)
#define PCF85063A_I2C_SDA_SOURCE (GPIO_PinSource7)
#define PCF85063A_I2C_AF		 (GPIO_AF_I2C1)

#define PCF85063A_FLAG_TIMEOUT ((uint32_t)0x1000)
#define PCF85063A_LONG_TIMEOUT ((uint32_t)(10 * PCF85063A_FLAG_TIMEOUT))

#define I2C_ADDR 0x51

// registar overview - crtl & status reg
#define RTC_CTRL_1 0x0
#define RTC_CTRL_2 0x01
#define RTC_OFFSET 0x02
#define RTC_RAM_by 0x03
// registar overview - time & data reg
#define RTC_SECOND_ADDR 0x04
#define RTC_MINUTE_ADDR 0x05
#define RTC_HOUR_ADDR	0x06
#define RTC_DAY_ADDR	0x07
#define RTC_WDAY_ADDR	0x08
#define RTC_MONTH_ADDR	0x09
#define RTC_YEAR_ADDR	0x0A	// years 0-99; calculate real year = 1970 + RCC reg year
// registar overview - alarm reg
#define RTC_SECOND_ALARM 0x0B
#define RTC_MINUTE_ALARM 0x0C
#define RTC_HOUR_ALARM	 0x0D
#define RTC_DAY_ALARM	 0x0E
#define RTC_WDAY_ALARM	 0x0F
// registar overview - timer reg
#define RTC_TIMER_VAL	0x10
#define RTC_TIMER_MODE	0x11
#define RTC_TIMER_TCF	0x08
#define RTC_TIMER_TE	0x04
#define RTC_TIMER_TIE	0x02
#define RTC_TIMER_TI_TP 0x01
// format
#define RTC_ALARM		   0x80	   // set AEN_x registers
#define RTC_ALARM_AIE	   0x80	   // set AIE ; enable/disable interrupt output pin
#define RTC_ALARM_AF	   0x40	   // set AF register ; alarm flag needs to be cleared for alarm
#define RTC_CTRL_2_DEFAULT 0x00
#define RTC_TIMER_FLAG	   0x08

#define RTC_MODE_UNKOWN (0)
#define RTC_MODE_12		(1)
#define RTC_MODE_24		(2)

#define TIME_ZONE 		(7) // timezone VietNam

class PCF85063A {
public:
	void init();
	enum CountdownSrcClock {
		TIMER_CLOCK_4096HZ	 = 0,
		TIMER_CLOCK_64HZ	 = 1,
		TIMER_CLOCK_1HZ		 = 2,
		TIMER_CLOCK_1PER60HZ = 3
	};

	void reset();
	void clearStopFlag();
	
	/* Get RTC times */
	uint32_t readDateTime();
	uint8_t getSecond();
	uint8_t getMinute();
	uint8_t getHour();
	uint8_t getHourLocal();
	uint8_t getDay();
	uint8_t getWeekday();
	uint8_t getMonth();
	uint16_t getYear();
	char* getMonthDay_c();
	char* getWeekDay_c();
	char* getMonth_c();

	/* Set RTC times */
	void setMode12or24Hour(uint8_t mode);
	void setHour(uint8_t hour);
	void setMinute(uint8_t minute);
	void setMonthDay(uint8_t mday);
	void setWeekDay(uint8_t wday);
	void setMonth(uint8_t month);
	void setYear(uint16_t year);
	void setTimestamp(long long timestamp);
	void setDateTime(uint8_t hour, uint8_t minute, uint8_t sec, uint8_t mday, uint8_t wday, uint8_t month, uint16_t yr);

	/* Print */
	void print_debug();

protected:
	int8_t getStatus();
	int8_t readMultiRegs(uint8_t *data, uint16_t len);
	int8_t writeMultiRegs(uint8_t *data, uint16_t len);
	int calculate_weekday(int year, int month, int day);
	uint32_t convertDatetimeToTimestamp(const struct tm &dateTime);

private:
	uint8_t decToBcd(uint8_t val);
	uint8_t bcdToDec(uint8_t val);
	/* time variables */
	struct tm dateTime;
};

extern PCF85063A rtc_PCF85063A;

#endif