#include "pcf85063a.h"

#include "app.h"
#include "app_dbg.h"
#include "xprintf.h"
#include "io_cfg.h"
#include "sys_ctrl.h"

#define SL_ERROR (0x00)
#define SL_OK 	(0x01)

PCF85063A rtc_PCF85063A;

// INIT
void PCF85063A::init() {
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(PCF85063A_I2C_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(PCF85063A_I2C_CLK, ENABLE);

	GPIO_PinAFConfig(PCF85063A_I2C_GPIO_PORT, PCF85063A_I2C_SCL_SOURCE, PCF85063A_I2C_AF);
	GPIO_PinAFConfig(PCF85063A_I2C_GPIO_PORT, PCF85063A_I2C_SDA_SOURCE, PCF85063A_I2C_AF);

	GPIO_InitStructure.GPIO_Pin	  = PCF85063A_I2C_SCL_PIN | PCF85063A_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(PCF85063A_I2C_GPIO_PORT, &GPIO_InitStructure);

	I2C_DeInit(PCF85063A_I2C);
	I2C_InitStructure.I2C_ClockSpeed		  = 100000;
	I2C_InitStructure.I2C_Mode				  = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle			  = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1		  = 0x00;
	I2C_InitStructure.I2C_Ack				  = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(PCF85063A_I2C, &I2C_InitStructure);

	I2C_Cmd(PCF85063A_I2C, ENABLE);

	if (getStatus() == SL_ERROR) {
		// APP_PRINT(NULL_TAG, "[ERR] start rtc address: 0X%X failed\n", I2C_ADDR);
	}
}

int8_t PCF85063A::getStatus() {
	uint32_t PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;

	/*!< Clear the PCF85063A_I2C AF flag */
	I2C_ClearFlag(PCF85063A_I2C, I2C_FLAG_AF);

	/*!< Enable PCF85063A_I2C acknowledgement if it is already disabled by other function */
	I2C_AcknowledgeConfig(PCF85063A_I2C, ENABLE);

	/*---------------------------- Transmission Phase ---------------------------*/

	/*!< Send PCF85063A_I2C START condition */
	I2C_GenerateSTART(PCF85063A_I2C, ENABLE);

	/*!< Test on PCF85063A_I2C EV5 and clear it */
	while ((!I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_SB)) && PCF85063A_Timeout) /*!< EV5 */
	{
		PCF85063A_Timeout--;
	}
	if (PCF85063A_Timeout == 0) {
		// APP_DBG(DBG_TAG, "RTC time out 1\n");
		return SL_ERROR;
	}

	PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;

	/*!< Send STLM75 slave address for write */
	I2C_Send7bitAddress(PCF85063A_I2C, (I2C_ADDR << 1), I2C_Direction_Transmitter);

	while ((!I2C_CheckEvent(PCF85063A_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && PCF85063A_Timeout) /* EV6 */
	{
		PCF85063A_Timeout--;
	}

	if ((I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_AF) != 0x00) || (PCF85063A_Timeout == 0)) {
		// APP_DBG(DBG_TAG, "RTC time out 2: %d\n", PCF85063A_Timeout);
		return SL_ERROR;
	}
	else {
		return SL_OK;
	}
}

int8_t PCF85063A::readMultiRegs(uint8_t *data, uint16_t len) {
	/* Enable the I2C peripheral */
	I2C_GenerateSTART(PCF85063A_I2C, ENABLE);

	/* Test on SB Flag */
	uint32_t PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
	while (!I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_SB)) {
		if ((PCF85063A_Timeout--) == 0)
			return SL_ERROR;
	}

	/* Send PCF85063A address for read */
	I2C_Send7bitAddress(PCF85063A_I2C, (I2C_ADDR << 1), I2C_Direction_Receiver);

	/* Test on ADDR Flag */
	PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(PCF85063A_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if ((PCF85063A_Timeout--) == 0)
			return SL_ERROR;
	}

	while (len > 0) {
		if (len < 2) {
			/*!< Disable Acknowledgement */
			I2C_AcknowledgeConfig(PCF85063A_I2C, DISABLE);

			/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
			(void)PCF85063A_I2C->SR2;

			/*!< Send STOP Condition */
			I2C_GenerateSTOP(PCF85063A_I2C, ENABLE);
		}

		/* Wait for the byte to be received */
		PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
		while (I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_RXNE) == RESET) {
			if ((PCF85063A_Timeout--) == 0)
				return SL_ERROR;
		}

		/*!< Read the byte received from the EEPROM */
		*data = I2C_ReceiveData(PCF85063A_I2C);
		data++;
		len--;
	}

	/*!< Re-Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(PCF85063A_I2C, ENABLE);

	return SL_OK;
}

int8_t PCF85063A::writeMultiRegs(uint8_t *data, uint16_t len) {
	/* Test on BUSY Flag */
	uint32_t PCF85063A_Timeout = PCF85063A_LONG_TIMEOUT;
	while (I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_BUSY)) {
		if ((PCF85063A_Timeout--) == 0)
			return SL_ERROR;
	}

	/* Enable the I2C peripheral */
	I2C_GenerateSTART(PCF85063A_I2C, ENABLE);

	/* Test on SB Flag */
	PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
	while (I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_SB) == RESET) {
		if ((PCF85063A_Timeout--) == 0)
			return SL_ERROR;
	}

	/* Transmit the slave address and enable writing operation */
	I2C_Send7bitAddress(PCF85063A_I2C, (I2C_ADDR << 1), I2C_Direction_Transmitter);

	/* Test on ADDR Flag */
	PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(PCF85063A_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((PCF85063A_Timeout--) == 0)
			return SL_ERROR;
	}

	for (int8_t idx = 0; idx < len; idx++) {
		/* Transmit the first address for r/w operations */
		I2C_SendData(PCF85063A_I2C, data[idx]);

		/* Test on TXE FLag (data sent) */
		PCF85063A_Timeout = PCF85063A_FLAG_TIMEOUT;
		while ((!I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(PCF85063A_I2C, I2C_FLAG_BTF))) {
			if ((PCF85063A_Timeout--) == 0)
				return SL_ERROR;
		}
	}

	/* Send STOP Condition */
	I2C_GenerateSTOP(PCF85063A_I2C, ENABLE);

	return SL_OK;
}

int PCF85063A::calculate_weekday(int year, int month, int day) {
	if (month < 3) {
		month += 12;
		year--;
	}

	int k = year % 100;
	int j = year / 100;

	// Zeller's Congruence formula
	int weekday = (day + 13 * (month + 1) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;

	// Convert to 0-indexed weekdays (0: Saturday, 1: Sunday, ..., 6: Friday)
	return (weekday + 6) % 7;
}

uint32_t PCF85063A::convertDatetimeToTimestamp(const struct tm &dateTime) {
	struct tm dateTimeRev = dateTime;
	dateTimeRev.tm_year += 70;
	return mktime(&dateTimeRev);	// time GMT+7
}

// PUBLIC
void PCF85063A::setDateTime(uint8_t hour, uint8_t minute, uint8_t second, uint8_t mday, uint8_t wday, uint8_t month, uint16_t yr) {
	uint32_t startTime = sys_ctrl_millis();
	dateTime.tm_sec	   = second;
	dateTime.tm_min	   = minute;
	dateTime.tm_hour   = hour;
	dateTime.tm_mday   = mday;
	dateTime.tm_wday   = wday;
	dateTime.tm_mon	   = month;
	dateTime.tm_year   = yr;
	yr				   = yr - 1970;	// convert to RTC year format 0-99

	uint8_t wData[8];
	wData[0]	 = RTC_SECOND_ADDR;
	wData[1]	 = decToBcd(dateTime.tm_sec);	// 0-59
	wData[2]	 = decToBcd(dateTime.tm_min);	// 0-59
	wData[3]	 = decToBcd(dateTime.tm_hour);	// 0-23
	wData[4]	 = decToBcd(dateTime.tm_mday);	// 1-31
	wData[5]	 = dateTime.tm_wday;			// 0-6
	wData[6]	 = decToBcd(dateTime.tm_mon);	// 1-12
	wData[7]	 = decToBcd(yr);
	int8_t errW1 = writeMultiRegs(wData, sizeof(wData));
	xprintf("[return write %d] - set hour: %d, minute: %d, second: %d, weekday: %d, day: %d, month: "
			"%d, year: %d\n", errW1, dateTime.tm_hour, dateTime.tm_min, dateTime.tm_sec, dateTime.tm_wday, mday, month, dateTime.tm_year);
}

void PCF85063A::setHour(uint8_t hour) {
	readDateTime();
	dateTime.tm_hour = hour;
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setMinute(uint8_t minute) {
	readDateTime();
	dateTime.tm_min = minute;
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setMonthDay(uint8_t mday) {
	readDateTime();
	dateTime.tm_mday = mday;
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setWeekDay(uint8_t wday) {
	readDateTime();
	dateTime.tm_wday = wday;
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setMonth(uint8_t month) {
	readDateTime();
	dateTime.tm_mon = month;
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setYear(uint16_t year) {
	readDateTime();
	dateTime.tm_year = (year - 1900);
	setDateTime(	dateTime.tm_hour, \
					dateTime.tm_min, \
					dateTime.tm_sec, \
					dateTime.tm_mday, \
					dateTime.tm_wday, \
					dateTime.tm_mon, \
					dateTime.tm_year + 1900); 
}

void PCF85063A::setTimestamp(long long timestamp) {
	xprintf("\nTimestamp = %lld", timestamp);
	time_t t = (time_t)timestamp;
	struct tm* tm_info = localtime(&t);

	xprintf("year%d", tm_info->tm_year+1900);

	/* Setup time to pcf85063a */
	rtc_PCF85063A.setDateTime(	tm_info->tm_hour, \
								tm_info->tm_min, \
								tm_info->tm_sec, \
								tm_info->tm_mday, \
								tm_info->tm_wday, \
								tm_info->tm_mon, \
								tm_info->tm_year+1900);
}

uint32_t PCF85063A::readDateTime() {
	uint8_t cmd = RTC_SECOND_ADDR;
	uint8_t rData[7];
	int8_t errWriteCode = writeMultiRegs(&cmd, 1);
	int8_t errReadCode	= readMultiRegs(rData, sizeof(rData));

	dateTime.tm_sec	 = bcdToDec(rData[0] & 0x7F);	 // ignore bit 7
	dateTime.tm_min	 = bcdToDec(rData[1] & 0x7F);
	dateTime.tm_hour = bcdToDec(rData[2] & 0x3F);	 // ignore bits 7 & 6
	dateTime.tm_mday = bcdToDec(rData[3] & 0x3F);
	dateTime.tm_wday = rData[4] & 0x07;				 // ignore bits 7,6,5,4 & 3
	dateTime.tm_mon	 = bcdToDec(rData[5] & 0x1F);	 // ignore bits 7,6 & 5
	dateTime.tm_year = bcdToDec(rData[6]) + 70; 	 // Datatime start in 1900 but rtc start in 1970 => +70

	return convertDatetimeToTimestamp(dateTime);	// time GMT+7
}

uint8_t PCF85063A::getSecond() {
	readDateTime();
	return dateTime.tm_sec; 
}

uint8_t PCF85063A::getMinute() {
	readDateTime();
	return dateTime.tm_min;
}

uint8_t PCF85063A::getHour() {
	readDateTime();
	return dateTime.tm_hour;
}

uint8_t PCF85063A::getHourLocal() {
	readDateTime();
	return dateTime.tm_hour + TIME_ZONE;
}

uint8_t PCF85063A::getDay() {
	readDateTime();
	return dateTime.tm_mday;
}

uint8_t PCF85063A::getWeekday() {
	readDateTime();
	return dateTime.tm_wday;
}

uint8_t PCF85063A::getMonth() {
	readDateTime();
	return dateTime.tm_mon;
}

uint16_t PCF85063A::getYear() {
	readDateTime();
	return dateTime.tm_year;
}


char* PCF85063A::getMonthDay_c() {
	const char* daysOfMonth[] = {
		"1st",	"2nd",	"3rd",	"4th",	"5th",	"6th",	"7th",
		"8th",	"9th",	"10th",	"11th",	"12th",	"13th",	"14th",
		"15th",	"16th",	"17th",	"18th",	"19th",	"20th",	"21st", 
		"22nd",	"23rd",	"24th",	"25th",	"26th",	"27th",	"28th",
		"29th",	"30th",	"31st"};
	readDateTime();
	return (char*)daysOfMonth[dateTime.tm_mday];
}

char* PCF85063A::getWeekDay_c() {
	const char* day_abbr[] = {
		"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
	};
	readDateTime();
	return (char*)day_abbr[dateTime.tm_wday];
}

char* PCF85063A::getMonth_c() {
	const char* month_abbr[] = {
		"Jan", "Feb", "Mar", "Apr",
		"May", "Jun", "Jul", "Aug",
		"Sep", "Oct", "Nov", "Dec"
	};
	readDateTime();
	return (char*)month_abbr[dateTime.tm_mon];
}


void PCF85063A::reset() {// datasheet 8.2.1.3.
	uint8_t wData[2];
	wData[0] = RTC_CTRL_1;
	wData[1] = 0x58;
	writeMultiRegs(wData, sizeof(wData));
}

void PCF85063A::clearStopFlag() {
	uint8_t wData[2];
	wData[0] = RTC_CTRL_1;
	wData[1] = 0x48;
	writeMultiRegs(wData, sizeof(wData));
}

void PCF85063A::setMode12or24Hour(uint8_t mode) {
	if (mode == RTC_MODE_12) {	  // set bit 1
		uint8_t wData[2];
		wData[0] = RTC_CTRL_1;
		wData[1] = 0x58 | (1 << 1);
		writeMultiRegs(wData, sizeof(wData));
	}
	else if (mode == RTC_MODE_24) {	   // clear bit 1, default is 24
		uint8_t wData[2];
		wData[0] = RTC_CTRL_1;
		wData[1] = 0x58 & ~(1 << 1);
		writeMultiRegs(wData, sizeof(wData));
	}
	else {}
		// APP_PRINT(NULL_TAG, "[ERR] mode invalid\n");
}

// PRIVATE
uint8_t PCF85063A::decToBcd(uint8_t val) {
	return ((val / 10 * 16) + (val % 10));
}

uint8_t PCF85063A::bcdToDec(uint8_t val) {
	return ((val / 16 * 10) + (val % 16));
}

void PCF85063A::print_debug() {
	readDateTime();
	xprintf("\n%dh-%dm-%ds : %s-%s-%s-%d", getHourLocal(), getMinute(), getSecond(), getWeekDay_c(), getMonthDay_c(), getMonth_c(), getYear());
}