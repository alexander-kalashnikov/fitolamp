#ifndef __RTC_H_
#define __RTC_H_

#define INIT_SECONDS 00
#define INIT_MINUTES 23
#define INIT_HOURS 1
#define INIT_DAY 4
#define INIT_MONTH 7
#define INIT_YEAR 19

void EnableRTCProtection(void);
void DisableRTCProtection(void);
void InitializeRTC(void);
uint8_t IsHoursBetweenNow(uint8_t min, uint8_t max);
void SetTime(uint8_t seconds, uint8_t minutes, uint8_t hours);
void GetTime(uint8_t * seconds, uint8_t * minutes, uint8_t * hours);
void SetAlarms(uint8_t start_hour, uint8_t start_minutes, uint8_t end_hour, uint8_t end_minutes);
void GetAlarms(uint8_t * start_hour, uint8_t * start_minutes, uint8_t * end_hour, uint8_t * end_minutes);
void ClearRTCProtection(void);

#endif
