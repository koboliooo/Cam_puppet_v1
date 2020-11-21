// https://qastack.fr/electronics/206113/how-do-i-use-the-printf-function-on-stm32
// https://electronics.stackexchange.com/questions/204996/stm32-st-link-cannot-connect-to-mcu-after-successful-programming
// example basic : https://github.com/miniwinwm/BluePillDemo
// vendor error :
// -> modify cfg to match chinese vendor id JElli.1 https://community.st.com/s/question/0D50X0000BUjpxv/error-in-initializing-stlink-device-reason-18-could-not-verify-st-device-abort-connection

#include "drv_trace.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "stm32f1xx_hal_uart.h"

extern UART_HandleTypeDef huart1;

static void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}
