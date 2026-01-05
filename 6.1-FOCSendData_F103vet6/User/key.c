#include "key.h"
#include "main.h"


void Key_Tick(void)
{
    static uint8_t key_state = 0;
    static uint8_t key_count = 0;

    if (HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin) == GPIO_PIN_RESET)
    {
        if (key_count < 20)
            key_count++;
        if (key_count == 20 && key_state == 0)
        {
            key_state = 1;
            Run.Runflag = !Run.Runflag;
        }
    }
    else
    {
        key_count = 0;
        key_state = 0;
    }
}
