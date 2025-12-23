#include "ADS8361.h"

#include "spi.h"
#include "gpio.h"
#include <math.h>

uint16_t spi_tx_buf[2] = {0x0000, 0x0000};
uint16_t spi_rx_buf[2];
int16_t Msin;
int16_t Mcos;
int16_t Ssin;
int16_t Scos;
uint8_t CH;
int16_t data;
uint8_t once = 1;

float N = 16;
float M_deg;
float S_deg;
float last_deg;
float deg;
int16_t k;

void ADS8631_GetRawData(void)
{
    HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)spi_tx_buf, (uint8_t *)spi_rx_buf, 2);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        CH = (uint8_t)(spi_rx_buf[0] >> 14);
        data = (spi_rx_buf[0] << 2) | (spi_rx_buf[1] >> 14);

        switch (CH)
        {
        case 0:
            Msin = data;
            break;
        case 1:
            Mcos = data;
            break;
        case 2:
            Ssin = data;
            break;
        case 3:

            Scos = data;
            M_deg = atan2f(Msin, Mcos);
            S_deg = atan2f(Ssin, Scos);
            k = (int16_t)roundf((N * S_deg - M_deg) / PI2);
            deg = (S_deg + PI2 * k) / N;
            deg = fmodf(deg, PI2);
            if (deg < 0)
                deg += PI2;
            if (once)
            {
                once = 0;
                last_deg = deg;
            }
            if ((deg - last_deg) >= PI2 / N)
                deg -= PI2 / N;
            else if ((deg - last_deg) < (-PI2 / N))
                deg += PI2 / N;
            last_deg = deg;
            break;
        default:
            break;
        }
        ADS8631_GetRawData();
    }
}