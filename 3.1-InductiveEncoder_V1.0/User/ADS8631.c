#include "spi.h"
#include "gpio.h"

uint16_t spi_tx_buf[2] = {0x0000, 0x0000};
uint16_t spi_rx_buf[2];

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
        
    }
}