/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

 /**************************************************************************//**
 * @file     ospi_hram_reg_access.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     12-Sep-2023
 * @brief    Utility functions to access the registers of the ISSI hram device.
 ******************************************************************************/
#include "ospi_drv.h"
#include "ospi_hram_reg_access.h"
#include <stdint.h>
#include <common/debug.h>

void ospi_setup_read_hram(ospi_cfg_t *ospi, uint32_t length)
{
    uint32_t t;



    ospi_disable(ospi);
    writel(ospi, ser, 0);



    t = (1 << DWC_SSI_CTRLR0_SPI_HE_OFFSET)
                |(SPI_OCTAL << DWC_SSI_CTRLR0_SPI_FRF_OFFSET)
                |(SPI_TMOD_RO << DWC_SSI_CTRLR0_TMOD_OFFSET)
                |(0xf << DWC_SSI_CTRLR0_DFS_OFFSET);



    writel(ospi, ctrlr0, t);



    writel(ospi, ctrlr1, length - 1);



    t = DWC_SPI_TRANS_TYPE_FRF_DEFINED                          /* OctalSPI transfer type */
                |(0xc << (DWC_SPI_CTRLR0_ADDR_L_OFFSET))        /* Address length - 48-bits */
                |(0 << DWC_SPI_CTRLR0_INST_L_OFFSET)            /* Instruction length - 0-bit */
                |(6 << DWC_SPI_CTRLR0_WAIT_CYCLES_OFFSET)       /* 6 Wait/Dummy cycles is the default value */
                |(1 << DWC_SPI_CTRLR0_SPI_DDR_EN_OFFSET)        /* SPI DDR Enable */
                |(1 << DWC_SPI_CTRLR0_SPI_RXDS_EN_OFFSET)       /* Data Strobe Enable */
                |(0 << DWC_SPI_CTRLR0_SPI_DM_EN_OFFSET)         /* SPI data mask enable */
                |(1 << DWC_SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET);  /* Enable rxds signaling during address & command phase of Hyper bus transfer */



    writel(ospi, spi_ctrlr0, t);



    ospi_enable(ospi);
}
//write 16bit data into the configuration register 0 of the ISSI device
void hyper_ram_command_write_conf_reg0(ospi_cfg_t *ospi, uint16_t data)
{
    int l = 0;
    uint32_t t;

    ospi_disable(ospi);
    writel(ospi, ser, 0);

    t = (1 << DWC_SSI_CTRLR0_SPI_HE_OFFSET)
            |(SPI_OCTAL << DWC_SSI_CTRLR0_SPI_FRF_OFFSET)
            |(SPI_TMOD_TO << DWC_SSI_CTRLR0_TMOD_OFFSET)
            |(0xf << DWC_SSI_CTRLR0_DFS_OFFSET);

    writel(ospi, ctrlr0, t);

    t = DWC_SPI_TRANS_TYPE_FRF_DEFINED                              /* OctalSPI transfer type */
            |(0xc << (DWC_SPI_CTRLR0_ADDR_L_OFFSET))                /* Address length - 48-bits */
            |(0 << DWC_SPI_CTRLR0_INST_L_OFFSET)                    /* Instruction length - 0-bit */
            |(0 << DWC_SPI_CTRLR0_WAIT_CYCLES_OFFSET)               /* 6 Wait/Dummy cycles is the default value */
            |(1 << DWC_SPI_CTRLR0_SPI_DDR_EN_OFFSET)                /* SPI DDR Enable */
            |(0 << DWC_SPI_CTRLR0_SPI_RXDS_EN_OFFSET)               /* Data Strobe Enable */
            |(0 << DWC_SPI_CTRLR0_SPI_DM_EN_OFFSET)                 /* SPI data mask enable */
            |(0 << DWC_SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET);

    writel(ospi, spi_ctrlr0, t);

    ospi_enable(ospi);

    /* Form the command to write into configuration register 0, refer to the ISSI data sheet */
    ospi->tx_buff[0] = 0x60000100;
    ospi->tx_buff[1] = 0x0;

    while (l < 2) {
        writel(ospi, datareg, ospi->tx_buff[l]);
        l++;
    }

    writel(ospi, datareg, data);

    writel(ospi, ser, 1);

    while ((readl(ospi, sr) & (SR_TF_EMPTY)) != SR_TF_EMPTY);
}

//get all the registers of the ISS hyperram device
void hyper_ram_get_regs(ospi_cfg_t *ospi)
{
	int l = 0, iter = 0;

    while (iter < 4) {
       ospi_setup_read_hram(ospi, 1);

        /* Refer to the ISSI data sheet for the following CA bytes */
        ospi->tx_buff[0] = 0xC0000000;
        ospi->tx_buff[1] = 0x00000000;
        ospi->tx_buff[2] = 0xC0000000;
        ospi->tx_buff[3] = 0x00000001;
        ospi->tx_buff[4] = 0xC0000100;
        ospi->tx_buff[5] = 0x00000000;
        ospi->tx_buff[6] = 0xC0000100;
        ospi->tx_buff[7] = 0x00000001;

        l = 0;

        while (l < 2) {
               writel(ospi, datareg, ospi->tx_buff[l + (2*iter)]);
               l++;
        }

        writel(ospi, ser, 1);

        while ((readl(ospi, sr) & (SR_TF_EMPTY)) != SR_TF_EMPTY);

        l = 0;

        while (ospi->regs->rxflr <= 0);

        while (l < 1) {
               ospi->rx_buff[l] = readl(ospi, datareg);
               l++;
        }

        writel(ospi, ser, 0);
        iter++;
    }
}

