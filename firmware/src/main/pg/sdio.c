/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if defined(USE_SDCARD_SDIO)

//#include "drivers/io.h"
#include "drivers/sdio.h"
//#include "pg/pg_ids.h"
#include "pg/sdio.h"

#ifndef SDCARD_SDIO_DMA_OPT
#define SDCARD_SDIO_DMA_OPT (DMA_OPT_UNUSED)
#endif

#ifndef SDIO_DEVICE
#define SDIO_DEVICE SDIOINVALID
#endif

#ifndef SDIO_USE_4BIT
#define SDIO_USE_4BIT false
#endif
#ifndef SDIO_CK_PIN
#define SDIO_CK_PIN NONE
#endif
#ifndef SDIO_CMD_PIN
#define SDIO_CMD_PIN NONE
#endif
#ifndef SDIO_D0_PIN
#define SDIO_D0_PIN NONE
#endif
#ifndef SDIO_D1_PIN
#define SDIO_D1_PIN NONE
#endif
#ifndef SDIO_D2_PIN
#define SDIO_D2_PIN NONE
#endif
#ifndef SDIO_D3_PIN
#define SDIO_D3_PIN NONE
#endif

sdioConfig_t sdioConfig;

void sdioConfig_Init(void)
{
	sdioConfig.clockBypass = 0;
	sdioConfig.useCache = 0;
	sdioConfig.use4BitWidth = SDIO_USE_4BIT;
	//sdioConfig.dmaopt = SDCARD_SDIO_DMA_OPT;
	//sdioConfig.device = SDIO_DEV_TO_CFG(SDIO_DEVICE);
}

#if ENABLE_SDIO_PIN_CONFIG

sdioPinConfig_t sdioPinConfig;

void sdioPinConfig_Init(void)
{
	sdioPinConfig.CKPin = IO_TAG(SDIO_CK_PIN);
	sdioPinConfig.CMDPin = IO_TAG(SDIO_CMD_PIN);
	sdioPinConfig.D0Pin = IO_TAG(SDIO_D0_PIN);
	sdioPinConfig.D1Pin = IO_TAG(SDIO_D1_PIN);
	sdioPinConfig.D2Pin = IO_TAG(SDIO_D2_PIN);
	sdioPinConfig.D3Pin = IO_TAG(SDIO_D3_PIN);
}
#endif

#endif
