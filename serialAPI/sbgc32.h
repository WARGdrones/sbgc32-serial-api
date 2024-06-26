/** ____________________________________________________________________
 *
 *	SBGC32 Serial API Library v1.1
 *
 *	@file		sbgc32.h
 *
 *	@brief		Assembly header file of the Library
 *	____________________________________________________________________
 *
 *	@attention	<h3><center>
 *				Copyright © 2023 BaseCam Electronics™.<br>
 *				All rights reserved.
 *				</center></h3>
 *
 *				<center><a href="https://www.basecamelectronics.com">
 *				www.basecamelectronics.com</a></center>
 *
 *	Licensed under the Apache License, Version 2.0 (the "License");
 *	you may not use this file except in compliance with the License.
 *	You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 *	Unless required by applicable law or agreed to in writing, software
 *	distributed under the License is distributed on an "AS IS" BASIS,
 *	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *	implied. See the License for the specific language governing
 *	permissions and limitations under the License.
 *	____________________________________________________________________
 */
/** ____________________________________________________________________
 *
 *  @defgroup	Common Common
 *		@brief	Common library block
 *
 * 	@defgroup	Sources Sources
 *		@brief	Sources Title Module
 *
 *	@defgroup	Drivers Drivers
 *		@brief	Drivers Title Module
 *  ____________________________________________________________________
 */

#ifndef		_SBGC32_H_
#define		_SBGC32_H_

#ifdef		__cplusplus
extern		"C" {
#endif
/*  = = = = = = = = = = = = = = = = = = = = = = = */

/* Specific configurations. Originate from the <serialAPI_ConfigTemplate.h> file */
#include	"serialAPI_Config.h"

/* Auxiliary code */
#include	"adjunct.h"

/* Kernel code */
#include	"core.h"

/* Enable used modules */
#if (SBGC_ADJVAR_MODULE)
	#include		"adjvar.h"
#endif

#if (SBGC_CALIB_MODULE)
	#include		"calib.h"
#endif

#if (SBGC_CONTROL_MODULE)
	#include		"gimbalControl.h"
#endif

#if (SBGC_EEPROM_MODULE)
	#include		"eeprom.h"
#endif

#if (SBGC_IMU_MODULE)
	#include		"imu.h"
#endif

#if (SBGC_PROFILES_MODULE)
	#include		"profiles.h"
#endif

#if (SBGC_REALTIME_MODULE)
	#include		"realtime.h"
#endif

#if (SBGC_SERVICE_MODULE)
	#include		"service.h"
#endif

/* Enable driver modules */
#if (SBGC_USE_LINUX_DRIVER)
	#include		"driver_Linux.h"
#endif

#if (SBGC_USE_STM32_DRIVER)
	#include		"driver_STM32.h"
#endif

/**	@addtogroup	Common
 *	@{
 */
TxRxStatus_t SBGC32_Init (GeneralSBGC_t *generalSBGC);
TxRxStatus_t SBGC32_Init_Custom (GeneralSBGC_t *generalSBGC, const char *dev, speed_t baud);

/**	@}
 */

#if (__DOXYGEN__)
	/**	@addtogroup	Configurations
	 *	@{
	 */
	/** General serial connection. See configuration file and look for this for your hardware */
	#define	SBGC_SERIAL_PORT

	/** Debug serial connection. See configuration file and look for this for your hardware */
	#define	DEBUG_SERIAL_PORT
	/**	@}
	 */
#endif


/*  = = = = = = = = = = = = = = = = = = = = = = = */
#ifdef 		__cplusplus
			}
#endif

/* Enable Arduino driver module */
#if (SBGC_USE_ARDUINO_DRIVER)
	 #include		"driver_Arduino.h"
#endif


/*  = = = = = = = = = = = = = = = = = = = = = = = */

#endif		/* _SBGC32_H_ */
