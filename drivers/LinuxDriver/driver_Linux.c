/** ____________________________________________________________________
 *
 *	@file		driver_Linux.c
 *
 *	@brief 		Linux OS driver source file
 *	____________________________________________________________________
 *
 *	@attention	<center><h3>
 *	Copyright © 2023 BaseCam Electronics™.</h3></center>
 *	<center>All rights reserved.</center>
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

#include "driver_Linux.h"

/**	@addtogroup	LinuxDriver
 *	@{
 */
/**	@brief	Initializes the driver object from GeneralSBGC_t
 *
 *	@param	*Driver - main hardware driver object
 *	@param	__USB_ADDR - (aka *char) path to connected SBGC32 device
 */
void DriverInit(void *Driver, __USB_ADDR, speed_t baud)
{
	Driver_t *drv = (Driver_t *)Driver;

	memcpy(&drv->device, dev, strlen(dev));

	drv->devFD = open(drv->device, O_RDWR | O_NOCTTY | O_NDELAY);

	if (drv->devFD == -1)
	{
		char errorStr[] = "SBGC Driver: Device not found!\n";
		PrintDebugData(errorStr, strlen(errorStr));
		return;
	}

	struct termios portConfigurations;

	tcgetattr(drv->devFD, &portConfigurations);

	if (baud != B115200 && baud != B230400)
	{
		fprintf(stderr, "SBGC Driver: unsupported baud rate, exiting\n");
		// TODO: Is this closed if the code does not fail here? :D (I know, it's not)
		close(drv->devFD);
		drv->devFD == -1;
		return;
	}

	cfsetispeed(&portConfigurations, baud);
	cfsetospeed(&portConfigurations, baud);

	portConfigurations.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
	portConfigurations.c_cflag |= CS8 | CREAD | CLOCAL;

	portConfigurations.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

	portConfigurations.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	portConfigurations.c_oflag &= ~OPOST;

	tcsetattr(drv->devFD, TCSANOW, &portConfigurations);

	//Set low latency mode, stackoverflow.com/a/43496519
	struct serial_struct serial;
	ioctl(drv->devFD, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
	ioctl(drv->devFD, TIOCSSERIAL, &serial);
}

/**	@brief	Closes the driver object
 *
 *	@param	*Driver - main hardware driver object
 */
void DriverClose(void *Driver)
{
	Driver_t *drv = (Driver_t *)Driver;

	if (drv->devFD != -1)
	{
		close(drv->devFD);
		drv->devFD = -1;
	}
}

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *														 Timer Functions
 */
/**	@brief	Gets current system time in milliseconds
 *
 *	@param	*Driver - main hardware driver object
 *
 *	@return	Current time
 */
ui32 GetTimeMs(void *Driver)
{
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	return ((spec.tv_sec & 0x000FFFFF) * 1000) + (spec.tv_nsec / 1.0e6);
}

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *															Tx Functions
 */
/**	@brief	Sends an amount of data to serial port
 *
 *	@param	*Driver - main hardware driver object
 *	@param	*data - transferred data
 *	@param	size - size of transferred data
 *
 *	@return	0
 */
ui8 PortTransmitData(void *Driver, ui8 *data, ui16 size)
{
	Driver_t *drv = (Driver_t *)Driver;

	ui16 count = 0;
	while (count < size)
	{
		write(drv->devFD, &data[count], 1);
		count++;
	}

	return 0;
}

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *															Rx Functions
 */
/**	@brief	Returns the number of available bytes
 *
 *	@param	*Driver - main hardware driver object
 *
 *	@return	Number of available bytes
 */
ui16 GetAvailableBytes(void *Driver)
{
	Driver_t *drv = (Driver_t *)Driver;

	ui16 bytes;
	ioctl(drv->devFD, FIONREAD, &bytes);
	return bytes;
}

/**	@brief	Receives byte from serial port
 *
 *	@param	*Driver - main hardware driver object
 *	@param	*data - data buffer
 *
 *	@return	Receipt status (0 - receiving in progress | 1 - received)
 */
ui8 PortReceiveByte(void *Driver, ui8 *data)
{
	Driver_t *drv = (Driver_t *)Driver;

	if (!GetAvailableBytes(drv))
		return 1;

	read(drv->devFD, data, 1);

	return 0;
}

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
 *														 Debug Functions
 */
/**	@brief	Prints debug data
 *
 *	@param	*data - debug data
 *	@param	length - size of debug data
 */
void PrintDebugData(char *data, ui16 length)
{
	ui16 count = 0;
	while (count < length)
		printf("%c", data[count++]);
}

/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
/*					https://www.basecamelectronics.com  			  */
/* __________________________________________________________________ */
