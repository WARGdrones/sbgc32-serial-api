/*	____________________________________________________________________
 *
 *	This is an example sketch for Arduino, that shows how to control
 *	SimpleBGC-driven gimbal via Serial API. API specs are available at
 *	http://www.basecamelectronics.com/serialapi/
 *	____________________________________________________________________
 */

#include <../../../../drivers/ArduinoDriver/driver_Arduino.h>

#include <../../../../sources/adjvar/adjvar.h>
#include <../../../../sources/gimbalControl/gimbalControl.h>
#include <../../../../sources/profiles/profiles.h>
#include <../../../../sources/realtime/realtime.h>
#include <../../../../sources/service/service.h>


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
/*   					Global Software Objects  					  */
/* __________________________________________________________________ */

            GeneralSBGC_t 			SBGC_1;

static 		ConfirmationState_t		Confirm;

static 		Control_t    			Control;
static 		ControlConfig_t    		ControlConfig;

static 		BoardInfo_t         	BoardInfo;
static 		BoardInfo3_t        	BoardInfo3;
static 		MainParams3_t       	MainParams3;
static 		MainParamsExt_t     	MainParamsExt;
static 		MainParamsExt2_t    	MainParamsExt2;
static 		MainParamsExt3_t   		MainParamsExt3;

static		RealTimeDataCustom_t	RealTimeDataCustom;
static		RealTimeData_t			RealTimeData;

static		AdjVarsGeneral_t		AdjVarsGeneral [3];

#if (SBGC_DEBUG_MODE)
	extern const
			AdjVarsDebugInfo_t		AdjVarsDebugInfoArray [];
#endif

static		DataStreamInterval_t	DataStreamInterval;

static		BeeperSettings_t		BeeperSettings;


static		ui8	DataStreamBuff [20];


TxRxStatus_t PrintBoardParameters (Profile_t slot);
TxRxStatus_t SBGC32_DemoControl (void);
void PrintDataStream (ui8 *pBuff);

/*  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */

void setup()
{
	/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
	/*                         Initialization                         */
	/* ______________________________________________________________ */

	/*  - - - - - - - - - - - - Hardware Init - - - - - - - - - - - - */

	/* Serial initialization */
    SBGC_SERIAL_PORT.begin(SBGC_SERIAL_SPEED);
	DEBUG_SERIAL_PORT.begin(DEBUG_SERIAL_SPEED);

	pinMode(SERIAL2_RX_PIN, INPUT_PULLUP);


	/*  - - - - - - - - SBGC Hardware-Software Init - - - - - - - - - */

  	/* High Layer Init */
  	SBGC32_DefaultInit(&SBGC_1, UartTransmitData, UartReceiveByte, GetAvailableBytes,
  			  		   UartTransmitDebugData, GetTimeMs, SBGC_PROTOCOL_V2);


	/* - - - - - - - - - High Layer Software Init - - - - - - - - - - */

	/* Control Configurations */
	ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].angleLPF = 6;
    ControlConfig.AxisCC[YAW].angleLPF = 7;

    ControlConfig.AxisCC[ROLL].angleLPF = 6;
    ControlConfig.AxisCC[PITCH].speedLPF = 6;
    ControlConfig.AxisCC[YAW].speedLPF = 7;
    ControlConfig.flags = RTCCF_CONTROL_CONFIG_FLAG_NO_CONFIRM;

    Control.controlMode[ROLL] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;
    Control.controlMode[PITCH] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;
    Control.controlMode[YAW] = CtrlM_MODE_ANGLE | CtrlF_CONTROL_FLAG_TARGET_PRECISE;

    Control.AxisC[ROLL].angle = 0;
    Control.AxisC[PITCH].angle = 0;
    Control.AxisC[YAW].angle = 0;

    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(30);
    Control.AxisC[YAW].speed = SPEED_TO_VALUE(30);

	/* Data Stream Configurations */
	DataStreamInterval.cmdID = CMD_REALTIME_DATA_CUSTOM;
	DataStreamInterval.intervalMs = 1000;
	DataStreamInterval.syncToData = STD_SYNC_OFF;

	/* For more information see the SBGC32_RequestRealTimeDataCustom function.
	   Total packets length = 20 bytes:
	   ui16 timestampMs						 i16 [3]				i16 [3]			i16 [3] */
	ui32 DataStreamIntervalConfig = RTDCF_STATOR_ROTOR_ANGLE | RTDCF_GYRO_DATA | RTDCF_ACC_DATA;
	memcpy(DataStreamInterval.config, &DataStreamIntervalConfig, sizeof(DataStreamIntervalConfig));

	/* Adj Vars Setting */
	#if (SBGC_DEBUG_MODE)
		/* Note: If your microprocessor has little size of RAM,
				 initialize these variables manually,
				 without AdjVarsDebugInfoArray [] */
		InitAdjVar(&AdjVarsGeneral[0], &AdjVarsDebugInfoArray[ADJ_VAL_ACC_LIMITER_ROLL]);
		InitAdjVar(&AdjVarsGeneral[1], &AdjVarsDebugInfoArray[ADJ_VAL_ACC_LIMITER_PITCH]);
		InitAdjVar(&AdjVarsGeneral[2], &AdjVarsDebugInfoArray[ADJ_VAL_ACC_LIMITER_YAW]);
		
	#else
		
		AdjVarsGeneral[0].ID = ADJ_VAL_ACC_LIMITER_ROLL;
		AdjVarsGeneral[0].minValue = 0;
		AdjVarsGeneral[0].maxValue = 1275;
		AdjVarsGeneral[0].varType = _UNSIGNED_SHORT_;

		AdjVarsGeneral[1].ID = ADJ_VAL_ACC_LIMITER_PITCH;
		AdjVarsGeneral[1].minValue = 0;
		AdjVarsGeneral[1].maxValue = 1275;
		AdjVarsGeneral[1].varType = _UNSIGNED_SHORT_;

		AdjVarsGeneral[2].ID = ADJ_VAL_ACC_LIMITER_YAW;
		AdjVarsGeneral[2].minValue = 0;
		AdjVarsGeneral[2].maxValue = 1275;
		AdjVarsGeneral[2].varType = _UNSIGNED_SHORT_;
		
	#endif


	/* - - - - - - - - - - - - Program Launch - - - - - - - - - - - - */

    /* SBGC32_Reset(&SBGC_1, RF_RESET_WITH_RESTORING_STATES, 500);
    SBGC32_CheckConfirmation(&SBGC_1, &Confirm, CMD_RESET);
    DELAY_MS_(5000); */

	PrintBoardParameters(P_CURRENT_PROFILE);

    SBGC32_ControlConfig(&SBGC_1, &ControlConfig, &Confirm);
    SBGC32_DemoControl();

	SBGC32_RequestDataStream(&SBGC_1, &DataStreamInterval, &Confirm);

	/*  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
}

void loop()
{
	/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
	/* 						  Start Worker Cycle					  */
	/* ______________________________________________________________ */

	SBGC32_ParseDataStream(&SBGC_1, DataStreamBuff, (SBGC_Command_t)DataStreamInterval.cmdID);
	PrintDataStream(DataStreamBuff);

	/*  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
}


TxRxStatus_t PrintBoardParameters (Profile_t slot)
{
    SBGC32_ReadBoardInfo(&SBGC_1, &BoardInfo, 0);
    SBGC32_ReadBoardInfo3(&SBGC_1, &BoardInfo3);

    SBGC32_ReadParams3(&SBGC_1, &MainParams3, slot);
    SBGC32_ReadParamsExt(&SBGC_1, &MainParamsExt, slot);
    SBGC32_ReadParamsExt2(&SBGC_1, &MainParamsExt2, slot);
    SBGC32_ReadParamsExt3(&SBGC_1, &MainParamsExt3, slot);

    SBGC32_ReadRealTimeData4(&SBGC_1, &RealTimeData);

    char boardVersionStr [4];
    char firmwareVersionStr [7];

    FormatBoardVersion(BoardInfo.boardVer, boardVersionStr);
    FormatFirmwareVersion(BoardInfo.firmwareVer, firmwareVersionStr);

    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)"Board Version: "));
    PrintMessage(&SBGC_1, TEXT_SIZE_(boardVersionStr));
    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)" \n"));
    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)"Firmware Version: "));
    PrintMessage(&SBGC_1, TEXT_SIZE_(firmwareVersionStr));
    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)" \n"));

    PrintStructElement(&SBGC_1, &BoardInfo3.flashSize, "Flash Size =", _UNSIGNED_CHAR_);

    PrintStructElement(&SBGC_1, &MainParams3.profileID, "Current profile #", _UNSIGNED_CHAR_);  // Note: -1
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[ROLL].p, "Roll P =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[ROLL].i, "Roll I =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[ROLL].d, "Roll D =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[PITCH].p, "Pitch P =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[PITCH].i, "Pitch I =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[PITCH].d, "Pitch D =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[YAW].p, "Yaw P =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[YAW].i, "Yaw I =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisCMP3[YAW].d, "Yaw D =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.ACC_LimiterAll, "ACC Limiter all = ", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisRC_MP3[ROLL].RC_MaxAngle, "RC Max Angle =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &MainParams3.AxisRC_MP3[YAW].RC_MinAngle, "RC Min Angle =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapROLL, "RC Map Roll =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapPITCH, "RC Map Pitch =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapYAW, "RC Map Yaw =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapCmd, "RC Map Cmd =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapFC_ROLL, "RC Map FC Roll =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParams3.RC_MapFC_PITCH, "RC Map FC Pitch =", _UNSIGNED_CHAR_);

    PrintStructElement(&SBGC_1, &MainParamsExt.LPF_Freq[ROLL], "LPF Frequency Roll =", _UNSIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &MainParamsExt.LPF_Freq[PITCH], "LPF Frequency Pitch =", _UNSIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &MainParamsExt.LPF_Freq[YAW], "LPF Frequency Yaw =", _UNSIGNED_SHORT_);

    PrintStructElement(&SBGC_1, &MainParamsExt2.frameIMU_LPF_Freq, "Frame IMU LPF Freq =", _UNSIGNED_CHAR_);
    PrintStructElement(&SBGC_1, &MainParamsExt2.timelapseTime, "Timelapse Time =", _UNSIGNED_SHORT_);

    PrintStructElement(&SBGC_1, &MainParamsExt3.motorStartupDelay, "Motor Startup DELAY_MS_ =", _UNSIGNED_SHORT_);

    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)" \n"));
	
	PrintStructElement(&SBGC_1, &RealTimeData.AxisRTD[ROLL].ACC_Data, "ACC Roll =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeData.AxisRTD[PITCH].ACC_Data, "ACC Pitch =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeData.AxisRTD[YAW].ACC_Data, "ACC Yaw =", _SIGNED_SHORT_);

    PrintStructElement(&SBGC_1, &RealTimeData.frameCamAngle[ROLL], "Roll Current Angle =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &RealTimeData.frameCamAngle[PITCH], "Pitch Current Angle =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &RealTimeData.frameCamAngle[YAW], "Yaw Current Angle =", _SIGNED_SHORT_);

    PrintStructElement(&SBGC_1, &RealTimeData.IMU_Temperature, "IMU Temperature =", _SIGNED_CHAR_);

    return SBGC_1._ParserCurrentStatus;
}


TxRxStatus_t SBGC32_DemoControl (void)
{
	/* Getting adjvars values */
	/* Note: AdjVarsGeneral.ID fields are already filled */
	SBGC32_GetAdjVarValues(&SBGC_1, AdjVarsGeneral, countof_(AdjVarsGeneral));

	/* Run the Demonstration Cycle */
	for (ui8 i = 0; i < 4; i++)
	{
		#if (SBGC_DEBUG_MODE)
			/* Printing */
			for (ui8 k = 0; k < countof_(AdjVarsGeneral); k++)
				PrintStructElement(&SBGC_1, &AdjVarsGeneral[k].value, AdjVarsGeneral[k].name, AdjVarsGeneral[k].varType);
		#endif

		Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(50);
		Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(-20);
		SBGC32_Control(&SBGC_1, &Control);
		DELAY_MS_(5000);

		Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(20);
		SBGC32_Control(&SBGC_1, &Control);
		DELAY_MS_(5000);

		Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(-50);
		SBGC32_Control(&SBGC_1, &Control);
		DELAY_MS_(5000);

		Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(-20);
		SBGC32_Control(&SBGC_1, &Control);
		DELAY_MS_(5000);

		Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(0);
		Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(0);
		SBGC32_Control(&SBGC_1, &Control);
		DELAY_MS_(5000);

		BeeperSettings.mode = BM_BEEPER_MODE_COMPLETE;
		SBGC32_PlayBeeper(&SBGC_1, &BeeperSettings);

		/* Adjustable Variables Re-Setting */
		for (ui8 k = 0; k < countof_(AdjVarsGeneral); k++)
			/* Toggle Min : Max adjvars contrast */
			EditAdjVarValue(&AdjVarsGeneral[k], ((i % 2 == 0) ? AdjVarsGeneral[k].maxValue : AdjVarsGeneral[k].minValue));

		SBGC32_SetAdjVarValues(&SBGC_1, AdjVarsGeneral, countof_(AdjVarsGeneral), &Confirm);
	}

	/* Saving all changed adjustable variables to EEPROM */
	/* SBGC32_SaveAllActiveAdjVarsToEEPROM(&SBGC_1, &Confirm);

	if (Confirm.cmdID == CMD_SAVE_PARAMS_3)
		for (ui8 i = 0; i < countof_(AdjVarsGeneral); i++)
			if (AdjVarsGeneral[i].saveFlag != SAVED)
				AdjVarsGeneral[i].saveFlag = SAVED; */

	/* or SBGC32_SaveAdjVarsToEEPROM(&SBGC_1, AdjVarsGeneral, countof_(AdjVarsGeneral), &Confirm); */

    return SBGC_1._ParserCurrentStatus;
}


void PrintDataStream (ui8 *pBuff)
{
	/* Preparing */
	ui8 BuffRPx = 2;  // ui16 timestampMs offset

	BuffRPx += ConvertWithPM(RealTimeDataCustom.frameCamAngle, &pBuff[BuffRPx],
							sizeof(RealTimeDataCustom.targetAngles), PM_DEFAULT_16BIT);
	BuffRPx += ConvertWithPM(RealTimeDataCustom.gyroData, &pBuff[BuffRPx],
							sizeof(RealTimeDataCustom.gyroData), PM_DEFAULT_16BIT);
	BuffRPx += ConvertWithPM(RealTimeDataCustom.ACC_Data, &pBuff[BuffRPx],
							sizeof(RealTimeDataCustom.ACC_Data), PM_DEFAULT_16BIT);

	/* Printing */
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[ROLL], "Frame Camera Angle Roll =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[PITCH], "Frame Camera Angle Pitch =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[YAW], "Frame Camera Angle Yaw =", _SIGNED_SHORT_);

	PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[ROLL], "Gyro Roll =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[PITCH], "Gyro Pitch =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[YAW], "Gyro Yaw =", _SIGNED_SHORT_);

	PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[ROLL], "ACC Roll =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[PITCH], "ACC Pitch =", _SIGNED_SHORT_);
	PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[YAW], "ACC Yaw =", _SIGNED_SHORT_);

	PrintMessage(&SBGC_1, TEXT_SIZE_((char*)"__________________________\n\n"));
}
