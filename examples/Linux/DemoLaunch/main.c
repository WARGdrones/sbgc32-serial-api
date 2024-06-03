#include "driver_Linux.h"

#include "adjvar.h"
#include "gimbalControl.h"
#include "profiles.h"
#include "realtime.h"
#include "service.h"
#include <string.h> 


/* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
/*   					Global Software Objects  					  */
/* __________________________________________________________________ */

static      GeneralSBGC_t 			SBGC_1;

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
static		DataStreamInterval_t	DataStreamIntervalMotorEvent;

static		BeeperSettings_t		BeeperSettings;


static		ui8	DataStreamBuff [128];
static		ui8	DataStreamBuffMot [128];


TxRxStatus_t PrintBoardParameters (Profile_t slot);
TxRxStatus_t SBGC32_DemoControl (void);
void PrintDataStream (ui8 *pBuff);

/*  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */

int main(int argc, char **argv)
{
    /* ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ */
    /*                         Initialization                         */
    /* ______________________________________________________________ */

    /*  - - - - - - - - SBGC Hardware-Software Init - - - - - - - - - */

    /* Driver Init */
    SBGC_1.Drv = malloc(sizeof(Driver_t));
    // read serial port from command line args
    char port[200] = "/dev/ttyUSB0";
    if (argc > 1)
    {
        strcpy(port, argv[1]);
    }
    printf("Using port %s\n", port);

    DriverInit(SBGC_1.Drv, port, B230400);

    /* High Layer Init */
    SBGC32_DefaultInit(&SBGC_1, PortTransmitData, PortReceiveByte, GetAvailableBytes,
                       PrintDebugData, GetTimeMs, SBGC_PROTOCOL_V2);


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

    Control.AxisC[PITCH].speed = SPEED_TO_VALUE(50);
    Control.AxisC[YAW].speed = SPEED_TO_VALUE(50);

    /* Data Stream Configurations */
    DataStreamInterval.cmdID = CMD_REALTIME_DATA_CUSTOM;
    DataStreamInterval.intervalMs = 10;
    DataStreamInterval.syncToData = STD_SYNC_ON;

    ui32 DataStreamIntervalConfig = RTDCF_STATOR_ROTOR_ANGLE | RTDCF_SYSTEM_POWER_STATE;
    memcpy(DataStreamInterval.config, &DataStreamIntervalConfig, sizeof(DataStreamIntervalConfig));

    /* - - - - - - - - - - - - Program Launch - - - - - - - - - - - - */

    SBGC32_Reset(&SBGC_1, RF_RESTART_CONFIRMATION, 500);
    SBGC32_CheckConfirmation(&SBGC_1, &Confirm, CMD_RESET);
    sleep(15);

    SBGC32_ControlConfig(&SBGC_1, &ControlConfig, &Confirm);

    TxRxStatus_t error = TX_RX_OK;

    // request custom rt data stream
    if (error = SBGC32_RequestDataStream(&SBGC_1, &DataStreamInterval, &Confirm))
    {
        printf("Data Stream Interval Setup error\n");
    }
    // // disable stream again
    sleep(2);
    DataStreamInterval.intervalMs = 0;
    // TODO: this always fails with RX_TIMEOUT_ERROR, regardless of SBGC_1.rxTimeout
    if (error = SBGC32_RequestDataStream(&SBGC_1, &DataStreamInterval, &Confirm))
    {
        printf("Data Stream Interval Stop error \n");
    }

    return 0;
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

    PrintStructElement(&SBGC_1, &MainParamsExt3.motorStartupDelay, "Motor Startup Delay =", _UNSIGNED_SHORT_);

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
        sleep(5);

        Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(20);
        SBGC32_Control(&SBGC_1, &Control);
        sleep(5);
        Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(-50);
        SBGC32_Control(&SBGC_1, &Control);
        sleep(5);

        Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(-20);
        SBGC32_Control(&SBGC_1, &Control);
        sleep(5);

        Control.AxisC[YAW].angle = DEGREE_TO_ANGLE_INT(0);
        Control.AxisC[PITCH].angle = DEGREE_TO_ANGLE_INT(0);
        SBGC32_Control(&SBGC_1, &Control);
        sleep(5);

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
    int rtsize = sizeof(RealTimeData_t);
    int size = sizeof(RealTimeDataCustom.timestampMs) + sizeof(RealTimeDataCustom.frameCamAngle) + sizeof(RealTimeDataCustom.systemPowerState);
    float time_per_byte_ms = 1000.0 / ( 230400.0 / 8.0);
    float transmission_time_ms =time_per_byte_ms *(float)size;
    
    /* Preparing */
    ui8 BuffRPx = 2;  // ui16 timestampMs offset

    // convert endianess of frameCamAngles
    BuffRPx += ConvertWithPM(RealTimeDataCustom.frameCamAngle, &pBuff[BuffRPx],
                             sizeof(RealTimeDataCustom.frameCamAngle), PM_DEFAULT_16BIT);
    // BuffRPx += ConvertWithPM(RealTimeDataCustom.gyroData, &pBuff[BuffRPx],
    //                          sizeof(RealTimeDataCustom.gyroData), PM_DEFAULT_16BIT);
    // BuffRPx += ConvertWithPM(RealTimeDataCustom.ACC_Data, &pBuff[BuffRPx],
    //                          sizeof(RealTimeDataCustom.ACC_Data), PM_DEFAULT_16BIT);
    
    // TODO: use PM_SYSTEM_POWER_STATE!
    BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState, &pBuff[BuffRPx],
                             sizeof(RealTimeDataCustom.systemPowerState), PM_SYSTEM_POWER_STATE);
    // convert endianess of systemPowerState
    //**************************************************************
    // MotorsPowerState_t motorsPower[3];
    // for (int i = 0; i < 3; i++)
    // {
    //     // i16 motPower
    //     BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.motorsPower[i].motPower, &pBuff[BuffRPx],
    //                              sizeof(RealTimeDataCustom.systemPowerState.motorsPower[i].motPower), PM_DEFAULT_16BIT);
    //     // ui16 motCurrent
    //     BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.motorsPower[i].motCurrent, &pBuff[BuffRPx],
    //                                 sizeof(RealTimeDataCustom.systemPowerState.motorsPower[i].motCurrent), PM_DEFAULT_16BIT);
    //     // i8 motTemp
    //     BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.motorsPower[i].motTemp, &pBuff[BuffRPx],
    //                                 sizeof(RealTimeDataCustom.systemPowerState.motorsPower[i].motTemp), PM_DEFAULT_8BIT);
    //     // ui16 motFlags
    //     BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.motorsPower[i].motFlags, &pBuff[BuffRPx],
    //                                 sizeof(RealTimeDataCustom.systemPowerState.motorsPower[i].motFlags), PM_DEFAULT_16BIT);
    //     // ui8 reserved[6]
    //     BuffRPx = BuffRPx + 6;

    // }
    // // SystemPowerStateState_t systemPowerState
    // BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.systemPowerState, &pBuff[BuffRPx],
    //                             sizeof(RealTimeDataCustom.systemPowerState.systemPowerState), PM_DEFAULT_8BIT);
    // // ui16 batteryVoltage
    //  BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.batteryVoltage, &pBuff[BuffRPx],
    //                             sizeof(RealTimeDataCustom.systemPowerState.batteryVoltage), PM_DEFAULT_16BIT);
    // // ui16 totalCurrent
    // BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.totalCurrent, &pBuff[BuffRPx],
    //                             sizeof(RealTimeDataCustom.systemPowerState.totalCurrent), PM_DEFAULT_16BIT);
    // //ui16 systemFlags
    // BuffRPx += ConvertWithPM(&RealTimeDataCustom.systemPowerState.systemFlags, &pBuff[BuffRPx],
    //                             sizeof(RealTimeDataCustom.systemPowerState.systemFlags), PM_DEFAULT_16BIT);
    //**************************************************************
    /* Printing */
    PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[ROLL], "Frame Camera Angle Roll =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[PITCH], "Frame Camera Angle Pitch =", _SIGNED_SHORT_);
    PrintStructElement(&SBGC_1, &RealTimeDataCustom.frameCamAngle[YAW], "Frame Camera Angle Yaw =", _SIGNED_SHORT_);

    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[ROLL], "Gyro Roll =", _SIGNED_SHORT_);
    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[PITCH], "Gyro Pitch =", _SIGNED_SHORT_);
    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.gyroData[YAW], "Gyro Yaw =", _SIGNED_SHORT_);

    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[ROLL], "ACC Roll =", _SIGNED_SHORT_);
    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[PITCH], "ACC Pitch =", _SIGNED_SHORT_);
    // PrintStructElement(&SBGC_1, &RealTimeDataCustom.ACC_Data[YAW], "ACC Yaw =", _SIGNED_SHORT_);

    PrintMessage(&SBGC_1, TEXT_SIZE_((char*)"__________________________\n\n"));
}
