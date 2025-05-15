#ifndef MOTORRM3508DRIVE_H
#define MOTORRM3508DRIVE_H
#define motor_RM3508_tx_header(motor)do{\
	motor.StdId = 0x200;\
	motor.ExtId = 0;\
	motor.RTR = CAN_RTR_DATA;\
	motor.DLC = 8;\
	motor.IDE=CAN_ID_STD;\
	motor.TransmitGlobalTime = DISABLE;\
	}while(0)
#define motor_RM3508_each_rx_header(motor,date)do{\
	motor.StdId = 0x201+date;\
	motor.ExtId = 0;\
	motor.RTR = CAN_RTR_DATA;\
	motor.DLC = 8;\
	motor.IDE=CAN_ID_STD;\
}while(0)
#define motor_RM3508_sFilterConfig(sFilterConfig)do{\
	 sFilterConfig.FilterActivation = ENABLE;\
	sFilterConfig.FilterBank = 0;\
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;\
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;\
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;\
	sFilterConfig.FilterIdHigh = 0x200<<5;\
	sFilterConfig.FilterIdLow = 0x0000;\
	sFilterConfig.FilterMaskIdHigh = 0x3ff<<5;\
	sFilterConfig.FilterMaskIdLow = 0x0000;\
}while(0)


void motor_RM3508_drive_Init(void);
#endif

