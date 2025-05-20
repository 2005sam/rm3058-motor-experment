#include "stm32f4xx_hal.h"
#include "MotorRM3508Drive.h"


#define motor_rx_fifo(num) CAN_RX_FIFO ## num
#define  motor_rx_number(i,j) i##j 
#define motor_tx_number(i) do{\
	tx_data[2*i-2] = motor##i>>8;\
	tx_data[2*i-1] = motor##i;\
}while(0)
#define motor_active_it(i) CAN_IT_RX_FIFO##i##_MSG_PENDING
#define CAN_Rx_FifoMsg_PendingCallback(i) void CAN_Rx_Fifo##i##_Msg_PendingCallback(CAN_HandleTypeDef *hcan)


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
	 sFilterConfig.FilterMaskIdHigh = 0x7fe<<5;\
	 sFilterConfig.FilterMaskIdLow = 0x0000;\
 }while(0)



static CAN_TxHeaderTypeDef tx_header_motor;
static CAN_RxHeaderTypeDef rx_header_motor[4];
static CAN_FilterTypeDef sFilterConfig; 
static CAN_HandleTypeDef hcan;
static uint32_t Rxfifo;
static char fifo_number;
struct rx_date_motor_rm3508_struct motor_rx_date;

//CAN1 init function
void motor_RM3508_Init(CAN_HandleTypeDef * hcan1,char fifo_number_input)
{
	fifo_number=fifo_number_input;
	if(fifo_number){
		Rxfifo = motor_rx_fifo(0);
	}else{
		Rxfifo = motor_rx_fifo(1);
	}
	hcan = *hcan1;
	int temp=0;
       	while (temp<4) {
		motor_RM3508_each_rx_header(rx_header_motor[temp],temp);
		temp++;
	}
	motor_RM3508_tx_header(tx_header_motor);
	motor_RM3508_sFilterConfig(sFilterConfig);
	if(fifo_number){
	HAL_CAN_ActivateNotification(&hcan,motor_active_it(0));
	}else{
		HAL_CAN_ActivateNotification(&hcan,motor_active_it(1));
	}
	HAL_CAN_ConfigFilter(&hcan,&sFilterConfig);
	HAL_CAN_Start(&hcan);

}
void moter_rm3508_tx_massage(uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4)
{
	uint32_t tx_mailbox;
	uint8_t tx_data[8] = {0};
	motor_tx_number(1);
	motor_tx_number(2);
	motor_tx_number(3);
	motor_tx_number(4);

	HAL_CAN_AddTxMessage(&hcan,&tx_header_motor,tx_data,&tx_mailbox);
}

struct rx_date_motor_rm3508_struct motor_rm3508_rx_massage(void)
{
	uint8_t rx_date[8];
	if(HAL_CAN_GetRxMessage(&hcan,Rxfifo,&rx_header_motor[0],rx_date)==HAL_OK)
	{

		motor_rx_date.angle = (rx_date[0]<<8 | rx_date[1])/8191.0f;
		motor_rx_date.rpm = (rx_date[2]<<8 | rx_date[3]);
		motor_rx_date.current = (rx_date[4]<<8 | rx_date[5]);
		motor_rx_date.temperture = rx_date[6];
		return motor_rx_date;
	}
	return motor_rx_date;
}	

CAN_Rx_FifoMsg_PendingCallback(0)
{
	if(fifo_number==1)
		return;
	struct rx_date_motor_rm3508_struct motor_rx_date_it;
	motor_rx_date_it = motor_rm3508_rx_massage();
	motor_rm3508_MSgPendingCallback(motor_rx_date_it,hcan);
}

CAN_Rx_FifoMsg_PendingCallback(1)
{
	if(fifo_number==0)
		return;
	struct rx_date_motor_rm3508_struct motor_rx_date_it;
	motor_rx_date_it = motor_rm3508_rx_massage();
	motor_rm3508_MSgPendingCallback(motor_rx_date_it,hcan);
}
