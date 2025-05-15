#include "stm32f4xx_hal.h"
#include "MotorRM3508Drive.h"
#define  motor_rx_number(i,j) i##j 
#define motor_tx_number(i) do{\
	tx_data[2*i-2] = motor##i>>8;\
	tx_data[2*i-1] = motor##i;\
}while(0)

static CAN_TxHeaderTypeDef tx_header_motor;
static CAN_RxHeaderTypeDef rx_header_motor[4]={0};
static CAN_FilterTypeDef sFilterConfig; 
static CAN_HandleTypeDef hacn;

//CAN1 init function
void motor_RM3508_Init(CAN_HandleTypeDef const *const hcan1)
{
	hacn = *hcan1;
	int temp=0;
       	while (temp<4) {
		motor_RM3508_each_rx_header(rx_header_motor[temp],temp);
		temp++;
	}
	motor_RM3508_tx_header(tx_header_motor);
	motor_RM3508_sFilterConfig(sFilterConfig);
}
void moter_rm3508_tx_massage(uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4)
{
	uint8_t tx_data[8] = {0};
	motor_tx_number(1);
	motor_tx_number(2);
	motor_tx_number(3);
	motor_tx_number(4);

	HAL_CAN_AddTxMessage(&hcan1,&tx_header_motor,tx_data,&tx_mailbox);
}
