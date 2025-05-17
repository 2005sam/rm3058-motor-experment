#include "stm32f4xx_hal.h"
#include "MotorRM3508Drive.h"
#define motor_rx_fifo(num) CAN_RX_FIFO ## num
#define  motor_rx_number(i,j) i##j 
#define motor_tx_number(i) do{\
	tx_data[2*i-2] = motor##i>>8;\
	tx_data[2*i-1] = motor##i;\
}while(0)
#define IT_FIFO_choose(i) CAN_IT_RX_FIFO ## i ## _MSG_PENDING

static CAN_TxHeaderTypeDef tx_header_motor;
static CAN_RxHeaderTypeDef rx_header_motor[4]={0};
static CAN_FilterTypeDef sFilterConfig; 
static CAN_HandleTypeDef hacn;
static uint32_t Rxfifo;
struct rx_date_motor_rm3508_struct motor_rx_date;

//CAN1 init function
void motor_RM3508_Init(CAN_HandleTypeDef const *const hcan1,char fifo_number)
{
	Rxfifo = motor_rx_fifo(fifo_number);
	hacn = *hcan1;
	int temp=0;
       	while (temp<4) {
		motor_RM3508_each_rx_header(rx_header_motor[temp],temp);
		temp++;
	}
	motor_RM3508_tx_header(tx_header_motor);
	motor_RM3508_sFilterConfig(sFilterConfig);
	HAL_CAN_ActiveNotification(&hacn,IT_FIFO_choose(fifo_number));

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

struct rx_date_motor_rm3508_struct motor_rm3508_rx_massage(void)
{
	uint8_t rx_date[8];
	if(HAL_CAN_GetRxMessage(&hcan1,Rxfifo,rx_header_motor[0],rx_date)==HAL_OK)
	{

		motor_rx_date.angle = (rx_date[0]<<8 | rx_date[1])/8191.0f;
		motor_rx_date.rpm = (rx_date[2]<<8 | rx_date[3]);
		motor_rx_date.current = (rx_date[4]<<8 | rx_date[5]);
		motor_rx_date.temperture = rx_date[6];
		return motor_rx_date;
	}
}	


