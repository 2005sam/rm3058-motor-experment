#ifndef MOTORRM3508DRIVE_H
#define MOTORRM3508DRIVE_H


struct rx_date_motor_rm3508_struct{float angle;uint16_t rpm;uint16_t current;uint8_t temperture;};



struct rx_date_motor_rm3508_struct motor_rm3508_rx_massage(void);
void motor_RM3508_Init(CAN_HandleTypeDef const *const hcan1,char fifo_number);
void moter_rm3508_tx_massage(uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4);
void __attribute__((weak)) motor_rm3508_MSgPendingCallback(struct rx_date_motor_rm3508_struct rx_date);

#endif

