/*
 * posture_control.h
 *
 *  Created on: 2025Äê3ÔÂ9ÈÕ
 *      Author: lenovo
 */

#ifndef CODE_POSTURE_CONTROL_H_
#define CODE_POSTURE_CONTROL_H_

typedef struct
{
    int16 accX;
    int16 accY;
    int16 accZ;
    int16 gyroX;
    int16 gyroY;
    int16 gyroZ;
} ICM_DATA;
extern ICM_DATA icm_data_lpf;
extern ICM_DATA re_icm_data;
extern float pitch_filiter;
void imu_data_get(void);
void dynamic_value_control(void);
void pit_isr_callback(void);
void find_ma(void);
void icm_init(void);
void imu_filiter(void);

#endif /* CODE_POSTURE_CONTROL_H_ */
