#ifndef _ICM_H_
#define _ICM_H_


extern float AngleX;
extern float AngleY;
extern float AngleZ;

extern char Offset_OK;

extern float gyro_x, gyro_y, gyro_z;
extern float acc_x, acc_y, acc_z;
extern float icm_gyro_x, icm_gyro_y, icm_gyro_z;

void icm20602_offset(void);
float invSqrt(float x);
void icm20602_GetData(void);
void IMUupdate(float, float, float, float, float, float);
void icm_update(void);
void icm_test(void);

#endif
