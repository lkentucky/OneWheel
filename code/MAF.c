/*
 * MAF.c
 *
 *  Created on: 2025年4月20日
 *      Author: yuan
 */
#include "MAF.h"

MovingAverage avgFilter;

float movingAverageFilter(MovingAverage *filter, float newValue)
{
    // 更新缓冲区并维护总和
    filter->sum -= filter->buffer[filter->index];
    filter->sum += newValue;
    filter->buffer[filter->index] = newValue;
    filter->index = (filter->index + 1) % WINDOW_SIZE;
    return filter->sum / WINDOW_SIZE;
}



