/* Hardware boundary for legacy controller tests. Mode2 tests supply real samples. */
#include "wheel_encoder.h"
void wheel_encoder_snapshot(wheel_encoder_sample_t *sample)
{
    sample->sequence=1;sample->age_ms=0;sample->valid=1;
    sample->speed_cps=0.0f;sample->dt_ms=20.0f;
    sample->position_count=0;sample->position_valid=1;sample->position_epoch=1;
}
uint8_t wheel_encoder_rebase_position(void) {return 1;}
