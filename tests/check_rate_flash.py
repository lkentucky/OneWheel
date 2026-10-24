"""Exercise the production gyro-test Flash extension with simulated EEPROM."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/flash.c').read_text(encoding='utf-8')
helpers = source[source.index('#define RATE_PARAMS_MAGIC'):source.index('//---')].replace('\n#endif', '')
assert 'rate_params_write();' in source[source.index('void write_flash(void)'):]
assert 'rate_params_read();' in source[source.index('void get_flash(void)'):]
prefix = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t uint32;
static uint32 flash[1024];
static float ground_angle_kp=1,ground_angle_rate_limit=10;
static float ground_rate_test_kp=25,ground_rate_test_limit=1000,ground_rate_test_sign=1;
static float pitch_balance_kp=0.5f, pitch_balance_rate_limit=20;
static float roll_rate_test_kp=5, roll_rate_test_limit=1000, roll_rate_test_sign=1;
#define my_flash_read(sector,page,type) (flash[page])
static void eeprom_page_program(uint32 s, uint32 p, uint32 *v) {
    assert(s==0 && p>=77 && p<=94); flash[p]=*v;
}
'''
main = r'''
int main(void) {
    memset(flash, 0xff, sizeof flash);
    rate_params_read(); assert(roll_rate_test_kp==5);
    roll_rate_test_kp=12.75f; roll_rate_test_limit=600; roll_rate_test_sign=-1;
    rate_params_write();
    roll_rate_test_kp=5; roll_rate_test_limit=1000; roll_rate_test_sign=1;
    rate_params_read();
    assert(roll_rate_test_kp==12.75f && roll_rate_test_limit==600 && roll_rate_test_sign==-1);
    flash[78]^=1; roll_rate_test_kp=5; rate_params_read(); assert(roll_rate_test_kp==5);
    roll_rate_test_kp=501; rate_params_write();
    roll_rate_test_kp=5; rate_params_read(); assert(roll_rate_test_kp==5);
    roll_rate_test_kp=8; rate_params_write(); flash[77]=0;
    roll_rate_test_kp=5; rate_params_read(); assert(roll_rate_test_kp==5);
    balance_params_read(); assert(pitch_balance_kp==0.5f);
    pitch_balance_kp=1.25f; pitch_balance_rate_limit=12; balance_params_write();
    pitch_balance_kp=0.5f; pitch_balance_rate_limit=20; balance_params_read();
    assert(pitch_balance_kp==1.25f && pitch_balance_rate_limit==12);
    flash[83]^=1; pitch_balance_kp=0.5f; balance_params_read(); assert(pitch_balance_kp==0.5f);
    pitch_balance_rate_limit=0; balance_params_write(); pitch_balance_rate_limit=20;
    balance_params_read(); assert(pitch_balance_rate_limit==20);
    roll_rate_test_kp=200;roll_rate_test_limit=3000;rate_params_write();
    roll_rate_test_limit=1000;rate_params_read();assert(roll_rate_test_limit==3000);
    roll_rate_test_limit=4001;rate_params_write();roll_rate_test_limit=1000;
    rate_params_read();assert(roll_rate_test_limit==1000);
    pitch_balance_kp=100;pitch_balance_rate_limit=20;balance_params_write();
    pitch_balance_kp=1;balance_params_read();assert(pitch_balance_kp==100);
    pitch_balance_kp=101;balance_params_write();pitch_balance_kp=1;
    balance_params_read();assert(pitch_balance_kp==1);
    roll_rate_test_kp=35;roll_rate_test_limit=4000;rate_params_write();
    roll_rate_test_limit=1000;rate_params_read();assert(roll_rate_test_limit==4000);
    ground_rate_test_kp=30;ground_rate_test_limit=1200;ground_rate_test_sign=-1;ground_params_write();
    ground_rate_test_kp=25;ground_rate_test_sign=1;ground_params_read();
    assert(ground_rate_test_kp==30 && ground_rate_test_limit==1200 && ground_rate_test_sign==-1);
    ground_angle_kp=2.5f;ground_angle_rate_limit=12;ground_angle_params_write();
    ground_angle_kp=1;ground_angle_params_read();assert(ground_angle_kp==2.5f && ground_angle_rate_limit==12);
    puts("PASS: gyro params round trip, legacy/partial/corrupt/out-of-range data rejected");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-rate-flash-') as directory:
    p = Path(directory)
    (p/'test.c').write_text(prefix+helpers+main, encoding='utf-8')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(p/'test.c'), '-o', str(p/'test.exe')], check=True)
    subprocess.run([str(p/'test.exe')], check=True)
