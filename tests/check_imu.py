"""Host regression of actual RB pipeline and EKF, with simulated SPI/STM."""
from pathlib import Path
import shutil
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="qcar-imu-") as directory:
    work=Path(directory)
    for name in ("imu.c","attitude_algorithm.c","attitude_algorithm.h"):
        shutil.copy2(root/"code"/name,work/name)
    (work/"zf_common_typedef.h").write_text("""
#ifndef HOST_TYPES
#define HOST_TYPES
#include <stdint.h>
#include <stdbool.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int16_t int16; typedef int32_t int32;
#define SPI_0 0
#define P20_13 13
#endif
""")
    (work/"zf_common_headfile.h").write_text('#include "zf_common_typedef.h"\ntypedef struct {float x,y,z;} Axis3f;\n')
    (work/"zf_common_interrupt.h").write_text('#include "zf_common_typedef.h"\nuint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);\n')
    (work/"zf_driver_delay.h").write_text('#include "zf_common_typedef.h"\nvoid system_delay_ms(uint32);\n')
    (work/"zf_driver_gpio.h").write_text('void gpio_high(int); void gpio_low(int);\n')
    (work/"zf_driver_spi.h").write_text('#include "zf_common_typedef.h"\nvoid spi_write_8bit_register(int,uint8,uint8); uint8 spi_read_8bit_register(int,uint8); void spi_read_8bit_registers(int,uint8,uint8*,uint32);\n')
    (work/"IfxStm.h").write_text('#include "zf_common_typedef.h"\n#define MODULE_STM0 host_stm\nextern int host_stm; uint32 IfxStm_getLower(int*); float IfxStm_getFrequency(int*);\n')
    for method in (1,2,3):
        exe=work/f"pipeline-{method}.exe"
        subprocess.run(['gcc','-x','c','-std=c99','-O2','-Wall','-Wextra',f'-DAHRS_METHOD={method}',
          '-I'+str(work),'-I'+str(root/'code'),'-I'+str(root/'libraries/zf_device'),
          str(work/'imu.c'),str(work/'attitude_algorithm.c'),str(root/'code/attitude.c'),
          str(root/'tests/imu_rb_test.c.in'),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    exe=work/'ekf.exe'
    subprocess.run(['gcc','-x','c','-std=c99','-O2','-Wall','-Wextra','-I'+str(root/'code'),
      str(root/'code/attitude.c'),str(root/'tests/attitude_ekf_test.c.in'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
