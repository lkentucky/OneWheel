from pathlib import Path
import math
import pytest
from qcar_firmware import FIELDS,PROFILES
from qcar_protocol import Protocol

def test_imu_input_actual_transport_frame():
    names=PROFILES['imu_input']
    assert len(names)==48 and len(set(names))==48 and set(names)<=set(FIELDS)
    protocol=Protocol()
    protocol.feed(('MCAR SUB '+','.join(names)+'\n').encode())
    values=protocol.feed((Path(__file__).parent/'fixtures/qcar_input.bin').read_bytes())['values']
    assert tuple(values)==tuple(names) and values['imu_status']==1
    assert [values['gyro_'+k+'_dps'] for k in 'xyz']==[1,2,3]
    assert [values['imu_pre_g'+k+'_dps'] for k in 'xyz']==pytest.approx([.5,1,1.5])
    assert [values['imu_pre_a'+k+'_g'] for k in 'xyz']==pytest.approx([.01,-.02,.99])
    assert values['imu_pre_norm_g']==pytest.approx(math.sqrt(.01**2+.02**2+.99**2),abs=1e-5)
    assert values['imu_reacquiring']==1
