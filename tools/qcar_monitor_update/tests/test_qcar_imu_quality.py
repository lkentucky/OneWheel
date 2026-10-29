from pathlib import Path
import pytest
from qcar_firmware import FIELDS,PROFILES
from qcar_protocol import Protocol

def test_imu_quality_actual_transport_frame():
    names=PROFILES['imu_quality']
    assert len(names)==47 and len(set(names))==47 and set(names)<=set(FIELDS)
    protocol=Protocol()
    protocol.feed(('MCAR SUB '+','.join(names)+'\n').encode())
    values=protocol.feed((Path(__file__).parent/'fixtures/qcar_quality.bin').read_bytes())['values']
    assert values['imu_quality']==3 and values['imu_status']==-5
    assert values['imu_reject_ms']==pytest.approx(600)
    assert values['imu_recovery_pct']==pytest.approx(42)
