from pathlib import Path
import pytest
from qcar_firmware import FIELDS,PROFILES
from qcar_parameters import V2_PARAMETERS,V3_PARAMETERS,V4_PARAMETERS,POSITION_PARAMETERS
from qcar_protocol import Protocol

def test_actual_position_frame_and_backward_profile_order():
    assert len(PROFILES['hold'])==48
    assert PROFILES['hold'][:43]==PROFILES['speed']
    assert set(PROFILES['hold'])<=set(FIELDS)
    p=Protocol();p.feed(('MCAR SUB '+','.join(PROFILES['hold'])+'\n').encode())
    values=p.feed((Path(__file__).parent/'fixtures/qcar_hold.bin').read_bytes())['values']
    assert values['wheel_position_count']==512 and values['wheel_ref_count']==0
    assert values['wheel_pos_error']==-512 and values['wheel_position_valid']==1
    assert values['wheel_target_cps']==-252 and values['wheel_speed_limit']==300
    assert values['wheel_tilt_offset']==pytest.approx(.252)

def test_scheme_version_sets_exclude_new_fields():
    assert len(V2_PARAMETERS)==15 and len(V3_PARAMETERS)==19 and len(V4_PARAMETERS)==21
    assert not (V3_PARAMETERS & POSITION_PARAMETERS)
    assert V4_PARAMETERS-V3_PARAMETERS==POSITION_PARAMETERS
