"""Firmware-produced Mode2 frame and editor/save validation consistency."""
from pathlib import Path
import pytest
from qcar_firmware import PROFILES
from qcar_parameters import PARAMETERS,SLIDER_LIMITS,SIGN_PARAMETERS,validate_parameter
from qcar_protocol import Protocol,slider_command

def test_mode2_actual_firmware_frame():
    p=Protocol();p.feed(('MCAR SUB '+','.join(PROFILES['speed'])+'\n').encode())
    values=p.feed((Path(__file__).parent/'fixtures/qcar_speed.bin').read_bytes())['values']
    assert len(values)==43 and values['control_mode']==2 and values['wheel_hold_state']==2
    assert values['wheel_target_cps']==0 and values['wheel_speed_cps']==500
    assert values['wheel_speed_error']==-500 and values['wheel_tilt_offset']==.5
    assert values['dual_pitch_target_deg']-values['dual_pitch_actual_deg']==.5
    assert values['wheel_feedback_valid']==1 and values['wheel_feedback_age_ms']==2

@pytest.mark.parametrize('name',list(PARAMETERS))
def test_scheme_validation_matches_command_validation(name):
    default=PARAMETERS[name][1]
    assert validate_parameter(name,default)==default
    assert slider_command(name,default)
    low,high=(-1,1) if name in SIGN_PARAMETERS else SLIDER_LIMITS[name]
    for value in (True,'0',float('nan'),float('inf'),low-1,high+1):
        with pytest.raises(ValueError):validate_parameter(name,value)
        with pytest.raises(ValueError):slider_command(name,value)
    if name in SIGN_PARAMETERS:
        for value in (-1,1):assert validate_parameter(name,value)==value
        with pytest.raises(ValueError):validate_parameter(name,0)
