"""Current Mode8 controls and parameter limits, independent from the GUI."""
import struct
import pytest
from qcar_firmware import FIELDS
from qcar_parameters import PARAMETERS, SLIDER_LIMITS, SIGN_PARAMETERS, PWM_PROFILE, DEBUG_PARAMETERS
from qcar_protocol import Protocol, slider_command

def test_all_editors_have_validation_and_valid_defaults():
    assert set(PARAMETERS)==set(SLIDER_LIMITS)|SIGN_PARAMETERS
    assert len(PARAMETERS)==21
    for name,(_,default,_) in PARAMETERS.items():
        assert slider_command(name,default).startswith(b'[slider,'+name.encode()+b',')

@pytest.mark.parametrize('name',DEBUG_PARAMETERS)
def test_independent_pwm_exact_commands_and_limits(name):
    for value in (-4000,-700,0,300,4000):
        assert slider_command(name,value)==f'[slider,{name},{value}]\n'.encode()
    for invalid in (-4001,4001,float('nan'),float('inf'),True):
        with pytest.raises(ValueError):slider_command(name,invalid)

def test_old_paired_name_rejected():
    with pytest.raises(ValueError):slider_command('debug_fly_pwm',300)

def test_debug_schema_preserves_both_actual_output_signs():
    assert len(PWM_PROFILE)==15 and set(PWM_PROFILE)<=set(FIELDS)
    protocol=Protocol()
    protocol.feed(('MCAR SUB '+','.join(PWM_PROFILE)+'\n').encode())
    values=dict.fromkeys(PWM_PROFILE,0)
    values.update(pwm_debug_enabled=1,debug_fly_left_pwm=300,debug_fly_right_pwm=-700,
                  motor_left_pwm=300,motor_right_pwm=-700,motor_ground_pwm=-500)
    frame=struct.pack('<15f',*(values[n] for n in PWM_PROFILE))+b'\x00\x00\x80\x7f'
    actual=protocol.feed(frame)['values']
    assert actual['motor_left_pwm']==300 and actual['motor_right_pwm']==-700
    assert actual['debug_fly_right_pwm']==-700

@pytest.mark.parametrize('name',sorted(SIGN_PARAMETERS))
def test_closed_loop_signs_remain_discrete(name):
    for sign in (-1,1):assert slider_command(name,sign)
    with pytest.raises(ValueError):slider_command(name,0)
