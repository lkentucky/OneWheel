"""The user photo is a default-value source, not an agent hardware test."""
import json
import re

import pytest

from test_lean_limits import ROOT, _compile_run
from test_imu_forward import param_host_source


def test_photo_values_load_from_real_firmware_defaults(tmp_path):
    snapshot = json.loads((ROOT/'tools/unicycle_station/docs/roll-defaults-20261109.json').read_text(encoding='utf-8'))
    values = snapshot['parameters']
    rows = ','.join('{"'+key+'",'+repr(float(value))+'f}' for key,value in values.items())
    source = param_host_source()+r'''
typedef struct {const char *name;float expected;} sample_t;
static const sample_t samples[]={'''+rows+r'''};
int main(void){
 param_init(); // no Flash record -> actual compile-time defaults
 for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);i++){
  float value;assert(param_get_by_name(samples[i].name,&value));
  assert(fabsf(value-samples[i].expected)<.000001f);
 }
 assert(R_RATE_KD==0 && g_param.r_rate_kd==0);
 // Existing valid Flash tuning still has precedence over compile defaults.
 assert(param_set_by_name("r_rate_kp",27));assert(param_save());param_init();
 assert(g_param.r_rate_kp==27);
 return 0;
}
'''
    _compile_run(tmp_path,source)


@pytest.mark.parametrize('relative',[
    'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json',
    'tools/unicycle_station/examples/default.workspace.json',
])
def test_profile_matches_photo_and_complete_parameter_name_set(relative):
    values=json.loads((ROOT/'tools/unicycle_station/docs/roll-defaults-20261109.json').read_text(encoding='utf-8'))['parameters']
    data=json.loads((ROOT/relative).read_text(encoding='utf-8'));profile=data.get('profile',data)
    rows={r['name']:r for r in profile['mock_parameters']}
    source=(ROOT/'code/Common/param.c').read_text(encoding='utf-8')
    names=re.findall(r'\{\s*"([^"]+)",\s*&g_param\.',source)
    assert len(names)==80 and set(rows)==set(names)
    for key,value in values.items():
        assert rows[key]['value']==value and profile['parameters'][key]['default']==value
    for key in ('r_rate_notch_hz','r_rate_notch_q','r_rate_notch_db'):
        assert rows[key]['flags']==21
