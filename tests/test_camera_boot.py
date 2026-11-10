"""Execute actual camera boot/manual command paths, without LCD/camera hardware."""
from pathlib import Path
import re,subprocess,os,shutil
import pytest
ROOT=Path(__file__).resolve().parents[1]
def function(text,signature):
 a=text.index(signature);b=text.index('{',a);depth=1;i=b+1
 while depth:
  depth+=(text[i]=='{')-(text[i]=='}');i+=1
 return text[a:i]+'\n'
@pytest.mark.parametrize('auto',[0,1])
def test_boot_policy_does_not_disable_manual_retry(tmp_path,auto,monkeypatch):
 monkeypatch.syspath_prepend(str(ROOT/'tools/unicycle_station/tests'))
 from test_camera_image_pipeline import camera_source
 from test_lean_limits import _compile_run
 source=camera_source().replace('#define QCAR_CAMERA_AUTO_START 0',f'#define QCAR_CAMERA_AUTO_START {auto}')
 source+=r'''
int main(void){
 init_failed=1;vision_core_init();assert(starts==QCAR_CAMERA_AUTO_START);
 assert(s_core_state==(QCAR_CAMERA_AUTO_START?VISION_CORE_FAILED:VISION_CORE_OFF));
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 assert(starts==QCAR_CAMERA_AUTO_START+1 && s_core_state==VISION_CORE_FAILED);
 init_failed=0;assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 assert(starts==QCAR_CAMERA_AUTO_START+2 && s_core_state==VISION_CORE_READY && exposures==1);
 assert(vision_command_request(VISION_CMD_SET_EXPOSURE,300));vision_core_run();assert(s_camera_exposure==300 && exposures==2);
 assert(vision_command_request(VISION_CMD_CALIB_IPM,0));vision_core_run();assert(s_ipm_collecting==1);
 return 0;
}
'''
 _compile_run(tmp_path,source)
@pytest.mark.parametrize('logging',[0,1])
def test_cpu1_camera_failure_cannot_clear_cpu0_lcd_by_default(tmp_path,logging):
 header=(ROOT/'libraries/zf_device/zf_device_mt9v03x.h').read_bytes().decode('utf-8',errors='replace')
 guard=re.search(r'#ifndef MT9V03X_INIT_LOG_ENABLE.*?#endif',header,re.S).group()
 src=(ROOT/'libraries/zf_device/zf_device_mt9v03x.c').read_bytes().decode('utf-8',errors='replace')
 body=function(src,'uint8 mt9v03x_init (void)')
 p=tmp_path/'camera.c';p.write_text(guard+'\n#define zf_log(x,msg) CAMERA_LCD_ERROR(msg)\n'+body,encoding='utf-8')
 compiler=os.environ.get('ONEWHEEL_HOST_TCC') or shutil.which('gcc')
 if not compiler:pytest.skip('Optional C preprocessor unavailable')
 r=subprocess.run([compiler,'-E',f'-DMT9V03X_INIT_LOG_ENABLE={logging}',str(p)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 assert r.stdout.count('CAMERA_LCD_ERROR(')==(2 if logging else 0)
