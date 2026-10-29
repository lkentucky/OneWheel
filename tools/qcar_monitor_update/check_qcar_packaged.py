"""Actual Qcar executable + firmware-produced frames over loopback UDP only."""
import argparse
import csv
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from qcar_firmware import PROFILES
from qcar_parameters import PWM_PROFILE


def check(exe,output):
    output.mkdir(parents=True,exist_ok=True)
    startup=output/'startup.json';report=output/'report.json'
    for path in (startup,report):path.unlink(missing_ok=True)
    environment=os.environ.copy()
    runtime=output/'runtime';runtime.mkdir(exist_ok=True)
    environment.update(TEMP=str(runtime),TMP=str(runtime),LOCALAPPDATA=str(output/'isolated_appdata'))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        probe.bind(('127.0.0.1',18081))
    process=subprocess.Popen([str(exe),'--data-dir',str(output/'state'),'--test-ports','--listen-test',
                              '--page','plot','--exit-after','5','--startup-report',str(startup),'--test-report',str(report)],
                             env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline=time.monotonic()+30
        while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert startup.exists(),('No startup report',process.poll())
        first=json.loads(startup.read_text('utf-8'))
        assert first['profile']=='qcar' and first['data_port']==18081 and first['remote_port']==5001,first
        def schema(name):return ('MCAR SUB '+','.join(PROFILES[name])+'\n').encode()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1',0));peer=sender.getsockname()
            to=('127.0.0.1',18081)
            startup_frame=(ROOT/'tests/fixtures/qcar_startup.bin').read_bytes()
            dual_frame=(ROOT/'tests/fixtures/qcar_dual.bin').read_bytes()
            sender.sendto(startup_frame,to) # not decoded until the field list arrives
            sender.sendto(schema('startup')+b'MCAR RATE 20\nMCAR STREAM 1\n',to)
            for _ in range(20):sender.sendto(startup_frame,to);time.sleep(.005)
            sender.sendto(schema('dual'),to)
            for _ in range(80):sender.sendto(dual_frame,to);time.sleep(.005)
            sender.sendto(schema('dual'),to) # periodic announcements must not reset the graph
            sender.sendto(b'bad frame',to)
            nan=bytearray(dual_frame);struct.pack_into('<f',nan,0,float('nan'));sender.sendto(nan,to)
            sender.sendto(b'MCAR SLIDER rate_kp 120\n',to) # not an acknowledgement of a pending write
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as foreign:
                foreign.bind(('127.0.0.2',0));foreign.sendto(dual_frame,to)
        process.wait(timeout=20)
        assert process.returncode==0,process.returncode
        result=json.loads(report.read_text('utf-8'))
        assert result['records']==100 and result['parse_errors']==2,result
        assert result['fields']==PROFILES['dual'],result
        assert result['latest']['dual_roll_pwm']==-2800 and result['latest']['dual_pitch_pwm']==-3000,result
        assert result['plotted_points']['dual_roll_actual_deg']==80,result
        assert result['plotted_points']['dual_pitch_actual_deg']==80,result
        assert result['waiting_packets']==1 and result['foreign_packets']==1,result
        assert not result['confirmed'] and result['remote_target']==list(peer),result
        saved=list(Path(result['output']).glob('*.csv'));assert len(saved)==1,saved
        with saved[0].open(encoding='utf-8-sig',newline='') as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==100 and float(rows[-1]['dual_pitch_pwm'])==-3000
        assert rows[0]['dual_pitch_pwm']=='',rows[0]
        proof=dict(profile='qcar',executable=str(exe),records=100,dual_curve_points=80,malformed_rejected=2,
                   before_schema_rejected=True,foreign_peer_rejected=True,unsolicited_ack_ignored=True,
                   source_port_learning=True,csv_rows=100,vehicle_contacted=False)
        (output/'proof.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(proof,ensure_ascii=False))
        check_mode8(exe, output/'mode8')
        check_encoder(exe, output/'encoder')
        check_speed(exe, output/'speed')
        check_hold(exe, output/'hold')
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=5)


def check_mode8(exe, output):
    """Exercise the packaged independent-PWM schema with loopback telemetry."""
    output.mkdir(parents=True,exist_ok=True)
    startup=output/'startup.json';report=output/'report.json'
    for path in (startup,report):path.unlink(missing_ok=True)
    environment=os.environ.copy()
    runtime=output/'runtime';runtime.mkdir(exist_ok=True)
    environment.update(TEMP=str(runtime),TMP=str(runtime),LOCALAPPDATA=str(output/'isolated_appdata'))
    process=subprocess.Popen([str(exe),'--data-dir',str(output/'state'),'--test-ports','--listen-test',
                              '--page','tuning','--exit-after','4','--startup-report',str(startup),
                              '--test-report',str(report)],env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline=time.monotonic()+30
        while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert startup.exists(),('No Mode8 startup report',process.poll())
        values=dict(telemetry_ms=1234,rate_seq=11,rate_state=2,pwm_debug_enabled=1,
                    debug_fly_left_pwm=300,debug_fly_right_pwm=-700,debug_ground_pwm=0,
                    motor_left_pwm=300,motor_right_pwm=-700,motor_ground_pwm=0,
                    flywheel_left_speed_raw=123,flywheel_right_speed_raw=-234,
                    driver_speed_frames=100,fault_reason=0,startup_scale=1)
        frame=struct.pack('<'+'f'*len(PWM_PROFILE),*(values[key] for key in PWM_PROFILE))+b'\x00\x00\x80\x7f'
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1',0))
            target=('127.0.0.1',18081)
            sender.sendto(('MCAR SUB '+','.join(PWM_PROFILE)+'\n').encode(),target)
            for _ in range(30):sender.sendto(frame,target);time.sleep(.01)
        process.wait(timeout=20)
        assert process.returncode==0,process.returncode
        result=json.loads(report.read_text('utf-8'))
        assert result['records']==30 and result['parse_errors']==0,result
        assert result['fields']==PWM_PROFILE and result['active_page']=='tuning',result
        for name,value in values.items():assert result['latest'][name]==value,(name,result)
        assert not result['confirmed'],result
        proof=dict(profile='mode8',records=30,left_pwm=300,right_pwm=-700,ground_pwm=0,
                   independent_channels=True,feedback_fields=True,vehicle_contacted=False)
        (output/'proof.json').write_text(json.dumps(proof,indent=2),encoding='utf-8')
        print(json.dumps(proof))
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=5)


def check_encoder(exe, output):
    """New observation profile decoded by the executable from a real C-produced frame."""
    output.mkdir(parents=True,exist_ok=True)
    startup=output/'startup.json';report=output/'report.json'
    environment=os.environ.copy()
    environment['LOCALAPPDATA']=str(output/'isolated_appdata')
    process=subprocess.Popen([str(exe),'--data-dir',str(output/'state'),'--test-ports','--listen-test',
                              '--page','plot','--exit-after','4','--startup-report',str(startup),
                              '--test-report',str(report)],env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline=time.monotonic()+30
        while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert startup.exists(),('No encoder startup report',process.poll())
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1',0));target=('127.0.0.1',18081)
            sender.sendto(('MCAR SUB '+','.join(PROFILES['encoder'])+'\n').encode(),target)
            frame=(ROOT/'tests/fixtures/qcar_encoder.bin').read_bytes()
            for _ in range(30):sender.sendto(frame,target);time.sleep(.01)
        process.wait(timeout=20)
        assert process.returncode==0
        result=json.loads(report.read_text('utf-8'))
        assert result['records']==30 and result['parse_errors']==0,result
        assert result['fields']==PROFILES['encoder'],result
        assert result['latest']['wheel_speed_cps']==-1000 and result['latest']['wheel_position_count']==-1000,result
        assert result['latest']['wheel_encoder_valid']==1 and result['latest']['wheel_position_valid']==1,result
        assert result['plotted_points']['wheel_speed_cps']==30,result
        assert result['plotted_points']['wheel_position_count']==30,result
        assert not result['confirmed'],result
        saved=list(Path(result['output']).glob('*.csv'));assert len(saved)==1
        with saved[0].open(encoding='utf-8-sig',newline='') as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==30 and float(rows[-1]['wheel_speed_cps'])==-1000
        proof=dict(profile='encoder',records=30,speed_cps=-1000,position_count=-1000,
                   encoder_curve_points=30,firmware_frame=True,csv_rows=30,vehicle_contacted=False)
        (output/'proof.json').write_text(json.dumps(proof,indent=2),encoding='utf-8')
        print(json.dumps(proof))
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=5)


def check_speed(exe, output):
    """Packaged Mode2 profile: no commands, isolated loopback only."""
    output.mkdir(parents=True,exist_ok=True)
    startup=output/'startup.json';report=output/'report.json'
    environment=os.environ.copy();environment['LOCALAPPDATA']=str(output/'isolated_appdata')
    process=subprocess.Popen([str(exe),'--data-dir',str(output/'state'),'--test-ports','--listen-test',
                              '--page','plot','--exit-after','4','--startup-report',str(startup),
                              '--test-report',str(report)],env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline=time.monotonic()+30
        while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert startup.exists(),('No Mode2 startup report',process.poll())
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1',0));target=('127.0.0.1',18081)
            sender.sendto(('MCAR SUB '+','.join(PROFILES['speed'])+'\n').encode(),target)
            frame=(ROOT/'tests/fixtures/qcar_speed.bin').read_bytes()
            for _ in range(30):sender.sendto(frame,target);time.sleep(.01)
        process.wait(timeout=20);assert process.returncode==0
        result=json.loads(report.read_text('utf-8'))
        assert result['records']==30 and result['parse_errors']==0,result
        assert result['fields']==PROFILES['speed'] and result['latest']['control_mode']==2,result
        assert result['latest']['wheel_speed_cps']==500 and result['latest']['wheel_tilt_offset']==.5,result
        for name in ('wheel_speed_cps','wheel_target_cps','wheel_tilt_offset'):
            assert result['plotted_points'][name]==30,result
        assert not result['confirmed'],result
        saved=list(Path(result['output']).glob('*.csv'));assert len(saved)==1
        with saved[0].open(encoding='utf-8-sig',newline='') as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==30 and float(rows[-1]['wheel_tilt_offset'])==.5
        proof=dict(profile='mode2_speed',records=30,speed_cps=500,tilt_offset=.5,control_mode=2,
                   curve_points=30,csv_rows=30,firmware_transport_frame=True,vehicle_contacted=False)
        (output/'proof.json').write_text(json.dumps(proof,indent=2),encoding='utf-8');print(json.dumps(proof))
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=5)


def check_hold(exe, output):
    """Validate the actual 48-channel C frame through the packaged application."""
    output.mkdir(parents=True,exist_ok=True)
    startup=output/'startup.json';report=output/'report.json'
    for path in (startup,report):path.unlink(missing_ok=True)
    environment=os.environ.copy();environment['LOCALAPPDATA']=str(output/'isolated_appdata')
    process=subprocess.Popen([str(exe),'--data-dir',str(output/'state'),'--test-ports','--listen-test',
                              '--page','plot','--exit-after','4','--startup-report',str(startup),
                              '--test-report',str(report)],env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        deadline=time.monotonic()+30
        while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
        assert startup.exists(),('No position-hold startup report',process.poll())
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1',0));target=('127.0.0.1',18081)
            sender.sendto(('MCAR SUB '+','.join(PROFILES['hold'])+'\n').encode(),target)
            frame=(ROOT/'tests/fixtures/qcar_hold.bin').read_bytes()
            for _ in range(30):sender.sendto(frame,target);time.sleep(.01)
        process.wait(timeout=20);assert process.returncode==0
        result=json.loads(report.read_text('utf-8'))
        assert result['records']==30 and result['parse_errors']==0,result
        assert result['fields']==PROFILES['hold'] and result['latest']['control_mode']==2,result
        expected=dict(wheel_position_count=512,wheel_ref_count=0,wheel_pos_error=-512,
                      wheel_target_cps=-252,wheel_speed_limit=300,wheel_position_valid=1)
        for name,value in expected.items():assert result['latest'][name]==value,(name,result)
        for name in ('wheel_pos_error','wheel_target_cps','wheel_speed_cps'):
            assert result['plotted_points'][name]==30,result
        assert not result['confirmed'],result
        saved=list(Path(result['output']).glob('*.csv'));assert len(saved)==1
        with saved[0].open(encoding='utf-8-sig',newline='') as stream:rows=list(csv.DictReader(stream))
        assert len(rows)==30 and float(rows[-1]['wheel_pos_error'])==-512
        proof=dict(profile='mode2_position',records=30,fields=48,position_count=512,
                   reference_count=0,position_error=-512,target_speed_cps=-252,
                   curve_points=30,csv_rows=30,firmware_transport_frame=True,vehicle_contacted=False)
        (output/'proof.json').write_text(json.dumps(proof,indent=2),encoding='utf-8');print(json.dumps(proof))
    finally:
        if process.poll() is None:process.kill();process.wait(timeout=5)


def check_imu_profiles(exe, output):
    """New estimator diagnostics through the real executable, loopback only."""
    for profile,fixture in [('imu_quality','qcar_quality.bin'),('imu_input','qcar_input.bin')]:
        folder=output/profile;folder.mkdir(parents=True,exist_ok=True)
        startup=folder/'startup.json';report=folder/'report.json'
        for path in (startup,report):path.unlink(missing_ok=True)
        environment=os.environ.copy();environment['LOCALAPPDATA']=str(folder/'appdata')
        process=subprocess.Popen([str(exe),'--data-dir',str(folder/'state'),'--test-ports','--listen-test',
            '--page','plot','--exit-after','6','--startup-report',str(startup),'--test-report',str(report)],
            env=environment,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            deadline=time.monotonic()+30
            while not startup.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.1)
            assert startup.exists(),('No startup report',profile,process.poll())
            names=PROFILES[profile];frame=(ROOT/'tests/fixtures'/fixture).read_bytes()
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
                sender.bind(('127.0.0.1',0))
                sender.sendto(('MCAR SUB '+','.join(names)+'\n').encode(),('127.0.0.1',18081))
                for _ in range(30):sender.sendto(frame,('127.0.0.1',18081));time.sleep(.01)
            process.wait(timeout=20)
            assert process.returncode==0
            result=json.loads(report.read_text('utf-8'))
            assert result['records']==30 and result['parse_errors']==0 and result['fields']==names,result
            assert result['latest']['imu_status']==1,result
            if profile=='imu_quality':
                assert result['latest']['imu_quality']==3 and result['latest']['imu_recovery_pct']==0,result
            else:
                assert result['latest']['imu_pre_gy_dps']==1 and result['latest']['gyro_y_dps']==2,result
                assert result['latest']['imu_reacquiring']==1,result
                assert result['plotted_points']['imu_pre_gy_dps']==30,result
            saved=list(Path(result['output']).glob('*.csv'));assert len(saved)==1
            with saved[0].open(encoding='utf-8-sig',newline='') as stream:rows=list(csv.DictReader(stream))
            assert len(rows)==30 and float(rows[-1]['imu_status'])==1
            proof=dict(profile=profile,fields=len(names),records=30,parse_errors=0,csv_rows=30,
                       raw_and_preprocessed=profile=='imu_input',firmware_transport_frame=True,vehicle_contacted=False)
            (folder/'proof.json').write_text(json.dumps(proof,indent=2),encoding='utf-8');print(json.dumps(proof))
        finally:
            if process.poll() is None:process.kill();process.wait(timeout=5)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('exe',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();check(args.exe.resolve(),args.output.resolve());check_imu_profiles(args.exe.resolve(),args.output.resolve())
