"""Qcar adaptation of the shared six-page monitor; no firmware changes."""
from collections import deque
from datetime import datetime
import ipaddress
import json
import math
from pathlib import Path
import time

import dearpygui.dearpygui as dpg
import app
from debug_log import LogLine
from link_health import LinkHealth
from packet_parser import FieldRule
from quick_commands import new_command
from transport import send_udp,MultiplexReceiver
from qcar_firmware import PROFILES
from qcar_protocol import Protocol,slider_command
from qcar_parameters import (PARAMETERS, DEBUG_PARAMETERS, LEGACY_PARAMETERS, V2_PARAMETERS, V3_PARAMETERS,
                             PWM_PROFILE, validate_parameter)

PROFILES = {**PROFILES, 'pwm_debug': PWM_PROFILE}
PROFILE_LABELS={'启动诊断 · 39通道':'startup','双轴直立 · 45通道':'dual',
                '角度环 · 28通道':'angle','三轴姿态 · 3通道':'attitude',
                'Mode8 PWM诊断 · 15通道':'pwm_debug','地轮编码器 · 15通道':'encoder',
                'Mode2速度环 · 43通道':'speed','Mode2位置保持 · 48通道':'hold','IMU可信度 · 47通道':'imu_quality','IMU原始/预处理 · 48通道':'imu_input'}
DEFAULT_PLOTS={'balance_actual_deg','balance_target_deg','rate_actual_dps','rate_target_dps',
               'dual_roll_actual_deg','dual_roll_target_deg','dual_pitch_actual_deg','dual_pitch_target_deg',
               'roll_deg','pitch_deg','yaw_deg','motor_left_pwm','motor_right_pwm','motor_ground_pwm',
               'debug_fly_left_pwm','debug_fly_right_pwm','debug_ground_pwm',
               'wheel_speed_cps','wheel_target_cps','wheel_tilt_offset','wheel_position_count',
               'wheel_pos_error','gyro_y_dps','imu_pre_gy_dps'}


class QuietLink(LinkHealth):
    def heartbeat(self,*args,**kwargs):return None


class QcarMonitor(app.Monitor):
    def __init__(self,args):
        self.protocol=Protocol()
        self.peer=None
        self.last_announced=()
        self.qcar_target=None
        self.pending={}
        self.confirmed={}
        self.qcar_states={key:'未发送' for key in PARAMETERS}
        self.waiting_packets=0
        self.foreign_packets=0
        self.qcar_message='等待车端MCAR SUB字段通知；先开始监听，再在板端Startup/WiFi connect。'
        super().__init__(args)
        self.link=QuietLink()

    def _build(self):
        self.single_port=True
        if self.multiplex is None:
            self.multiplex=MultiplexReceiver();self.receivers=self.multiplex.channels
        super()._build()
        dpg.configure_viewport(0,title='OneWheel TC264 Monitor - Mode2 Speed/Position Hold')
        dpg.configure_item('receive_mode',items=['Qcar JustFloat UDP'],enabled=False)
        dpg.set_value('receive_mode','Qcar JustFloat UDP')
        dpg.set_value('remote_port',self.settings.get('remote_port',5001))
        dpg.set_value('remote_ip',self.settings.get('remote_ip',''))
        self.qcar_target=(dpg.get_value('remote_ip'),dpg.get_value('remote_port'))
        dpg.set_value('listen_note','电脑监听8081；模块命令端口默认5001。数据与MCAR文本共用一个UDP端口，自动识别模块地址；不发送旧版ALIVE。')
        dpg.set_value('params_intro','OneWheel JustFloat + MCAR SUB：Mode2双轴平衡、地轮速度环与位置保持、Mode8独立PWM及编码器观测；开始监听不启动电机，Start/Stop、模式和Flash保存使用板载菜单。位置保持参数仅RAM，不保存板载Flash；位置环100ms、速度环约20ms，首次就绪锁定位置，Stop→Start重设保持点。位置环需实车调试，不宣称已解决平衡。')
        dpg.configure_item('image_connection',show=False)
        dpg.configure_item('nav_camera',enabled=False,label='04  图像（固件未发送）')
        dpg.set_value('camera_intro','当前Qcar UDP协议未发送摄像头图像。')
        # Preserve the rules table, replace obsolete Mcar field presets.
        heading=dpg.get_item_children('rules_panel',1)[0]
        dpg.delete_item(heading)
        with dpg.group(horizontal=True,parent='rules_panel',before='rules_table'):
            self.heading('设备订阅字段（自动同步）')
            dpg.add_combo(list(PROFILE_LABELS),default_value='启动诊断 · 39通道',tag='qcar_profile',width=240)
            dpg.add_button(label='请求此方案',callback=self.guard(self.subscribe_profile))
            dpg.add_button(label='查询订阅/周期',callback=self.guard(lambda:self.command('GET?')))
        with dpg.group(horizontal=True,parent='params_tab',before='rules_panel'):
            dpg.add_button(label='暂停遥测（非停车）',callback=self.guard(lambda:self.command('STREAM 0')))
            dpg.add_button(label='恢复遥测',callback=self.guard(lambda:self.command('STREAM 1')))
            dpg.add_input_int(label='周期ms',tag='qcar_period',default_value=20,min_value=2,max_value=1000,min_clamped=True,max_clamped=True,width=155)
            dpg.add_button(label='设置周期',callback=self.guard(lambda:self.command('RATE '+str(dpg.get_value('qcar_period')))))
        dpg.add_text('',tag='qcar_connection',parent='params_tab',before='rules_panel',wrap=1200)
        self.load_rules([FieldRule(n,'float32',4*i) for i,n in enumerate(PROFILES['startup'])])
        self.collect_rules()
        dpg.delete_item('tuning_tab',children_only=True)
        with dpg.group(parent='tuning_tab'):
            self.heading('Qcar 在线参数 / 设备回执')
            dpg.add_text('',tag='qcar_state',wrap=1250)
            dpg.add_text('启停、Mode切换、Flash保存请操作板载菜单。修改在RAM中立即生效；本地保存方案不等于写入板载Flash。',wrap=1250,color=(255,190,75))
            dpg.add_text('Mode8：先板端选8并Start，再设置下方三路PWM。无PID/倾角保护，需架空；Start/Stop/切模式清零。',wrap=1250)
            with dpg.group(horizontal=True):
                dpg.add_button(label='订阅Mode8设定/实际输出',callback=self.guard(self.subscribe_pwm_debug))
                dpg.add_button(label='三路PWM归零（非Stop）',callback=self.guard(self.zero_pwm_debug))
            dpg.add_text('',tag='qcar_pwm_diagnostic',wrap=1250)
            with dpg.group(horizontal=True):
                dpg.add_button(label='订阅地轮编码器（停车手转）',callback=self.guard(self.subscribe_encoder))
            dpg.add_text('',tag='qcar_encoder_diagnostic',wrap=1250)
            with dpg.group(horizontal=True):
                dpg.add_button(label='订阅Mode2速度环（43通道）',callback=self.guard(self.subscribe_speed))
            dpg.add_text('',tag='qcar_speed_diagnostic',wrap=1250)
            with dpg.group(horizontal=True):
                dpg.add_button(label='订阅Mode2位置保持（48通道）',callback=self.guard(self.subscribe_hold))
            dpg.add_text('位置环100ms、速度环约20ms；首次就绪锁定位置，Stop→Start重设保持点；速度环KP和KI都为0时位置环不产生倾角修正。',tag='qcar_hold_note',wrap=1250)
            dpg.add_text('',tag='qcar_hold_diagnostic',wrap=1250)
            dpg.add_text('',tag='qcar_readonly',wrap=1250)
            with dpg.group(horizontal=True):
                dpg.add_input_text(tag='qcar_scheme',hint='本地方案路径；留空使用数据目录/qcar_scheme.json',width=650)
                dpg.add_button(label='保存本地编辑方案',callback=self.guard(self.save_qcar_scheme))
                dpg.add_button(label='导入编辑方案',callback=self.guard(self.load_qcar_scheme))
            dpg.add_text(self.qcar_message,tag='qcar_tuning_message',wrap=1250)
            with dpg.table(header_row=True,resizable=True,borders_innerH=True,borders_innerV=True,scrollY=True,height=420,width=-1):
                for label in ('参数','编辑值','最近匹配写回执','说明','应用状态'):dpg.add_table_column(label=label)
                for key,(label,default,help_text) in PARAMETERS.items():
                    with dpg.table_row():
                        dpg.add_text(label+'\n'+key,wrap=230)
                        dpg.add_input_double(tag='qcar_edit_'+key,default_value=default,width=-1,step=0,format='%.8g')
                        dpg.add_text('未确认',tag='qcar_ack_'+key)
                        dpg.add_text(help_text,wrap=260)
                        with dpg.group():
                            dpg.add_button(label='应用此项',callback=self.guard(lambda k=key:self.apply_qcar(k)))
                            dpg.add_text('未发送',tag='qcar_result_'+key,wrap=240)
        if not self.quick_path.exists():
            commands=[]
            for label,payload,note in [('查询当前订阅','GET?','不读取全部PID'),('列出可订阅字段','LIST?','最多48字段同时上传'),
                                       ('暂停遥测','STREAM 0','仅停止数据上传，不停车'),('恢复遥测','STREAM 1','不启动电机')]:
                row=new_command(label);row.update(content=payload,ending='LF',note=note);commands.append(row)
            self.quick_config.update(target_port=5001,commands=commands)
            self.rebuild_quick_rows()

    def load_rules(self,rules):
        super().load_rules(rules)
        for row in self.rule_rows:
            for child in dpg.get_item_children(row,1):dpg.configure_item(child,enabled=False)

    def collect_rules(self):
        changed=super().collect_rules()
        for name in self._plot_names:
            if not getattr(self,'_chosen_schema',None)==tuple(r.name for r in self.rules):
                dpg.set_value('check_'+name,name in DEFAULT_PLOTS)
        self._chosen_schema=tuple(r.name for r in self.rules)
        return changed

    def resize_layout(self):
        width,height=dpg.get_item_rect_size('content')
        if width<100 or height<100:return
        field_width=max((dpg.get_text_size(r.name)[0] for r in self.rules),default=300)
        sidebar=dpg.get_value('plot_sidebar_width')
        key=(round(width),round(height),self.active_page,sidebar,round(field_width))
        if key==self._layout_size:return
        self._layout_size=key
        content=max(880,int(width)-32)
        dpg.configure_item('udp_listen_panel',width=(content-8)//2)
        dpg.configure_item('listen_note',wrap=(content-8)//2-20)
        dpg.configure_item('csv_path',width=max(230,content-450))
        dpg.configure_item('rules_panel',height=360)
        dpg.configure_item('running_logs',height=180)
        mode_width=max(dpg.get_text_size(t)[0] for t in ('UTF-8 文字','十六进制'))
        dpg.configure_item('send_mode',width=max(220,round(mode_width)+55))
        for tag in ('params_intro','plot_intro','quick_intro','quick_target','quick_help','quick_binary_help','quick_status',
                    'qcar_state','qcar_readonly','qcar_tuning_message','qcar_connection','qcar_pwm_diagnostic',
                    'qcar_encoder_diagnostic','qcar_speed_diagnostic','qcar_hold_diagnostic','qcar_hold_note'):
            dpg.configure_item(tag,wrap=content)
        dpg.configure_item('quick_file_path',width=max(220,content-400))
        plot_height=max(500,int(height)-140)
        maximum=max(360,min(850,content-340))
        sidebar=max(360,min(maximum,sidebar))
        dpg.configure_item('plot_sidebar_width',max_value=maximum)
        dpg.set_value('plot_sidebar_width',sidebar)
        dpg.configure_item('plot_controls',width=sidebar,height=plot_height)
        dpg.configure_item('values_table',height=round(plot_height*.42),inner_width=round(field_width)+170)
        dpg.configure_item('value_name_column',init_width_or_weight=round(field_width)+24)
        dpg.configure_item('plot_choices_panel',height=round(plot_height*.28))
        dpg.configure_item('plot',height=plot_height)
        dpg.configure_item('pose',width=content,height=max(350,int(height)-250))

    def _target(self):
        ip=dpg.get_value('remote_ip').strip()
        ipaddress.IPv4Address(ip)
        port=dpg.get_value('remote_port')
        if not 1<=port<=65535:raise ValueError('模块命令端口必须为1～65535')
        if not any(r.running for r in self.receivers.values()):raise ValueError('先开始监听，才能接收车辆回执')
        return ip,port

    def command(self,text):
        target=self._target()
        send_udp((text.strip()+'\n').encode('ascii'),*target)
        self.status('已发送，等待设备回复：'+text.split(' ',1)[0])

    def subscribe_profile(self):
        key=PROFILE_LABELS[dpg.get_value('qcar_profile')]
        self.command('SUB '+','.join(PROFILES[key])+'\nRATE 20')

    def _invalidate_ack(self,reason):
        self.pending.clear();self.confirmed.clear()
        self.qcar_states={key:reason for key in PARAMETERS}

    def start(self):
        if self.multiplex.receiver.running:return
        self.protocol=Protocol();self.peer=None;self.last_announced=()
        self._invalidate_ack('新接收会话，未确认')
        self.latest.clear();self.last_data=0
        self._start_shared()
        self.status('JustFloat监听已启动，等待车端MCAR SUB；这不是电机Start。')

    def stop(self):
        super().stop()
        self._invalidate_ack('监听已停止，确认失效')
        self.peer=None;self.protocol=Protocol();self.last_announced=()

    def set_receive_mode(self):pass

    def _accept_peer(self,address):
        if self.peer is None:
            self.peer=address
            if not dpg.get_value('remote_ip').strip():
                dpg.set_value('remote_ip',address[0])
                dpg.set_value('remote_port',address[1])
            self.qcar_target=(dpg.get_value('remote_ip').strip(),dpg.get_value('remote_port'))

    def observe_module(self,ip):
        # The Qcar receiver owns explicit peer selection; no legacy ALIVE/binding.
        return

    def receive(self):
        for packet in self.multiplex.receiver.drain(4096):
            configured=dpg.get_value('remote_ip').strip()
            if (self.peer and packet.address!=self.peer) or (configured and configured!=packet.address[0]):
                self.foreign_packets+=1;continue
            binary=bool(self.protocol.fields) and len(packet.data)==4*len(self.protocol.fields)+4 and packet.data.endswith(b'\x00\x00\x80\x7f')
            payloads=packet.data.splitlines() if not binary and packet.data.startswith(b'MCAR ') else [packet.data]
            for payload in payloads:
                if not self.protocol.fields and not payload.startswith(b'MCAR '):
                    self.waiting_packets+=1;continue
                try:event=self.protocol.feed(payload)
                except (ValueError,UnicodeError):
                    self.parse_errors+=1;continue
                self._accept_peer(packet.address)
                kind=event['kind']
                if kind=='schema':
                    fields=event['fields']
                    if fields!=self.last_announced:
                        for label,profile in PROFILE_LABELS.items():
                            if tuple(PROFILES[profile])==fields:dpg.set_value('qcar_profile',label)
                        self.last_announced=fields
                    if tuple(r.name for r in self.rules)!=fields:
                        self.load_rules([FieldRule(n,'float32',4*i) for i,n in enumerate(fields)])
                        self.collect_rules()
                        self.latest.clear();self.last_data=0;self.has_pose=False
                    self.qcar_message=f'已同步{len(fields)}通道，等待对应数据。'
                if kind=='data':
                    values=event['values'];self.latest.update(values)
                    self.last_data=packet.stamp;self.data_times.append(packet.stamp)
                    for alias,name in [('roll','roll_deg'),('pitch','pitch_deg'),('yaw','yaw_deg')]:
                        if name in values:self.latest[alias]=values[name]
                    if self.history_origin is None:self.history_origin=packet.stamp
                    for name,value in values.items():self.history.setdefault(name,deque(maxlen=self.max_samples)).append((packet.stamp,value))
                    self.data_file.write(json.dumps(dict(timestamp=datetime.fromtimestamp(packet.stamp).isoformat(timespec='milliseconds'),address=packet.address,values=values),ensure_ascii=False)+'\n')
                    self.data_records+=1;self.receivers['data'].received+=1
                else:
                    self.receivers['log'].received+=1
                    self.append_logs([LogLine(packet.stamp,f'{packet.address[0]}:{packet.address[1]}',payload.decode('ascii'))])
                    if kind=='slider':
                        key=event['name'];request=self.pending.get(key)
                        if request and request['target']==packet.address and time.monotonic()-request['time']<=2 and math.isclose(request['value'],event['value'],rel_tol=1e-6,abs_tol=1e-6):
                            self.confirmed[key]=event['value'];self.qcar_states[key]='设备已确认RAM写入（未保存Flash）';del self.pending[key]
                    elif kind=='error':
                        self.qcar_message=event['text']
                        for key in self.pending:self.qcar_states[key]='设备报错，未确认：'+event['text']
                        self.pending.clear()

    def apply_qcar(self,key):
        payload=slider_command(key,dpg.get_value('qcar_edit_'+key))
        target=self._target()
        if self.pending:raise ValueError('等待上一项回执或超时后再应用')
        self.confirmed.pop(key,None)
        send_udp(payload,*target)
        self.pending[key]=dict(target=target,value=float(dpg.get_value('qcar_edit_'+key)),time=time.monotonic())
        self.qcar_states[key]='已发送，尚未确认'

    def subscribe_pwm_debug(self):
        self.command('SUB '+','.join(PWM_PROFILE)+'\nRATE 20')

    def subscribe_encoder(self):
        self.command('SUB '+','.join(PROFILES['encoder'])+'\nRATE 20')

    def subscribe_speed(self):
        self.command('SUB '+','.join(PROFILES['speed'])+'\nRATE 20')

    def subscribe_hold(self):
        self.command('SUB '+','.join(PROFILES['hold'])+'\nRATE 20')

    def zero_pwm_debug(self):
        # Clear outputs; this neither switches mode nor disables the controller.
        if self.pending:raise ValueError('等待上一项回执或超时后再归零')
        target=self._target()
        payload=b''.join(slider_command(key,0) for key in DEBUG_PARAMETERS)
        send_udp(payload,*target)
        now=time.monotonic()
        for key in DEBUG_PARAMETERS:
            dpg.set_value('qcar_edit_'+key,0)
            self.confirmed.pop(key,None)
            self.pending[key]=dict(target=target,value=0.0,time=now)
            self.qcar_states[key]='已发送归零，尚未确认'

    def refresh_tuning(self):
        target=(dpg.get_value('remote_ip').strip(),dpg.get_value('remote_port'))
        if target!=self.qcar_target:
            self._invalidate_ack('发送对象已更改，确认失效')
            self.peer=None;self.protocol=Protocol();self.latest.clear();self.last_data=0
            self.last_announced=()
            self.qcar_target=target
        for key,request in list(self.pending.items()):
            if time.monotonic()-request['time']>2:
                self.qcar_states[key]='回执超时，不能确认；未自动重发';del self.pending[key]
        fresh=self.last_data and time.time()-self.last_data<1 and any(r.running for r in self.receivers.values())
        if fresh:
            control_mode=self.latest.get('control_mode')
            if control_mode is not None:
                try:shown_mode=f'{control_mode:g}'
                except (TypeError,ValueError):shown_mode=str(control_mode)
                mode_text='实际ControlMode='+shown_mode+('（未启动）' if control_mode==0 else '')
            else:
                axis=self.latest.get('rate_axis')
                mode=8 if self.latest.get('pwm_debug_enabled')==1 else 5 if self.latest.get('dual_enabled')==1 else (7 if axis==1 else 6) if axis in (0,1) and self.latest.get('rate_only')==1 else (4 if axis==1 else 3) if axis in (0,1) and self.latest.get('rate_only')==0 else '未订阅模式判据'
                mode_text=f'推得Mode={mode}'
            control=self.latest.get('rate_state')
            meaning={0:'停止',1:'等待IMU',2:'控制运行',-1:'IMU/样本故障',-2:'倾角保护',-3:'参数故障',-4:'编码器故障/位置不连续'}.get(control,'未订阅')
            if control==1 and control_mode==2:meaning='等待IMU/编码器窗口'
            state=f'{mode_text}；设备State={control}（{meaning}）；IMU={self.latest.get("imu_status","未订阅")}；F={self.latest.get("fault_reason","未订阅")}'
        else:state='遥测未到达或已过期，当前控制/IMU状态未知'
        dpg.set_value('qcar_state',state+'；屏幕Roll=内部pitch，屏幕Pitch=内部roll。')
        def value_text(name):
            value=self.latest.get(name) if fresh else None
            return f'{value:g}' if value is not None else '未订阅/过期'
        diagnostic='；'.join(label+': '+value_text(name) for label,name in (
            ('左设定','debug_fly_left_pwm'),('右设定','debug_fly_right_pwm'),('地轮设定','debug_ground_pwm'),
            ('左下发','motor_left_pwm'),('右下发','motor_right_pwm'),('地轮下发','motor_ground_pwm'),
            ('左转速反馈','flywheel_left_speed_raw'),('右转速反馈','flywheel_right_speed_raw')))
        dpg.set_value('qcar_pwm_diagnostic',diagnostic)
        encoder_fields=('wheel_encoder_delta','wheel_speed_cps','wheel_position_count','wheel_encoder_seq',
                        'wheel_encoder_valid','wheel_position_valid','wheel_encoder_errors')
        if fresh and any(name in self.latest for name in encoder_fields):
            def encoder_text(name):
                value=self.latest.get(name)
                return f'{value:g}' if value is not None else '等待接收'
            encoder='；'.join(label+': '+encoder_text(name) for label,name in (
                ('delta','wheel_encoder_delta'),('speed(cnt/s)','wheel_speed_cps'),
                ('position(count)','wheel_position_count'),('seq','wheel_encoder_seq'),
                ('valid','wheel_encoder_valid'),('pos_valid','wheel_position_valid'),
                ('errors','wheel_encoder_errors')))
            encoder+='。valid只表示采样时序有效，不证明编码器接线正常；pos_valid=0表示长停顿等造成累计位置不连续；此观测不启动电机。'
        else:
            encoder='等待“地轮编码器”订阅的新鲜数据；未收到时显示等待而非0。此观测不启动电机。'
        dpg.set_value('qcar_encoder_diagnostic',encoder)
        speed_fields=('wheel_target_cps','wheel_speed_cps','wheel_tilt_offset','wheel_speed_integral',
                      'wheel_feedback_age_ms','wheel_feedback_valid','wheel_hold_state')
        if fresh and any(name in self.latest for name in speed_fields):
            valid=self.latest.get('wheel_feedback_valid')
            valid_text=({1:'有效',0:'无效'}.get(valid,'等待接收') if valid is not None else '未订阅')
            speed='；'.join((
                '速度目标='+value_text('wheel_target_cps')+' cnt/s',
                '实际速度='+value_text('wheel_speed_cps')+' cnt/s',
                '倾角修正='+value_text('wheel_tilt_offset')+'°',
                'PI积分='+value_text('wheel_speed_integral'),
                '反馈年龄='+value_text('wheel_feedback_age_ms')+'ms',
                '反馈有效='+valid_text,
                '外环状态='+value_text('wheel_hold_state')))
        else:
            speed='等待“Mode2速度环”订阅的新鲜数据（速度目标/实际、倾角修正、PI积分、反馈年龄/有效性、外环状态）；未收到或过期不显示0。'
        dpg.set_value('qcar_speed_diagnostic',speed)
        hold_markers=('wheel_ref_count','wheel_pos_error','wheel_pos_kp','wheel_speed_limit')
        if fresh and any(name in self.latest for name in hold_markers):
            def hold_text(name):
                value=self.latest.get(name)
                return f'{value:g}' if value is not None else '等待接收'
            pos_valid=self.latest.get('wheel_position_valid')
            pos_valid_text=({1:'有效',0:'无效'}.get(pos_valid,'等待接收') if pos_valid is not None else '未订阅')
            reference=hold_text('wheel_ref_count')+' count'
            if self.latest.get('control_mode')==0 or self.latest.get('rate_state')==1:
                reference='未锁定（停车/等待就绪）'
            hold='；'.join((
                '累计位置='+hold_text('wheel_position_count')+' count',
                '参考位置='+reference,
                '控制位置误差='+hold_text('wheel_pos_error')+' count',
                '目标速度='+hold_text('wheel_target_cps')+' cnt/s',
                '位置有效='+pos_valid_text))
            hold+='。F10表示位置不连续；State=-4也可能是编码器无效/过期，请结合F判断。'
            if self.latest.get('wheel_speed_kp')==0 and self.latest.get('wheel_speed_ki')==0:
                hold+='速度PI已关闭，位置请求不会产生倾角修正。'
        else:
            hold='等待“Mode2位置保持”订阅的新鲜数据（累计位置/参考位置/误差/目标速度/位置有效标志）；F10表示位置不连续。'
        dpg.set_value('qcar_hold_diagnostic',hold)
        fields=('balance_kp','rate_kp','rate_sign','balance_rate_limit_dps','rate_pwm_limit',
                'fly_angle_kp','fly_rate_kp','fly_sign','fly_rate_limit_dps','ground_angle_kp','ground_rate_kp','ground_sign','ground_rate_limit_dps')
        readonly='；'.join(f'{key}={self.latest.get(key,"未订阅")}' for key in fields) if fresh else '等待新鲜遥测；设备参数不能从编辑默认值推断。'
        dpg.set_value('qcar_readonly',readonly)
        dpg.set_value('qcar_tuning_message',self.qcar_message)
        dpg.set_value('qcar_connection',f'字段={len(self.protocol.fields)}；设备周期={self.protocol.period_ms}ms；遥测开关={self.protocol.stream}；等待字段表的数据包={self.waiting_packets}；其他设备包={self.foreign_packets}')
        for key in PARAMETERS:
            value=self.confirmed.get(key)
            dpg.set_value('qcar_ack_'+key,f'{value:g}' if value is not None else '未确认')
            dpg.set_value('qcar_result_'+key,self.qcar_states[key])

    def refresh(self):
        super().refresh()
        if self.active_page=='imu' and (not all(k in self.latest for k in ('roll','pitch','yaw')) or not self.last_data or time.time()-self.last_data>=1):
            dpg.set_value('angles','当前订阅不含完整三轴姿态或数据已过期；选择“三轴姿态”方案，不能从单轴角度拼造3D。')

    def _scheme_path(self):
        return Path(dpg.get_value('qcar_scheme').strip()) if dpg.get_value('qcar_scheme').strip() else app.STORE/'qcar_scheme.json'

    def save_qcar_scheme(self):
        values={key:validate_parameter(key,dpg.get_value('qcar_edit_'+key)) for key in PARAMETERS}
        path=self._scheme_path();path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(dict(version='QCAR_JUSTFLOAT_4',values=values),ensure_ascii=False,indent=2),encoding='utf-8')
        self.qcar_message='仅保存本地方案（21项），没有发送参数或保存板载Flash。'

    def load_qcar_scheme(self):
        data=json.loads(self._scheme_path().read_text('utf-8-sig'))
        if not isinstance(data,dict):raise ValueError('方案文件必须是JSON对象')
        version=data.get('version');values=data.get('values')
        if version=='QCAR_JUSTFLOAT_4':expected=frozenset(PARAMETERS)
        elif version=='QCAR_JUSTFLOAT_3':expected=V3_PARAMETERS
        elif version=='QCAR_JUSTFLOAT_2':expected=V2_PARAMETERS
        elif version=='QCAR_JUSTFLOAT_1':expected=LEGACY_PARAMETERS
        else:raise ValueError('仅支持QCAR_JUSTFLOAT_4（21项）、V3（原19项）、V2（原15项）或V1（5项）方案')
        if not isinstance(values,dict) or frozenset(values)!=expected:
            raise ValueError(f'{version}方案必须恰好包含{len(expected)}项且不能有多余字段')
        checked={}
        for key in expected:
            try:checked[key]=validate_parameter(key,values[key])
            except ValueError as exc:raise ValueError(f'{key} 无效：{exc}') from None
        for key,number in checked.items():dpg.set_value('qcar_edit_'+key,number)
        self.qcar_message=f'已导入{version}（{len(checked)}项）到编辑栏，没有发送修改。'

    def run(self):
        dpg.show_viewport();started=time.monotonic()
        if self.args.listen_test:self.start()
        self.show_page(self.args.page)
        try:
            while dpg.is_dearpygui_running():
                callbacks=dpg.get_callback_queue()
                if callbacks:dpg.run_callbacks(callbacks)
                self.refresh();dpg.render_dearpygui_frame()
                if self.args.startup_report:
                    Path(self.args.startup_report).write_text(json.dumps(dict(profile='qcar',started=True,storage=str(app.STORE),data_port=dpg.get_value('data_port'),remote_port=dpg.get_value('remote_port'))),encoding='utf-8')
                    self.args.startup_report=None
                if self.args.exit_after and time.monotonic()-started>=self.args.exit_after:
                    if self.data_records:self.save_csv()
                    self.save_log()
                    if self.args.test_report:
                        Path(self.args.test_report).write_text(json.dumps(dict(profile='qcar',records=self.data_records,parse_errors=self.parse_errors,fields=self.protocol.fields,latest=self.latest,confirmed=self.confirmed,output=str(self.output),active_page=self.active_page,peer=self.peer,remote_target=[dpg.get_value('remote_ip'),dpg.get_value('remote_port')],
                            plotted_points={n:len(dpg.get_value('series_'+n)[0]) for n in self._plot_names if dpg.get_value('check_'+n)},waiting_packets=self.waiting_packets,foreign_packets=self.foreign_packets)),encoding='utf-8')
                    break
        finally:
            self.stop();self.sync_quick_commands();self.save_quick_commands();self.save_settings()
            self.log_file.close();self.data_file.close();dpg.destroy_context()
