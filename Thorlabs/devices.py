"""
Created on Tue Jun  30 18:23:39 2023
@author: Harun
"""

import thorlabs_apt_protocol as apt
import serial
import pandas as pd
import numpy as np
from time import sleep

class LTS150:
    def __init__(self, portn="/dev/ttyUSB0", baud=115200, dest=0x50, source=1, chan_ident=1):
        self.portn=portn
        self.baud=baud
        self.source=source
        self.dest=dest
        self.chan_ident=chan_ident
        self.max_vel=329790656
        self.max_acc=45034
        self.max_pos=409600

    def initialize(self):
        port = serial.Serial(self.portn, self.baud, rtscts=True, timeout=0.1)
        port.rts = True
        port.reset_input_buffer()
        port.reset_output_buffer()
        port.rts = False
        # This message is sent on start up to notify the controller of the 
        # source and destination addresses. A client application must send 
        # this message as part of its initialization process.
        port.write(apt.hw_no_flash_programming(self.dest, self.source))
        self.port = port
        print('Device Initialized')
        return port

    def prnt(self, msg_list):
        # Printing the received messages from the controller
        unpacker = apt.Unpacker(self.port)
        all_msg=[]
        for msg in unpacker:
            all_msg.append(msg)
            # print(msg)
        #breakpoint()
        all_msg = np.array(all_msg, dtype=object)
        all_msg = pd.DataFrame(all_msg, columns=msg_list)
        return all_msg
        
    def update(self):
        # Update the variables by asking the device        
        p_hardware = self.msgp_hardware()
        self.var_serial_number = p_hardware['serial_number']
        self.var_model_number = p_hardware['model_number']
        self.var_type = p_hardware['type']
        self.var_firmware_version = p_hardware['firmware_version']
        self.var_hw_version = p_hardware['hw_version']
        self.var_no_channels = p_hardware['nchs']
        
        p_velparam = self.msgp_velparam()
        self.var_min_velocity = int(p_velparam['min_velocity'].iloc[0])
        self.var_acceleration = int(p_velparam['acceleration'].iloc[0])
        self.var_max_velocity = int(p_velparam['max_velocity'].iloc[0])
    
        p_jogparam = self.msgp_jogparam()
        self.var_jog_mode = int(p_jogparam['jog_mode'].iloc[0])
        self.var_jog_step_size = int(p_jogparam['step_size'].iloc[0])
        self.var_jog_min_velocity = int(p_jogparam['min_velocity'].iloc[0])
        self.var_jog_acceleration = int(p_jogparam['acceleration'].iloc[0])
        self.var_jog_max_velocity = int(p_jogparam['max_velocity'].iloc[0])
        self.var_jog_stop_mode = int(p_jogparam['stop_mode'].iloc[0])
        
        p_backlash = self.msgp_backlash()
        self.var_backlash_distance = int(p_backlash['backlash_distance'].iloc[0])
        
        p_relmov = self.msgp_relmov()
        self.var_relative_distance = int(p_relmov['relative_distance'].iloc[0])
        
        p_absmov=self.msgp_absmov()
        self.var_absolute_position = int(p_absmov['absolute_position'].iloc[0])
        
        p_homeparam = self.msgp_homeparam()
        self.var_home_direction = int(p_homeparam['home_dir'].iloc[0])
        self.var_home_limit_switch = int(p_homeparam['limit_switch'].iloc[0])
        self.var_home_velocity = int(p_homeparam['home_velocity'].iloc[0])
        self.var_home_offset_distance = int(p_homeparam['offset_distance'].iloc[0])
        
        
        self.jog_f()
        sleep(2)
        self.jog_r()
        sleep(2)
        p_jog=self.msgp_jog()
        sleep(2)
        self.var_jog_position = p_jog['position']
        self.var_jog_velocity = p_jog['velocity'].iloc[0]
        self.var_jog_homed = p_jog['homed'].iloc[0]
        print('Data Updated >>>>>>>>>>>>>>>>>>>')

#####################################################################################
# High-Level Methods for working with the motor
#####################################################################################
# port.write(apt.mot_move_velocity(source=1, dest=0x50, chan_ident=1, direction=2))

    def move_abspos(self, position):
        # Set the object to a absolute position from 0mm to 150mm in integer
        self.position = int(float(position)*self.max_pos)
        self.msgs_absmov(self.position)
        self.port.write(apt.mot_move_absolute(self.dest, self.source, self.chan_ident))

    def move_reldis(self, relative_distance=1): 
        # Shift the object relative to its current position
        self.relative_distance = int(float(relative_distance)*self.max_pos)
        self.msgs_relmov(relative_distance=self.relative_distance)
        self.port.write(apt.mot_move_relative(self.dest, self.source, self.chan_ident))
               
    def jog_f(self):
        self.port.write(apt.mot_move_jog(self.dest,self.source, self.chan_ident, direction=1)) # to 150
            
    def jog_r(self):
        self.port.write(apt.mot_move_jog(self.dest, self.source, self.chan_ident, direction=2)) # to Home
    
    def stop_hard(self):
        self.port.write(apt.mot_move_stop(self.dest, self.source, self.chan_ident, stop_mode=1))
    
    def stop_soft(self):
        self.port.write(apt.mot_move_stop(self.dest, self.source, self.chan_ident, stop_mode=2))
    
    def disconnect(self):
        self.port.close()
        print('Device disconnected')
        
    def move_home(self):
        self.port.write(apt.mot_move_home(self.dest, self.source, self.chan_ident))
        
######################################################################
# High-Level Methods for setting simple parameters of the device
######################################################################
    def set_jog_vel(self, max_vel):
        # Give a value from 0 to 100%
        self.var_jog_max_velocity = int((float(max_vel)/100) * self.max_vel);
        self.var_jog_min_velocity = int(0);
        self.msgs_jogparam(self.var_jog_step_size, self.var_jog_min_velocity, self.var_jog_acceleration, self.var_jog_max_velocity)
        
    def set_jog_acc(self, acc):
        # Give the value from 0 to 100%
        self.var_jog_acceleration = int((float(acc)/100) * self.max_acc)
        self.msgs_jogparam(self.var_jog_step_size, self.var_jog_min_velocity, self.var_jog_acceleration, self.var_jog_max_velocity)
        
    def set_jog_step_size(self, step_size):
        # Give a value for the step size in mm
        self.var_jog_step_size = int(float(step_size * self.max_pos))
        self.msgs_jogparam(self.var_jog_step_size, self.var_jog_min_velocity, self.var_jog_acceleration, self.var_jog_max_velocity)
    
    def set_vel(self, max_vel):
        # Give a value from 0 to 100%
        self.var_min_velocity = int(0)
        self.var_max_velocity = int((float(max_vel)/100) * self.max_vel)
        self.msgs_velparam(self.var_min_velocity, self.var_acceleration, self.var_max_velocity)
    
    def set_acc(self, acc):
        # Give the value from 0 to 100%
        self.var_acceleration = int((float(acc)/100) * self.max_acc)
        self.msgs_velparam(self.var_min_velocity, self.var_acceleration, self.var_max_velocity)
        
    def set_home_vel(self, vel):
        # Give the value from 0 to 100%
        self.var_home_velocity = int((float(vel)/100) * self.max_vel)
        self.msgs_homeparam(self.var_home_velocity, self.var_home_offset_distance)
    
    
#####################################################
# The following Methods are to communicate with the controller of the hardware. msgp_...-methods are to get an answer from the controller and msgs_...-methods are to send messages to the controller, with which you can change settings of the controller.
#####################################################

    def msgp_jog(self):
        msg_list=[
         'msg', 'msgid', 'dest', 'source', 'chan_ident', 'position', 'velocity',
         'forward_limit_switch', 'reverse_limit_switch', 'forward_limit_soft', 'reverse_limit_soft', 'moving_forward',
         'moving_reverse', 'jogging_forward', 'jogging_reverse', 'motor_connected', 'homing', 'homed', 'initializing', 'tracking', 'settled', 'motion_error', 'instrument_error', 'interlock', 'overtemp', 'voltage_fault', 'commutation_error', 'dig_ins', 'motor_current_limit_reached', 'encoder_fault', 'overcurrent', 'current_fault', 'power_ok', 'active', 'error', 'channel_enabled']
        # for jog
        try:
            p_jog = self.prnt(msg_list)
            return p_jog
        except:
            print('Error reading fun: msgp_jog')
            return 0

    def msgp_hardware(self):
        # Sent to request hardware information from the controller
        msg_list2=[
        'msg', 'msgid', 'dest', 'source', 'serial_number', 'model_number', 'type', 
        'firmware_version', 'hw_version', 'mod_state', 'nchs']
        
        # for hardwareinfo
        self.port.write(apt.hw_req_info(self.dest, self.source))
        try:
            p_hardware=self.prnt(msg_list2)
            return p_hardware
        except:
            print('Error setting fun: msgp_hardware')
            return 0

    def msgp_velparam(self):
        # MGMSG_MOT_SET_VELPARAMS
        msg_list3=[
        'msg', 'msgid', 'dest', 'source', 'chan_ident', 'min_velocity',
        'acceleration', 'max_velocity']
        # for hardwareinfo
        self.port.write(apt.mot_req_velparams(self.dest, self.source, self.chan_ident))
        try:
            p_velparam=self.prnt(msg_list3)
            return p_velparam
        except:
            print('Error reading fun: msgs_velparam')
            return 0
        
    def msgs_velparam(self, min_velocity, acceleration, max_velocity):
        # Used to set the trapezoidal velocity parameters for the specified
        # motor channel. For DC servo controllers, the velocity is set in
        # encoder counts/sec and acceleration is set in encoder counts/sec/sec.
        # For stepper motor controllers the velocity is set in microsteps/sec
        # and acceleration is set in microsteps/sec/sec.
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_velparams(self.dest, self.source,
                                             self.chan_ident,
                                             min_velocity=min_velocity,
                                             acceleration=acceleration,
                                             max_velocity=max_velocity))
            print('Done')
        except:
            print('Error setting fun: msgs_velparam')
        return 0

    def msgp_jogparam(self):
        # MGMSG_MOT_SET_JOGPARAMS
        msg_list4=[
            'msg', 'msgid', 'dest', 'source', 'chan_ident', 'jog_mode', 'step_size', 
            'min_velocity', 'acceleration', 'max_velocity', 'stop_mode']
        # for hardwareinfo
        self.port.write(apt.mot_req_jogparams(self.dest, self.source, self.chan_ident))
        try:
            p_jogparam=self.prnt(msg_list4)
            return p_jogparam
        except:
            print('Error reading fun: msgp_jogparam')
            return 0

    def msgs_jogparam(self, step_size, min_velocity, acceleration, max_velocity, jog_mode=2, stop_mode=2):
        # Used to set the velocity jog parameters for the specified motor
        # channel, For DC servo controllers, values set in encoder counts.
        # For stepper motor controllers the values is set in microsteps
        # for hardwareinfo:
        try:
            self.port.write(apt.mot_set_jogparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             jog_mode=jog_mode,
                                             step_size=step_size,
                                             min_velocity=min_velocity,
                                             acceleration=acceleration,
                                             max_velocity=max_velocity,
                                             stop_mode=stop_mode))
            
            print('Done')
        except:
            print('Error setting fun: msgs_jogparam')
        return 0

    def msgp_backlash(self):
        # MGMSG_MOT_SET_GENMOVEPARAMS
        msg_list5=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'backlash_distance']
        # for hardwareinfo
        self.port.write(apt.mot_req_genmoveparams(self.dest,self.source, self.chan_ident))
        try:
            p_backlash=self.prnt(msg_list5)
            return p_backlash
        except:
            print('Error reading fun: msgp_backlash')
            return 0
        
    def msgs_backlash(self, backlash_distance):
        # Used to set the general move parameters for the specified motor
        # channel. At this time this refers specifically to the backlash settings
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_genmoveparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             backlash_distance=backlash_distance))                                         
            print('Done backlash_distance')
        except:
            print('Error setting fun: msgs_backlash')
        return 0

    def msgp_relmov(self):
        # MGMSG_MOT_SET_MOVERELPARAMS
        msg_list7=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'relative_distance']
        # for hardwareinfo
        self.port.write(apt.mot_req_moverelparams(self.dest,self.source, self.chan_ident))
        try:
            p_relmov=self.prnt(msg_list7)
            return p_relmov
        except:
            print('Error reading fun: msgp_relmov')
            return 0
        
    def msgs_relmov(self, relative_distance= 0):
        # Used to set the general move parameters for the specified motor
        # channel. At this time this refers specifically to the backlash settings
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_moverelparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             relative_distance=relative_distance))                                         
            print('Done relative_distance')
        except:
            print('Error setting fun: msgs_relmov')
        return 0

    def msgp_absmov(self):
        # MGMSG_MOT_SET_MOVEABSPARAMS
        msg_list8=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'absolute_position']
        # for hardwareinfo
        self.port.write(apt.mot_req_moveabsparams(self.dest,self.source, self.chan_ident))
        try:
            p_absmov=self.prnt(msg_list8)
            return p_absmov
        except:
            print('Error reading fun: msgp_absmov')
            return 0
    
    def msgs_absmov(self, absolute_position=0):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_moveabsparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             absolute_position=absolute_position))                                         
            print('Done absolute_position')
        except:
            print('Error setting fun: msgs_absmov')
        return 0

    def msgp_homeparam(self):
        # MGMSG_MOT_SET_HOMEPARAMS
        msg_list9=['msg', 'msgid', 'dest', 'source','chan_ident', 
               'home_dir', 'limit_switch', 'home_velocity', 'offset_distance']
        
        # for hardwareinfo
        self.port.write(apt.mot_req_homeparams(self.dest,self.source, self.chan_ident))
        try:
            p_homeparam=self.prnt(msg_list9)
            return p_homeparam
        except:
            print('Error reading fun: msgp_homeparam')
            return 0

    def msgs_homeparam(self, home_velocity, offset_distance,
                       home_dir=2, limit_switch=1, ):
        try:
            self.port.write(apt.mot_set_homeparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             home_dir=home_dir, 
                                             limit_switch=limit_switch, 
                                             home_velocity=home_velocity,
                                             offset_distance=offset_distance))                                         
            print('Done homeparam')
        except:
            print('Error setting fun: msgs_homeparam')
        return 0

    def msgp_limitsw(self):
        # MGMSG_MOT_SET_LIMSWITCHPARAMS
        msg_list10=['msg', 'msgid', 'dest', 'source',
               'chan_ident','cw_hardlimit', 'ccw_hardlimit', 'cw_softlimit',
               'ccw_softlimit', 'soft_limit_mode']
        
        self.port.write(apt.mot_req_limswitchparams(self.dest, self.source, self.chan_ident))
        try:
            p_limitsw=self.prnt(msg_list10)
            return p_limitsw
        except:
            print('Error reading fun: msgp_limitsw')
            return 0

    def msgs_limitsw(self, cw_softlimit, ccw_softlimit, cw_hardlimit=2,
                     ccw_hardlimit=2, soft_limit_mode=0):
        try:
            self.port.write(apt.mot_set_limswitchparams(
                                             self.dest, self.source,
                                             self.chan_ident,                                        
                                             cw_hardlimit=cw_hardlimit, 
                                             ccw_hardlimit=ccw_hardlimit, 
                                             cw_softlimit=cw_softlimit, 
                                             ccw_softlimit=ccw_softlimit, 
                                             soft_limit_mode=soft_limit_mode))                                         
            print('Done limitsw')
        except:
            print('Error setting fun: msgs_limitsw')
        return 0
        
        
class LTS300(LTS150):
    def __init__(self, portn="/dev/ttyUSB0", baud=115200, dest=0x50, source=1, chan_ident=1):
        super().__init__(portn=portn, baud=baud, dest=dest, source=source, chan_ident=chan_ident)