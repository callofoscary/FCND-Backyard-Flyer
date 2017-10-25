
from pymavlink import mavutil
import os
import threading
import time
from . import connection
from . import message_types as mt


# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

# a constant that isn't defined in Mavlink but is useful for PX4
PX4_MODE_OFFBOARD = 6

# useful masks for sending commands
MASK_IGNORE_POSITION = 0x007
MASK_IGNORE_VELOCITY = 0x038
MASK_IGNORE_ACCELERATION = 0x1C0
MASK_IGNORE_YAW = 0x400
MASK_IGNORE_YAW_RATE = 0x800

MASK_IS_FORCE = (1 << 9)
MASK_IS_TAKEOFF = 0x1000
MASK_IS_LAND = 0x2000
MASK_IS_LOITER = 0x3000

class MavlinkConnection(connection.Connection):

    def __init__(self, device, threaded=False):
        # IMPORTANT NOTE: if threaded is true, the thread will die immediately once all other processes stop
        # which means that if f threaded is true, drone needs to maintain a while loop or something on the main thread or else tis thread will die immediately
        # 
        super().__init__()
        if device is not "":
            self._master = mavutil.mavlink_connection(device)
        self._threaded = threaded
        if threaded:
            self._read_handle = threading.Thread(target=self.read_loop)
            self._read_handle.daemon = True
        else:
            self._read_handle = None
        self._running = False

        self._target_system = 1
        self._target_component = 1

    def testcallback(self):
        print("testing callback")
        self.notify_message_listeners('test', [])


    def read_loop(self):
        # this is the main loop that wait for a mavlink messages, parses it accordingly, and triggers the corresponding callback(s)
        # NOTE: since a single mavlink message can correspond to multiple custom messages, this loop needs to be handled by this class
        last_msg_time = time.time()
        timeout = 5  # the number of seconds to wait of no messages before calling a connection terminated
        while (self._running):
            current_time = time.time()
            msg = self.wait_for_message()

            # if we haven't heard a message in a given amount of time, send a signal to kill everything
            if current_time - last_msg_time > timeout:
                print("timeout too long!")
                self.notify_message_listeners(mt.MSG_CONNECTION_CLOSED, 0)
                # TODO: maybe want to set running to false here...?

            # if no message or a bad message was received, just move along
            if msg is None:
                continue

            # update the time of the last message
            last_msg_time = current_time

            # this does indeed get timestamp, should double check format
            # TODO: deice on timestamp format for messages
            timestamp = msg._timestamp

            # parse out the message based on the type and call
            # the appropriate callbacks
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # parse out the gps position and trigger that callback
                gps = mt.GlobalFrameMessage(timestamp, float(msg.lat)/1e7, float(msg.lon)/1e7, float(msg.alt)/1000)
                self.notify_message_listeners(mt.MSG_GLOBAL_POSITION, gps)

                # parse out the velocity and trigger that callback
                vel = mt.LocalFrameMessage(timestamp, float(msg.vx)/100, float(msg.vy)/100, float(msg.vx)/100)
                self.notify_message_listeners(mt.MSG_VELOCITY, vel)

            elif msg.get_type() == 'HEARTBEAT':
                motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                # TODO: determine if want to broadcast all current mode types, not just boolean on manual
                guided_mode = False

                # extract whether or not we are in offboard mode for PX4 (the main mode)
                main_mode = (msg.custom_mode & 0x000F0000) >> 16
                if main_mode == PX4_MODE_OFFBOARD:
                    guided_mode = True

                state = mt.StateMessage(timestamp, motors_armed, guided_mode)
                self.notify_message_listeners(mt.MSG_STATE, state)

            elif msg.get_type() == 'LOCAL_POSITION_NED':
                # parse out the local positin and trigger that callback
                pos = mt.LocalFrameMessage(timestamp, msg.x, msg.y, msg.z)
                self.notify_message_listeners(mt.MSG_LOCAL_POSITION, pos)

                # parse out the velocity and trigger that callback
                vel = mt.LocalFrameMessage(timestamp, msg.vx, msg.vy, msg.vz)
                self.notify_message_listeners(mt.MSG_VELOCITY, vel)

            elif msg.get_type() == 'HOME_POSITION':
                # TODO: decode position information
                home = mt.GlobalFrameMessage(timestamp, float(msg.latitude)/1e7, float(msg.longitude)/1e7, float(msg.altitude)/1000)
                self.notify_message_listeners(mt.MSG_GLOBAL_HOME, home)

            elif msg.get_type() == 'POSITION_TARGET_LOCAL_NED':
                print(msg)

            #elif msg.get_type() == 'ATTITUDE':



            # TODO: parse out additional message types



    def wait_for_message(self):
        # NOTE: this returns a mavlink message
        # this function should not be called outside of this class!
        msg = self._master.recv_match(blocking=True,timeout=10)
        if msg is None:
            # no message received
            return None
        else:
            if(msg.get_type() == 'BAD_DATA'):
                # no message that is useful
                return None
                
            # send a heartbeat message back, since this needs to be constantly sent so the autopilot knows this exists
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                           0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

            # pass the message along to be handled by this class
            return msg


    def start(self):
        # start the main thread
        self._running = True
        if self._threaded:
            self._read_handle.start()
        else:
            # NOTE: this is a full blocking function here!!
            # TODO: find a correct way to terminate the read loop
            self.read_loop()


    def stop(self):
        self._running = False
        if self._threaded:
            self._read_handle.join()
        else:
            self._master.close()

    def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        # send a command long helper function
        confirmation = 0  # may want this as an input....
        self._master.mav.command_long_send(
            self._target_system, self._target_component,
            command_type, confirmation,
            param1, param2, param3, param4, param4, param6, param7)

    def arm(self):
        # send an arm command through mavlink
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def disarm(self):
        # send a disarm command
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def take_control(self):
        mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = PX4_MODE_OFFBOARD
        custom_sub_mode = 0  # not used for manual/offboard
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode)

    def release_control(self):
        mode = mavutil.mavlink.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  # tells system to use PX4 custom commands
        custom_mode = 1  # 1 == manual control
        custom_sub_mode = 0  # not used for manual/offboard
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, custom_mode, custom_sub_mode)

    def cmd_attitude(self, yaw, pitch, roll, collective):
        time_boot_ms = 0  # this does not need to be set to a specific time
        # TODO: convert the attitude to a quaternion
        q = [0, 0, 0, 0]
        mask = 0b00000111
        self._master.mav.set_attitude_target_send(
            time_boot_ms, self._target_system, self._target_component, mask,
            q, 0, 0, 0, collective)

    def cmd_attitude_rate(self, yaw_rate, pitch_rate, roll_rate, collective):
        time_boot_ms = 0  # this does not need to be set to a specific time
        q = [0, 0, 0, 0]
        mask = 0b10000000
        self._master.mav.set_attitude_target_send(
            time_boot_ms, target_system, target_component, mask,
            q, roll_rate, pitch_rate, yaw_rate, collective)        

    def cmd_velocity(self, vn, ve, vd, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = (MASK_IGNORE_YAW_RATE | MASK_IGNORE_ACCELERATION | MASK_IGNORE_POSITION)
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            heading, 0)

    def cmd_motors(self, motor1, motor2, motor3, motor4):
        pass

    def cmd_position(self, n, e, d, heading):
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = (MASK_IGNORE_YAW_RATE | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY)
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            heading, 0)

    def takeoff(self, n, e, d):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = MASK_IS_TAKEOFF
        mask |= (MASK_IGNORE_YAW_RATE | MASK_IGNORE_YAW | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY)
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            0, 0)

    def land(self, n, e):
        # for mavlink to PX4 need to specify the NED location for landing
        # since connection doesn't keep track of this info, have drone send it
        # abstract away that part in the drone class
        d = 0  # going to land, so just set d to 0
        time_boot_ms = 0  # this does not need to be set to a specific time
        mask = MASK_IS_LAND
        mask |= (MASK_IGNORE_YAW_RATE | MASK_IGNORE_YAW | MASK_IGNORE_ACCELERATION | MASK_IGNORE_VELOCITY)
        self._master.mav.set_position_target_local_ned_send(
            time_boot_ms, self._target_system, self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            0, 0)

    def set_home_position(self, lat, lon, alt):
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, lat, lon, alt)
