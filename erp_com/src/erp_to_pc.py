# ros
import rospy
import serial
# from erp_com.msg import Status, Cmd
from vehicle_states_msgs import ControlModeState, EStopState, GearState, SpeedState, SteeringState, BrakeState, EncoderState, AliveState

# built-in
import struct
import numpy as np

PORT = '/dev/ttyUSB0'                   # port number
BAUDRATE = 115200                       # baudrate
ERPS = serial.Serial(PORT, BAUDRATE)
STAR_BITS = "535458"

class ERPHandler:
    def __init__(self):
        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)
        rospy.loginfo("serial %s connected", PORT)

        self.control_mode_state_pub = rospy.Publisher("/erp42_control_mode_state", ControlModeState, queue_size=3)
        self.e_stop_state_pub = rospy.Publisher("/erp42_e_stop_state", EStopState, queue_size=3)
        self.gear_state_pub = rospy.Publisher("/erp42_gear_state", GearState, queue_size=3)
        self.speed_state_pub = rospy.Publisher("/erp42_speed_state", SpeedState, queue_size=3)
        self.steering_state_pub = rospy.Publisher("/erp42_steering_state", SteeringState, queue_size=3)
        self.brake_state_pub = rospy.Publisher("/erp42_brake_state", BrakeState, queue_size=3)
        self.encoder_state_pub = rospy.Publisher("/erp42_encoder_state", EncoderState, queue_size=3)
        self.alive_state_pub = rospy.Publisher("/erp42_alive_state", AliveState, queue_size=3)
        self.flag = True

        
    def erp_to_pc(self):
        packet = self.serial.read(18)
        
        if self.flag:
            print("[RECEIVE] first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(STAR_BITS) != 0:
            end, data = packet.hex().split(STAR_BITS)
            packet = bytes.fromhex(STAR_BITS + data + end)

        control_mode_msg, e_stop_msg, gear_msg, speed_msg, steering_msg, brake_msg, encoder_msg, alive_msg = self.Packet2ErpMsgs(packet)

        print("[RECEIVE] steer: ", str(round(steering_msg.data / 71, 2)).ljust(5),
              "degree | speed: ", str(speed_msg.data).ljust(3),
              "kph | brake: ", str(brake_msg.data).ljust(3),
              " | gear: ", str(gear_msg.data).ljust(1))
        
        self.control_mode_state_pub.publish(control_mode_msg)
        self.e_stop_state_pub.publish(e_stop_msg)
        self.gear_state_pub.publish(gear_msg)
        self.speed_state_pub.publish(speed_msg)
        self.steering_state_pub.publish(steering_msg)
        self.brake_state_pub.publish(brake_msg)
        self.encoder_state_pub.publish(encoder_msg)
        self.alive_state_pub.publish(alive_msg)
    

    def Packet2ErpMsgs(self, byte):
        packet = struct.unpack('<BBBBBBhhBiBBB', byte)

        control_mode_msg = ControlModeState()
        e_stop_msg = EStopState()
        gear_msg = GearState()
        speed_msg = SpeedState()
        steering_msg = SteeringState()
        brake_msg = BrakeState()
        encoder_msg = EncoderState()
        alive_msg = AliveState()

        control_mode_msg.data = packet[3]
        e_stop_msg.data = bool(packet[4])
        gear_msg.data = packet[5]
        speed_msg.data = packet[6]
        steering_msg.data = -packet[7]
        brake_msg.data = packet[8]
        encoder_msg.data = -np.int32(packet[9])
        alive_msg.data = packet[10]

        return control_mode_msg, e_stop_msg, gear_msg, speed_msg, steering_msg, brake_msg, encoder_msg, alive_msg


if __name__ == "__main__":
    rospy.init_node("erp_to_pc", anonymous=False)
    node = ERPHandler()
    rate = rospy.Rate(50)   # 50 Hz
    while not rospy.is_shutdown():
        node.erp_to_pc()
        rate.sleep()