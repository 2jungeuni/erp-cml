# ros
import rospy
import serial
from erp_com.msg import Status, Cmd

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


        self.msg_pub = rospy.Publisher("/erp42_status", Status, queue_size=3)
        self.packet.e_stop = False
        self.flag = True

        
    def receive_packet(self):
        self.packet = Cmd()
        packet = self.serial.read(18)
        if self.flag:
            print("[RECEIVE] first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(STAR_BITS) != 0:
            end, data = packet.hex().split(STAR_BITS)
            packet = bytes.fromhex(STAR_BITS + data + end)
        msg = self.Packet2ErpMsg(packet)

        print("[RECEIVE] steer: ", msg.steer, " | speed: ", msg.speed, 
              " | brake: ", msg.brake, " | gear: ", msg.gear)
        self.msg_pub.publish(msg)

    
    def Packet2ErpMsg(self, byte):
        packet = struct.unpack('<BBBBBBhhBiBBB', byte)
        msg = Status()
        msg.control_mode = packet[3]
        msg.e_stop = bool(packet[4])
        msg.gear = packet[5]
        msg.speed = packet[6]
        msg.steer = -packet[7]
        msg.brake = packet[8]
        msg.encoder = -np.int32(packet[9])
        msg.alive = packet[10]
        return msg


if __name__ == "__main__":
    rospy.init_node("erp_to_pc", anonymous=False)
    node = ERPHandler()
    rate = rospy.Rate(50)   # 50 Hz
    while not rospy.is_shutdown():
        node.receive_packet()
        rate.sleep()