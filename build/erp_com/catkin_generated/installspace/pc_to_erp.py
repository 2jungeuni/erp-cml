# ros
import rospy
import serial
from std_msgs.msg import Int32
from erp_com.msg import Status, Cmd

# built-in
import time
import struct
import numpy as np

PORT = '/dev/ttyUSB0'  # port number
BAUDRATE = 115200   # baudrate
ERPS = serial.Serial(PORT, BAUDRATE)

STAR_BITS = "535458"


def ErpMsg2Packet(msg, alive):
    header = "STX".encode()
    tail = "\r\n".encode()

    data = struct.pack(
        ">BBBHhBB",
        1,
        msg.e_stop,
        msg.gear,
        msg.speed,
        msg.steer,
        msg.brake,
        alive
    )

    packet = header + data + tail
    return packet


class ERPHandler:
    def __init__(self):
        rospy.init_node("pc_to_erp", anonymous=True)

        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)

        rospy.loginfo("serial %s connected", PORT)
        self.alive = 0
        self.packet = Cmd()
        self.packet.e_stop = False
        self.flag = True

        self.msg_pub = rospy.Publisher("/erp42_status",
                                       Status,
                                       queue_size=3)

    def send_packet(self, steer, speed, brake, gear):
        self.packet.steer = int(steer)
        self.packet.speed = int(speed)
        self.packet.brake = int(brake)
        self.packet.gear = int(gear)
        packet = ErpMsg2Packet(self.packet, self.alive)
        self.serial.write(packet)

        self.alive += 1
        if self.alive == 256:
            self.alive = 0

        print("[SEND] steer: ", self.packet.steer, " | speed: ", self.packet.speed, " | brake: ", self.packet.brake,
              " | gear: ", self.packet.gear)


if __name__ == "__main__":
    handler = ERPHandler()

    steer = 71 * 10
    speed = 10 * 1  #10 * 10
    brake = 20
    gear = 0

    while not rospy.is_shutdown():
        handler.send_packet(steer, speed, brake, gear)