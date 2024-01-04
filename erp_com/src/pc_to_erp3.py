# ros
import rospy
import serial
from erp_com.msg import Cmd

# built-in
import struct

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

        rospy.Subscriber("/erp42_command", Cmd, self.packet_send_callback)
        self.steer = 0
        self.speed = 0
        self.brake = 0
        self.gear = 0
        self.e_stop = 0
        self.alive = 0

        
    def packet_send_callback(self, data):
        self.steer = int(data.steer)
        self.speed = int(data.speed)
        self.brake = int(data.brake)
        self.gear = int(data.gear)
        self.e_stop = data.e_stop

        packet = self.ErpMsg2Packet()
        self.serial.write(packet)

        self.alive += 1
        if self.alive == 256:
            self.alive = 0

        print("[SEND] steer: ", self.packet.steer, " | speed: ", self.packet.speed, 
              " | brake: ", self.packet.brake, " | gear: ", self.packet.gear)


    def ErpMsg2Packet(self):
        header = "STX".encode()
        tail = "\r\n".encode()

        data = struct.pack(
            ">BBBHhBB",
            1,
            self.e_stop,
            self.gear,
            self.speed,
            self.steer,
            self.brake,
            self.alive
        )

        packet = header + data + tail
        return packet


if __name__ == "__main__":
    rospy.init_node("pc_to_erp", anonymous=False)
    rate = rospy.Rate(1)
    handler = ERPHandler()
    while not rospy.is_shutdown():
        rate.sleep()