# ros
import rospy
import serial
from std_msgs.msg import UInt8, Bool, Int32, String

# built-in
import struct
import string

PORT = '/dev/ttyUSB0'                   # port number
BAUDRATE = 115200                       # baudrate


class ERPHandler:
    def __init__(self):
        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)
        rospy.loginfo("serial %s connected", PORT)
        

        rospy.Subscriber("/e_stop", Bool, self.e_stop_callback)
        rospy.Subscriber("/gear", UInt8, self.gear_callback)
        rospy.Subscriber("/speed", UInt8, self.speed_callback)
        rospy.Subscriber("/steer", Int32, self.steer_callback)
        rospy.Subscriber("/brake", UInt8, self.brake_callback)

        self.e_stop = 0  # 1. bool  | 0: off, 1: on
        self.gear = 0    # 2. uint8 | 0: fw, 1: natural, 2: back
        self.speed = 0   # 3. uint8 | 0 ~ 200 [kph * 10, 최대 20kph]? -> 6이면 0.6km/h
        self.steer = 0   # 4. int32 | -2000 ~ 2000 (-27.77 ~ 27.77), right: +
        self.brake = 200 # 5. uint8 | 1->0: no braking, 33->200: full braking
        self.alive = 0


    def e_stop_callback(self, data):
        self.e_stop = data.data

    def gear_callback(self, data):
        self.gear = data.data

    def speed_callback(self, data):
        self.speed = data.data

    def steer_callback(self, data):
        self.steer = data.data

    def brake_callback(self, data):
        self.brake = data.data


    def pc_to_erp(self):
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

        self.serial.write(packet)
        self.alive += 1
        if self.alive == 256:
            self.alive = 0

        print("[SEND] steer: ", self.steer, " | speed: ", self.speed, 
              " | brake: ", self.brake, " | gear: ", self.gear)



if __name__ == "__main__":
    rospy.init_node("sender")
    node = ERPHandler()
    rate = rospy.Rate(50)   # Publish for 50 Hz
    while not rospy.is_shutdown():
        node.pc_to_erp()
        rate.sleep()