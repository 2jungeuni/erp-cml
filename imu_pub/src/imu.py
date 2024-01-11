#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import Imu


PORT = '/dev/ttyUSB0'
BAUDRATE = 115200


class ImuReader:
    def __init__(self):
        self.serial_port = PORT
        self.baud_rate = BAUDRATE
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        rospy.loginfo("Start publishing for '/imu/data' topic!")


    def publish_imu_data(self):
        data = self.ser.readline().decode('ascii').rstrip()
        parts = self.checksum_filter(data)

        if parts is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            
            imu_msg.orientation.x = float(parts[1])
            imu_msg.orientation.y = float(parts[2])
            imu_msg.orientation.z = float(parts[3])
            imu_msg.orientation.w = float(parts[4])

            imu_msg.linear_acceleration.x = float(parts[5])
            imu_msg.linear_acceleration.y = float(parts[6])
            imu_msg.linear_acceleration.z = float(parts[7])

            imu_msg.angular_velocity.x = float(parts[8])
            imu_msg.angular_velocity.y = float(parts[9])
            imu_msg.angular_velocity.z = float(parts[10])

            imu_msg.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.imu_pub.publish(imu_msg)


    def checksum_filter(self, data):
        if '*' in data:
            nmea_data, checksum_str = data.split('*')
            
            # 체크섬계산: $와 * 사이의 모든 문자에 대한 XOR 연산 ($, *는 계산에서 제외)
            real_checksum = 0
            for char in nmea_data[1:]:
                real_checksum ^= ord(char)

            try:
                # 체크섬 비교
                input_checksum = int(checksum_str, 16)
                if real_checksum != input_checksum:
                    rospy.logwarn("Checksum does not match: calculated {}, input {}"
                                  .format(real_checksum, input_checksum))
                    return None
            except ValueError:
                rospy.logwarn("Invalid checksum: {}".format(checksum_str))
                return None
        else:
            rospy.logwarn("No checksum found in data: {}".format(data))
            return None
        
        parts = nmea_data.split(',')

        if len(parts) != 11:
            rospy.logwarn("Unexpected number of data parts: %d" % len(parts))
            return None
        
        return parts



if __name__ == '__main__':
    rospy.init_node('imu_reader_node', anonymous=False)
    node = ImuReader()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        node.publish_imu_data()
        rate.sleep()
