#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import Imu

class ImuReader:
    def __init__(self, serial_port, baud_rate):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)

    def parse_imu_data(self, data):
        if '*' in data:
            # 체크섬 분리
            nmea_data, checksum_str = data.split('*')
            calculated_checksum = self.calculate_checksum(nmea_data[1:])  # $ 문자는 제외하고 계산
            try:
                # 체크섬 비교
                input_checksum = int(checksum_str, 16)
                if calculated_checksum != input_checksum:
                    rospy.logwarn("Checksum does not match: calculated {}, input {}".format(calculated_checksum, input_checksum))
                    return None  # 체크섬이 일치하지 않으면 None을 반환
            except ValueError:
                rospy.logwarn("Invalid checksum: {}".format(checksum_str))
                return None  # 체크섬 형식이 잘못된 경우 None을 반환
        else:
            rospy.logwarn("No checksum found in data: {}".format(data))
            return None  # 체크섬이 없는 경우 None을 반환
        
        parts = nmea_data.split(',')


        if len(parts) != 11:
            rospy.logerr("Unexpected number of data parts: %d" % len(parts))
            return None

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        # 쿼터니언 데이터 (Orientation)
        imu_msg.orientation.x = float(parts[1])
        imu_msg.orientation.y = float(parts[2])
        imu_msg.orientation.z = float(parts[3])
        imu_msg.orientation.w = float(parts[4])

        # 선형 가속도 데이터 (Linear Acceleration)
        imu_msg.linear_acceleration.x = float(parts[5])
        imu_msg.linear_acceleration.y = float(parts[6])
        imu_msg.linear_acceleration.z = float(parts[7])

        # 각속도 데이터 (Angular Velocity)
        imu_msg.angular_velocity.x = float(parts[8])
        imu_msg.angular_velocity.y = float(parts[9])
        imu_msg.angular_velocity.z = float(parts[10])

        # 가속도와 각속도의 공분산은 알려지지 않았으므로 기본값을 사용합니다.
        imu_msg.linear_acceleration_covariance = [-1 if i == 0 else 0 for i in range(9)]
        imu_msg.angular_velocity_covariance = [-1 if i == 0 else 0 for i in range(9)]
        imu_msg.orientation_covariance = [-1 if i == 0 else 0 for i in range(9)]

        return imu_msg
    
    def calculate_checksum(self, nmea_str):
        # 체크섬은 $와 * 사이의 모든 문자에 대한 XOR 연산의 결과입니다.
        # $와 *는 체크섬 계산에서 제외됩니다.
        checksum = 0
        for char in nmea_str:
            checksum ^= ord(char)
        return checksum

    def read_and_publish(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').rstrip()
                imu_msg = self.parse_imu_data(line)
                if imu_msg is not None:
                    self.imu_pub.publish(imu_msg)
                    rospy.loginfo(imu_msg)
            rospy.sleep(0.02)  # 10Hz


if __name__ == '__main__':
    rospy.init_node('imu_reader_node', anonymous=False)
    imu_reader = ImuReader('/dev/ttyUSB0', 115200)
    
    try:
        imu_reader.read_and_publish()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS Interrupt Exception: %s" % str(e))
        pass
    except serial.serialutil.SerialException as e:
        rospy.logerr("Serial Exception: %s" % str(e))
        pass
