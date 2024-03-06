#!/usr/bin/env python
# import rospy
# from nmea_msgs.msg import Sentence
# import pynmea2

# def parse_nmea_sentence(msg):
#     if msg.sentence.startswith('$GNGGA') or msg.sentence.startswith('$GPGGA'):
#         try:
#             gga = pynmea2.parse(msg.sentence)
#             latitude = gga.latitude
#             longitude = gga.longitude
#             altitude = gga.altitude
#             print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} M")
#         except pynmea2.ParseError as e:
#             rospy.logwarn(f"Failed to parse GGA sentence: {e}")
#     elif msg.sentence.startswith('$GNVTG') or msg.sentence.startswith('$GPVTG'):
#         try:
#             vtg = pynmea2.parse(msg.sentence)
#             true_track = vtg.true_track
#             speed_kmph = vtg.spd_over_grnd_kmph
#             print(f"True Track: {true_track} deg, {speed_kmph} km/h")
#         except pynmea2.ParseError as e:
#             rospy.logwarn(f"Failed to parse VTG sentence: {e}")

# def listener():
#     rospy.init_node('gps_data_extractor', anonymous=True)
#     rospy.Subscriber('/nmea_sentence', Sentence, parse_nmea_sentence)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()


# mattplotlib를 이용한 실시간 그래프 그리기 및 전체 경로 folium 시각화
import rospy
from nmea_msgs.msg import Sentence
import pynmea2
import matplotlib.pyplot as plt
import folium

# 위도와 경도를 저장할 리스트 초기화
latitudes = []
longitudes = []

def plot_gps_coordinates(lat, lon):
    latitudes.append(lat)
    longitudes.append(lon)

    plt.clf()  # 이전에 그려진 그래프를 지웁니다.
    plt.scatter(longitudes, latitudes, c='blue', marker='.')  # 현재까지의 위도와 경도를 그래프에 표시합니다.
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GPS Coordinates')
    plt.pause(0.05)  # 그래프를 업데이트하고 잠시 기다립니다.

def parse_nmea_sentence(msg):
    if msg.sentence.startswith('$GNGGA') or msg.sentence.startswith('$GPGGA'):
        try:
            gga = pynmea2.parse(msg.sentence)
            plot_gps_coordinates(gga.latitude, gga.longitude)
        except pynmea2.ParseError as e:
            rospy.logwarn(f"Failed to parse GGA sentence: {e}")

def listener():
    rospy.init_node('gps_data_visualization', anonymous=True)
    rospy.Subscriber('/nmea_sentence', Sentence, parse_nmea_sentence)
    
    plt.ion()  # 대화형 모드 활성화
    plt.show()  # 초기 그래프 윈도우를 보여줍니다.

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    
    # folium 패키지를 이용해서 실제 지도에 위도, 경도 표시하기
    middle_lat = min(latitudes) + (max(latitudes) - min(latitudes)) / 2
    middle_lng = min(longitudes) + (max(longitudes) - min(longitudes)) / 2

    center = [middle_lat, middle_lng]

    m = folium.Map(location=center, zoom_start=12)

    for idx in range(len(longitudes)):
        folium.CircleMarker(location=[latitudes[idx], longitudes[idx]], 
                            color='blue', 
                            radius=1, 
                            fill=True).add_to(m)

    m.save("result.html")   



# import rospy
# from nmea_msgs.msg import Sentence
# import pynmea2
# from geometry_msgs.msg import Pose2D
# import utm

# def parse_nmea_sentence(msg):
#     pub_pose = rospy.Publisher('/gps/pose2d', Pose2D, queue_size=10)
#     pose_msg = Pose2D()

    
        
#     if msg.sentence.startswith('$GNVTG') or msg.sentence.startswith('$GPVTG'):
#         vtg = pynmea2.parse(msg.sentence)
#         if vtg.true_track is not None: 
#             pose_msg.theta = float(vtg.true_track)
#             if msg.sentence.startswith('$GNGGA') or msg.sentence.startswith('$GPGGA'):
#                 gga = pynmea2.parse(msg.sentence)
#                 utm_coord = utm.from_latlon(gga.latitude, gga.longitude)
#                 pose_msg.x = utm_coord[0]  # UTM easting
#                 pose_msg.y = utm_coord[1]  # UTM northing
#                 pub_pose.publish(pose_msg)
    
    

# if __name__ == '__main__':
#     rospy.init_node('gps_data_to_pose2d')
#     rospy.Subscriber('/nmea_sentence', Sentence, parse_nmea_sentence)
#     rospy.spin()
    


# import rospy
# from nmea_msgs.msg import Sentence
# import pynmea2
# import matplotlib.pyplot as plt
# import utm

# # 위도와 경도를 저장할 리스트
# latitudes = []
# longitudes = []

# def plot_and_save_gps_coordinates(lat, lon):
#     latitudes.append(lat)
#     longitudes.append(lon)

#     # Matplotlib를 이용한 시각화
#     plt.clf()
#     plt.scatter(longitudes, latitudes, c='red', marker='o')
#     plt.xlabel('Longitude')
#     plt.ylabel('Latitude')
#     plt.title('GPS Coordinates Visualization')
#     plt.pause(0.05)

# def save_to_file():
#     with open('gps_coordinates.txt', 'w') as file:
#         for lat, lon in zip(latitudes, longitudes):
#             file.write(f"{lat}, {lon}\n")
#     print("GPS coordinates have been saved to gps_coordinates.txt")

# def parse_nmea_sentence(msg):
#     if msg.sentence.startswith('$GNGGA') or msg.sentence.startswith('$GPGGA'):
#         try:
#             gga = pynmea2.parse(msg.sentence)
#             # UTM 변환 없이 원시 위도 및 경도 사용
#             plot_and_save_gps_coordinates(gga.latitude, gga.longitude)
#         except pynmea2.ParseError as e:
#             rospy.logwarn(f"Failed to parse GGA sentence: {e}")

# if __name__ == '__main__':
#     rospy.init_node('gps_data_visualization_and_saving')
#     rospy.Subscriber('/nmea_sentence', Sentence, parse_nmea_sentence)
    
#     plt.ion()
#     plt.show()

#     rospy.on_shutdown(save_to_file)  # 노드 종료 시 데이터 저장
#     rospy.spin()