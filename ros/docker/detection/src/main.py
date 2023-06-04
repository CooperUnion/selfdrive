import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from lane_detection import Lanes

# DEFINES
# Subscribed Topics
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  

#Published Topics
LANE_TOPIC = '/lane_detection/rgb/image_rect_color'
LANE_OFFSET = '/lane_offset'

bridge = CvBridge()

class Subscriber():
    def __init__(self):
        self.cv_img = None
        rospy.Subscriber(IMAGE_TOPIC, Image, self.img2cv)

    def img2cv(self,data):
        # rospy.loginfo('Video Frame Received')
        # Converts image topic stream to cv image
        # Possible encoding schemes: 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]     
        # Can optionally do color or pixel depth conversion
        self.cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

class Publisher(self):
    def __init__(self):
        self.lane_pub =  rospy.Publisher(LANE_TOPIC, Image, queue_size=2)
        self.offset_pub = rospy.Publisher(LANE_OFFSET, Int16, queue_size=2)
        self.rate = rospy.Rate(100)
    def pub_lane(self,cv_image):
        img_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.lane_pub.publish(img_msg)
        self.rate.sleep()
    def pub_offset(self,offset):
        msg = Int16()
        msg.data = offset
        self.offset_pub.publish(msg.data)
        self.rate.sleep()

def main():

    rospy.init_node('lane_detection', anonymous=True)
    sub = Subscriber()
    pub = Publisher()
    lanes = Lanes()
    
    while not rospy.is_shutdown():
        offset = lanes.detection(sub.cv_img)
        print(offset)
        pub.pub_lane(lanes.source_img)
        pub.pub_offset(offset)

if __name__ ==  'main':
    main()