import rospy
import cv2
import gi
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

def on_new_buffer(appsink):
    sample = appsink.emit('pull-sample')
    buffer = sample.get_buffer()
    caps = sample.get_caps()
    success, array = buffer.extract_dup(0, buffer.get_size())
    if success:
        array = np.frombuffer(array, dtype=np.uint8)
        image = cv2.imdecode(array, 1)
        
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        pub.publish(image_message)
    
    return Gst.FlowReturn.OK

def main():
    global pub
    rospy.init_node('gstreamer_to_ros', anonymous=True)
    pub = rospy.Publisher('/FPV_video', Image, queue_size=2)

    Gst.init(None)
    pipeline_str = 'udpsrc port=5003 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink name=appsink sync=false'
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name('appsink')
    appsink.connect('new-sample', on_new_buffer)
    
    pipeline.set_state(Gst.State.PLAYING)
    rospy.spin()
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    main()
