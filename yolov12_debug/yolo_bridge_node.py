import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import json
import zmq
import numpy as np
import time

class YoloBridgeNode:
    def __init__(self):
        rospy.init_node('yolo_bridge_node', anonymous=True)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/iris_0/camera/image_raw')
        self.detections_topic = rospy.get_param('~detections_topic', '/iris_0/yolo/detections')
        self.annotated_image_topic = rospy.get_param('~annotated_image_topic', '/iris_0/yolo/annotated_image')
        
        # Publishers
        self.detections_pub = rospy.Publisher(self.detections_topic, String, queue_size=10)
        self.annotated_image_pub = rospy.Publisher(self.annotated_image_topic, Image, queue_size=10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ZeroMQ context and sockets
        self.context = zmq.Context()
        self.sender = self.context.socket(zmq.PUSH)
        self.sender.bind("tcp://127.0.0.1:5555")
        
        self.receiver = self.context.socket(zmq.PULL)
        self.receiver.bind("tcp://127.0.0.1:5556")
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        # Start result listener thread
        import threading
        self.receive_thread = threading.Thread(target=self.receive_results)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        rospy.loginfo(f"YoloBridgeNode initialized. Listening on {self.image_topic}")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Encode image as JPEG and send to external script
            _, jpeg_img = cv2.imencode('.jpg', cv_image)
            img_data = jpeg_img.tobytes()
            
            # Send image data along with header info
            self.sender.send_multipart([b"image", img_data, bytes(msg.header.frame_id, 'utf-8')])
            
        except CvBridgeError as e:
            rospy.logerr(e)
            
    def receive_results(self):
        """Thread that receives results from the YOLO processing script"""
        while not rospy.is_shutdown():
            try:
                # Receive detection results and annotated image
                frames = self.receiver.recv_multipart()
                
                if len(frames) == 3 and frames[0] == b"result":
                    # Parse detection results
                    detections_json = frames[1].decode('utf-8')
                    self.detections_pub.publish(detections_json)
                    
                    # Decode and publish annotated image
                    frame_id = frames[2].decode('utf-8')
                    img_data = np.frombuffer(self.receiver.recv(), dtype=np.uint8)
                    annotated_img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                    
                    # Convert back to ROS Image
                    img_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
                    img_msg.header.stamp = rospy.Time.now()
                    img_msg.header.frame_id = frame_id
                    self.annotated_image_pub.publish(img_msg)
                    
            except Exception as e:
                rospy.logerr(f"Error receiving results: {e}")
                time.sleep(0.1)
                
    def shutdown(self):
        """Clean shutdown of ZMQ connections"""
        self.sender.close()
        self.receiver.close()
        self.context.term()

if __name__ == '__main__':
    try:
        node = YoloBridgeNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass