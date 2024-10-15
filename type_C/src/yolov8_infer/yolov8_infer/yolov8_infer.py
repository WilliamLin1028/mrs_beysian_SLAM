import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator


class yolov8_infer_node(Node):

    def __init__(self):
        super().__init__('yolov8_infer')

        self.model = YOLO("yolov8n.pt")

        self.yolo_img_pub = self.create_publisher(
            CompressedImage,
            '/yolov8_infer/compressed',
            10
        )

        self.yolo_result_pub = self.create_publisher(
            Detection2DArray,
            "/yolov8_infer/result",
            10
        )

        self.img_compressed_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.result_msg = Detection2DArray()

    def det_result_add(self, bbox, classes, score):
        detection2d = Detection2D()
        detection2d.id = self.model.names[int(classes)]
        x1, y1, x2, y2 = bbox
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        center_x = (x1+x2)/2.0
        center_y = (y1+y2)/2.0

        detection2d.bbox.center.position.x = center_x
        detection2d.bbox.center.position.y = center_y

        detection2d.bbox.size_x = float(x2-x1)
        detection2d.bbox.size_y = float(y2-y1)

        obj_pose = ObjectHypothesisWithPose()
        obj_pose.hypothesis.class_id = self.model.names[int(classes)]
        obj_pose.hypothesis.score = float(score)

        # not yet implemented
        world_x, world_y = -1.0, -1.0
        obj_pose.pose.pose.position.x = world_x
        obj_pose.pose.pose.position.y = world_y

        detection2d.results.append(obj_pose)
        self.result_msg.detections.append(detection2d)



    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            stream_img = bridge.compressed_imgmsg_to_cv2(msg)
            results = self.model(stream_img)

            self.result_msg.detections.clear()
            self.result_msg.header.frame_id = "camera"
            self.result_msg.header.stamp = self.get_clock().now().to_msg()

            for r in results:
                annotator = Annotator(stream_img)
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    s = box.conf
                    annotator.box_label(b, self.model.names[int(c)])

                    self.det_result_add(b, c, s)
            
            img = annotator.result()
            
            infer_img = bridge.cv2_to_compressed_imgmsg(img)
            self.yolo_img_pub.publish(infer_img)
            self.yolo_result_pub.publish(self.result_msg)
            
            

        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    node = yolov8_infer_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
