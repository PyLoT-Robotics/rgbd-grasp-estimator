"""Aligned depth + CameraInfo + timestamped ROI adapter. No arm commands."""
from dataclasses import fields
import json
import math
from .core import Intrinsics,GraspConfig,estimate


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from cv_bridge import CvBridge,CvBridgeError

    class GraspNode(Node):
        def __init__(self):
            super().__init__('rgbd_grasp')
            defaults=GraspConfig();values={f.name:self.declare_parameter(f.name,getattr(defaults,f.name)).value for f in fields(defaults)}
            self.config=GraspConfig(**values)
            self.age=float(self.declare_parameter('max_age_sec',.5).value)
            self.sync=float(self.declare_parameter('sync_tolerance_sec',.1).value)
            if not math.isfinite(self.age) or self.age<=0 or not math.isfinite(self.sync) or self.sync<0:raise ValueError('Invalid freshness/synchronization limits')
            self.bridge=CvBridge();self.info=None;self.roi=None
            depth_topic=self.declare_parameter('depth_topic','/camera/aligned_depth_to_color/image_raw').value
            info_topic=self.declare_parameter('camera_info_topic','/camera/color/camera_info').value
            self.create_subscription(CameraInfo,info_topic,self.camera_info,qos_profile_sensor_data)
            self.create_subscription(String,'~/roi',self.region,10)
            self.create_subscription(Image,depth_topic,self.depth,qos_profile_sensor_data)
            self.poses=self.create_publisher(PoseStamped,'~/pose',10)
            self.result=self.create_publisher(String,'~/result',10)

        def camera_info(self,msg):self.info=msg

        def region(self,msg):
            try:
                data=json.loads(msg.data)
                if not isinstance(data,dict) or not isinstance(data['frame_id'],str):raise ValueError('ROI needs frame_id')
                stamp=float(data['stamp'])
                if not math.isfinite(stamp) or stamp<=0:raise ValueError('ROI needs positive stamp')
                self.roi=data
            except (ValueError,TypeError,KeyError,CvBridgeError) as e:
                self.roi=None;self.get_logger().warning(str(e),throttle_duration_sec=5.)

        def depth(self,msg):
            try:
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                now=self.get_clock().now().nanoseconds*1e-9
                if self.info is None or self.roi is None:raise ValueError('CameraInfo and ROI are required')
                if stamp<=0 or not -.1<=now-stamp<=self.age or abs(stamp-float(self.roi['stamp']))>self.sync:raise ValueError('Depth/ROI are stale or unsynchronized')
                if msg.header.frame_id!=self.roi['frame_id'] or msg.header.frame_id!=self.info.header.frame_id:raise ValueError('Aligned depth, ROI and intrinsics must use the same optical frame')
                if (msg.width,msg.height)!=(self.info.width,self.info.height):raise ValueError('CameraInfo/image size mismatch')
                if msg.encoding not in ['16UC1','32FC1']:raise ValueError('Expected 16UC1 mm or 32FC1 metres')
                image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
                k=self.info.k;intrinsics=Intrinsics(k[0],k[4],k[2],k[5])
                candidate=estimate(image,intrinsics,self.roi['bbox'],self.config,'mm' if msg.encoding=='16UC1' else 'm')
                if candidate is None:
                    self.result.publish(String(data=json.dumps({'valid':False,'reason':'no_supported_cluster','stamp':stamp})));return
                candidate.update(valid=True,stamp=stamp,frame_id=msg.header.frame_id)
                self.result.publish(String(data=json.dumps(candidate,allow_nan=False)))
                pose=PoseStamped();pose.header=msg.header
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=candidate['position_m']
                q=pose.pose.orientation;q.x,q.y,q.z,q.w=candidate['orientation_xyzw'];self.poses.publish(pose)
            except (ValueError,TypeError,KeyError,CvBridgeError) as e:
                self.result.publish(String(data=json.dumps({'valid':False,'reason':str(e)})))
                self.get_logger().warning(str(e),throttle_duration_sec=5.)

    rclpy.init(args=args);node=None
    try:node=GraspNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
