"""Optional isolated DDS tests with synthetic publishers; no hardware access."""
import os
import signal
import subprocess
import sys
import time
import json
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class Harness:
    def __init__(self,package,domain,parameters):
        self.context=Context();rclpy.init(context=self.context,domain_id=domain)
        self.node=Node(package+'_test',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
        args=[sys.executable,'-c',f'from {package}.node import main; main()','--ros-args']
        for key,value in parameters.items():args+=['-p',f'{key}:={value}']
        self.process=subprocess.Popen(args,env=self.env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)

    def wait(self,predicate,publish=None,timeout=8.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            assert self.process.poll() is None,self.process.stdout.read().decode()
            if publish:publish()
            self.executor.spin_once(timeout_sec=.025)
        assert predicate()

    def call(self,name):
        from std_srvs.srv import Trigger
        client=self.node.create_client(Trigger,name);self.wait(client.service_is_ready)
        future=client.call_async(Trigger.Request());self.wait(future.done)
        return future.result()

    def close(self):
        try:os.killpg(self.process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=2)
        output=self.process.stdout.read().decode();self.process.stdout.close()
        self.executor.shutdown();self.node.destroy_node();rclpy.shutdown(context=self.context)
        assert self.process.returncode==0,output

def test_real_depth_roi_pipeline_and_unsynchronized_input_rejection():
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    import numpy as np
    h=Harness('rgbd_grasp',85,{'depth_topic':'/grasp_test/depth','camera_info_topic':'/grasp_test/info'})
    results=[];poses=[]
    try:
        info_pub=h.node.create_publisher(CameraInfo,'/grasp_test/info',qos_profile_sensor_data)
        depth_pub=h.node.create_publisher(Image,'/grasp_test/depth',qos_profile_sensor_data)
        roi_pub=h.node.create_publisher(String,'/rgbd_grasp/roi',10)
        h.node.create_subscription(String,'/rgbd_grasp/result',lambda m:results.append(json.loads(m.data)),10)
        h.node.create_subscription(PoseStamped,'/rgbd_grasp/pose',lambda m:poses.append(m),10)
        def send(stale=False):
            stamp=h.node.get_clock().now().to_msg();seconds=stamp.sec+stamp.nanosec*1e-9
            info=CameraInfo();info.header.frame_id='optical';info.header.stamp=stamp;info.width=info.height=40
            info.k=[200.,0.,19.5,0.,200.,19.5,0.,0.,1.];info_pub.publish(info)
            roi_pub.publish(String(data=json.dumps({'bbox':[0,0,40,40],'stamp':seconds-1 if stale else seconds,'frame_id':'optical'})))
            # Deliver calibration and ROI before the depth callback.
            h.executor.spin_once(timeout_sec=.02)
            msg=Image();msg.header=info.header;msg.height=msg.width=40;msg.encoding='16UC1';msg.step=80
            msg.data=np.full((40,40),1000,dtype=np.uint16).tobytes();depth_pub.publish(msg)
        h.wait(lambda:bool(poses),send)
        assert poses[-1].pose.position.z==pytest.approx(.97)
        results.clear();count=len(poses)
        h.wait(lambda:any(not r['valid'] and 'unsynchronized' in r['reason'] for r in results),lambda:send(True))
    finally:h.close()
