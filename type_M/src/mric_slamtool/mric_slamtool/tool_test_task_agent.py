import sys

import rclpy
from rclpy.node import Node

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask

robot_name = 'robot_B'

class TestTaskAgentClient(Node):

    def __init__(self):
        super().__init__('test_task_agnet_client')
        self.cli = self.create_client(UpdateTaskList, f'{robot_name}_update_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdateTaskList.Request()

    def send_request(self,a):
        self.req.send_msg = a
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = TestTaskAgentClient()
    future = minimal_client.send_request('Update task list.')
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result : {}'.format(response.new_task_list))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()