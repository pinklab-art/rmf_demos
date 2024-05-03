import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory
from rmf_fleet_msgs.msg import FleetState

class MapSwitcher(Node):
    def __init__(self, map_files):
        super().__init__('map_switcher')
        self.map_files = map_files
        self.current_level = "L1"
        self.client = self.create_client(LoadMap, 'map_server/load_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map_server to be available...')
        self.get_logger().info('Service available.')
        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_state_callback,
            10
        )
        self.is_request_active = False  # 현재 요청 상태를 추적하는 변수

    def fleet_state_callback(self, msg):
        if not msg.robots:
            self.get_logger().info('No robots in the fleet state message.')
            return
        # self.get_logger().info(f'Current level: {self.current_level}')
        # self.get_logger().info(f'Robot location: {msg.robots[0].location.level_name}')
        level_name = msg.robots[0].location.level_name
        if level_name != self.current_level and not self.is_request_active:
            self.current_level = level_name
            self.switch_map('rmf_demos_maps', 'pinklab', self.map_files[level_name])

    def switch_map(self, package_name, map_name, map_path):
        self.is_request_active = True
        full_path = get_package_share_directory(package_name) + '/' + map_name + '/' + map_path
        self.get_logger().info(f'Trying to load map: {full_path}')
        req = LoadMap.Request()
        req.map_url = full_path
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_map_response)

    def handle_map_response(self, future):
        try:
            response = future.result()
            if response.result == response.RESULT_SUCCESS:
                self.get_logger().info('Map loaded successfully')
            else:
                self.get_logger().error(f'Failed to load map, error code: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Exception while handling map load response: {str(e)}')
        finally:
            self.is_request_active = False


def main(args=None):
    rclpy.init(args=args)
    map_files = {
        'L1': sys.argv[1],
        'L2': sys.argv[2]
    }
    map_switcher = MapSwitcher(map_files)
    rclpy.spin(map_switcher)
    map_switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
