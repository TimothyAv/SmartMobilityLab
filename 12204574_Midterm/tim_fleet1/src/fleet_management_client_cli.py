import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tim_fleet.msg import FleetManagement

class FleetManagementClient(Node):

    def __init__(self):
        super().__init__('fleet_management_client')
        self._action_client = ActionClient(self, FleetManagement, 'fleet_management')

    async def send_request(self, fleet_size):
        goal_msg = FleetManagement.Goal()
        goal_msg.fleet_size = fleet_size
        future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().result.vehicle_routes
        else:
            return []

def main(args=None):
    rclpy.init(args=args)
    fleet_management_client = FleetManagementClient()
    # Usage: python fleet_management_client.py <fleet_size>
    fleet_size = int(sys.argv[1])
    routes = fleet_management_client.send_request(fleet_size)
    print("Vehicle Routes:")
    for route in routes:
        print(route)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

