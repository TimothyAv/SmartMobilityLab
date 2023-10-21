# Fleet Management ROS2 Application

This is a ROS2-based Fleet Management application that includes an Action Server, Action Client, and two sample scenarios.

## Action Definition

### `fleet_management.action`

This action definition includes the following messages:

- **Goal**: `int32 fleet_size`
- **Result**: `string[] vehicle_routes`
- **Feedback**: `float32 completion_percentage`

## Components

### Action Server (`fleet_management_server.py`)

The Action Server (`fleet_management_server.py`) receives fleet size requests, performs fleet management logic, and returns calculated routes as the Action Result. It uses the Fleet Management Action definition to accomplish this.

To run the Action Server:
```shell
ros2 run your_package_name fleet_management_server.py
```

### Action Client CLI (`fleet_management_client_cli.py`)

The Action Client CLI (`fleet_management_client_cli.py`) allows users to request fleet management tasks by specifying the fleet size. It sends the request to the server, receives the routes in response, and displays them to the user.

To run the Action Client CLI:
```shell
ros2 run your_package_name fleet_management_client_cli.py <fleet_size>
```

### Professional CLI (`fleet_management_cli.py`)

This Professional CLI (`fleet_management_cli.py`) is designed using the `click` library. It provides an option for users to allocate and route vehicles by specifying the fleet size. It internally calls the Action Client CLI to perform the task.

To run the Professional CLI:
```shell
ros2 run your_package_name fleet_management_cli.py --fleet-size <fleet_size>
```

## Scenarios

### Scenario 1 (`scenario_1.py`)

- **Goal description:** Deliver packages to three locations in a small town.
- **Fleet size:** 5
- **Expected output (vehicle routes):**
  - Vehicle 1: Route A
  - Vehicle 2: Route B
  - Vehicle 3: Route C
  - Vehicle 4: Route A
  - Vehicle 5: Route B

To run Scenario 1:
```shell
python scenario_1.py
```

### Scenario 2 (`scenario_2.py`)

- **Goal description:** Perform courier services in a city.
- **Fleet size:** 10
- **Expected output (vehicle routes):**
  - Vehicle 1: Route X
  - Vehicle 2: Route Y
  - Vehicle 3: Route Z
  - Vehicle 4: Route X
  - Vehicle 5: Route Y
  - Vehicle 6: Route Z
  - Vehicle 7: Route X
  - Vehicle 8: Route Y
  - Vehicle 9: Route Z
  - Vehicle 10: Route X

To run Scenario 2:
```shell
python scenario_2.py
```

## Running the Application

Make sure to replace `your_package_name` with the actual name of your ROS2 package in the provided commands. Before running the application, ensure your ROS2 environment is set up correctly.

For additional information and detailed instructions, please refer to the package documentation and the respective code files.

