# Scenario 1
# Goal description: Deliver packages to three locations in a small town.
# Fleet size: 5
# Expected output (vehicle routes):
# Vehicle 1: Route A
# Vehicle 2: Route B
# Vehicle 3: Route C
# Vehicle 4: Route A
# Vehicle 5: Route B

fleet_size = 5

# Call the fleet management client with the fleet size
routes = fleet_management_client.send_request(fleet_size)

print("Vehicle Routes:")
for i, route in enumerate(routes):
    print(f"Vehicle {i + 1}: {route}")

