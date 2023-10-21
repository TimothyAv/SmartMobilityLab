# Scenario 2
# Goal description: Perform courier services in a city.
# Fleet size: 10
# Expected output (vehicle routes):
# Vehicle 1: Route X
# Vehicle 2: Route Y
# Vehicle 3: Route Z
# Vehicle 4: Route X
# Vehicle 5: Route Y
# Vehicle 6: Route Z
# Vehicle 7: Route X
# Vehicle 8: Route Y
# Vehicle 9: Route Z
# Vehicle 10: Route X

fleet_size = 10

# Call the fleet management client with the fleet size
routes = fleet_management_client.send_request(fleet_size)

print("Vehicle Routes:")
for i, route in enumerate(routes):
    print(f"Vehicle {i + 1}: {route}")

