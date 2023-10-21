import click

@click.command()
@click.option('--fleet-size', type=int, help='Specify the fleet size')
def fleet_management_cli(fleet_size):
    from tim_fleet.fleet_management_client import FleetManagementClient

    client = FleetManagementClient()
    routes = client.send_request(fleet_size)

    click.echo("Vehicle Routes:")
    for route in routes:
        click.echo(route)

if __name__ == '__main__':
    fleet_management_cli()

