"""Official submission entrypoint: replacement for blue-team rescue only."""
from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.solutions.my_drone_place_example import MyDronePlaceExample
from swarm_rescue.solutions.blue_policy import MyDroneRescue


def drone_class_for_mode(mode: str):
    if TeamMode.from_string(mode) == TeamMode.PLACE:
        return MyDronePlaceExample
    return MyDroneRescue
