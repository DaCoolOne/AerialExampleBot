import math

from rlbot.agents.base_agent import SimpleControllerState

from util.vec import Vec3
from util.orientation import Orientation

"""

Todo:

1. Create an aerial turn controller
    a. Make the bot point to a forward vector
    b. How to do the up vector

2. Create a delta v function

3. Create a function to go for the aerial

4. How to improve our code

"""


def clamp(n, a, b):
    return max(min(n, b), a)


def align_car_to(controller: SimpleControllerState, angular_velocity: Vec3, rotation: Orientation,
                 forward: Vec3, up: Vec3):

    local_forward = rotation.cast_local(forward)

    ang_vel_local = rotation.cast_local(angular_velocity)

    pitch_angle = math.atan2(-local_forward.z, local_forward.x)
    yaw_angle = math.atan2(-local_forward.y, local_forward.x)

    pitch_angular_velocity = ang_vel_local.y
    yaw_angular_velocity = ang_vel_local.z

    p = 4
    d = 0.9

    controller.pitch = clamp(-pitch_angle * p + pitch_angular_velocity * d, -1, 1)
    controller.yaw = clamp(-yaw_angle * p - yaw_angular_velocity * d, -1, 1)


def delta_v(displacement: Vec3, initial_velocity: Vec3, time: float, gravity: float) -> Vec3:
    return Vec3(
        (2 * (displacement.x - initial_velocity.x * time)) / (time * time),
        (2 * (displacement.y - initial_velocity.y * time)) / (time * time),
        (2 * (displacement.z - initial_velocity.z * time)) / (time * time) - gravity
    )


def look_for_aerial(ball_prediction, current_time, car_loc: Vec3, car_vel: Vec3, gravity):
    slices = ball_prediction.slices
    for i in range(0, ball_prediction.num_slices):
        s = slices[i]
        loc = Vec3(s.physics.location)
        delta_t = s.game_seconds - current_time

        if delta_t <= 0:
            continue

        dv = delta_v(loc - car_loc, car_vel, delta_t, gravity)

        if dv.length() < 1060:
            return loc, s.game_seconds, dv

    return None, None, None

