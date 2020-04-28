import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from rlbot.utils.game_state_util import Vector3, Physics, Rotator, CarState, GameState

from util.orientation import Orientation
from util.vec import Vec3
from random import uniform
from util.aerial_utils import align_car_to, look_for_aerial, delta_v


class MyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()

        self.is_aerialing = False

        self.alignment_vector = Vec3()

        self.alignment_timer = -1000

        self.target_loc = Vec3()
        self.dv_length = 0
        self.game_seconds = 0

        self.time_on_ground = 0
        self.time_in_air = 0

        self.p_time = 0

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:

        delta_time = packet.game_info.seconds_elapsed - self.p_time
        self.p_time = packet.game_info.seconds_elapsed

        ball_prediction = self.get_ball_prediction_struct()

        if ball_prediction is None:
            c = SimpleControllerState()
            c.throttle = 1
            return c

        my_car = packet.game_cars[self.index]
        car_loc = Vec3(my_car.physics.location)
        car_vel = Vec3(my_car.physics.velocity)
        car_rot = Orientation(my_car.physics.rotation)
        car_ang_vel = Vec3(my_car.physics.angular_velocity)

        dv = Vec3()

        self.controller_state.throttle = 1
        if self.is_aerialing:

            self.target_loc, self.game_seconds, dv = look_for_aerial(
                ball_prediction, packet.game_info.seconds_elapsed, car_loc, car_vel,
                packet.game_info.world_gravity_z
            )

            self.time_in_air += delta_time

            if self.target_loc is None or (my_car.has_wheel_contact and self.time_in_air > 0.2):
                self.is_aerialing = False
                self.controller_state.boost = False
                self.time_in_air = 0
            else:
                delta_t = self.game_seconds - packet.game_info.seconds_elapsed

                car_to_target = self.target_loc - car_loc
                dv = delta_v(car_to_target, car_vel, delta_t,
                             packet.game_info.world_gravity_z)
                align_car_to(self.controller_state, car_ang_vel, car_rot, dv, car_rot.up)

                self.controller_state.boost = car_rot.forward.dot(dv.normalized()) > 0.6 and dv.length() > 400

                self.controller_state.jump = False
        else:

            self.controller_state.jump = False
            self.controller_state.boost = False

            self.target_loc, self.game_seconds, dv = look_for_aerial(
                ball_prediction, packet.game_info.seconds_elapsed, car_loc, car_vel,
                packet.game_info.world_gravity_z
            )
            self.time_on_ground += delta_time

            if self.target_loc is not None and self.time_on_ground > 0.2:
                self.dv_length = dv.length()
                self.is_aerialing = True

                self.time_on_ground = 0

                self.controller_state.jump = True

        if dv is None:
            dv = Vec3()

        self.renderer.begin_rendering()

        self.renderer.draw_line_3d(car_loc.render(), (car_loc + dv).render(),
                                   self.renderer.blue())

        self.renderer.end_rendering()

        """self.set_game_state(GameState(cars={self.index: CarState(
            physics=Physics(
                location=Vector3(0, 0, 1000),
                velocity=Vector3(0, 0, 0)
            )
        )}))"""

        return self.controller_state


def find_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff


def draw_debug(renderer, car, ball, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(car.physics.location, ball.physics.location, renderer.white())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    renderer.end_rendering()
