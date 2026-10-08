#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import json
import logging
import math
import signal
import time

import carla

# Simulation step: 100 Hz base tick, so the IMU publishes at 100 Hz. Sensors with
# a "sensor_tick" attribute (the GNSS, 0.1 s) publish at their own lower rate.
#
# Deliberately not exactly 0.01. On each tick the server keeps stepping until
# its game time reaches "time at the tick + this value" (a double), but each
# frame only advances by this value rounded to float32. float32(0.01) is
# 2.2e-10 below 0.01, so one frame falls just short and the server runs a
# second one: two frames per world.tick(). float32(0.0100000005) is above it,
# so a single frame reaches the target. The step differs from 0.01 s by 7e-10 s.
FIXED_DELTA_SECONDS = 0.0100000005

# Time to wait after spawning the vehicle, before the sensors exist, so the drop
# onto the ground and the suspension settling never reach the topics.
SETTLE_SECONDS = 2.0

# Time the vehicle stays parked with the sensors publishing, for the static IMU
# initialisation (needs more than 10 s of stationary data).
STATIC_SECONDS = 12.0

LOG_EVERY_N_TICKS = 500


def _setup_town(client, config):
    world = client.get_world()
    town = config.get("town")

    if town and not world.get_map().name.endswith(town):
        logging.info("Loading town: {}".format(town))
        world = client.load_world(town)
        world.tick()
        logging.info("Town loaded: {}".format(world.get_map().name))
    else:
        logging.info("Using current map: {}".format(world.get_map().name))

    return world


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    return  world.spawn_actor(
        bp,
        map_.get_spawn_points()[0],
        attach_to=None)


def _setup_sensors(world, vehicle, sensors_config):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"]
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"]
            )
        )

        sensors.append(
            world.spawn_actor(
                bp,
                wp,
                attach_to=vehicle
            )
        )

        sensors[-1].enable_for_ros()

    return sensors


def _update_spectator(world, vehicle, distance=8.0, height=3.0, pitch=-15.0):
    """Move the spectator to a third-person chase view behind the vehicle."""
    transform = vehicle.get_transform()
    yaw = transform.rotation.yaw

    # Unit forward vector of the vehicle (CARLA's left-handed UE convention).
    forward = carla.Vector3D(
        x=math.cos(math.radians(yaw)),
        y=math.sin(math.radians(yaw)),
        z=0.0
    )

    location = transform.location - forward * distance
    location.z += height

    spectator_transform = carla.Transform(
        location=location,
        rotation=carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)
    )

    world.get_spectator().set_transform(spectator_transform)


class _Pacer:
    """Ticks the world at real-time pace and reports the measured rate."""

    def __init__(self, world, vehicle, target_dt):
        self._world = world
        self._vehicle = vehicle
        self._target_dt = target_dt

        # Real-time reference: wall-clock time and simulation time at the same instant.
        self._wall_ref = None
        self._sim_ref = None

        self._tick_count = 0
        self._window_wall = None
        self._window_sim = None
        self._window_frame = None

    def tick(self):
        frame = self._world.tick()
        _update_spectator(self._world, self._vehicle)

        sim_now = self._world.get_snapshot().timestamp.elapsed_seconds
        wall_now = time.perf_counter()

        if self._wall_ref is None:
            self._wall_ref, self._sim_ref = wall_now, sim_now
            self._window_wall, self._window_sim, self._window_frame = wall_now, sim_now, frame

        self._tick_count += 1
        if self._tick_count % LOG_EVERY_N_TICKS == 0:
            wall_elapsed = wall_now - self._window_wall
            logging.info(
                "Real-time factor: %.2f (target 1.00), client tick rate: %.2f Hz, "
                "server frames per tick: %.2f",
                (sim_now - self._window_sim) / wall_elapsed,
                LOG_EVERY_N_TICKS / wall_elapsed,
                (frame - self._window_frame) / LOG_EVERY_N_TICKS
            )
            self._window_wall, self._window_sim, self._window_frame = wall_now, sim_now, frame

        # Pace on simulation time rather than on the number of client ticks: the
        # server may advance more than one frame per tick, and counting ticks
        # would then let the simulation run faster than real time.
        sleep_time = self._wall_ref + (sim_now - self._sim_ref) - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -0.1:
            # More than 0.1 s behind real time (the server cannot keep up): move
            # the reference instead of trying to "catch up" in a burst.
            self._wall_ref, self._sim_ref = time.perf_counter(), sim_now

    def run_for(self, seconds):
        """Tick for the given amount of simulation time."""
        for _ in range(int(round(seconds / self._target_dt))):
            self.tick()


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        with open(args.file) as f:
            config = json.load(f)

        world = _setup_town(client, config)

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
        world.apply_settings(settings)

        applied = world.get_settings()
        logging.info(
            "Applied settings -- synchronous_mode=%s fixed_delta_seconds=%s",
            applied.synchronous_mode, applied.fixed_delta_seconds
        )

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        vehicle = _setup_vehicle(world, config)

        # Hold the vehicle in place until the autopilot (or a control command) takes over.
        vehicle.apply_control(carla.VehicleControl(hand_brake=True))

        pacer = _Pacer(world, vehicle, settings.fixed_delta_seconds)

        logging.info("Letting the vehicle settle (%.1f s)...", SETTLE_SECONDS)
        pacer.run_for(SETTLE_SECONDS)

        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        logging.info("Publishing stationary data (%.1f s)...", STATIC_SECONDS)
        pacer.run_for(STATIC_SECONDS)

        vehicle.set_autopilot(config.get("autopilot", False))

        logging.info("Running...")

        while True:
            pacer.tick()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        if original_settings:
            world.apply_settings(original_settings)

        for sensor in sensors:
            sensor.destroy()

        if vehicle:
            vehicle.destroy()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA ROS2 native')
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', default='config.json', help='Configuration JSON file (default: config.json)')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug', help='print debug information')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('Listening to server %s:%s', args.host, args.port)

    # Containers stop with SIGTERM; translate it into KeyboardInterrupt so the
    # cleanup in main() runs (destroy actors, restore world settings).
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    main(args)
