#!/usr/bin/env python

from base import VehicleBase
from dronekit import VehicleMode, LocationGlobalRelative
import time


class VehiclePX4(VehicleBase):
    def __init__(self, gcs, addr):
        super(VehiclePX4, self).__init__(gcs, addr)

        self.add_handler("takeoff", self.takeoff)
        self.add_handler("land", self.land)
        self.add_handler("rtl", self.rtl)
        self.add_handler("goto", self.goto)
        self.add_handler("rc", self.rc)

    def run(self):
        print("Type: %s" % self.vehicle._vehicle_type)
        print("Armed: %s" % self.vehicle.armed)
        print("System status: %s" % self.vehicle.system_status.state)
        print("GPS: %s" % self.vehicle.gps_0)
        print("Alt: %s" % self.vehicle.location.global_relative_frame.alt)

        super(VehiclePX4, self).run()

    def arm(self):
        if self.vehicle.armed:
            return

        print("Pre-arm checks")

        # Don't arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Wait for vehicle to arm
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

    def takeoff(self, data):
        self.arm()

        targetAlt = data and data["relAlt"] or 10
        # Take off to target altitude
        self.vehicle.simple_takeoff(targetAlt)

        return (None, None)

    def land(self, data):
        self.vehicle.mode = VehicleMode("LAND")
        return (None, None)

    def rtl(self, data):
        self.vehicle.mode = VehicleMode("RTL")
        return (None, None)

    def rc(self, data):
        if data is None:
            print("Warning! rc message with no data")
        else:
            roll = data["roll"]
            pitch = data["pitch"]
            yaw = data["yaw"]
            throttle = data["throttle"]
            gimbal = data["gimbal"]

    def goto(self, data):
        self.arm()

        if data is None:
            return (None, "no data")
        else:
            lat = data["lat"]
            lon = data["lon"]
            rel_alt = data["relAlt"]

            p = LocationGlobalRelative(lat, lon, rel_alt)
            self.vehicle.simple_goto(p)
            print("Goto lat: %s, lon: %s, alt: %s" % (lat, lon, rel_alt))

            return ("OK", None)
