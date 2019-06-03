#!/usr/bin/env python

from base import VehicleBase
from pymavlink import mavutil
from dronekit import VehicleMode
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

        #self.rc_pub = rospy.Publisher("/mavros/manual_control/control", ManualControl, queue_size=10)
        #self.rc_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        #self.goto_pub = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)

    def takeoff(self, data):
        #self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system, self.vehicle._master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)

        print("Pre-arm checks")
        targetAlt = 10

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

        print("Taking off!")
        # Take off to target altitude
        self.vehicle.simple_takeoff(targetAlt)

        # Wait for takeoff to finish
        while self.vehicle.location.global_relative_frame.alt < targetAlt * 0.95:
            print("Altitude: %s" % self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)

        return (None, None)

    def land(self, data):
        return (None, "not implemented")

        #rospy.wait_for_service('/mavros/cmd/land')

        #try:
        #    land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        #    resp = land(x, y)

        #    if resp.success is True:
        #        cb(None)
        #    else:
        #        cb(resp.result)

        #except rospy.ServiceException as err:
        #    cb("%s" % err)

    def rtl(self, data):
        return (None, "not implemented")

    def rc(self, data):
        if data is None:
            print("Warning! rc message with no data")
        else:
            roll = data["roll"]
            pitch = data["pitch"]
            yaw = data["yaw"]
            throttle = data["throttle"]
            gimbal = data["gimbal"]

        #msg = ManualControl()
        #msg.x = pitch
        #msg.y = roll
        #msg.z = throttle
        #msg.r = yaw

        #self.rc_pub.publish(msg)

    def goto(self, data):
        if data is None:
            return (None, "no data")
        else:
            lat = data["lat"]
            lon = data["lon"]
            rel_alt = data["relAlt"]
            print("Goto lat: %s, lon: %s, alt: %s" % (lat, lon, rel_alt))

            return ("OK", None)

        #msg = GlobalPositionTarget()
        #msg.coordinate_frame = msg.FRAME_GLOBAL_REL_ALT
        #msg.latitude = lat
        #msg.longitude = lon
        #msg.altitude = rel_alt

        #self.goto_pub.publish(msg)
        #cb()

