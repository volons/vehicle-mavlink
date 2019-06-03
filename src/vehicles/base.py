#!/usr/bin/env python

import Queue
import thread
import time
from dronekit import connect
from pymavlink import mavutil

#from sensor_msgs.msg import NavSatFix, BatteryState
#from std_msgs.msg import Float64
from models.message import Message
from config.config import Config

class VehicleBase(object):
    '''
    Abstract class, does not implement vehicle methods (takeoff, land, etc.)
    '''

    def __init__(self, gcs, addr):
        self.rel_alt = 0
        self.hdg = 0
        self.addr = addr
        self.gcs = gcs
        self.queue = Queue.Queue()
        self.gcs.add_listener("message", self.queue_message)
        self.handlers = {}
        self.vehicle = None

        self.add_handler("info", self.info)

    def start(self):
        print("Connect to url: %s" % self.addr)
        self.vehicle = connect(self.addr, wait_ready=True, heartbeat_timeout=15)
        thread.start_new_thread(self.send_status_loop, ())

        self.run()

    def run(self):
        while True:
            msg = self.queue.get()

            if msg is None:
                break

            self.on_message(msg)
            self.queue.task_done()

    def queue_message(self, msg):
        if msg is not None:
            self.queue.put(msg)

    def add_handler(self, msg_type, handler):
        self.handlers[ msg_type ] = handler

    def on_message(self, msg):
        typ = msg.get_type()

        handler = self.handlers.get(typ, None)
        if handler is not None:
            res, err = handler(msg.data) or (None, None)
            # only reply to requests
            if msg.verb == Message.REQ:
                self.reply(msg, res, err)
        else:
            print("unknown message of type '%s'" % typ)

    def info(self, data):
        conf = Config.get()

        return ({
            "name": conf.name,
            "model": conf.model
        }, None)


    def reply(self, replyTo, result, error):
        msg = Message("reply", {
            "id": replyTo.id,
            "result": result,
            "error": error
        })
        self.send(msg)

    def send(self, msg):
        if self.gcs is not None:
            self.gcs.send_message(msg)

    def takeoff(self, cb):
        cb("not implemented")

    def land(self, cb):
        cb("not implemented")

    def rtl(self, cb):
        cb("not implemented")

    def rc(self, roll, pitch, yaw, throttle, gimbal):
        pass

    def goto(self, lat, lon, relAlt, cb):
        cb("not implemented")

    def send_status_loop(self):
        while True:
            pos = self.vehicle.location.global_relative_frame
            hdg = self.vehicle.heading
            if pos is not None:
                self.send(Message("position", {
                    "lat": pos.lat,
                    "lon": pos.lon,
                    "alt": pos.alt,
                    "relAlt": pos.alt,
                    "hdg": hdg
                }))
                #print("Pos: lat: %s, lon: %s, alt: %s" % (pos.lat, pos.lon, pos.alt))

            bat = self.vehicle.battery
            if bat is not None:
                self.send(Message("battery", {
                    "current": bat.current,
                    "percent": bat.level,
                    "voltage": bat.voltage
                }))
                #print("Battery: current: %s, percent: %s, voltage: %s" % (bat.current, bat.level, bat.voltage))

            time.sleep(1) # send every second



