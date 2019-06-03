#!/usr/bin/env python2

import sys, getopt
import thread

from conn.ws import WsConn
from vehicles.px4 import VehiclePX4
from config.config import Config

class App(object):
    def start(self):
        self.readArgs()

        conf = Config.read(self.configfile)

        self.gcsConn = WsConn(conf.gcs + "/vehicle?token=" + conf.token)
        print("Connect to url: %s" % conf.mavlink)
        self.vehicle = VehiclePX4(self.gcsConn, conf.mavlink)

        thread.start_new_thread(self.gcsConn.connect, ())
        self.vehicle.start()


    def readArgs(self):
        self.configfile = ""

        try:
            opts, args = getopt.getopt(sys.argv, "hc:", ["help","config="])
        except getopt.GetoptError:
            print("test.py -i <inputfile> -o <outputfile>")
            sys.exit(2)

        for opt, arg in opts:
            if opt in ("-h", "--help"):
                print("main.py -c <configfile>")
                sys.exit()
            elif opt in ("-c", "--config"):
                self.configfile = arg

if __name__ == '__main__':
    App().start()
