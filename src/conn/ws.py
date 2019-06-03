#!/usr/bin/env python

import websocket
import Queue
import thread
import time

from libs.events import EventDispatcher
from models.message import Message

class WsConn(EventDispatcher):
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.msgQueue = Queue.Queue()
        self.tries = 0

        super(WsConn, self).__init__()

    def connect(self):
        #websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp(self.url,
            on_message = self.on_message,
            on_error = self.on_error,
            on_close = self.on_close
        )
        self.ws.on_open = self.on_open

        while True:
            self.ws.run_forever()
            self.tries = min(self.tries + 1, 10)
            time.sleep(self.tries)

    def on_message(self, message):
        msg = Message.parseJSON(message);

        if msg is not None:
            self.trigger("message", msg)

    def on_error(self, err):
        print("websocket error %s" % repr(err))
        self.trigger("error", err)

    def on_close(self):
        print("websocket closed")
        self.trigger("close")

    def on_open(self):
        print("connected to gate")
        self.tries = 0
        thread.start_new_thread(self.run, ())

    def run(self, *args):
        while True:
            msg = self.msgQueue.get()

            if msg is None:
                break

            self.ws.send(msg)
            self.msgQueue.task_done()

        self.ws.close()
        print("thread terminating...")

    def send_message(self, message):
        msg = message.stringify()
        if msg is not None:
            self.msgQueue.put(msg)

    def close(self):
        self.msgQueue.put(None)

