#!/usr/bin/env python

class EventDispatcher(object):
    def __init__(self):
        self._listeners = {}

    def add_listener(self, event, listener):
        if event in self._listeners:
            listeners = self._listeners[event]
        else:
            listeners = []
            self._listeners[event] = listeners

        listeners.append(listener)

    def trigger(self, event, data=None):
        if event in self._listeners:
            listeners = self._listeners[event]
            for listener in listeners:
                listener(data)
