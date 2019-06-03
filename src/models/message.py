#!/usr/bin/env python

import json

class Message(object):
    REQ = 0
    UPD = 0
    counter = 0

    @staticmethod
    def parseJSON(string):
        try:
            data = json.loads(string)
            return Message(
                typ=data.get("type", None),
                data=data.get("data", None),
                verb=Message.verb_from_str(data.get("verb", "req")),
                id=data.get("id", None)
            )
        except ValueError:
            return None

    @staticmethod
    def verb_from_str(verb):
        if verb == "upd":
            return Message.UPD

        return Message.REQ

    @staticmethod
    def unique_id():
        Message.counter += 1
        return str(Message.counter)

    def __init__(self, typ=None, data=None, verb=REQ, id=None):
        self.id = id if isinstance(id, basestring) else Message.unique_id()
        self.type = typ.split(":") if isinstance(typ, basestring) else []
        self.verb = verb
        self.data = data if data is not None else {}

    def get_type(self, i=None):
        if i is None:
            return ":".join(self.type)
        if i < len(self.type):
            return self.type[i]
        else:
            return None

    def stringify(self):
        obj = {
            "type": ":".join(self.type),
            "id": self.id
        }

        if self.data:
            obj["data"] = self.data

        return json.dumps(obj)
