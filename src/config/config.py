import os

defaults = {
    "gcs_url": "ws://hive:8081",
    "mavlink": "0.0.0.0:14590",
    "token": "dev",
    "name": "Peuplier",
    "model": "Custom"
}


class Config(object):
    _instance = None

    @classmethod
    def get(cls):
        return cls._instance

    @classmethod
    def read(cls, filename):
        if filename in ("", None):
            cls._instance = cls({});
        else:
            with open(filename) as configfile:
                data = json.load(configfile)

            cls._instance = cls(data)

        return cls._instance

    def __init__(self, config):
        self.gcs = self.readValue("gcs_url", config)
        self.mavlink = self.readValue("mavlink", config)
        self.token = self.readValue("token", config)
        self.name = self.readValue("name", config)
        self.model = self.readValue("model", config)

    def readValue(self, name, config):
        # Get value from config then env variable then default value
        return config.get(name,
                os.environ.get("VOLONS_" + name.upper(),
                    defaults.get(name, None)))

