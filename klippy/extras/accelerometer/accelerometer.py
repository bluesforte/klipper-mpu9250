import importlib

accel_chip_infos = {
        "adxl345": { "package" : "adxl345",     "class" : "ADXL345" },
        "mpu9250": { "package" : "invensense",  "class" : "MPU9250" },
        "mpu6050": { "package" : "invensense",  "class" : "MPU6050" },
    }

class Accelerometer:
    def __init__(self, config):
        self.accel_chip_info = config.getchoice('chip', accel_chip_infos)(config)
        mod = importlib.import_module(self.accel_chip_info["package"], ".")
        self.accel_class = getattr(mod, self.accel_chip_info["class"])
        self.accelerometer = self.accel_class(config)
    


def load_config(config):
    return Accelerometer(config)