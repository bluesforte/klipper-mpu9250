import importlib

accel_chip_infos = {
        "adxl345": { "module" : "adxl345",     "class" : "ADXL345" },
        "mpu9250": { "module" : "invensense",  "class" : "MPU9250" },
        "mpu6050": { "module" : "invensense",  "class" : "MPU6050" },
    }

class Accelerometer:
    def __init__(self, config):
        self.accel_chip_info = config.getchoice('chip', accel_chip_infos)
        mod = importlib.import_module(self.accel_chip_info["module"], "extras.accelerometer")
        self.accel_class = getattr(mod, self.accel_chip_info["class"])
        self.accelerometer = self.accel_class(config)



def load_config(config):
    return Accelerometer(config)