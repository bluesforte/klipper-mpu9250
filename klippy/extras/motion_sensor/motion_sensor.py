import importlib

sensor_chip_infos = {
        "adxl345": { "module" : ".adxl345",     "class" : "ADXL345" },
        "mpu6050": { "module" : ".invensense",  "class" : "MPU6050" },
        "mpu9250": { "module" : ".invensense",  "class" : "MPU9250" },
    }

class MotionSensor:
    def __init__(self, config):
        self.sensor_chip_info = config.getchoice('chip', sensor_chip_infos)
        package_name = '.'.join(self.__module__.split('.')[:-1])
        mod = importlib.import_module(name=self.sensor_chip_info["module"], 
            package=package_name)
        self.sensor_class = getattr(mod, self.sensor_chip_info["class"])
        self.sensor = self.sensor_class(config)

def load_config(config):
    return MotionSensor(config)