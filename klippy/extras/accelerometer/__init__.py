from . import accelerometer

def load_config(config):
    return accelerometer.load_config(config)

def load_config_prefix(config):
    name = config.get_name().split()[-1]
    if name == "accelerometer":
        raise config.error(
            "Section name [accelerometer accelerometer] is not valid. "
            "Please choose a different postfix.")
    return accelerometer.load_config(config)