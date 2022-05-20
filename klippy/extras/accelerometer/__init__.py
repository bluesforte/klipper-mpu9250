import accelerometer

def load_config(config):
    return accelerometer.load_config(config)

def load_config_prefix(config):
    if not config.has_section('accelerometer'):
        raise config.error(
            "A primary [accelerometer] section must be defined in printer.cfg "
            "to use auxilary displays")
    name = config.get_name().split()[-1]
    if name == "accelerometer":
        raise config.error(
            "Section name [accelerometer accelerometer] is not valid. "
            "Please choose a different postfix.")
    return accelerometer.load_config(config)