import json
from _def import CMD_SET

def load_config(config_path='config.json'):
    with open(config_path, 'r') as f:
        return json.load(f)

def get_available_ports(config):
    """
    Get a list of all available ports based on the configuration.
    
    :param config: The configuration dictionary
    :return: List of available ports in format "pcb_port-position"
    """
    ports = []
    for valve in config['selector_valves']:
        pcb_port = valve['pcb_port']
        max_position = valve['max_position']
        connection = valve['connection']
        
        if connection:
            connected_position = connection['position']
            for position in range(1, max_position + 1):
                if position != connected_position:
                    ports.append(f"{pcb_port}-{position}")
        else:
            for position in range(1, max_position + 1):
                ports.append(f"{pcb_port}-{position}")
    
    return sorted(ports)

def create_port_mapping(config):
    """
    Create a mapping between simplified port numbers (1 to n) and actual port identifiers.
    
    :param config: The configuration dictionary
    :return: A tuple of two dictionaries (simplified_to_actual, actual_to_simplified)
    """
    available_ports = get_available_ports(config)
    simplified_to_actual = {i+1: port for i, port in enumerate(available_ports)}
    actual_to_simplified = {port: i+1 for i, port in enumerate(available_ports)}
    return simplified_to_actual, actual_to_simplified

def get_valve_path(config, target_port):
    """
    Determine the path of selector valves that need to be opened to reach the target port.
    
    :param config: The configuration dictionary
    :param target_port: The target port in format "pcb_port-position" (e.g., "3-1")
    :return: List of ports to open in order
    """
    target_pcb, target_pos = map(int, target_port.split('-'))
    path = [(target_pcb, target_pos)]
    
    while True:
        current_valve = next((v for v in config['selector_valves'] if v['pcb_port'] == target_pcb), None)
        if not current_valve or not current_valve['connection']:
            break
        
        connection = current_valve['connection']
        target_pcb = connection['destination_selector_valve']
        target_pos = connection['position']
        path.append((target_pcb, target_pos))
    
    return [f"{pcb}-{pos}" for pcb, pos in reversed(path)]

def open_selector_valve_path(controller, config, simplified_port, port_mapping):
    """
    Open the necessary selector valves to reach the target port.
    
    :param controller: The FluidController instance
    :param config: The configuration dictionary
    :param simplified_port: The simplified port number (1 to n)
    :param port_mapping: The mapping from simplified to actual ports
    """
    actual_port = port_mapping[simplified_port]
    path = get_valve_path(config, actual_port)
    for port in path:
        pcb_port, position = map(int, port.split('-'))
        controller.send_command(CMD_SET.SET_ROTARY_VALVE, pcb_port, position)

def get_simplified_ports(config):
    """
    Get a list of simplified port numbers (1 to n).
    
    :param config: The configuration dictionary
    :return: List of simplified port numbers
    """
    return list(range(1, len(get_available_ports(config)) + 1))
