"""Resolve missing declared passive tails from a read-only EtherCAT scan.

Only optional sensor positions may disappear. All other identities and the main
coupler's branch connections must match the declared physical layout.
"""

from itertools import combinations
import re


def resolve_sensor_topology(topology, sensors, scan):
    master = topology['master_id']
    blocks = re.findall(rf'=== Master {master}, Slave (\d+) ===(.*?)(?==== Master|\Z)', scan, re.S)
    observed = []
    for position, block in blocks:
        identity = {'ring_position': int(position)}
        for label, key in (('Vendor Id', 'vendor_id'), ('Product code', 'product_code'),
                           ('Revision number', 'revision_number')):
            match = re.search(label + r':\s*(0x[0-9a-fA-F]+)', block)
            if match is None:
                raise ValueError(f'Missing {label} at slave {position}')
            identity[key] = int(match[1], 16)
        observed.append(identity)
    if not observed or [x['ring_position'] for x in observed] != list(range(len(observed))):
        raise ValueError('Incomplete or unordered EtherCAT scan')
    expected = sorted(topology['responders'], key=lambda x: x['ring_position'])
    optional = {sensor['ring_position'] for sensor in sensors}
    if not optional.issubset({x['ring_position'] for x in expected}):
        raise ValueError('Optional sensor is absent from the declared physical topology')
    missing_count = len(expected) - len(observed)
    if missing_count < 0 or missing_count > len(optional):
        raise ValueError('Unexpected responder count; only declared optional sensors may be missing')
    candidates = []
    for missing in combinations(sorted(optional), missing_count):
        remaining = [x for x in expected if x['ring_position'] not in missing]
        if any(any(actual[k] != wanted[k] for k in ('vendor_id', 'product_code', 'revision_number'))
               for actual, wanted in zip(observed, remaining)):
            continue
        mapping = {item['ring_position']: index for index, item in enumerate(remaining)}
        connections = topology['branches'] + topology['internal_ports']
        if any(not re.search(
                rf"^\s+{item['main_esc_port']}\s+(?:MII|EBUS)\s+up\s+open\s+yes\s+{mapping[item['downstream']]}\b",
                blocks[0][1], re.M) for item in connections):
            continue
        candidates.append((mapping, set(missing)))
    if len(candidates) != 1:
        raise ValueError('EtherCAT identities or branch wiring do not uniquely match the declared layout')
    return candidates[0]
