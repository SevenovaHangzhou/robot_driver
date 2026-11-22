import json
from pathlib import Path


def apply_approved_fit(spheres):
    path = Path(__file__).resolve().parents[1] / 'config/curobo/approved_head_carriage_fit.json'
    document = json.loads(path.read_text())
    for name, entry in document['links'].items():
        spheres[name] = entry['spheres']
    return document
