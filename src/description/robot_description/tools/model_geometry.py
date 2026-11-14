"""Small URDF geometry helpers shared by the offline asset builder and tests.

Transforms are parent_T_child, metres/radians, URDF fixed-axis XYZ RPY.
This module is for offline model checks; omitted joint values mean zero,
not an estimate of a running robot's state.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np


def rotation(rpy):
    r, p, y = map(float, rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def transform(element):
    result = np.eye(4)
    if element is not None:
        result[:3, :3] = rotation(element.get('rpy', '0 0 0').split())
        result[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    return result


def set_origin(parent, matrix):
    origin = parent.find('origin')
    if origin is None:
        origin = ET.SubElement(parent, 'origin')
    origin.set('xyz', ' '.join(format(float(x), '.17g') for x in matrix[:3, 3]))
    r = matrix[:3, :3]
    pitch = math.atan2(-r[2,0], math.hypot(r[0,0], r[1,0]))
    if abs(math.cos(pitch)) > 1e-10:
        angles = (math.atan2(r[2,1],r[2,2]), pitch, math.atan2(r[1,0],r[0,0]))
    else:
        angles = (math.atan2(-r[1,2],r[1,1]), pitch, 0.0)
    origin.set('rpy', ' '.join(format(float(x), '.17g') for x in angles))


def frames(model, positions=None):
    """Return root_T_link for a complete single-tree URDF. Missing q means zero."""
    positions = positions or {}
    links = [link.get('name') for link in model.findall('link')]
    joints = list(model.findall('joint'))
    if len(set(links)) != len(links):
        raise ValueError('duplicate link')
    children = [joint.find('child').get('link') for joint in joints]
    roots = set(links) - set(children)
    if len(roots) != 1 or len(set(children)) != len(children):
        raise ValueError('URDF is not a single tree')
    result = {next(iter(roots)): np.eye(4)}
    while joints:
        progress = False
        for joint in joints[:]:
            name = joint.get('name')
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if child not in links or parent not in links:
                raise ValueError('unknown joint link: ' + name)
            if parent not in result:
                continue
            motion = np.eye(4)
            kind = joint.get('type')
            if kind != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                norm = np.linalg.norm(axis)
                if norm < 1e-12:
                    raise ValueError('zero axis: ' + name)
                axis = axis / norm
                q = float(positions.get(name, 0.0))
                if not np.isfinite(q):
                    raise ValueError('nonfinite position: ' + name)
                if kind == 'prismatic':
                    motion[:3, 3] = axis * q
                elif kind in ('revolute', 'continuous'):
                    x, y, z = axis
                    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    motion[:3, :3] = np.eye(3) + math.sin(q)*cross + (1-math.cos(q))*(cross@cross)
                else:
                    raise ValueError('unsupported joint type: ' + kind)
            result[child] = result[parent] @ transform(joint.find('origin')) @ motion
            joints.remove(joint)
            progress = True
        if not progress:
            raise ValueError('cyclic or disconnected URDF')
    if set(result) != set(links):
        raise ValueError('disconnected link')
    return result


def resolve_mesh(root, uri):
    prefix = 'package://robot_description/'
    if not uri.startswith(prefix):
        raise ValueError('nonportable mesh URI: ' + uri)
    path = (Path(root) / uri[len(prefix):]).resolve()
    if not path.is_relative_to(Path(root).resolve()) or not path.is_file():
        raise ValueError('mesh missing or outside package: ' + uri)
    return path
