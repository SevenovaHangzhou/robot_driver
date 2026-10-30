#!/usr/bin/env python3
"""Convert the current URDF to fixed-base MJCF, retaining all moving joints."""
from pathlib import Path
import copy
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compatible_mesh(path):
    count = struct.unpack('<I', path.read_bytes()[80:84])[0]
    if path.stat().st_size != 84 + 50 * count:
        raise ValueError(f'Expected binary STL: {path}')
    if count <= 200000:
        return path
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.fromfile(path, dtype=dtype, offset=84, count=count)
    vertices, indices = np.unique(triangles['vertices'].reshape(-1, 3), axis=0, return_inverse=True)
    output = ROOT / 'mujoco/meshes' / f'{path.stem}_{sha(path)[:12]}.obj'
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w') as stream:
        np.savetxt(stream, vertices, fmt='v %.9g %.9g %.9g')
        np.savetxt(stream, indices.reshape(-1, 3) + 1, fmt='f %d %d %d')
    return output


def write_xml(root, path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def main():
    source = ROOT / 'urdf/robot_alfav3_mobile.urdf'
    robot = ET.parse(source).getroot()
    mapping = {j.get('name'): {'armature': 0} for j in robot.findall('joint')
               if j.get('type') != 'fixed'}
    for mesh in robot.findall('.//mesh'):
        prefix = 'package://robot_description/'
        filename = mesh.get('filename')
        if not filename.startswith(prefix):
            raise ValueError('Unsupported mesh URI: ' + filename)
        path = ROOT / filename[len(prefix):]
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', '../' + path.relative_to(ROOT).as_posix())
    imported = copy.deepcopy(robot)
    # Upstream CoACD STL files concatenate independent convex components.
    # One MuJoCo mesh would convexify their union and fill intended clearances.
    collision_components = []
    dtype = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    for link in imported.findall('link'):
        for collision in list(link.findall('collision')):
            mesh = collision.find('geometry/mesh')
            if mesh is None or '/mesh/arms/collision/' not in mesh.get('filename'):
                continue
            path = (ROOT/'urdf'/mesh.get('filename')).resolve()
            triangles = np.fromfile(path,dtype=dtype,offset=84)
            vertices, inverse = np.unique(triangles['vertices'].reshape(-1,3),axis=0,return_inverse=True)
            faces = inverse.reshape(-1,3)
            graph = coo_matrix((np.ones(len(faces)*3),
                               (faces.ravel(),faces[:,[1,2,0]].ravel())),shape=(len(vertices),len(vertices)))
            count, labels = connected_components(graph,directed=False)
            if count == 1:
                continue
            link.remove(collision)
            outputs = []
            for component in range(count):
                part = triangles[labels[faces[:,0]] == component]
                dest = ROOT/'mujoco/meshes'/f'{path.stem}_{sha(path)[:12]}_{component:02d}.stl'
                dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_bytes(b'CoACD connected convex component'.ljust(80,b' ')+struct.pack('<I',len(part))+part.tobytes())
                replacement = copy.deepcopy(collision)
                replacement.find('geometry/mesh').set('filename','../'+dest.relative_to(ROOT).as_posix())
                if replacement.get('name'):
                    replacement.set('name',replacement.get('name')+f'_{component}')
                link.append(replacement)
                outputs.append(dest.relative_to(ROOT).as_posix())
            collision_components.append(dict(source=path.relative_to(ROOT).as_posix(),components=outputs))
    mesh_files = {}
    mesh_aliases = {}
    for mesh in imported.findall('.//mesh'):
        path = (ROOT / 'urdf' / mesh.get('filename')).resolve()
        # Several chassis and torso assets have identical basenames but differ
        # in content. Unique import aliases prevent MuJoCo asset-name collisions.
        alias = mesh_aliases.setdefault(path, f'description_mesh_{len(mesh_aliases):03d}.stl')
        mesh_files[alias] = compatible_mesh(path)
        mesh.set('filename', alias)
    extension = ET.SubElement(imported, 'mujoco')
    ET.SubElement(extension, 'compiler', discardvisual='false', fusestatic='false',
                  strippath='false', inertiafromgeom='false')
    spec = mujoco.MjSpec.from_string(ET.tostring(imported, encoding='unicode'))
    for mesh in spec.meshes:
        mesh.file = str(mesh_files[mesh.file])
        # Legacy sensors contain open surface meshes. Body inertias always come
        # from the URDF; shell mode only allows those mesh assets to compile.
        mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
    # Delay contact activation until planar collision meshes become thin boxes.
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0
    for joint in spec.joints:
        joint.armature = mapping[joint.name]['armature']
    spec.option.timestep = 0.001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    xml = ET.fromstring(spec.to_xml())
    # MjSpec's default text serializer rounds values. Restore authoritative
    # transforms, mass, COM and full inertia with 17-digit precision.
    fmt = lambda values: ' '.join(f'{float(x):.17g}' for x in values)
    bodies = {b.get('name'): b for b in xml.findall('.//body')}
    for link in robot.findall('link'):
        body = bodies[link.get('name')]
        src = link.find('inertial')
        if src is None:
            continue
        origin = src.find('origin')
        rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        inertia = src.find('inertia')
        xx, yy, zz, xy, xz, yz = [float(inertia.get(k)) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']]
        tensor = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
        dest = body.find('inertial')
        if dest is None:
            dest = ET.SubElement(body, 'inertial')
        dest.attrib.clear()
        dest.attrib.update(pos=origin.get('xyz', '0 0 0'), mass=src.find('mass').get('value'),
                           fullinertia=fmt([tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
    for joint in robot.findall('joint'):
        body = bodies[joint.find('child').get('link')]
        origin = joint.find('origin')
        body.set('pos', origin.get('xyz', '0 0 0'))
        quat = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_quat()
        body.set('quat', fmt(quat[[3, 0, 1, 2]]))
        if joint.get('name') in mapping:
            dest = body.find('joint')
            dest.set('axis', joint.find('axis').get('xyz'))
            limit = joint.find('limit')
            if joint.get('type') != 'continuous':
                dest.set('range', f'{limit.get("lower")} {limit.get("upper")}')
    for link in imported.findall('link'):
        body = bodies[link.get('name')]
        for tag, group in [('visual', '1'), ('collision', '0')]:
            geoms = [g for g in body.findall('geom') if g.get('group', '0') == group]
            sources = link.findall(tag)
            assert len(geoms) == len(sources), (link.get('name'), tag)
            for geom, src in zip(geoms, sources):
                origin = src.find('origin')
                if origin is not None:
                    geom.set('pos', origin.get('xyz', '0 0 0'))
                    quat = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_quat()
                    geom.set('quat', fmt(quat[[3, 0, 1, 2]]))
    xml.find('compiler').set('inertiafromgeom', 'false')
    xml.find('compiler').set('fusestatic', 'false')
    # Visualization/control frames only; no mass or calibrated transform changes.
    for side, color in [('right', '1 0.25 0.12 1'), ('left', '0.15 0.55 1 1')]:
        ET.SubElement(bodies[side + '_tool0'], 'site', name=side + '_ee',
                      pos='0 0 0', size='0.009', group='2', rgba=color)
    for mesh in xml.findall('asset/mesh'):
        mesh.set('file', '../' + Path(mesh.get('file')).relative_to(ROOT).as_posix())
    # Direct force/torque actuators avoid introducing uncalibrated position gains.
    actuator = ET.SubElement(xml, 'actuator')
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        effort = float(joint.find('limit').get('effort'))
        ET.SubElement(actuator, 'motor', name=name + '_motor', joint=name,
                      gear='1', ctrllimited='true', ctrlrange=f'{-effort} {effort}',
                      forcelimited='true', forcerange=f'{-effort} {effort}')
    # MuJoCo cannot construct a convex hull for a coplanar collision mesh.
    # Keep its visual mesh; represent only the collision surface by a 1 mm box.
    planar_collisions = []
    for asset in xml.findall('asset/mesh'):
        collision_geoms = [g for g in xml.findall('.//worldbody//geom')
                           if g.get('mesh') == asset.get('name') and g.get('group', '0') == '0']
        if not collision_geoms:
            continue
        path = (ROOT/'mujoco'/asset.get('file')).resolve()
        if path.suffix != '.stl':
            continue
        dtype = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
        vertices = np.fromfile(path,dtype=dtype,offset=84)['vertices'].reshape(-1,3).astype(float)
        vertices *= np.fromstring(asset.get('scale','1 1 1'),sep=' ')
        center = vertices.mean(axis=0)
        _, singular, vt = np.linalg.svd(vertices-center,full_matrices=False)
        if singular[-1] > max(singular[0]*1e-8,1e-12):
            continue
        basis = vt.T
        if np.linalg.det(basis)<0:
            basis[:,-1]*=-1
        projected = (vertices-center)@basis
        low, high = projected.min(axis=0), projected.max(axis=0)
        box_center = center + basis@((low+high)/2)
        half_size = np.maximum((high-low)/2,0.0005)
        for geom in collision_geoms:
            original = Rotation.from_quat(np.fromstring(geom.get('quat','1 0 0 0'),sep=' ')[[1,2,3,0]])
            geom.set('pos',fmt(np.fromstring(geom.get('pos','0 0 0'),sep=' ')+original.apply(box_center)))
            quat = (original*Rotation.from_matrix(basis)).as_quat()
            geom.set('quat',fmt(quat[[3,0,1,2]]))
            geom.set('type','box');geom.set('size',fmt(half_size));geom.attrib.pop('mesh')
        planar_collisions.append(dict(mesh=asset.get('file'),collision_instances=len(collision_geoms),
                                      minimum_thickness_m=0.001))
    # Display detailed visuals; compute contacts only on collision geometry.
    for geom in xml.findall('.//worldbody//geom'):
        collision = geom.get('group', '0') == '0'
        geom.set('contype', '1' if collision else '0')
        geom.set('conaffinity', '1' if collision else '0')
        if collision:
            geom.set('rgba', '0.5 0.5 0.5 0')
    # Explicitly filter assembly interfaces, including children of the fixed base.
    contact = ET.SubElement(xml, 'contact')
    for joint in robot.findall('joint'):
        ET.SubElement(contact, 'exclude', body1=joint.find('parent').get('link'),
                      body2=joint.find('child').get('link'))
    write_xml(xml, ROOT / 'mujoco/robot.xml')
    scene = ET.Element('mujoco', model=robot.get('name'))
    ET.SubElement(scene, 'include', file='robot.xml')
    ET.SubElement(scene, 'include', file='environment.xml')
    write_xml(scene, ROOT / 'mujoco/scene.xml')
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'mujoco/scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nq == model.nv == model.nu == len(mapping)
    report = dict(source='urdf/robot_alfav3_mobile.urdf', source_sha256=sha(source),
                  mujoco_version=mujoco.__version__, nq=model.nq, nv=model.nv, nu=model.nu,
                  base='fixed base_footprint; all URDF moving joints retained',
                  contacts=True, collision_group=0, visual_group=1,
                  collision_visible=False,
                  contact_exclusions='directly connected URDF parent/child links',
                  actuators='direct force/torque motors; URDF effort limits',
                  velocity_limits='URDF velocity limits are not enforced by MuJoCo',
                  planar_collision_proxies=planar_collisions,
                  convex_collision_components=collision_components)
    (ROOT / 'mujoco/conversion_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('convex_collision_components', 'planar_collision_proxies')}, indent=2))


if __name__ == '__main__':
    main()
