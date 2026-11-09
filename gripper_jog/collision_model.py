"""Import the artifact's arm collision geometry; never derive it from visuals."""
from hashlib import sha256
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET

import mujoco

from .errors import JogError


def arm_collision_links(root, names):
    joints = {j.get('name'): j for j in root.findall('joint')}
    children = {}
    for joint in joints.values():
        children.setdefault(joint.find('parent').get('link'), []).append(joint.find('child').get('link'))
    result = {}
    for side in ('left', 'right'):
        arm = [joints[n] for n in names if n.startswith(side + '_')]
        if len(arm) != 7:
            raise JogError('collision_model', '碰撞模型需要每臂完整的 7 个受控关节')
        descendants = {j.find('child').get('link') for j in arm}
        pending = list(descendants)
        while pending:
            for link in children.get(pending.pop(), []):
                if link not in descendants:
                    descendants.add(link)
                    pending.append(link)
        result[side] = descendants
    if result['left'] & result['right']:
        raise JogError('collision_model', '双臂碰撞链不能共享下游连杆')
    return result


def artifact_model(xml, names, model_share=None, *, collisions=False):
    root = ET.fromstring(xml)
    groups = arm_collision_links(root, names) if collisions else {}
    arm_links = set().union(*groups.values()) if groups else set()
    share = Path(model_share).resolve() if model_share else None
    assets, sources = {}, []
    for tag in ('ros2_control', 'mujoco', 'gazebo', 'transmission'):
        for element in root.findall(tag):
            root.remove(element)
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        for i, collision in enumerate(link.findall('collision')):
            if link.get('name') not in arm_links:
                link.remove(collision)
                continue
            collision.set('name', f'jog_collision_{link.get("name")}_{i}')
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise JogError('collision_model', 'URDF 碰撞体必须包含一个明确的几何形状')
            mesh = geometry.find('mesh')
            if mesh is not None:
                uri = mesh.get('filename', '')
                prefix = 'package://robot_description/'
                relative = PurePosixPath(uri.removeprefix(prefix))
                if (share is None or not uri.startswith(prefix) or relative.is_absolute()
                        or '..' in relative.parts):
                    raise JogError('collision_asset', '碰撞 mesh 必须来自指定的 robot_description package')
                source = share.joinpath(relative).resolve()
                if not source.is_relative_to(share) or not source.is_file() or source.suffix.lower() != '.stl':
                    raise JogError('collision_asset', '碰撞 STL 缺失、不受支持或路径越界')
                raw = source.read_bytes()
                key = f'jog_collision_mesh_{len(assets)}.stl'
                assets[key] = raw
                mesh.set('filename', key)
                sources.append({'link': link.get('name'), 'uri': uri, 'sha256': sha256(raw).hexdigest()})
            elif geometry[0].tag not in ('box', 'sphere', 'cylinder'):
                raise JogError('collision_model', '不支持的 URDF 碰撞几何')
    if collisions:
        controlled = {j.find('child').get('link') for j in root.findall('joint') if j.get('name') in names}
        for link in root.findall('link'):
            name = link.get('name')
            physical = name in controlled or link.find('inertial') is not None
            if name in arm_links and physical and not link.findall('collision'):
                raise JogError('collision_model', f'受控连杆或末端工具 {name} 缺少碰撞几何')
    extension = ET.SubElement(root, 'mujoco')
    ET.SubElement(extension, 'compiler', fusestatic='false', discardvisual='true', inertiafromgeom='false')
    try:
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), assets)
    except ValueError as exc:
        raise JogError('collision_model', f'运行 URDF 碰撞模型加载失败：{exc}') from exc
    return model, groups, sources
