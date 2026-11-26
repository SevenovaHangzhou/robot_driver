"""Rebuild the default model from the approved preview without additional wheel rotation."""
from pathlib import Path
from copy import deepcopy
import json,hashlib
import xml.etree.ElementTree as ET
import numpy as np
from model_geometry import rotation, set_origin, transform
import yaml
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'preview/chassis_head_20261124'

def build(root=ROOT):
    source=root/'preview/chassis_head_20261124'
    model=ET.parse(source/'urdf/preview_corrected.urdf').getroot()
    for i in range(1, 5):
        joint = model.find(f"joint[@name='wheel{i}_joint']")
        link = model.find(f"link[@name='wheel{i}']")
        # Re-express the hub frame. Front and rear mounts face opposite ways.
        old_T_hub = np.eye(4)
        old_T_hub[:3, :3] = rotation((0, -np.pi / 2, 0))
        set_origin(joint, transform(joint.find('origin')) @ old_T_hub)
        for item in link:
            if item.tag in ('visual', 'collision', 'inertial'):
                set_origin(item, old_T_hub.T @ transform(item.find('origin')))
    for variant in ['canonical','visual','collision']:
        m=deepcopy(model)
        for link in m.findall('link'):
            if variant!='canonical':
                for child in link.findall('collision' if variant=='visual' else 'visual'):link.remove(child)
        path=root/('urdf/robot_alfav3_mobile.urdf' if variant=='canonical' else f'urdf/variants/robot_{variant}.urdf')
        ET.indent(m,space='  ');ET.ElementTree(m).write(path,encoding='utf-8',xml_declaration=True)
    for src,dst in [('fixed_body_boxes.yaml','fixed_body_boxes.yaml'),('collision_spheres.preview.yaml','collision_spheres.yaml')]:
        (root/'config/curobo'/dst).write_bytes((source/'config/curobo'/src).read_bytes())
    rev='chassis-head-default-20261125'
    for name,field in [('config/model_variants.yaml','revision'),('config/motion_scope.yaml','model_revision'),('config/poses/home.yaml','model_revision'),('config/poses/stow.yaml','model_revision')]:
        p=root/name;d=yaml.safe_load(p.read_text());d[field]=rev;p.write_text(yaml.safe_dump(d,sort_keys=False,allow_unicode=True))
    p=root/'config/alfav3_mobile_manifest.json';d=json.loads(p.read_text());d.update(revision=rev,link_count=len(model.findall('link')),movable_joint_count=sum(j.get('type')!='fixed' for j in model.findall('joint')),fixed_box_count=3)
    ss=yaml.safe_load((root/'config/curobo/collision_spheres.yaml').read_text())['collision_spheres'];d['sphere_count']=sum(len(x) for x in ss.values())
    d['default_model_source']={'urdf':'preview/chassis_head_20261124/urdf/preview_corrected.urdf','preview_commit':'2a7141b52db75b7c5f8cca43d534cf3d94f29352','wheel_orientation':'zero-pose wheel geometry from approved preview; wheel hub local Z aligned to axle','limitations':'head sphere fit candidate; chest camera has no sphere representation; some source inertia uses old provisional fallback; not full planning/dynamics validation'}
    d['collision_scope']='232 unchanged arm/carriage spheres + 7 candidate head spheres; three approved base_link boxes; chest camera has mesh but no spheres; not a conservative coverage or dynamics certificate'
    d['composition']['head_and_sensors']='approved v3.4.2 mapped TF/meshes from preview; original arm kinematics retained'
    d['composition']['base_collision']='three boxes 001/002/004; 003 removed'
    d['head_collision_fit']={k:v for k,v in d['head_collision_fit'].items() if k=='arm_carriage'}
    d['candidate_head_collision_fit']={'source':'preview/chassis_head_20261124/config/curobo/collision_spheres.preview.yaml','method':'vertex clusters plus 1mm; candidate, not full collision validation'}
    for i in range(1,5):
        d['joint_overrides'][f'steer{i}_joint']='Approved preview mount and orientation, without additional +90deg correction'
    for a in d['artifacts'].values():a['sha256']=hashlib.sha256((root/a['path']).read_bytes()).hexdigest()
    p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
    paths=sorted({m.get('filename').removeprefix('package://robot_description/') for m in model.findall('.//mesh')})
    p=root/'config/mesh_assets.json';p.write_text(json.dumps({'entry':'urdf/robot_alfav3_mobile.urdf','files':paths,'sha256':{x:hashlib.sha256((root/x).read_bytes()).hexdigest() for x in paths}},indent=2)+'\n')

if __name__=='__main__':build()
