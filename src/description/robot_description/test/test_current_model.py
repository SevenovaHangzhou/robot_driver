"""Contracts for the composed robot and its three asset representations."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from model_geometry import frames, resolve_mesh, rotation, transform
ENTRY=ROOT/'urdf/robot_alfav3_mobile.urdf'
VARIANTS=yaml.safe_load((ROOT/'config/model_variants.yaml').read_text())
MANIFEST=json.loads((ROOT/'config/alfav3_mobile_manifest.json').read_text())

def structure(e):
    return [e.tag,dict(sorted(e.attrib.items())),[structure(c) for c in e]]

def test_canonical_and_projection_trees():
    m=ET.parse(ENTRY).getroot()
    links=[l.get('name') for l in m.findall('link')]
    assert len(links)==47  # +8 ultrasonic transducer frames (user 2026-11-13)
    assert sum(j.get('type')!='fixed' for j in m.findall('joint'))==25
    for key in ('canonical_urdf','visual_urdf','collision_urdf'):
        p=ROOT/VARIANTS[key]
        result=subprocess.run(['check_urdf',str(p)],capture_output=True,text=True)
        assert result.returncode==0,result.stdout+result.stderr
        v=ET.parse(p).getroot()
        assert [structure(j) for j in v.findall('joint')]==[structure(j) for j in m.findall('joint')]
        assert [l.get('name') for l in v.findall('link')]==links
        assert set(frames(v))==set(links)
        if key=='visual_urdf':assert v.findall('.//visual') and not v.findall('.//collision')
        if key=='collision_urdf':assert v.findall('.//collision') and not v.findall('.//visual')
    assert set((ROOT/'urdf').rglob('*.urdf'))=={ROOT/VARIANTS[k] for k in ('canonical_urdf','visual_urdf','collision_urdf')}

def test_assets_portable_complete_and_hashed():
    manifest=json.loads((ROOT/'config/mesh_assets.json').read_text())
    paths=set()
    for key in ('canonical_urdf','visual_urdf','collision_urdf'):
        for mesh in ET.parse(ROOT/VARIANTS[key]).findall('.//mesh'):
            paths.add(str(resolve_mesh(ROOT,mesh.get('filename')).relative_to(ROOT)))
    assert paths==set(manifest['files'])
    assert paths=={str(p.relative_to(ROOT)) for p in (ROOT/'mesh').rglob('*') if p.is_file()}
    for p in paths:assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==manifest['sha256'][p]
    for a in MANIFEST['artifacts'].values():assert hashlib.sha256((ROOT/a['path']).read_bytes()).hexdigest()==a['sha256']

def test_github_arm_joint_semantics_preserved():
    m=ET.parse(ENTRY).getroot()
    assert len(MANIFEST['github_arm_joint_contract'])==14
    for name,original in MANIFEST['github_arm_joint_contract'].items():
        assert structure(m.find(f"joint[@name='{name}']"))==original
    for side in ('left','right'):
        assert m.find(f"joint[@name='{side}_joint1']/parent").get('link')=='arm_carriage'
        bounds=m.find(f"joint[@name='{side}_joint6']/limit")
        assert np.allclose([float(bounds.get(k)) for k in ('lower','upper')],np.deg2rad([-110,110]),rtol=0,atol=1e-12)
    assert m.find("joint[@name='left_joint6']/axis").get('xyz')=='0 0 -1'
    assert m.find("joint[@name='right_joint6']/axis").get('xyz')=='0 0 1'

def test_complete_github_inventory_and_local_joint_contract():
    m=ET.parse(ENTRY).getroot()
    upstream=MANIFEST['github_full_joint_contract']
    current={j.get('name'):j for j in m.findall('joint')}
    assert len(upstream)==36
    added_links={'lidar_main','lidar_rear'}|{f'ultrasonic_channel_{i}_link' for i in range(1,9)}
    added_joints={'lidar_main_fixed','lidar_rear_fixed'}|{f'ultrasonic_channel_{i}_joint' for i in range(1,9)}
    assert set(current)==set(upstream)|added_joints
    assert {l.get('name') for l in m.findall('link')}==set(MANIFEST['github_link_names'])|added_links
    assert set(MANIFEST['joint_overrides'])=={'base_footprint_to_base_link','updown','left_tool0_fixed','right_tool0_fixed'}
    for name,original in upstream.items():
        j=current[name]
        # No joint may disappear or move to a different parent, including
        # joints with an explicitly overridden local origin.
        children={c[0]:c[1] for c in original[2]}
        assert j.get('type')==original[1]['type']
        assert j.find('parent').attrib==children['parent']
        assert j.find('child').attrib==children['child']
        if name not in MANIFEST['joint_overrides']:
            assert structure(j)==original, name
    assert 'head_joint' not in current and 'head_pitch_joint' not in current

def test_head_sensor_follows_original_dynamic_chain():
    m=ET.parse(ENTRY).getroot()
    expected={
        'H1_joint':('arm_carriage','H1_Link',[.267,0,.36515],[np.pi,0,0]),
        'H2_joint':('H1_Link','H2_Link',[-.02,0,.24625009252407],[np.pi/2,0,0]),
        'mid360_joint':('H2_Link','mid360_Link',[.05857,.066019,0],[-1.5708,0,-.37885]),
        'Leye_joint':('H2_Link','Leye_link',[.032866,.11171,-.06],[-1.5708,0,0]),
        'Reye_joint':('H2_Link','Reye_Link',[.032866,.11171,.06],[-1.5708,0,0]),
    }
    for name,(parent,child,xyz,rpy) in expected.items():
        j=m.find(f"joint[@name='{name}']")
        assert j.find('parent').get('link')==parent
        assert j.find('child').get('link')==child
        t=transform(j.find('origin'))
        assert np.allclose(t[:3,3],xyz,rtol=0,atol=1e-12)
        assert np.allclose(t[:3,:3],rotation(rpy),rtol=0,atol=1e-12)
    poses=[]
    for q in ({},{'H1_joint':.4,'H2_joint':-.3},{'updown':-.4,'H1_joint':-.4,'H2_joint':.3}):
        f=frames(m,q);poses.append(f['mid360_Link'])
        for name in ('mid360_joint','Leye_joint','Reye_joint'):
            parent,child,xyz,rpy=expected[name]
            relative=np.linalg.inv(f[parent])@f[child]
            assert np.allclose(relative,transform(m.find(f"joint[@name='{name}']/origin")),atol=1e-12)
    assert not np.allclose(poses[0],poses[1])
    assert not np.allclose(poses[1],poses[2])

def test_wheel_joints_retained_without_individual_collision():
    m=ET.parse(ENTRY).getroot()
    f0=frames(m)
    for i in range(1,5):
        for kind in ('steer','wheel'):
            name=f'{kind}{i}'
            link=m.find(f"link[@name='{name}']")
            assert link.find('visual') is not None and link.find('inertial') is not None
            assert not link.findall('collision')
            assert not np.allclose(frames(m,{name+'_joint':.5})[name],f0[name])

def test_root_lift_and_fixed_radars():
    m=ET.parse(ENTRY).getroot()
    for q in ({},{'updown':-.8,'H1_joint':.3,'H2_joint':-.2}):
        f=frames(m,q)
        assert np.allclose(f['base_link'][:3,3],[0,0,.335],atol=1e-12)
        assert np.allclose(f['base_link'][:3,:3],np.eye(3),atol=1e-12)
        for name,xyz,rpy in [('lidar_main',[.453313447977869,0,.398099404],[0,15,0]),('lidar_rear',[-.047,0,1.959],[0,-90,0])]:
            assert np.allclose(f[name][:3,3],xyz,atol=1e-12)
            assert np.allclose(f[name][:3,:3],rotation(np.deg2rad(rpy)),atol=1e-12)
            assert m.find(f"joint[@name='{name}_fixed']/parent").get('link')=='base_link'
            assert len(m.find(f"link[@name='{name}']"))==0
    j=m.find("joint[@name='updown']")
    assert j.find('axis').get('xyz')=='0 0 -1'
    assert np.allclose(transform(j.find('origin'))[:3,3],[.174,0,1.68])

def test_tcp_distance_and_axes():
    m=ET.parse(ENTRY).getroot()
    for q in ({},{'left_joint5':.6,'left_joint6':-.8,'left_joint7':.3,'right_joint5':-.6,'right_joint6':.8,'right_joint7':-.3}):
        f=frames(m,q)
        for side in ('left','right'):
            axis=f[side+'_link6'][:3,:3]@np.fromstring(m.find(f"joint[@name='{side}_joint6']/axis").get('xyz'),sep=' ')
            delta=f[side+'_tool0'][:3,3]-f[side+'_link6'][:3,3]
            assert np.isclose(np.linalg.norm(np.cross(axis,delta)),.2342,atol=1e-10)
            t=np.linalg.inv(f[side+'_link7'])@f[side+'_tool0']
            assert np.allclose(t[:3,3],[0,0,.2342],atol=1e-12)
            assert np.allclose(t[:3,:3],rotation([0,0,-np.pi/2]),atol=1e-12)

def test_ft_sensor_parent_and_mount():
    m=ET.parse(ENTRY).getroot()
    for side,sign in [('left',-1),('right',1)]:
        l=m.find(f"link[@name='{side}_ft_sensor_link']")
        assert l is not None and len(l)==0
        j=m.find(f"joint[@name='{side}_ft_sensor_fixed']")
        assert j.find('parent').get('link')==side+'_tool_link'
        t=transform(j.find('origin'))
        assert np.allclose(t[:3,3],[0,sign*.0072,0])
        assert np.allclose(t[:3,:3],rotation([sign*np.pi/2,0,0]))

def test_limits_and_inertials():
    m=ET.parse(ENTRY).getroot()
    limits=yaml.safe_load((ROOT/'config/joint_limits_alfav3_mobile.yaml').read_text())['joints']
    js={j.get('name'):j for j in m.findall('joint') if j.get('type')!='fixed'}
    assert set(limits)==set(js)
    for name,j in js.items():
        u='m' if j.get('type')=='prismatic' else 'rad'
        pairs=[('max_velocity_'+u+'_s','velocity'),('max_effort','effort')]
        if j.get('type')!='continuous':
            pairs += [('lower_position_'+u,'lower'),('upper_position_'+u,'upper')]
        else:
            assert 'lower_position_'+u not in limits[name] and 'upper_position_'+u not in limits[name]
        for field,attr in pairs:
            assert np.isclose(limits[name][field],float(j.find('limit').get(attr)))
        if name.startswith(('left_joint','right_joint')):
            assert np.isclose(float(j.find('limit').get('velocity')),np.pi/6)
            assert np.isclose(limits[name]['max_acceleration_rad_s2'],np.pi/3)
    for l in m.findall('link'):
        a=l.find('inertial')
        if a is None:continue
        assert float(a.find('mass').get('value'))>0
        i={k:float(v) for k,v in a.find('inertia').attrib.items()}
        matrix=np.array([[i['ixx'],i['ixy'],i['ixz']],[i['ixy'],i['iyy'],i['iyz']],[i['ixz'],i['iyz'],i['izz']]])
        assert np.isfinite(matrix).all() and np.linalg.eigvalsh(matrix).min()>0

def test_sphere_box_contract_and_references():
    m=ET.parse(ENTRY).getroot();links={l.get('name') for l in m.findall('link')};joints={j.get('name') for j in m.findall('joint')}
    spheres=yaml.safe_load((ROOT/VARIANTS['curobo_spheres']).read_text())['collision_spheres']
    assert set(spheres)<=links and 'base_link' not in spheres
    assert sum(map(len,spheres.values()))==MANIFEST['sphere_count']
    assert len(spheres['arm_carriage'])==20
    assert {'H1_Link','H2_Link','mid360_Link'} <= set(spheres)
    assert 'head' not in spheres and 'head_yaw' not in spheres
    for ss in spheres.values():
        for s in ss:assert np.isfinite(s['center']).all() and len(s['center'])==3 and 0<s['radius']<1
    boxes=yaml.safe_load((ROOT/VARIANTS['fixed_body_boxes']).read_text())
    assert boxes['required'] and boxes['frame']=='base_link' and len(boxes['cuboids'])==4
    for b in boxes['cuboids']:assert np.all(np.array(b['dimensions'])>0)
    cfg=yaml.safe_load((ROOT/VARIANTS['curobo_config']).read_text())['kinematics']
    assert set(cfg['collision_link_names'])==set(spheres)
    assert set(cfg['cspace']['joint_names'])|set(cfg['lock_joints'])=={j.get('name') for j in m.findall('joint') if j.get('type')!='fixed'}
    assert len(cfg['cspace']['joint_names'])==15 and len(cfg['lock_joints'])==10
    assert {'H1_joint','H2_joint'} <= set(cfg['lock_joints'])
    assert 'mid360_Link' in cfg['tool_frames']
    assert all(n in spheres for pair in cfg['self_collision_ignore'].values() for n in pair)
    for e in ET.parse(ROOT/'srdf/robot_alfav3_mobile.srdf').getroot().iter():
        for attr in ('link1','link2','base_link','tip_link','child_link'):
            if attr in e.attrib:assert e.get(attr) in links
        if e.tag=='joint':assert e.get('name') in joints

def test_new_head_spheres_cover_mesh_surface_samples():
    pytest.importorskip('trimesh')
    from audit_collision_coverage import audit
    report=audit(ROOT)
    for name in ('H1_Link','H2_Link','mid360_Link'):
        assert name in report['links']
        assert report['links'][name]['covered_fraction']==1.0
        assert report['links'][name]['maximum_sampled_gap_m']==0.0
        fit=MANIFEST['head_collision_fit'][name]
        assert fit['maximum_assigned_triangle_vertex_gap_m'] <= -0.0004999
    # Eye sight-range meshes are visualization only, not physical obstacles.
    for name in ('Leye_link','Reye_Link'):
        link=ET.parse(ENTRY).getroot().find(f"link[@name='{name}']")
        assert link.find('collision') is None and link.find('inertial') is None
        assert name not in report['links']

def test_fk_independent_parser():
    yourdfpy=pytest.importorskip('yourdfpy')
    m=ET.parse(ENTRY).getroot();u=yourdfpy.URDF.load(ENTRY,load_meshes=False,build_scene_graph=True)
    rng=np.random.default_rng(117)
    for _ in range(32):
        q={j.name:float(rng.uniform(-np.pi,np.pi) if j.type=='continuous' else rng.uniform(j.limit.lower,j.limit.upper)) for j in u.actuated_joints}
        u.update_cfg(q)
        for name,t in frames(m,q).items():assert np.allclose(t,u.get_transform(name),atol=1e-10),name
