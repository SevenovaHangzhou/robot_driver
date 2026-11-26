"""Contracts for the composed robot and its three asset representations."""
import hashlib
from copy import deepcopy
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

def expected_joint_contract(name, original):
    expected = deepcopy(original)
    limits = MANIFEST['arm_module_limits']
    if name in limits['velocity_rad_s']:
        for tag, attributes, _ in expected[2]:
            if tag == 'limit':
                attributes['velocity'] = str(limits['velocity_rad_s'][name])
                attributes['effort'] = format(limits['effort_nm'][name], 'g')
    return expected

def test_canonical_and_projection_trees():
    m=ET.parse(ENTRY).getroot()
    links=[l.get('name') for l in m.findall('link')]
    assert len(links)==49  # +8 ultrasonic frames (2026-11-13), +suspension_link (2026-11-18)
    assert sum(j.get('type')!='fixed' for j in m.findall('joint'))==26
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
    assert all((ROOT/p).is_file() for p in paths)
    for p in paths:assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==manifest['sha256'][p]
    for a in MANIFEST['artifacts'].values():assert hashlib.sha256((ROOT/a['path']).read_bytes()).hexdigest()==a['sha256']

def test_github_arm_joint_semantics_preserved():
    m=ET.parse(ENTRY).getroot()
    assert len(MANIFEST['github_arm_joint_contract'])==14
    for name,original in MANIFEST['github_arm_joint_contract'].items():
        assert structure(m.find(f"joint[@name='{name}']"))==expected_joint_contract(name, original)
    for side in ('left','right'):
        assert m.find(f"joint[@name='{side}_joint1']/parent").get('link')=='arm_carriage'
        bounds=m.find(f"joint[@name='{side}_joint6']/limit")
        assert np.allclose([float(bounds.get(k)) for k in ('lower','upper')],np.deg2rad([-110,110]),rtol=0,atol=1e-12)
    assert m.find("joint[@name='left_joint6']/axis").get('xyz')=='0 0 -1'
    assert m.find("joint[@name='right_joint6']/axis").get('xyz')=='0 0 1'

def test_complete_github_inventory_and_local_joint_contract():
    # The approved sensor/chassis update adds only the chest camera to main's tree.
    m=ET.parse(ENTRY).getroot()
    source=ET.parse(ROOT/'preview/chassis_head_20261124/urdf/preview_corrected.urdf').getroot()
    assert {l.get('name') for l in m.findall('link')}=={l.get('name') for l in source.findall('link')}
    for j in m.findall('joint'):
        original=source.find(f"joint[@name='{j.get('name')}']")
        assert j.get('type')==original.get('type')
        assert j.find('parent').attrib==original.find('parent').attrib
        assert j.find('child').attrib==original.find('child').attrib
        if j.get('name') in {f'wheel{i}_joint' for i in range(1, 5)}:
            for tag in ('limit', 'axis'):
                assert structure(j.find(tag)) == structure(original.find(tag))
            continue
        assert structure(j)==structure(original)


def test_head_sensor_follows_original_dynamic_chain():
    from model_geometry import frames
    m=ET.parse(ENTRY).getroot()
    source=ET.parse(ROOT/'preview/chassis_head_20261124/source/v342_source.urdf').getroot()
    for lift in (0.,-.5,-1.):
        for yaw,pitch in ((0,0),(.3,-.2)):
            a=frames(m,{'updown':lift,'H1_joint':yaw,'H2_joint':pitch})
            b=frames(source,{'updown':lift+1.05,'H1_joint':yaw,'H2_joint':pitch})
            for dest,src in [('H1_Link','H1_Link'),('H2_Link','H2_Link'),('mid360_Link','head_mid360'),('Leye_link','head_camera_left'),('chest_camera_left','chest_camera_left')]:
                np.testing.assert_allclose(np.linalg.inv(a['base_link'])@a[dest],b[src],atol=1e-7,rtol=0)


def test_wheel_joints_retained_with_new_collision_meshes():
    m=ET.parse(ENTRY).getroot()
    f0=frames(m)
    for i in range(1,5):
        for kind in ('steer','wheel'):
            name=f'{kind}{i}'
            link=m.find(f"link[@name='{name}']")
            assert link.find('visual') is not None and link.find('collision') is not None
            assert not np.allclose(frames(m,{name+'_joint':.5})[name],f0[name])


def test_wheel_poses_match_approved_preview_at_all_steering_angles():
    m=ET.parse(ENTRY).getroot()
    approved=ET.parse(ROOT/'preview/chassis_head_20261124/urdf/preview_corrected.urdf').getroot()
    for i in range(1,5):
        for angle in (0.,np.pi/2,-np.pi/2,.7):
            link=f'wheel{i}'
            q={f'steer{i}_joint':angle,'suspension':.1}
            current,expected=frames(m,q),frames(approved,q)
            for tag in ('visual','collision','inertial'):
                path=f"link[@name='{link}']/{tag}/origin"
                np.testing.assert_allclose(current[link] @ transform(m.find(path)),
                                           expected[link] @ transform(approved.find(path)),atol=1e-8)
            axle=current[link][:3,:3] @ np.fromstring(m.find(f"joint[@name='{link}_joint']/axis").get('xyz'),sep=' ')
            np.testing.assert_allclose(axle,[np.sin(angle),np.cos(angle),0],atol=1e-7)
            spun=frames(m,{**q,f'{link}_joint':.8})[link]
            # STL narrow dimension is mesh Y. Its world direction stays on the hub at any spin.
            mesh_y=transform(m.find(f"link[@name='{link}']/visual/origin"))[:3,1]
            assert abs(np.dot(spun[:3,:3] @ mesh_y,axle)) == pytest.approx(1.)


def test_root_lift_and_fixed_radars():
    m=ET.parse(ENTRY).getroot()
    source=ET.parse(ROOT/'preview/chassis_head_20261124/source/v342_source.urdf').getroot()
    reference=frames(source)
    for q in (0.,-.8):
        f=frames(m,{'updown':q})
        np.testing.assert_allclose(f['base_link'][:3,3],[.07478297093,0,.37],atol=1e-12)
        for dest,src in [('lidar_main','front_mid360'),('lidar_rear','behind_mid360'),('imu_link','imu_link')]:
            np.testing.assert_allclose(np.linalg.inv(f['base_link'])@f[dest],reference[src],atol=1e-12)
    j=m.find("joint[@name='updown']")
    assert j.find('child').get('link')=='arm_carriage'
    assert [float(j.find('limit').get(k)) for k in ('lower','upper','velocity')]==[-1.033,.017,.1666]


def test_suspension_lifts_only_the_rear_axle():
    m=ET.parse(ENTRY).getroot()
    f0,f1=frames(m),frames(m,{'suspension':.1})
    for name in ('steer3','steer4','wheel3','wheel4'):
        np.testing.assert_allclose(f1[name][:3,3]-f0[name][:3,3],[0,0,.1],atol=1e-12)
    for name in ('steer1','steer2','base_link','lidar_main','imu_link'):
        np.testing.assert_allclose(f1[name],f0[name],atol=1e-12)
    np.testing.assert_allclose(f0['steer3'][:2,3],[-.305,.305],atol=1e-10)
    np.testing.assert_allclose(f0['steer4'][:2,3],[-.305,-.305],atol=1e-10)


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
            index=int(name[-1])-1
            speed=np.deg2rad([75,75,75,100.2,180,180,180][index])
            effort=[647,647,484,459,204,107,36][index]
            assert np.isclose(float(j.find('limit').get('velocity')),speed)
            assert float(j.find('limit').get('effort'))==effort
            assert np.isclose(MANIFEST['arm_module_limits']['velocity_rad_s'][name],speed)
            assert MANIFEST['arm_module_limits']['effort_nm'][name]==effort
            assert np.isclose(limits[name]['max_acceleration_rad_s2'],np.pi/3)
    for l in m.findall('link'):
        a=l.find('inertial')
        if a is None:continue
        assert float(a.find('mass').get('value'))>0
        i={k:float(v) for k,v in a.find('inertia').attrib.items()}
        matrix=np.array([[i['ixx'],i['ixy'],i['ixz']],[i['ixy'],i['iyy'],i['iyz']],[i['ixz'],i['iyz'],i['izz']]])
        assert np.isfinite(matrix).all() and np.linalg.eigvalsh(matrix).min()>0

def test_regeneration_preserves_documented_module_limits():
    from compose_robot_model import apply_arm_module_limits
    model=ET.parse(ENTRY).getroot()
    original=structure(model)
    for name in MANIFEST['arm_module_limits']['velocity_rad_s']:
        limit=model.find(f"joint[@name='{name}']/limit")
        limit.set('velocity',str(np.pi/6))
        limit.set('effort','1')
    assert apply_arm_module_limits(model)==MANIFEST['arm_module_limits']
    assert structure(model)==original
    cspace=yaml.safe_load((ROOT/VARIANTS['curobo_config']).read_text())['kinematics']['cspace']
    expected=[.4 if name=='updown' else np.pi/3 for name in cspace['joint_names']]
    assert np.allclose(cspace['max_acceleration'],expected,rtol=0,atol=1e-12)

def test_sphere_box_contract_and_references():
    m=ET.parse(ENTRY).getroot();links={l.get('name') for l in m.findall('link')};joints={j.get('name') for j in m.findall('joint')}
    spheres=yaml.safe_load((ROOT/VARIANTS['curobo_spheres']).read_text())['collision_spheres']
    assert set(spheres)<=links and 'base_link' not in spheres
    assert sum(map(len,spheres.values()))==MANIFEST['sphere_count']
    assert len(spheres['arm_carriage'])==30
    assert {'H1_Link','H2_Link','mid360_Link'} <= set(spheres)
    assert 'head' not in spheres and 'head_yaw' not in spheres
    for ss in spheres.values():
        for s in ss:assert np.isfinite(s['center']).all() and len(s['center'])==3 and 0<s['radius']<1
    boxes=yaml.safe_load((ROOT/VARIANTS['fixed_body_boxes']).read_text())
    assert boxes['required'] and boxes['frame']=='base_link' and len(boxes['cuboids'])==3
    for b in boxes['cuboids']:assert np.all(np.array(b['dimensions'])>0)
    cfg=yaml.safe_load((ROOT/VARIANTS['curobo_config']).read_text())['kinematics']
    assert set(cfg['collision_link_names'])==set(spheres)
    assert set(cfg['cspace']['joint_names'])|set(cfg['lock_joints'])=={j.get('name') for j in m.findall('joint') if j.get('type')!='fixed'}
    assert len(cfg['cspace']['joint_names'])==15 and len(cfg['lock_joints'])==11
    assert {'H1_joint','H2_joint','suspension'} <= set(cfg['lock_joints'])
    assert 'mid360_Link' in cfg['tool_frames']
    assert all(n in spheres for pair in cfg['self_collision_ignore'].values() for n in pair)
    for e in ET.parse(ROOT/'srdf/robot_alfav3_mobile.srdf').getroot().iter():
        for attr in ('link1','link2','base_link','tip_link','child_link'):
            if attr in e.attrib:assert e.get(attr) in links
        if e.tag=='joint':assert e.get('name') in joints

def test_approved_fit_matches_published_spheres_and_reports_gaps():
    published=yaml.safe_load((ROOT/VARIANTS['curobo_spheres']).read_text())
    preview=yaml.safe_load((ROOT/'preview/chassis_head_20261124/config/curobo/collision_spheres.preview.yaml').read_text())
    assert published==preview
    original=json.loads((ROOT/'config/curobo/approved_head_carriage_fit.json').read_text())
    assert published['collision_spheres']['arm_carriage']==original['links']['arm_carriage']['spheres']
    assert 'chest_camera_left' not in published['collision_spheres'] # documented gap, not invented geometry


def test_fk_independent_parser():
    yourdfpy=pytest.importorskip('yourdfpy')
    m=ET.parse(ENTRY).getroot();u=yourdfpy.URDF.load(ENTRY,load_meshes=False,build_scene_graph=True)
    rng=np.random.default_rng(117)
    for _ in range(32):
        q={j.name:float(rng.uniform(-np.pi,np.pi) if j.type=='continuous' else rng.uniform(j.limit.lower,j.limit.upper)) for j in u.actuated_joints}
        u.update_cfg(q)
        for name,t in frames(m,q).items():assert np.allclose(t,u.get_transform(name),atol=1e-10),name
