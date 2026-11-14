#!/usr/bin/env python3
"""Preview three representations from one robot revision (no motion commands)."""
import argparse
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import viser
import yaml
import yourdfpy
from viser.extras import ViserUrdf
from tf_inspector import TfInspector


def quaternion(matrix):
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return (q[3], *q[:3])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--port', type=int, default=8291)
    args = parser.parse_args()
    root = args.root.resolve()
    config = yaml.safe_load((root / 'config/model_variants.yaml').read_text())
    entry = root / config['canonical_urdf']
    robot = yourdfpy.URDF.load(entry, load_meshes=False, build_scene_graph=True)
    server = viser.ViserServer(host='127.0.0.1', port=args.port)
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/ground', width=10, height=15, plane='xy')
    placements = {'visual': -3.6, 'collision': 0.0, 'spheres': 3.6}
    views = []
    roots = {}
    titles = []
    tf_roots = []
    tf_handles = []
    for name, offset in placements.items():
        roots[name]=server.scene.add_frame('/'+name, position=(0, offset, 0), show_axes=False)
        titles.append(server.scene.add_label('/'+name+'/title', {
            'visual': '外观 / Visual', 'collision': '简化碰撞 / Collision',
            'spheres': 'cuRobo / 球＋盒'}[name], position=(0, 0, 2.8)))
        if name != 'spheres':
            path = root / config[name+'_urdf']
            model = yourdfpy.URDF.load(path, load_meshes=name=='visual',
                load_collision_meshes=name=='collision', build_scene_graph=name=='visual',
                build_collision_scene_graph=name=='collision',
                filename_handler=lambda fname: str(root / fname.removeprefix('package://robot_description/')))
            views.append(ViserUrdf(server, model, root_node_name='/'+name+'/model',
                load_meshes=name=='visual', load_collision_meshes=name=='collision',
                mesh_color_override=(.72,.75,.82,.85),
                collision_mesh_color_override=(.15,.55,.9,.85)))
        tf_roots.append(server.scene.add_frame('/'+name+'/tf', show_axes=False))
        handles = {}
        for link in robot.link_map:
            handles[link] = server.scene.add_frame('/'+name+'/tf/'+link,
                axes_length=.08, axes_radius=.003, show_axes=True)
        tf_handles.append(handles)

    spheres = yaml.safe_load((root / config['curobo_spheres']).read_text())['collision_spheres']
    sphere_frames = {}
    count = 0
    for link, values in spheres.items():
        sphere_frames[link] = server.scene.add_frame('/spheres/model/'+link, show_axes=False)
        for i, sphere in enumerate(values):
            if sphere['radius'] <= 0:
                continue
            server.scene.add_icosphere(f'/spheres/model/{link}/{i}',
                radius=sphere['radius'], position=tuple(sphere['center']),
                color=(50,180,110), subdivisions=2)
            count += 1
    boxes = yaml.safe_load((root / config['fixed_body_boxes']).read_text())
    box_frame = server.scene.add_frame('/spheres/boxes', show_axes=False)
    for box in boxes['cuboids']:
        q = Rotation.from_euler('xyz', box['rpy_rad']).as_quat()
        server.scene.add_box('/spheres/boxes/'+box['name'],
            dimensions=tuple(box['dimensions']), position=tuple(box['center']),
            wxyz=(q[3],*q[:3]), color=(235,155,45), opacity=.9)

    server.gui.add_markdown('## 同一模型，三种表示\n版本：`'+config['revision']+'`\n\n'
        f'同一URDF运动链；{count}个有效球＋{len(boxes["cuboids"])}盒。'
        '仅左右平移摆放，无旋转/高度补偿。基础几何，无负载、无规划膨胀。'
        '\n\n三个雷达：lidar_main、lidar_rear、头部mid360_Link。头部保留GitHub父子链。'
        '\n\n原臂/底盘球盒仍有约55mm最大采样缺口；新头部球根据其网格重新生成。前后雷达仅为坐标标记，头部雷达有实际网格。')
    mode=server.gui.add_dropdown('显示模式',options=['TF单机','三栏对照'],initial_value='TF单机')
    opacity = server.gui.add_slider('外观/碰撞不透明度', min=0., max=1., step=.05, initial_value=.25)
    sliders = {}
    markers = {}
    for name in placements:
        for link, color in [('lidar_main',(240,60,170)), ('lidar_rear',(20,210,220)),
                            ('mid360_Link',(250,190,30))]:
            if link not in robot.link_map:
                continue
            frame = server.scene.add_frame(f'/{name}/sensors/{link}', axes_length=.12, axes_radius=.004)
            server.scene.add_icosphere(f'/{name}/sensors/{link}/origin', radius=.018, color=color, subdivisions=2)
            server.scene.add_label(f'/{name}/sensors/{link}/name', link, position=(0,0,.07))
            markers[(name,link)] = frame

    inspector=TfInspector(server,robot,mode,roots,tf_handles,titles,markers,opacity)

    def update():
        q = {name: slider.value for name, slider in sliders.items()}
        robot.update_cfg(q)
        for view in views:
            view.update_cfg(q)
        poses = {name: robot.get_transform(name) for name in robot.link_map}
        for handles in tf_handles:
            for link, handle in handles.items():
                handle.position = poses[link][:3,3]
                handle.wxyz = quaternion(poses[link])
        for link, handle in sphere_frames.items():
            handle.position = poses[link][:3,3]
            handle.wxyz = quaternion(poses[link])
        box_frame.position = poses[boxes['frame']][:3,3]
        box_frame.wxyz = quaternion(poses[boxes['frame']])
        for (_,link), handle in markers.items():
            handle.position = poses[link][:3,3]
            handle.wxyz = quaternion(poses[link])
        inspector.update(poses)

    @opacity.on_update
    def change_opacity(_):
        for view in views:
            for mesh in view._meshes:
                mesh.opacity = opacity.value

    with server.gui.add_folder('关节（rad / m）', expand_by_default=False):
        for joint in robot.actuated_joints:
            lower, upper = (-np.pi, np.pi) if joint.type=='continuous' else (joint.limit.lower, joint.limit.upper)
            sliders[joint.name] = server.gui.add_slider(joint.name, min=float(lower), max=float(upper),
                step=.001, initial_value=float(np.clip(0, lower, upper)))
            sliders[joint.name].on_update(lambda _: update())
    reset = server.gui.add_button('全部归零')
    fit_button = server.gui.add_button('当前模式全部入镜')
    head_button = server.gui.add_button('聚焦头部雷达（中间模型）')

    @reset.on_click
    def zero(_):
        for slider in sliders.values():
            slider.value = float(np.clip(0,slider.min,slider.max))
        update()

    def camera(client):
        # Fit all three models even in the narrow in-app browser side panel.
        fov = float(np.deg2rad(50))
        aspect = max(float(client.camera.aspect), .3)
        width=5.6 if mode.value=='TF单机' else 11.8
        distance = max(4., width/(2*np.tan(fov/2)*aspect))
        client.camera.position = (distance,0,1.25+distance*.15)
        client.camera.look_at = (0,0,1.25)
        client.camera.up_direction = (0,0,1)
        client.camera.fov = fov

    @fit_button.on_click
    def fit_all(_):
        for client in server.get_clients().values():
            camera(client)

    @head_button.on_click
    def focus_head(_):
        position=robot.get_transform('mid360_Link')[:3,3]
        for client in server.get_clients().values():
            client.camera.position=position+np.array([1.3,-.6,.55])
            client.camera.look_at=position
            client.camera.up_direction=(0,0,1)

    @server.on_client_connect
    def on_connect(client):
        camera(client)

    @mode.on_update
    def mode_camera(_):
        for client in server.get_clients().values():camera(client)

    update()
    inspector.change_mode()
    change_opacity(None)
    print('READY http://127.0.0.1:'+str(args.port), flush=True)
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
