#!/usr/bin/env python3
"""View reproducible arm collision poses or export a self-contained HTML demo."""
import argparse
import base64
import io
import json
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[1]
POSES = [
    ('零位', None, 0),
    ('左 J2：末端 / 底座', 'left_joint2', -90),
    ('右 J2：末端 / 底座', 'right_joint2', 90),
    ('左 J4：末端 / 底座', 'left_joint4', 150),
    ('右 J4：末端 / 底座', 'right_joint4', -150),
    ('左 J6：原限位端点', 'left_joint6', 110),
    ('左 J6：新限位端点', 'left_joint6', 135),
    ('右 J6：原限位端点', 'right_joint6', 110),
    ('右 J6：新限位端点', 'right_joint6', 135),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--html', type=Path, help='Export HTML instead of opening the 3D viewer')
    args = parser.parse_args()
    if args.html:
        os.environ.setdefault('MUJOCO_GL', 'egl')
    import mujoco as mj
    import numpy as np

    model = mj.MjModel.from_xml_path(str(ROOT / 'mujoco/scene.xml'))
    data = mj.MjData(model)
    original = model.geom_rgba.copy()
    option = mj.MjvOption()
    option.geomgroup[0] = 0

    def pose(index):
        title, joint, degrees = POSES[index]
        mj.mj_resetData(model, data)
        if joint:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
            data.qpos[model.jnt_qposadr[jid]] = np.deg2rad(degrees)
        mj.mj_forward(model, data)
        model.geom_rgba[:] = original
        contacts = [c for c in data.contact if c.dist < -1e-5]
        bodies = {int(model.geom_bodyid[g]) for c in contacts for g in c.geom}
        for g in range(model.ngeom):
            if model.geom_group[g] == 1 and int(model.geom_bodyid[g]) in bodies:
                model.geom_rgba[g] = [1, .12, .06, 1]
        depth = max((-c.dist * 1000 for c in contacts), default=0)
        return dict(title=title, joint=joint, degrees=degrees,
                    contacts=len(contacts), depth=round(depth, 3))

    if args.html:
        from PIL import Image
        frames = []
        with mj.Renderer(model, height=600, width=960) as renderer:
            camera = mj.MjvCamera()
            camera.lookat[:] = [.1, 0, 1.05]
            camera.distance = 3.5
            camera.azimuth = 135
            camera.elevation = -18
            for index in range(len(POSES)):
                frame = pose(index)
                renderer.update_scene(data, camera=camera, scene_option=option)
                buffer = io.BytesIO()
                Image.fromarray(renderer.render()).save(buffer, format='JPEG', quality=88)
                frame['image'] = 'data:image/jpeg;base64,' + base64.b64encode(buffer.getvalue()).decode()
                frames.append(frame)
        html = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>双臂限位 · MuJoCo 碰撞演示</title>
<style>body{margin:0;background:#101722;color:#eef3fb;font:16px system-ui}main{max-width:1100px;margin:auto;padding:26px}h1{font-size:28px}p{color:#b8c6d9;line-height:1.6}button{padding:10px 15px;margin:4px;border:1px solid #41516a;border-radius:8px;background:#20304a;color:white;cursor:pointer}button.active{background:#3168b8}img{width:100%;border-radius:12px}#status{padding:16px;background:#1b2739;border-radius:10px;font-size:19px}.bad{color:#ff8c77}.ok{color:#85dfb5}</style>
<main><h1>双臂限位 · MuJoCo 碰撞演示</h1>
<p>MuJoCo 3.15.0 实际渲染。点击姿态或自动播放；碰撞部件标红。其他关节均为零位，升降处于上端点。这里展示的是固定姿态的碰撞检测结果。</p>
<button id="play" onclick="toggle()">▶ 自动播放</button><div id="buttons"></div>
<p id="status"></p><img id="view"><p>J1～J7：±160° / −92°～+122° / ±160° / ±150° / ±160° / ±135° / ±160°。零位及这些示例不能覆盖所有组合姿态。J6 可对比原 ±110° 与新 ±135° 端点。</p></main>
<script>const frames=FRAME_DATA;let index=0,timer=null;
const buttons=document.getElementById('buttons');frames.forEach((f,i)=>{let b=document.createElement('button');b.textContent=f.title;b.onclick=()=>show(i);buttons.appendChild(b)});
function show(i){index=i;let f=frames[i];document.getElementById('view').src=f.image;document.getElementById('status').innerHTML=`<b>${f.title}</b> · ${f.joint?f.joint+' = '+f.degrees+'°':'全部关节为 0'} · <span class="${f.contacts?'bad':'ok'}">${f.contacts?'碰撞接触 '+f.contacts+' 个，最大穿透 '+f.depth+' mm':'无碰撞接触'}</span>`;[...buttons.children].forEach((b,j)=>b.className=i===j?'active':'')}
function toggle(){if(timer){clearInterval(timer);timer=null}else{timer=setInterval(()=>show((index+1)%frames.length),2200)}document.getElementById('play').textContent=timer?'❚❚ 暂停':'▶ 自动播放'}
document.onkeydown=e=>{if(e.key==='ArrowRight')show((index+1)%frames.length);if(e.key==='ArrowLeft')show((index+frames.length-1)%frames.length)};show(0);</script></html>'''
        args.html.parent.mkdir(parents=True, exist_ok=True)
        args.html.write_text(html.replace('FRAME_DATA', json.dumps(frames, ensure_ascii=False)))
        print(args.html.resolve())
        return

    import mujoco.viewer
    index = 0
    pending = []

    def key_callback(key):
        if key in (262, 32, 78):  # Right, Space, N
            pending.append(1)
        elif key in (263, 80):  # Left, P
            pending.append(-1)

    print('鼠标旋转/缩放；右方向键、N 或空格：下一姿态；左方向键或 P：上一姿态。')
    print(pose(index))
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [.1, 0, 1.05]
        viewer.cam.distance = 3.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.opt.geomgroup[0] = 0
        while viewer.is_running():
            with viewer.lock():
                while pending:
                    index = (index + pending.pop(0)) % len(POSES)
                    print(pose(index), flush=True)
            viewer.sync()
            time.sleep(.02)


if __name__ == '__main__':
    main()
