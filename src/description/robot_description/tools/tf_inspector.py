"""TF inspector UI: complete link inventory, detail panel and screen-side labels.

Label placement is client-camera dependent. It never changes the underlying FK.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation


class TfInspector:
    def __init__(self, server, robot, mode, roots, tf_roots, titles, markers, opacity):
        self.server, self.robot, self.mode = server, robot, mode
        self.roots, self.tf_roots, self.titles, self.markers = roots, tf_roots, titles, markers
        self.clients = {}
        self.poses = {}
        self.parents = {j.child:j for j in robot.joint_map.values()}
        self.order = []
        def visit(link,depth):
            self.order.append((link,depth))
            for j in robot.joint_map.values():
                if j.parent==link:visit(j.child,depth+1)
        visit(robot.base_link,0)
        self.enabled = server.gui.add_checkbox('显示TF坐标轴', initial_value=True)
        self.names = server.gui.add_checkbox('显示分列名称与引线', initial_value=True)
        self.scale = server.gui.add_slider('坐标轴长度（m）', min=.03,max=.25,step=.01,initial_value=.10)
        self.selected = server.gui.add_dropdown('选择坐标系', options=[n for n,_ in self.order],initial_value='mid360_Link')
        self.focus = server.gui.add_button('聚焦所选TF')
        self.detail = server.gui.add_markdown('')
        self.search = server.gui.add_text('筛选TF名称 / 关节', initial_value='')
        self.checks = {}
        with server.gui.add_folder(f'完整TF树 · {len(self.order)}项（逐项显隐）',expand_by_default=False):
            self.show_all=server.gui.add_button('全部显示')
            self.hide_all=server.gui.add_button('全部隐藏')
            for name,depth in self.order:
                joint=self.parents.get(name)
                hint='根坐标系' if joint is None else f'{joint.parent} → {name} / {joint.name} ({joint.type})'
                self.checks[name]=server.gui.add_checkbox('  '*depth+name,initial_value=True,hint=hint)
                self.checks[name].on_update(lambda _:self.refresh())
        for control in (self.names,self.enabled,self.search):
            control.on_update(lambda _:self.refresh())
        self.selected.on_update(lambda _:self.refresh())
        self.focus.on_click(lambda _:self.focus_selected())
        self.show_all.on_click(lambda _:self.toggle_all(True))
        self.hide_all.on_click(lambda _:self.toggle_all(False))
        self.scale.on_update(lambda _:self.refresh())
        self.mode.on_update(lambda _:self.change_mode())
        self.opacity=opacity
        server.on_client_connect(self.connect)
        server.on_client_disconnect(lambda client:self.clients.pop(client.client_id,None))

    def toggle_all(self,value):
        for c in self.checks.values():c.value=value
        self.refresh()

    def visible(self,name):
        term=self.search.value.strip().lower()
        j=self.parents.get(name)
        return self.checks[name].value and (not term or term in name.lower() or (j and term in j.name.lower()))

    def change_mode(self):
        single=self.mode.value=='TF单机'
        for key,root in self.roots.items():
            root.visible=(key=='visual') if single else True
            root.position=(0,0,0) if single else (0,{'visual':-3.6,'collision':0,'spheres':3.6}[key],0)
        for title in self.titles:title.visible=not single
        for marker in self.markers.values():marker.visible=not single
        self.refresh()

    def update(self,poses):
        self.poses=poses
        self.refresh()

    def refresh(self):
        single=self.mode.value=='TF单机'
        # Global frame axes stay on the real geometry. Labels alone are displaced.
        for idx,frames in enumerate(self.tf_roots):
            active=single and idx==0 or not single and idx==1
            for name,frame in frames.items():
                frame.visible=bool(active and self.enabled.value and self.visible(name))
                frame.axes_length=self.scale.value*(1.5 if name==self.selected.value else 1)
        if self.poses:
            n=self.selected.value;j=self.parents.get(n);t=self.poses[n]
            local=np.eye(4) if j is None else j.origin
            angles=Rotation.from_matrix(local[:3,:3]).as_euler('xyz',degrees=True)
            def fmt(v):return ', '.join(f'{float(x):.6f}' for x in v)
            self.detail.content=(f'**{n}**\n\n父：`{j.parent if j else "— 根"}` · 关节：`{j.name if j else "—"}`'
                f'\n\n局部安装 XYZ (m)：`{fmt(local[:3,3])}`'
                f'\n\n局部安装 RPY (°)：`{fmt(angles)}`'
                f'\n\n当前 footprint XYZ (m)：`{fmt(t[:3,3])}`'
                '\n\n红X · 绿Y · 蓝Z；局部安装为URDF origin，不含当前关节运动。')
        for client,data in list(self.clients.values()):
            self.layout(client,data)

    def connect(self,client):
        data={'labels':{},'last':0.,'busy':False}
        for n,_ in self.order:
            label=client.scene.add_label('/tf_names/'+n,n,visible=False,
                font_size_mode='screen',font_screen_scale=1.,depth_test=False,anchor='center-left')
            data['labels'][n]=label
        self.clients[client.client_id]=(client,data)
        @client.camera.on_update
        def changed(_):
            now=time.monotonic()
            if now-data['last']<.04:return
            data['last']=now
            self.layout(client,data)
        self.layout(client,data)

    def layout(self,client,data):
        if data['busy']:return
        data['busy']=True
        try:
            active=[n for n,_ in self.order if self.enabled.value and self.names.value and self.visible(n) and n in self.poses]
            for n,h in data['labels'].items():h.visible=n in active
            if not active:
                client.scene.add_line_segments('/tf_name_lines',points=np.zeros((0,2,3)),colors=(120,130,150),thickness=.8,thickness_units='screen')
                return
            camera=client.camera
            q=camera.wxyz;basis=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
            cam=np.array(camera.position);look=np.array(camera.look_at)
            # Viser cameras use OpenCV axes: +Z forward, +X right, -Y up.
            depth=max(.3,float(np.dot(look-cam,basis[:,2])))
            h=depth*np.tan(camera.fov/2);w=h*camera.aspect
            coords={n:(self.poses[n][:3,3]-cam)@basis for n in active}
            ranked=sorted(active,key=lambda n:coords[n][0]/max(.01,coords[n][2]))
            split=(len(ranked)+1)//2
            lines=[]
            for sign,group in [(-1,ranked[:split]),(1,ranked[split:])]:
                group.sort(key=lambda n:-coords[n][1]/max(.01,coords[n][2]),reverse=True)
                # Fixed screen-space rows: even coincident frames have distinct labels.
                ys=np.linspace(.80,-.83,len(group)) if len(group)>1 else [0.]
                for n,y in zip(group,ys):
                    pos=cam+depth*basis[:,2]+sign*.94*w*basis[:,0]-y*h*basis[:,1]
                    label=data['labels'][n];label.position=pos
                    label.anchor='center-left' if sign<0 else 'center-right'
                    label.text=('▶ ' if n==self.selected.value else '')+n
                    lines.append([self.poses[n][:3,3],pos])
            client.scene.add_line_segments('/tf_name_lines',points=np.array(lines),colors=(125,135,155),thickness=.8,thickness_units='screen')
        except (AttributeError,RuntimeError):
            # A connecting/disconnecting client may not have its camera yet.
            pass
        finally:data['busy']=False

    def focus_selected(self):
        if self.selected.value not in self.poses:return
        p=self.poses[self.selected.value][:3,3]
        self.checks[self.selected.value].value=True
        for client,_ in self.clients.values():
            client.camera.position=p+np.array([1.3,-.4,.5])
            client.camera.look_at=p
            client.camera.up_direction=(0,0,1)
