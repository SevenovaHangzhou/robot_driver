#!/usr/bin/env python3
"""Report sampled mesh coverage of the published sphere/box approximation.

Vertices and triangle centroids are checked, not a volumetric proof. Gaps are
reported without changing approved radii, padding, or collision exclusions.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

from model_geometry import frames, resolve_mesh, rotation, transform


def audit(root):
    m=ET.parse(root/'urdf/robot_alfav3_mobile.urdf').getroot()
    fk=frames(m)
    spheres=yaml.safe_load((root/'config/curobo/collision_spheres.yaml').read_text())['collision_spheres']
    boxes=yaml.safe_load((root/'config/curobo/fixed_body_boxes.yaml').read_text())
    result={}
    for l in m.findall('link'):
        name=l.get('name')
        points=[]
        for collision in l.findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:raise ValueError('coverage audit currently expects mesh collision')
            shape=trimesh.load_mesh(resolve_mesh(root,mesh.get('filename')),process=False)
            t=transform(collision.find('origin'))
            sample=np.concatenate([shape.vertices,shape.triangles_center])
            sample*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            points.append(sample@t[:3,:3].T+t[:3,3])
        if not points:continue
        points=np.concatenate(points)
        key=name
        if name.endswith('_tool_link'):
            # The source PR82 represents the attached physical tool on link7.
            key=name.split('_')[0]+'_link7'
            t=np.linalg.inv(fk[key])@fk[name]
            points=points@t[:3,:3].T+t[:3,3]
        if key in spheres:
            ss=spheres[key]
            centers=np.array([s['center'] for s in ss]);radii=np.array([s['radius'] for s in ss])
            distance=np.min(np.linalg.norm(points[:,None,:]-centers[None,:,:],axis=-1)-radii,axis=1)
            method='sphere_union_surface_sample'
        elif key=='base_link':
            distances=[]
            for b in boxes['cuboids']:
                local=(points-np.array(b['center']))@rotation(b['rpy_rad'])
                q=np.abs(local)-np.array(b['dimensions'])/2
                distances.append(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0))
            distance=np.min(distances,axis=0);method='four_box_union_surface_sample'
        else:
            result[name]={'status':'missing_collision_representation','samples':len(points)}
            continue
        result[name]={'method':method,'samples':len(points),
            'covered_fraction':float(np.mean(distance<=1e-6)),
            'maximum_sampled_gap_m':float(max(0,distance.max()))}
    return {'status':'measured_not_conservative_certificate','links':result,
            'method':'all mesh vertices and face centroids; no radius inflation; local coverage invariant under each rigid link FK',
            'maximum_sampled_gap_m':max(v.get('maximum_sampled_gap_m',0) for v in result.values())}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--output',type=Path)
    a=p.parse_args();result=audit(a.root.resolve())
    text=json.dumps(result,indent=2)+'\n';print(text)
    if a.output:a.output.write_text(text)
