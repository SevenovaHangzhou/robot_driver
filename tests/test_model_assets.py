from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from gripper_jog.model import Chain
from gripper_jog.model_assets import ModelAssets
from gripper_jog.errors import JogError


def make_assets(tmp_path, xml, uri='package://robot_description/meshes/tool.STL'):
    (tmp_path/'urdf').mkdir()
    (tmp_path/'meshes').mkdir()
    (tmp_path/'meshes/tool.STL').write_bytes(b'offline fixture mesh')
    (tmp_path/'package.xml').write_text('<package><name>robot_description</name></package>')
    xml = xml.replace('<link name="l7"/>', f'<link name="l7"><visual><geometry><mesh filename="{uri}"/></geometry></visual></link>')
    source = tmp_path/'urdf/robot.urdf'
    source.write_text(xml)
    return source, Chain(xml, 'base', 'l7')


def test_display_shares_planning_identity_and_exact_tcp_frames(tmp_path, xml):
    source, chain = make_assets(tmp_path, xml)
    assets = ModelAssets(source)
    info = assets.info(chain)
    assert info['model_hash'] == chain.model_hash
    assert info['frame'] == chain.base and info['tip'] == chain.tip
    root = ET.fromstring(info['urdf'])
    assert root.find('link[@name="l7"]/visual/geometry/mesh').get('filename') == '/mesh/robot_description/meshes/tool.STL'
    assert assets.meshes['robot_description/meshes/tool.STL'].read_bytes() == b'offline fixture mesh'
    source.write_text(source.read_text().replace('analytic_fixture','changed_fixture'))
    with pytest.raises(JogError) as e:
        ModelAssets(source).info(chain)
    assert e.value.code == 'model_mismatch'


def test_complete_robot_keeps_visuals_outside_selected_arm(tmp_path, xml):
    xml = xml.replace('<link name="base"/>', '<link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>')
    xml = xml.replace('</robot>', '<link name="other_arm"><visual><geometry><sphere radius=".1"/></geometry></visual></link><joint name="other_fixed" type="fixed"><parent link="base"/><child link="other_arm"/></joint></robot>')
    source, chain = make_assets(tmp_path, xml)
    root = ET.fromstring(ModelAssets(source).info(chain)['urdf'])
    assert root.find('link[@name="base"]/visual') is not None
    assert root.find('link[@name="other_arm"]/visual') is not None


@pytest.mark.parametrize('uri', ['https://example.com/model.stl', 'package://unknown/tool.STL',
    'package://robot_description/../outside.STL', 'package://robot_description/meshes/missing.STL'])
def test_assets_fail_closed_on_missing_or_foreign_paths(tmp_path, xml, uri):
    source, _ = make_assets(tmp_path, xml, uri)
    with pytest.raises(JogError): ModelAssets(source)


def test_portable_relative_mesh_stays_inside_explicit_share(tmp_path, xml):
    source, chain = make_assets(tmp_path, xml, '../meshes/tool.STL')
    with pytest.raises(JogError):
        ModelAssets(source)
    assets = ModelAssets(source, tmp_path, allow_relative=True)
    assert '/mesh/robot_description/meshes/tool.STL' in assets.info(chain)['urdf']
    source.write_text(source.read_text().replace('../meshes/tool.STL', '../../outside.STL'))
    with pytest.raises(JogError):
        ModelAssets(source, tmp_path, allow_relative=True)
