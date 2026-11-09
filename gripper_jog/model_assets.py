"""Read-only visual assets from the exact planning URDF; no model copies."""
from pathlib import Path, PurePosixPath
from urllib.parse import quote
import xml.etree.ElementTree as ET

from .errors import JogError


class ModelAssets:
    def __init__(self, urdf, share=None, *, allow_relative=False):
        source = Path(urdf).resolve()
        self.xml = source.read_text()
        self.share = Path(share).resolve() if share else source.parent.parent
        manifest = self.share / 'package.xml'
        self.package = ET.fromstring(manifest.read_text()).findtext('name') if manifest.is_file() else 'robot_description'
        root = ET.fromstring(self.xml)
        self.meshes = {}
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                link.remove(collision)
            for mesh in link.findall('visual/geometry/mesh'):
                name = mesh.get('filename', '')
                prefix = f'package://{self.package}/'
                if name.startswith(prefix):
                    relative = name[len(prefix):]
                    if '..' in PurePosixPath(relative).parts:
                        raise JogError('model_asset_invalid', '模型 mesh 路径越界')
                    path = self.share.joinpath(relative).resolve()
                elif allow_relative and not url_has_scheme(name) and not Path(name).is_absolute():
                    path = (source.parent / name).resolve()
                    if not path.is_relative_to(self.share):
                        raise JogError('model_asset_invalid', '模型 mesh 路径越界')
                    relative = path.relative_to(self.share).as_posix()
                else:
                    raise JogError('model_asset_invalid', '模型包含未配置的 mesh package 或外部 URL')
                key = self.package + '/' + relative
                if '..' in PurePosixPath(relative).parts or path.suffix.lower() not in {'.stl', '.dae'} or not path.is_relative_to(self.share) or not path.is_file():
                    raise JogError('model_asset_missing', '模型可视 mesh 缺失或路径越界')
                self.meshes[key] = path
                mesh.set('filename', '/mesh/' + quote(key, safe='/._-'))
        self.browser_urdf = ET.tostring(root, encoding='unicode')

    def info(self, chain):
        if self.xml != getattr(chain, 'source_xml', self.xml):
            raise JogError('model_mismatch', '显示模型与规划模型不一致')
        return {'urdf': self.browser_urdf, 'model_hash': chain.model_hash,
                'frame': chain.base, 'tip': chain.tip, 'tcp_xyz_m': chain.tcp[:3, 3].tolist()}


def url_has_scheme(name):
    return ':' in name or not name
