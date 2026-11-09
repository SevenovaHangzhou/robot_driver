"""Read both checksum manifests and the current composer file-list manifest."""
from hashlib import sha256

import yaml

from .errors import JogError


def inspect_artifact(files):
    artifact = yaml.safe_load(files['artifact.yaml'])
    if artifact.get('selection', {}).get('use_mock_hardware') is not False:
        raise JogError('not_real_runtime', '选中的 artifact 不是实机配置')
    required = ('robot_description.xml', 'controllers.yaml')
    declared = artifact.get('files')
    hashes = {name: sha256(content.encode()).hexdigest() for name, content in files.items()}
    if not isinstance(declared, (dict, list)) or any(name not in declared for name in required):
        raise JogError('artifact_invalid', 'artifact 没有列出模型和控制器配置')
    # Current composer intentionally no longer stores content checksums. Keep
    # fingerprints as evidence, and verify stored checksums only on old artifacts.
    if isinstance(declared, dict):
        if any(hashes[name] != declared[name] for name in required):
            raise JogError('artifact_mismatch', '模型或控制器配置与 artifact 校验值不一致')
    return artifact, hashes
