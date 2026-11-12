"""Build a physics-only, checksummed R4 bundle; never edit viz assets."""

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np

from experiments.r3 import Case, load_scene
from experiments.run_all import ROOT, SCENE
from rl.live import atomic_json
from sim.resources import wait_for_simulation
from sim.rig import DriveRig


def stripped_scene(destination):
    """Keep collision geometry and explicit inertias, replace visual meshes."""
    root = ET.parse(SCENE).getroot()
    compiler = root.find('compiler')
    if compiler.get('inertiafromgeom') != 'false':
        raise ValueError('Cannot strip visual meshes when inertia is inferred')
    board_meshes = {asset.get('name') for asset in root.findall('./asset/mesh')
                    if Path(asset.get('file', '')).name.startswith('board_c')}
    # Resolve the default class rather than assuming its collision masks.
    visual_default = root.find("./default/default[@class='vis']/geom")
    if (visual_default.get('contype'), visual_default.get('conaffinity')) != ('0', '0'):
        raise ValueError('Visual geoms have collision masks')
    board_geoms = [g for g in root.findall('.//body/geom') if g.get('mesh') in board_meshes]
    if not board_geoms or any(g.get('class') != 'vis' or
        int(g.get('contype', '0')) != 0 or int(g.get('conaffinity', '0')) != 0
        for g in board_geoms):
        raise ValueError('Board meshes absent or used by a physics geom; stop')
    stripped_count = 0
    for geom in root.findall('.//geom'):
        geom.attrib.pop('material', None)
        if geom.get('mesh') is not None and geom.get('class') == 'vis':
            if int(geom.get('contype', '0')) or int(geom.get('conaffinity', '0')):
                raise ValueError('A visual mesh has explicit collision masks')
            geom.attrib.pop('mesh')
            geom.set('type', 'box')
            geom.set('size', '.001 .001 .001')
            stripped_count += 1
    used_meshes = {g.get('mesh') for g in root.findall('.//geom')}
    used_fields = {g.get('hfield') for g in root.findall('.//geom')}
    asset = root.find('asset')
    for item in list(asset):
        if ((item.tag == 'mesh' and item.get('name') not in used_meshes) or
            (item.tag == 'hfield' and item.get('name') not in used_fields) or
            item.tag in ('texture', 'material')):
            asset.remove(item)
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Absolute paths are used only during the local proof. The bundled XML is
    # restored to relative filenames below, after proof has succeeded.
    for item in asset:
        if 'file' in item.attrib:
            item.set('file', str(SCENE.parent/item.get('file')))
    for key in ('meshdir', 'texturedir'):
        compiler.set(key, str(SCENE.parent))
    ET.ElementTree(root).write(destination, encoding='unicode')
    return root, dict(board_meshes_removed=sorted(board_meshes),
                      board_visual_geoms=len(board_geoms), visual_meshes_replaced=stripped_count)


def prove_meshes(destination):
    root, report = stripped_scene(destination)
    original = load_scene(Case(settle_limit=True))
    stripped = mujoco.MjModel.from_xml_path(str(destination))
    stripped.geom_friction[:, 0] = .9
    stripped.actuator_ctrlrange[:] = [-13, 13]
    mass_error = float(np.max(np.abs(original.body_mass-stripped.body_mass)))
    inertia_error = float(np.max(np.abs(original.body_inertia-stripped.body_inertia)))
    if mass_error or inertia_error:
        raise AssertionError('Stripped scene changes mass/inertia; stop')
    wait_for_simulation()
    a, b = (DriveRig(Case(settle_limit=True), model=model) for model in (original, stripped))
    settle_error = float(np.max(np.abs(a.data.qpos-b.data.qpos)))
    qpos_error = qvel_error = 0.
    # One second after the same complete R3 settle, including the electronics.
    for _ in range(100):
        for rig in (a, b):
            rig.advance(rig.drive.speeds(0., np.radians(30)))
        qpos_error = max(qpos_error, float(np.max(np.abs(a.data.qpos-b.data.qpos))))
        qvel_error = max(qvel_error, float(np.max(np.abs(a.data.qvel-b.data.qvel))))
    if max(settle_error, qpos_error, qvel_error) > 1e-12:
        raise AssertionError('Stripped scene changes trajectory; stop')
    report.update(body_mass_max_abs=mass_error, body_inertia_max_abs=inertia_error,
        settle_qpos_max_abs=settle_error, one_second_qpos_max_abs=qpos_error,
        one_second_qvel_max_abs=qvel_error, body_count=original.nbody,
        model_body_mass=original.body_mass.tolist(), model_body_inertia=original.body_inertia.tolist(),
        proof='Explicit inertias, zero board collision masks, identical settle and 1 s open-loop trajectory')
    return root, report


def make_bundle(out):
    out = Path(out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    # All intermediates stay beside the bundle output and are owned
    # by this process; TemporaryDirectory removes them on completion.
    with tempfile.TemporaryDirectory(prefix='r4_bundle_', dir=out.parent) as temporary:
        assert Path(temporary).resolve().is_relative_to(out.parent)
        root, report = prove_meshes(Path(temporary)/'stripped.xml')
        compiler = root.find('compiler')
        compiler.set('meshdir', '.')
        compiler.set('texturedir', '.')
        assets = {}
        for item in root.find('asset'):
            if 'file' in item.attrib:
                source = Path(item.get('file'))
                assets['viz/scene/'+source.name] = source.read_bytes()
                item.set('file', source.name)
        files = dict(assets)
        files['viz/scene/rover_scene.xml'] = ET.tostring(root, encoding='utf-8')
        for folder in ('sim', 'rl', 'colab'):
            for path in sorted((ROOT/folder).glob('*.py')):
                files[path.relative_to(ROOT).as_posix()] = path.read_bytes()
        for name in ('__init__.py', 'r3.py', 'run_all.py', 'turn_sweep.py'):
            files['experiments/'+name] = (ROOT/'experiments'/name).read_bytes()
        for name in ('LIVE_FORMAT.md',):
            path = ROOT/'docs'/name
            if path.exists():
                files['docs/'+name] = path.read_bytes()
        files['mesh_proof.json'] = json.dumps(report, indent=2).encode('utf-8')
        files['manifest.json'] = json.dumps({name: hashlib.sha256(content).hexdigest()
                                            for name, content in files.items()}, indent=2).encode('utf-8')
        with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
            for name, content in files.items():
                archive.writestr(name, content)
    checksum = hashlib.sha256(out.read_bytes()).hexdigest()
    out.with_suffix(out.suffix+'.sha256').write_text(checksum+'  '+out.name+'\n', encoding='ascii')
    atomic_json(out.with_suffix('.mesh_check.json'), report)
    print(json.dumps(dict(bundle=str(out), bytes=out.stat().st_size, sha256=checksum,
                          mesh_check=report), indent=2))
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('build/rover-r4.zip'))
    args = parser.parse_args()
    make_bundle(args.out)
