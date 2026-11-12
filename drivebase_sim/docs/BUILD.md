# Building the model and running the experiments

Run the commands from the repository root unless noted otherwise.

Python 3.12 with `mujoco` (4.0), `numpy`, `trimesh`, `shapely`, `mapbox-earcut`, `fast_simplification`,
`imageio`.

```
cd viz
python build_rover.py        # writes scene/rover_scene.xml (+ meshes, terrain, texture)
python preview_rover.py      # settles, drives over a rock with a simple speed loop, writes stills to out/
python -m mujoco.viewer --mjcf scene/rover_scene.xml
```

The control board meshes are not in this repository. To include them, clone the team's
`Rover_Control_Board_v1` KiCad repository into `ref/`. Both `ref/` and
`board3d/` stay out of git. From the repository root:

```
kicad-cli pcb export glb --force --subst-models -o board3d/single_board_body.glb ref/Rover_Control_Board_v1/single_board/single_board.kicad_pcb
python viz/board_mesh.py board3d/single_board_body.glb board3d/mj --faces 120000
```

Without them, `build_rover.py` puts a plain 100 x 100 mm board in its place.

## Results

The findings are summarised in [FINDINGS.md](FINDINGS.md); plots and data are under `../results/`.

## Experiment commands

```powershell
python -m pytest tests --basetemp $env:TEMP\rover-sim-bt -p no:cacheprovider -q
python -m experiments.run_all
python -m experiments.check_results
python -m experiments.r3
```
