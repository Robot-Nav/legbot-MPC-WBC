# Vbot MuJoCo Forward Walk

`scripts/mujoco_legbot_walk.py` connects the legbot MJCF model in this repository to
a MuJoCo forward walking simulation.

The script uses `models/MJCF/legbot/legbot_mpc_scene.xml` as the source scene,
rewrites its `meshdir` to the checked-in STL directory
`models/legbot/xmls/meshes`, removes non-robot marker arrows, and writes:

```text
models/MJCF/legbot/legbot_mpc_scene_project.xml
```

The original XML is not modified.

## Run

Use the conda environment that contains MuJoCo and NumPy:

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8
```

Run one command speed:

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 8
```

Open the MuJoCo viewer:

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 20 --viewer --realtime
```

Use 500 Hz control with the same 1000 Hz physics:

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 8 --control-hz 500
```

Generate the MuJoCo XML only:

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --prepare-xml
```

## Controller

The controller is a diagonal trot:

- swing legs follow cycloid foot trajectories and analytic leg IK
- stance legs add foot-space ground reaction control through MuJoCo Jacobians,
  using `tau = -J^T F`
- horizontal force tracks commanded forward velocity
- vertical force supports body weight and regulates height
- pitch/roll feedback suppresses forward tipping and side roll
- generated simulation XML scales motor force limits with `--torque-scale`
  defaulting to `1.8`; the source model is left untouched
- physics runs at 1000 Hz; control can run at `--control-hz 1000` or `500`
- viewer sync defaults to every 16 physics steps, close to 60 Hz display

Validated headless in the `mujoco` conda environment for 8 s:

```text
height=0.2775m, control_hz=1000
cmd=0.50m/s avg=0.56m/s
cmd=1.00m/s avg=1.09m/s
cmd=1.50m/s avg=1.56m/s
cmd=2.00m/s avg=1.83m/s

height=0.2775m, control_hz=500
cmd=0.50m/s avg=0.56m/s
cmd=1.00m/s avg=1.10m/s
cmd=1.50m/s avg=1.56m/s
cmd=2.00m/s avg=1.83m/s
```

The health check fails if average speed leaves the walking range, body height
drops too low, or roll/pitch exceed the configured stability limits.
