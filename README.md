# wlsea-asset-processing

Convert a SolidWorks assembly to URDF without launching SolidWorks. Geometry comes from the tessellation saved in each `.SLDPRT`. Joint axes come from motor shafts and gripper racks.

## Setup

Python 3.13 and [uv](https://docs.astral.sh/uv/) are required.

```bash
uv sync
```

That installs the Python packages (`cascadio`, `trimesh`, `numpy`, `mujoco`, `viser`). It does not install `cadmpeg`.

### cadmpeg

`cadmpeg` reads SolidWorks files. The converter uses it to decode the saved tessellation inside each part. Without it, the build falls back to a STEP export, which often produces shard-like meshes or empty solids.

`cadmpeg` is a separate open-source tool: [github.com/cadmpeg/cadmpeg](https://github.com/cadmpeg/cadmpeg). On Linux or macOS:

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/cadmpeg/cadmpeg/releases/latest/download/cadmpeg-installer.sh | sh
```

The installer puts `cadmpeg` on your user `PATH` (typically `~/.local/bin`). Open a new shell and check:

```bash
cadmpeg --version
```

Any of these is enough for the converter to find it:

1. `cadmpeg` is on `PATH`.
2. `CADMPEG` is set to the binary, for example `export CADMPEG=$HOME/.local/bin/cadmpeg`.
3. The binary is copied to `.tools/cadmpeg` in this repository. That directory is gitignored, so each machine keeps its own copy:

```bash
mkdir -p .tools
cp "$(command -v cadmpeg)" .tools/cadmpeg
```

To build from source instead of using the installer, install Rust 1.88 or later and run:

```bash
git clone https://github.com/cadmpeg/cadmpeg
cd cadmpeg
cargo install --path crates/cadmpeg
```

## Convert

Run from this directory. `--model` selects the joint layout. The SolidWorks trees are not part of this repository; the defaults look for them next to it (`../s3机械臂_1`, `../S5`, `../S7`). On another machine, pass `--assembly`.

```bash
uv run python convert_to_urdf.py --model s3
uv run python convert_to_urdf.py --model s5
uv run python convert_to_urdf.py --model s7

uv run python convert_to_urdf.py \
  --assembly /path/to/robot.SLDASM \
  --output output/my_arm
```

`--model s3` is the only layout that reads `Origin_*` coordinate systems from the assembly. `s5` and `s7` have no such frames; their revolute joints are placed on the drive shafts. `--view` opens the URDF in a browser with viser.

Each run writes `output/<name>/<name>.urdf` and one STL per link under `output/<name>/meshes/`.

Check the result:

```bash
uv run python scripts/check_build.py output/s3_arm/s3_arm.urdf
```

The checker exits 1 when a link is larger than 0.5 m or has an edge longer than 0.25 m. On the S3 arm that means a placement or tessellation bug. A part that is itself that large, such as the S7 1 m hose, trips the same limits.

Joint rules differ by arm, and several quantities are still placeholders. See [GUIDE.md](GUIDE.md) for the current layout, known issues, and missing data.
