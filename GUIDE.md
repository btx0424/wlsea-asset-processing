# Guide for future work

The converter works for three arms, but the way those arms are described is inconsistent. Geometry and file reading are shared. Joints are not: S3, S5, and S7 each have their own rules, and two of those rules were revised after the first guess turned out to be a motor shaft instead of the hinge. This note is the map for cleaning that up. It also lists what is still wrong or unknown.

## How the code is split today

`convert_to_urdf.py` is about 1,300 lines and does every stage:

| Stage | Where | Shared? |
|---|---|---|
| Read SolidWorks blocks, component tree, `swTransform` | `convert_to_urdf.py` | yes |
| `sw16_to_se3`, `cs13_to_se3`, `part_axis` | `src/conversion/sw_math.py` | yes |
| Saved tessellation, then STEP | `src/conversion/mesh_part.py` | yes |
| Which link a part belongs to, and which shaft is the joint | `joint_axes_world` / `assign_links` for S3; `_s5_kinematics` and `_s7_kinematics` for the others | no |
| Write URDF, merge fixed joints | `convert_to_urdf.py` | yes, but it reads the global `_ACTIVE_LINKS` |
| Viewer | `view()` in the same file | yes |

S3 is the module-level table `LINKS` plus `Origin_*` frames. S5 and S7 return their own tables and assign `_ACTIVE_LINKS` for the rest of that process. `coordinate_systems`, `joint_axes_world`, and `assign_links` still look at `LINKS`, so they only make sense for S3.

A second copy of the math and meshing helpers lives in `.cursor/skills/solidworks-to-urdf/scripts/`. The package copies are the ones the converter imports. The skill text still says every revolute axis is local Y. That is true for the S3 motor shafts and false for the S5 worm wheels and the S7 elbow bevel.

`src/conversion/__init__.py` still prints `Hello from conversion!`, and that is the `conversion` console script. The real entry point is `python convert_to_urdf.py`.

## What to aim for

One description per arm, and one function that builds a URDF from it. The description should be data: link name, parent, joint type, and how to find the frame and the axis. S3, S5, and S7 should go through that same function.

Keep these rules in the shared code, because they already survived all three arms:

- Transpose the 3x3 in `swTransform`. Do not transpose an `Origin_*` coordinate system the same way.
- Mesh the tessellation saved in the part. Use STEP only when that tessellation is missing, and bake the GLB scene graph.
- A part origin can sit far from the solid. Body links follow the mesh centroid.
- A jaw rack is long, so its centroid sits on the centerline. Split jaws by the part origin.
- A name that contains `电机轴` or `蜗杆` is a motor. The hinge may be a different part: a pin (`固定轴`), a bearing, a worm wheel, or a bevel.
- The viewer rotates about the URDF `<axis>`.

A new arm should not require a new 150-line function. It should require a new description, or frames placed in SolidWorks under the `Origin_*` names S3 already uses.

Suggested order:

1. Turn `_ACTIVE_LINKS` into an argument. Stop writing a global during `build()`.
2. Move S3's `joint_axes_world` and `assign_links` into the same kind of description S5 and S7 already return, so all three call one writer.
3. Delete the skill copies of `sw_math.py` and `mesh_part.py`, or make them import the package. Update the skill so it does not say "local Y" for every arm.
4. Point the `conversion` script at `convert_to_urdf.py` and remove the hello stub.
5. Add a small test that loads each URDF, checks parent/child names, and checks that a 90 degree motion keeps a point on the joint axis fixed. The S5 and S7 bugs were exactly that test failing.
6. Only after that, parse mates. Mates are the information the heuristics are guessing.

## What to do in SolidWorks

The highest-leverage change is to mark each hinge in the assembly. `--model s3` already reads that. S5 and S7 only have the three default planes, so the converter guessed from part names and twice chose a motor shaft instead of the hinge.

**Put a coordinate system on every joint.** Name them `Origin_global`, `Origin_pitch`, `Origin_roll`, and so on, one per child link, with the origin on the hinge. `Origin_global` is the arm root. These are the link frames `--model s3` uses.

**Keep the hinge part distinct from the motor, and model the hinge along a part axis.**

- A part whose name contains `电机轴` is a motor. On the S3 arm its shaft runs along the part's local Y, and that Y is the revolute axis. The `Origin_*` frame does not have to point its own Z along the joint.
- A rack whose name contains `齿条` slides along the part's local X. The two jaws get opposite signs so positive motion opens both.
- Name the actual pin, bearing, worm wheel, or bevel as itself (`固定轴`, `蜗轮`, `锥齿轮`). On S5 the worm is the motor and the worm wheel is the joint. On S7 `固定轴1` is the shoulder and the elbow bevel is the pitch. `电机轴2` is a different pair of joints farther down the arm.

**Make each rigid link its own subassembly.** S7's `底座装配`, `大臂装配`, and `小臂装配` could be assigned as links. S5 is one flat assembly, so parts were split by a guessed curve along the arm. If a motor inside a link is supposed to move something, put that mechanism in its own subassembly instead of leaving it inside a rigid one. The S7 upper-arm four-bar and timing belts are still frozen into `upper_link` for that reason.

**Save real files and a real tessellation.**

- Rebuild and save parts so SolidWorks stores the display tessellation. Parts without it are meshed from a STEP export, which comes out as shards or an empty solid. On S3 that is `S2连接件`, `夹爪安装座`, `夹爪限位盖`, `夹爪底座`, `S3连接件V2`, and `限位销`.
- Do not leave parts as SolidWorks temp copies. The S5 RoboMaster internals and the copied FAMR86 fingers point at `C:\Users\...\Temp\...` and are skipped. Keep `FAMR86-新 (1).STEP` next to the assembly if the finger parts stay virtual.
- Fix or suppress broken external references such as `调试底座`. Suppressed and hidden parts are already skipped, which is the right way to drop the debug base and the 1 m hose.

**Leave the home pose as the active configuration.** The URDF zero is that configuration (`盖板` on S3, `默认` on S5 and S7).

Mass, joint limits, and positive direction are not read from the assembly today. S3 uses one recorded mass. S5 and S7 assume aluminum at 2700 kg/m3, and every revolute limit is +/- pi. Setting those in SolidWorks will not show up in the URDF until the converter is taught to read them. A coordinate system whose Z is the positive hinge direction, plus limits written next to that name, is the piece to add when that happens.

## Known issues

**Joints**

- S5 and S7 joint axes were inferred from part names and mesh shape. They have been corrected once each (worm versus worm wheel; motor shaft versus `固定轴1` and the elbow bevel) and are still not checked against a mate or a drawing.
- Positive direction is whichever way the part's axis column points. It has not been matched to a convention such as "positive lifts the arm."
- Limits are +/- pi for revolute joints and +/- 20 mm for the jaws. Effort and velocity are constants (`20` and `1`).
- The S7 upper arm is a linkage (`电机轴1` four times, `连接杆2`, timing belts) collapsed into one rigid `upper_link`. Those internal degrees of freedom are not in the URDF.
- `wrist_yaw` on S7 has a frame and no mesh of its own. The vertical `电机轴2` stays on `wrist_pitch`; only the gripper yaws.
- S3 bakes the fixed catch joint into the roll mesh. The catch frame is not a link in the written URDF.

**Geometry**

- Parts with no saved tessellation fall back to STEP. On S3 that set is `S2连接件`, `夹爪安装座`, `夹爪限位盖`, `夹爪底座`, `S3连接件V2`, and `限位销`. STEP from `cadmpeg convert` can be an untrimmed surface (a shard) or an empty solid.
- `WL-S3ARM-016 S3连接件.SLDPRT` and `WLT-S5ARM-004 外壳4.SLDPRT` make `cadmpeg convert` fail. The display list still meshes them.
- S7 motor cans and several small screws (`DM_4340_3D_2`, `4310_202409130001`, `M2_5-L6`, and the other `*_2.SLDPRT` virtual pieces) decode to an empty STEP and are omitted.
- `scripts/check_build.py` treats a link larger than 0.5 m, or an edge longer than 0.25 m, as a bug. That matches the S3 arm. The S7 hose `aob02-h4545-l1000` is a real 1 m tessellation, and the upper-arm shells are 0.38 m with 0.25 m edges, so S7 fails the checker on purpose.
- Link colors are a fixed palette. They are not the SolidWorks appearance colors. The red finger tips on the S3 render are appearances, not separate links.

**Files the archives do not contain**

- S3 warns about `调试底座` (`D:\Machine\设计\s3机械臂\step\111\调试底座.SLDPRT`).
- S5 gripper fingers are temp copies (`Copy of FA86...^FAMR86-新 (1).STEP-1`). The build substitutes `FAMR86-新 (1).STEP` from the S3 tree. The RoboMaster M3508 internals are the same kind of temp copy and have no substitute, so that motor is hollow.
- S7 still skips `WL-S3ARM-036柔性夹爪平台`, `夹爪安装板`, and two fastener configurations whose names do not match the files in `s3机械臂_1`.

**Packaging**

- `--assembly` does not select a kinematic model. An unfamiliar assembly with `--model` left at the default is built as S3 and will fail the `Origin_*` check, or worse, attach the S3 table to the wrong geometry.
- Default assembly paths are siblings of this repo (`../s3机械臂_1`, `../S5`, `../S7`). The CAD is not versioned here.
- `README.md` still says joint axes come from motor shafts. That sentence is only true for the S3 roll and the S7 gripper roll.

## Missing information

These are the facts the heuristics are standing in for. None of them are in the files we have.

| Needed | Why |
|---|---|
| Joint list per arm | Which pairs of parts are actually a hinge, and in which order |
| Axis and origin of each joint | A mate, a named coordinate system, or a drawing. Part-name search has already picked the motor instead of the hinge |
| Positive direction and range | From the mechanism or the controller, not from the part's local axis |
| Assembly mass and per-link mass | S3 uses one number taken from SolidWorks, `3.8939075156825225` kg, spread by mesh volume. S5 and S7 use aluminum at 2700 kg/m3 because the assemblies store no mass |
| Inertia about the real center of mass | The mesh inertia assumes uniform density and uses a box when the mesh is not closed |
| Which internal linkages are degrees of freedom | The S7 upper-arm four-bar and timing belts are rigid in the URDF. That may be wrong |
| SolidWorks appearances | If link color should match the CAD |
| A home pose other than the saved configuration | The URDF zero is whatever configuration was active (`盖板` on S3, `默认` on S5 and S7) |

The durable fix for the joint rows is to put `Origin_*` coordinate systems on the hinges in SolidWorks, the way S3 already does, and to record the positive direction next to each name. Until those exist, every new arm will need another hand-written kinematic function, which is how this file got into its current shape.
