# Thesis Batch Evaluation Scripts

These scripts are the public, path-cleaned versions of the automation used during the thesis experiments.

## Scripts

- `run_drake_static_batch.py`
  - Runs `DrakeStatic` for many grasp hypotheses from a CSV.
  - Writes `<input>_withDrakeResults.csv` by default.

- `run_drake_dynamic_batch.py`
  - Runs `DrakeDynamic` for many grasps using configurable force/torque CLI flags.
  - Writes `<input>_withDrakeDynamicResults.csv` by default.

## Common Input Format

Expected semicolon-separated CSV columns:

- `position_x`, `position_y`, `position_z`
- `orientation_x`, `orientation_y`, `orientation_z`, `orientation_w`
- `gripper_opening`
- `score`
- id column as either `graspID` or `id`

If `matrix_name` exists, both scripts only use rows with `X_Drake_Grasp`.

## Example

From Drake workspace root:

```bash
python examples/simple_gripper/thesis_eval/run_drake_static_batch.py \
  --input_csv /home/dan/Projects/MasterThesisFolder/Code/Sprayer/object_1_UOGPLog_heightCorrected.csv \
  --mesh_path /home/dan/Projects/MasterThesisFolder/Code/Sprayer/Sprayer.obj \
  --uogp_object Sprayer \
  --ids 0,1,2
```

```bash
python examples/simple_gripper/thesis_eval/run_drake_dynamic_batch.py \
  --input_csv /home/dan/Projects/MasterThesisFolder/Code/Sprayer/object_1_UOGPLog_heightCorrected.csv \
  --mesh_path /home/dan/Projects/MasterThesisFolder/Code/Sprayer/Sprayer.obj \
  --uogp_object Sprayer \
  --ids 0,1,2 \
  --force_magnitude 180.5 \
  --force_direction y \
  --moment_direction x
```
