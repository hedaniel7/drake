#!/usr/bin/env python3
"""Batch runner for DrakeDynamic perturbation simulation.

Adapted from the thesis automation scripts with hardcoded paths removed.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional

from _drake_batch_common import (
    default_binary_path,
    min_z_from_obj,
    parse_drake_contacts,
    parse_grasp_rows,
    run_drake_command,
)

FIELDNAMES = [
    "id",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "gripper_opening",
    "score",
    "manual_correction",
    "table_correction",
    "advance_sim_to",
    "force_start",
    "force_end",
    "force_magnitude",
    "torque_magnitude",
    "SelectForceDirection",
    "SelectMomentDirection",
    "useNegativeForceAxis",
    "useNegativeMomentAxis",
    "num_contacts",
    "drake_command",
    "F_Ac_W_c1_x",
    "F_Ac_W_c1_y",
    "F_Ac_W_c1_z",
    "p_WC_c1_x",
    "p_WC_c1_y",
    "p_WC_c1_z",
    "F_Ac_W_c2_x",
    "F_Ac_W_c2_y",
    "F_Ac_W_c2_z",
    "p_WC_c2_x",
    "p_WC_c2_y",
    "p_WC_c2_z",
    "object_com_x",
    "object_com_y",
    "object_com_z",
    "status",
]


def parse_id_list(ids_raw: Optional[str]) -> Optional[List[int]]:
    if not ids_raw:
        return None
    return [int(x.strip()) for x in ids_raw.split(",") if x.strip()]


def resolve_table_correction(args: argparse.Namespace) -> float:
    if args.table_correction is not None:
        return args.table_correction
    if args.mesh_path is None:
        raise ValueError("Pass either --table_correction or --mesh_path.")
    return min_z_from_obj(args.mesh_path)


def command_for_grasp(
    binary: Path,
    grasp: Dict[str, object],
    args: argparse.Namespace,
    table_correction: float,
) -> List[str]:
    pos = grasp["position"]
    quat = grasp["orientation"]

    cmd = [
        str(binary),
        f"--position={pos[0]},{pos[1]},{pos[2]}",
        f"--orientation={quat[0]},{quat[1]},{quat[2]},{quat[3]}",
        f"--gripper_opening={grasp['gripper_opening']}",
        f"--manual_correction={args.manual_correction}",
        f"--table_correction={table_correction}",
        f"--force_start={args.force_start}",
        f"--force_end={args.force_end}",
        f"--force_magnitude={args.force_magnitude}",
        f"--torque_magnitude={args.torque_magnitude}",
        f"--SelectForceDirection={args.force_direction}",
        f"--SelectMomentDirection={args.moment_direction}",
    ]

    if not args.use_height_correction:
        cmd.append("--NoHeightCorrection")
    if args.uogp_object:
        cmd.append(f"--uogp_object={args.uogp_object}")
    if args.advance_sim_to is not None:
        cmd.append(f"--advanceSimTo={args.advance_sim_to}")
    if args.use_negative_force_axis:
        cmd.append("--useNegativeForceAxis")
    if args.use_negative_moment_axis:
        cmd.append("--useNegativeMomentAxis")

    return cmd


def append_results(
    output_csv: Path,
    grasp: Dict[str, object],
    drake_command: List[str],
    contacts,
    p_wo_w,
    status: str,
    args: argparse.Namespace,
    table_correction: float,
    num_contacts: int,
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    write_header = not output_csv.exists()

    row = {
        "id": grasp["grasp_id"],
        "position_x": grasp["position"][0],
        "position_y": grasp["position"][1],
        "position_z": grasp["position"][2],
        "orientation_x": grasp["orientation"][0],
        "orientation_y": grasp["orientation"][1],
        "orientation_z": grasp["orientation"][2],
        "orientation_w": grasp["orientation"][3],
        "gripper_opening": grasp["gripper_opening"],
        "score": grasp["score"],
        "manual_correction": args.manual_correction,
        "table_correction": table_correction,
        "advance_sim_to": args.advance_sim_to,
        "force_start": args.force_start,
        "force_end": args.force_end,
        "force_magnitude": args.force_magnitude,
        "torque_magnitude": args.torque_magnitude,
        "SelectForceDirection": args.force_direction,
        "SelectMomentDirection": args.moment_direction,
        "useNegativeForceAxis": args.use_negative_force_axis,
        "useNegativeMomentAxis": args.use_negative_moment_axis,
        "num_contacts": num_contacts,
        "drake_command": " ".join(drake_command),
        "status": status,
    }

    # Fill contact columns with N/A first.
    for key in (
        "F_Ac_W_c1_x",
        "F_Ac_W_c1_y",
        "F_Ac_W_c1_z",
        "p_WC_c1_x",
        "p_WC_c1_y",
        "p_WC_c1_z",
        "F_Ac_W_c2_x",
        "F_Ac_W_c2_y",
        "F_Ac_W_c2_z",
        "p_WC_c2_x",
        "p_WC_c2_y",
        "p_WC_c2_z",
        "object_com_x",
        "object_com_y",
        "object_com_z",
    ):
        row[key] = "N/A"

    if contacts:
        if len(contacts) >= 1:
            c1 = contacts[0]
            row.update(
                {
                    "F_Ac_W_c1_x": c1["F_c_W"][0],
                    "F_Ac_W_c1_y": c1["F_c_W"][1],
                    "F_Ac_W_c1_z": c1["F_c_W"][2],
                    "p_WC_c1_x": c1["p_WC_W"][0],
                    "p_WC_c1_y": c1["p_WC_W"][1],
                    "p_WC_c1_z": c1["p_WC_W"][2],
                }
            )
        if len(contacts) >= 2:
            c2 = contacts[1]
            row.update(
                {
                    "F_Ac_W_c2_x": c2["F_c_W"][0],
                    "F_Ac_W_c2_y": c2["F_c_W"][1],
                    "F_Ac_W_c2_z": c2["F_c_W"][2],
                    "p_WC_c2_x": c2["p_WC_W"][0],
                    "p_WC_c2_y": c2["p_WC_W"][1],
                    "p_WC_c2_z": c2["p_WC_W"][2],
                }
            )

    if p_wo_w is not None:
        row.update(
            {
                "object_com_x": p_wo_w[0],
                "object_com_y": p_wo_w[1],
                "object_com_z": p_wo_w[2],
            }
        )

    with output_csv.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES, delimiter=";")
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(description="Batch-run DrakeDynamic force/torque perturbation tests.")
    parser.add_argument("--input_csv", type=Path, required=True, help="Input grasp CSV (semicolon-separated).")
    parser.add_argument(
        "--output_csv",
        type=Path,
        default=None,
        help="Output CSV path. Default: <input>_withDrakeDynamicResults.csv",
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=default_binary_path("DrakeDynamic"),
        help="Path to DrakeDynamic binary (default: bazel-bin/examples/simple_gripper/DrakeDynamic).",
    )
    parser.add_argument("--mesh_path", type=Path, default=None, help="OBJ mesh path used to auto-compute table correction.")
    parser.add_argument("--table_correction", type=float, default=None, help="Explicit table correction override.")
    parser.add_argument("--uogp_object", type=str, default=None, help="Object folder name passed to Drake binary.")
    parser.add_argument("--manual_correction", type=float, default=0.0)
    parser.add_argument("--advance_sim_to", type=float, default=0.7)
    parser.add_argument("--ids", type=str, default=None, help="Optional comma-separated grasp ids.")

    parser.add_argument("--force_start", type=float, default=0.55)
    parser.add_argument("--force_end", type=float, default=0.56)
    parser.add_argument("--force_magnitude", type=float, default=180.5)
    parser.add_argument("--torque_magnitude", type=float, default=0.0)
    parser.add_argument("--force_direction", type=str, default="y")
    parser.add_argument("--moment_direction", type=str, default="x")
    parser.add_argument("--use_negative_force_axis", action="store_true")
    parser.add_argument("--use_negative_moment_axis", action="store_true")

    parser.add_argument(
        "--use_height_correction",
        action="store_true",
        help="Do not pass --NoHeightCorrection (disabled by default to match thesis runs).",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress per-line subprocess output.")
    args = parser.parse_args()

    if args.output_csv is None:
        args.output_csv = args.input_csv.with_name(
            f"{args.input_csv.stem}_withDrakeDynamicResults{args.input_csv.suffix}"
        )

    table_correction = resolve_table_correction(args)
    ids = parse_id_list(args.ids)
    # If matrix_name exists, only consume grasp pose rows (X_Drake_Grasp).
    # For post-processed CSVs (without matrix_name), parse_grasp_rows will keep all rows.
    grasps = parse_grasp_rows(args.input_csv, ids=ids, only_matrix_name="X_Drake_Grasp")

    if not args.binary.exists():
        raise FileNotFoundError(f"Drake binary not found: {args.binary}")

    if args.uogp_object is None and args.mesh_path is not None:
        args.uogp_object = args.mesh_path.stem

    print(f"Loaded {len(grasps)} grasp rows from {args.input_csv}")
    print(f"Writing results to {args.output_csv}")
    print(f"Using table_correction={table_correction}")

    for i, grasp in enumerate(grasps, start=1):
        cmd = command_for_grasp(binary=args.binary, grasp=grasp, args=args, table_correction=table_correction)

        print(f"\n[{i}/{len(grasps)}] Running grasp_id={grasp['grasp_id']}")
        output = run_drake_command(cmd, cwd=args.binary.parent, verbose=not args.quiet)

        contacts, p_wo_w, num_contacts = parse_drake_contacts(output)
        status = "SUCCESS" if num_contacts in (2, 3) else "FAILED"

        append_results(
            output_csv=args.output_csv,
            grasp=grasp,
            drake_command=cmd,
            contacts=contacts,
            p_wo_w=p_wo_w,
            status=status,
            args=args,
            table_correction=table_correction,
            num_contacts=num_contacts,
        )

    print("\nDone.")


if __name__ == "__main__":
    main()
