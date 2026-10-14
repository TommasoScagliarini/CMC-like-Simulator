"""Generate English tables for the 35/84 asymmetric observation contract.

Uses only the Python standard library and portable pathlib paths. The generated
tables describe feature semantics independently of any subject or saved rollout.
An optional runtime receipt validates the complete feature order before writing.
No simulator, neural checkpoint, or training framework is loaded.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class Feature:
    name: str
    description: str


ACTOR_FEATURES = (
    Feature("gait_phase_sin", "Sine of prescribed gait-clock phase; dimensionless."),
    Feature("gait_phase_cos", "Cosine of prescribed gait-clock phase; dimensionless."),
    Feature("pros_knee_angle", "Physical prosthetic-knee angle [rad]."),
    Feature("pros_knee_angle_vel", "Physical prosthetic-knee speed [rad/s]."),
    Feature("pros_ankle_angle", "Physical prosthetic-ankle angle [rad]."),
    Feature("pros_ankle_angle_vel", "Physical prosthetic-ankle speed [rad/s]."),
    Feature("SEA_Knee_motor_angle", "Knee motor angle [rad]."),
    Feature("SEA_Knee_motor_speed", "Knee motor speed [rad/s]."),
    Feature("SEA_Ankle_motor_angle", "Ankle motor angle [rad]."),
    Feature("SEA_Ankle_motor_speed", "Ankle motor speed [rad/s]."),
    Feature("online_left_normal_grf_bw", "Prosthetic-foot normal force / body weight."),
    Feature("online_left_in_contact", "Physical prosthetic-foot contact flag [0/1]."),
    Feature("online_left_heel_strike", "Accepted prosthetic-side HS pulse [0/1]."),
    Feature("online_left_toe_off", "Accepted prosthetic-side TO pulse [0/1]."),
    Feature("online_left_gait_phase_sin", "Sine of online prosthetic-side gait phase."),
    Feature("online_left_gait_phase_cos", "Cosine of online prosthetic-side gait phase."),
    Feature("online_left_cycle_duration_s", "Latest available prosthetic-side cycle duration [s]."),
    Feature("phase_fsm_wait_hs", "FSM waiting for HS [0/1]."),
    Feature("phase_fsm_stance_after_hs", "FSM in stance after HS [0/1]."),
    Feature("phase_fsm_swing_after_to", "FSM in swing after TO [0/1]."),
    Feature("phase_expected_hs", "Next expected event is HS [0/1]."),
    Feature("phase_expected_to", "Next expected event is TO [0/1]."),
    Feature("phase_stance_elapsed_norm", "Stance elapsed / 2.2 s; clipped to [0,1.5]."),
    Feature("phase_swing_elapsed_norm", "Swing elapsed / 2.6 s; clipped to [0,1.5]."),
    Feature("phase_cycle_progress_credit", "FSM cycle progress: 0, 0.25, 0.5 or 1."),
    Feature("pros_knee_angle_previous_endpoint", "Last knee endpoint after slew limiting [rad]."),
    Feature("pros_knee_angle_served_ref", "Knee reference served by the governor [rad]."),
    Feature("pros_knee_angle_served_ref_vel", "Served knee-reference speed [rad/s]."),
    Feature("pros_knee_angle_served_ref_accel", r"Served knee-reference acceleration [rad/s$^2$]."),
    Feature("pros_knee_angle_sea_u", "Last normalized knee SEA command [-1,1]."),
    Feature("pros_ankle_angle_previous_endpoint", "Last ankle endpoint after slew limiting [rad]."),
    Feature("pros_ankle_angle_served_ref", "Ankle reference served by the governor [rad]."),
    Feature("pros_ankle_angle_served_ref_vel", "Served ankle-reference speed [rad/s]."),
    Feature("pros_ankle_angle_served_ref_accel", r"Served ankle-reference acceleration [rad/s$^2$]."),
    Feature("pros_ankle_angle_sea_u", "Last normalized ankle SEA command [-1,1]."),
)


def _critic_features() -> tuple[Feature, ...]:
    features = [
        Feature("pros_knee_angle_sea_u_abs", r"Absolute knee SEA command $|u_k|$."),
        Feature("pros_knee_angle_sea_u_saturated", r"Knee command saturation flag: $|u_k|\ge0.98$."),
        Feature("pros_ankle_angle_sea_u_abs", r"Absolute ankle SEA command $|u_a|$."),
        Feature("pros_ankle_angle_sea_u_saturated", r"Ankle command saturation flag: $|u_a|\ge0.98$."),
        Feature("pros_knee_angle_target", "Served knee reference, same as slot 26 [rad]."),
        Feature("pros_knee_angle_target_vel", "Served knee-reference speed, same as slot 27 [rad/s]."),
        Feature("pros_knee_angle_tracking_error", "Served reference minus physical knee angle [rad]."),
        Feature("pros_ankle_angle_target", "Served ankle reference, same as slot 31 [rad]."),
        Feature("pros_ankle_angle_target_vel", "Served ankle-reference speed, same as slot 32 [rad/s]."),
        Feature("pros_ankle_angle_tracking_error", "Served reference minus physical ankle angle [rad]."),
    ]
    biological_coordinates = (
        ("pelvis_tx", "Pelvis x translation", "m", "m/s"),
        ("pelvis_ty", "Pelvis y translation", "m", "m/s"),
        ("pelvis_tz", "Pelvis z translation", "m", "m/s"),
        ("pelvis_tilt", "Pelvis tilt", "rad", "rad/s"),
        ("pelvis_list", "Pelvis list", "rad", "rad/s"),
        ("pelvis_rotation", "Pelvis rotation", "rad", "rad/s"),
        ("hip_flexion_r", "Sound-side hip flexion", "rad", "rad/s"),
        ("knee_angle_r", "Sound-side knee angle", "rad", "rad/s"),
        ("ankle_angle_r", "Sound-side ankle angle", "rad", "rad/s"),
        ("hip_flexion_l", "Prosthetic-side biological hip flexion", "rad", "rad/s"),
        ("hip_adduction_l", "Prosthetic-side biological hip adduction", "rad", "rad/s"),
    )
    for name, description, position_unit, velocity_unit in biological_coordinates:
        features.extend((
            Feature(name, f"{description}: physical coordinate [{position_unit}]."),
            Feature(f"{name}_vel", f"{description}: physical speed [{velocity_unit}]."),
            Feature(f"{name}_kin_ref", f"{description}: prescribed IK reference [{position_unit}]."),
        ))
    features.extend((
        Feature("online_right_normal_grf_bw", "Sound-foot normal force / body weight."),
        Feature("online_right_in_contact", "Physical sound-foot contact flag [0/1]."),
        Feature("online_right_heel_strike", "Sound-side HS pulse [0/1]."),
        Feature("online_right_toe_off", "Sound-side TO pulse [0/1]."),
        Feature("online_right_gait_phase", "Online sound-side gait phase [0,1]."),
        Feature("online_right_cycle_duration_s", "Latest available sound-side cycle duration [s]."),
    ))
    return tuple(features)


CRITIC_EXTRA_FEATURES = _critic_features()
FULL_FEATURES = ACTOR_FEATURES + CRITIC_EXTRA_FEATURES


def validate_schema(receipt_path: Path) -> None:
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    for key, expected in (
        ("actor_feature_names", ACTOR_FEATURES),
        ("observation_feature_names", FULL_FEATURES),
    ):
        actual_names = receipt[key]
        expected_names = [feature.name for feature in expected]
        if actual_names != expected_names:
            raise ValueError(f"Feature names or ordering differ for {key}.")


def _feature_name(name: str) -> str:
    # Breaks are permitted after underscores; the smaller monospaced font also
    # keeps long feature names within the fixed name column on A4 pages.
    escaped = name.replace("_", r"\_\allowbreak{}")
    return r"{\fontsize{7.6}{9.2}\selectfont\texttt{" + escaped + "}}"


def _table(features: Sequence[Feature], start_index: int) -> str:
    lines = [
        r"{\footnotesize",
        r"\begin{longtable}{@{}r p{7.2cm} p{7.5cm}@{}}",
        r"\toprule Index & Feature name & Meaning and units\\\midrule\endhead",
    ]
    for index, feature in enumerate(features, start=start_index):
        lines.append(f"{index} & {_feature_name(feature.name)} & {feature.description}\\\\")
    lines.extend((r"\bottomrule\end{longtable}", "}"))
    return "\n".join(lines) + "\n"


def write_tables(output_directory: Path | None = None, receipt_path: Path | None = None) -> dict[str, int]:
    if len(ACTOR_FEATURES) != 35 or len(CRITIC_EXTRA_FEATURES) != 49:
        raise ValueError("The observation contract must contain 35 actor and 49 extra critic features.")
    if len({feature.name for feature in FULL_FEATURES}) != 84:
        raise ValueError("Observation feature names must be unique.")
    if receipt_path is not None:
        validate_schema(Path(receipt_path))
    destination = Path(output_directory) if output_directory is not None else Path(__file__).resolve().parent
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "actor_features.tex").write_text(_table(ACTOR_FEATURES, 0), encoding="utf-8")
    shared_prefix = (
        "The critic receives all 84 features. Indices 0--34 are identical to the actor table; "
        "the following 49 features occupy indices 35--83.\n\n"
    )
    (destination / "critic_features.tex").write_text(shared_prefix + _table(CRITIC_EXTRA_FEATURES, 35), encoding="utf-8")
    return {"actor_feature_count": 35, "critic_extra_feature_count": 49, "critic_feature_count": 84}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--receipt", type=Path, default=None, help="Optional runtime feature-schema validation.")
    arguments = parser.parse_args()
    print(json.dumps(write_tables(arguments.output_dir, arguments.receipt), indent=2))


if __name__ == "__main__":
    main()
