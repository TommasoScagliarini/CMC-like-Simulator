"""TensorBoard logging for the baseline-MLP PPO pipeline.

Two pieces, both opt-in from ``train_ppo_mlp.py``:

1. ``RewardComponentsCallback`` (an RLlib ``RLlibCallback``) aggregates the
   per-step reward components/losses (exposed by ``RewardShapingWrapper`` and the
   env in ``info``) into env-runner metrics, so they land in
   ``result["env_runners"]`` once per training iteration.
2. ``make_tb_writer`` / ``log_result_scalars`` write a flat view of the training
   ``result`` (env-runner + learner scalars, including the reward components
   above) to a ``torch.utils.tensorboard.SummaryWriter`` under the run's
   ``output_dir/tensorboard``.

Kept in its own importable module (not ``__main__``) so the callback class is
picklable by reference for Ray worker processes.
"""

from __future__ import annotations

import math
from typing import Any, Iterator, Mapping

from ray.rllib.callbacks.callbacks import RLlibCallback

# Rolling window (in steps) for smoothing reward metrics, matching RLlib's own
# metric-smoothing convention. ``reduce="mean"`` requires a window or ema_coeff
# on this Ray version (``clear_on_reduce`` is deprecated).
_SMOOTHING_WINDOW = 100

# Loss terms produced by the env in ``info["reward_terms"]`` worth logging.
_LOSS_KEYS = (
    "tracking_loss",
    "tracking_position_loss",
    "tracking_velocity_loss",
    "reference_loss",
    "reference_position_loss",
    "bio_loss",
    "bio_position_loss",
    "effort_loss",
    "smoothness_loss",
    "command_rate_loss",
    "segment_delta_loss",
    "segment_knot_delta_loss",
    "qdot_ref_loss",
    "qddot_ref_loss",
    "jerk_ref_loss",
    "reference_governor_loss",
    "reference_velocity_limit_fraction",
    "reference_acceleration_limit_fraction",
    "reference_jerk_limit_fraction",
    "u_rate_loss",
    "saturation_loss",
    "sea_saturation_loss",
    "sea_torque_error_loss",
    "sea_tau_spring_effort_loss",
    "sea_tau_spring_rate_loss",
    "sea_motor_speed_loss",
    "sea_motor_accel_loss",
    "sea_motor_power_loss",
    "sea_tau_input_saturation_fraction",
    "pros_knee_angle_sea_tau_spring_rms_nm",
    "pros_ankle_angle_sea_tau_spring_rms_nm",
    "pros_knee_angle_sea_tau_spring_abs_max_nm",
    "pros_ankle_angle_sea_tau_spring_abs_max_nm",
    "pros_knee_angle_sea_tau_spring_rate_rms_nm_s",
    "pros_ankle_angle_sea_tau_spring_rate_rms_nm_s",
    "pros_knee_angle_sea_tau_spring_rate_abs_max_nm_s",
    "pros_ankle_angle_sea_tau_spring_rate_abs_max_nm_s",
    "policy_action_clip_loss",
    "policy_action_clip_fraction",
    "policy_action_clip_abs_max",
    "sound_imitation_loss",
    "served_imitation_loss",
    "safety_loss",
    "contact_load_loss",
    "contact_overload_loss",
    "swing_unloading_loss",
    "grf_slip_loss",
    "phase_regularity_loss",
    "phase_event_progress_score",
    "landing_window_contact_score",
    "landing_window_active",
    "invalid_event_loss",
    "contact_validity_loss",
    "invalid_event_count",
    "phase_fsm_state_id",
    "phase_event_order_loss",
    "phase_period_loss",
    "phase_timeout_loss",
    "phase_stance_timeout_loss",
    "phase_swing_timeout_loss",
    "phase_timeout_exceeded",
    "phase_pending_cycle_credit",
    "phase_cycle_complete_bonus",
    "phase_clawback_penalty",
    "phase_failure_extra_penalty",
    "phase_cycle_failed_this_step",
    "phase_stance_fraction_loss",
    "phase_stance_contact_fraction",
    "phase_stance_mean_load_bw",
    "phase_cycle_knee_excursion_rad",
    "phase_cycle_ankle_excursion_rad",
    "phase_cycle_rejected_this_step",
    "reserve_residual_loss",
    "reserve_norm_loss",
    "residual_norm_loss",
    "pelvis_height_loss",
    "morphology_loss",
    "morphology_knee_loss",
    "morphology_ankle_loss",
    "fsm_morphology_phase",
    "u_abs_max",
    "u_saturation_fraction",
    "terminated",
    "truncated",
)


def _as_finite_float(value: Any) -> float | None:
    try:
        fv = float(value)
    except (TypeError, ValueError):
        return None
    return fv if math.isfinite(fv) else None


class RewardComponentsCallback(RLlibCallback):
    """Log reward components + losses as per-iteration env-runner means.

    Reads the last step's ``info`` from the episode: ``reward_components`` (added
    by ``RewardShapingWrapper``) and ``reward_terms`` (the env's raw losses).
    Defensive by design: any logging error is swallowed so a metrics bug can
    never crash sampling.
    """

    def on_environment_created(self, *, env, env_context, **kwargs) -> None:
        from env_factory import assign_episode_start_offset_for_runner

        assign_episode_start_offset_for_runner(env, env_context)

    def on_episode_step(self, *, episode, metrics_logger=None, **kwargs) -> None:
        if metrics_logger is None:
            return
        try:
            info = episode.get_infos(-1)
        except Exception:
            return
        if not isinstance(info, dict):
            return

        start_offset = _as_finite_float(info.get("episode_start_offset_s"))
        if start_offset is not None:
            offset_label = f"{start_offset:.6f}".replace("-", "m").replace(".", "p")
            metrics_logger.log_value(
                f"episode_start_steps/offset_{offset_label}s",
                1.0,
                reduce="lifetime_sum",
            )

        components = info.get("reward_components")
        if isinstance(components, dict):
            for key, value in components.items():
                self._log(metrics_logger, f"reward/{key}", value)

        terms = info.get("reward_terms")
        if isinstance(terms, dict):
            for key in _LOSS_KEYS:
                if key in terms:
                    self._log(metrics_logger, f"reward_loss/{key}", terms[key])
            for key, value in terms.items():
                if (
                    key.startswith("pros_knee_angle_sea_")
                    or key.startswith("pros_ankle_angle_sea_")
                    or "_reference_" in key
                    or "_tracking_" in key
                    or "_bio_" in key
                    or key.endswith("_reward_q_range")
                    or key.endswith("_reward_qdot_range")
                    or key.startswith("grf_penetration_")
                    or key.startswith("grf_ankle_moment_flip_")
                    or key.startswith("contact_")
                    or key.startswith("phase_")
                    or key.startswith("prosthetic_")
                    or key.startswith("morphology_")
                    or key.startswith("fsm_morphology_")
                    or key.startswith("reserve_")
                    or key.startswith("residual_")
                    or key.startswith("pelvis_")
                    or key.endswith("_imitation_loss")
                    or "_imitation_position_" in key
                    or "_imitation_velocity_" in key
                ):
                    self._log(metrics_logger, f"reward_diagnostic/{key}", value)

        gait = info.get("online_gait")
        if isinstance(gait, dict):
            sides = gait.get("sides")
            if isinstance(sides, dict):
                for side in ("left", "right"):
                    side_info = sides.get(side)
                    if not isinstance(side_info, dict):
                        continue
                    for key in (
                        "normal_force_bw",
                        "in_contact",
                        "heel_strike",
                        "toe_off",
                        "cycle_duration_s",
                        "gait_phase",
                    ):
                        if key in side_info:
                            self._log(
                                metrics_logger,
                                f"gait/{side}/{key}",
                                side_info[key],
                            )

        end_reason = info.get("end_reason")
        if isinstance(end_reason, str) and end_reason:
            safe_reason = end_reason.replace(":", "_").replace("/", "_")
            metrics_logger.log_value(
                f"episode_end/{safe_reason}",
                1.0,
                reduce="lifetime_sum",
            )

    @staticmethod
    def _log(metrics_logger, key: str, value: Any) -> None:
        fv = _as_finite_float(value)
        if fv is None:
            return
        # Rolling-window mean (smoothed like RLlib's own env-runner metrics).
        metrics_logger.log_value(key, fv, reduce="mean", window=_SMOOTHING_WINDOW)


def make_tb_writer(log_dir):
    """Create a torch TensorBoard SummaryWriter at ``log_dir`` (created if needed)."""
    from torch.utils.tensorboard import SummaryWriter

    return SummaryWriter(log_dir=str(log_dir))


def _iter_scalars(node: Mapping[str, Any], prefix: str = "") -> Iterator[tuple[str, float]]:
    for key, value in node.items():
        full = f"{prefix}/{key}" if prefix else str(key)
        if isinstance(value, Mapping):
            yield from _iter_scalars(value, full)
            continue
        fv = _as_finite_float(value)
        if fv is not None:
            yield full, fv


def log_result_scalars(writer, result: Mapping[str, Any], step: int) -> int:
    """Write env-runner + learner scalar metrics from ``result`` at ``step``.

    Returns the number of scalars written. Nested dicts are flattened with ``/``;
    non-scalar / non-finite values are skipped. The reward components logged by
    ``RewardComponentsCallback`` (``reward/*`` / ``reward_loss/*``) are promoted to
    top-level TensorBoard sections; everything else keeps its ``env_runners/`` /
    ``learners/`` prefix.
    """
    written = 0
    for subtree in ("env_runners", "learners"):
        node = result.get(subtree)
        if not isinstance(node, Mapping):
            continue
        for key, value in _iter_scalars(node):
            if (
                key.startswith("reward/")
                or key.startswith("reward_loss/")
                or key.startswith("reward_diagnostic/")
                or key.startswith("gait/")
                or key.startswith("episode_end/")
                or key.startswith("episode_start_steps/")
            ):
                tag = key  # dedicated top-level section for reward tuning
            else:
                tag = f"{subtree}/{key}"
            writer.add_scalar(tag, value, step)
            written += 1
    return written
