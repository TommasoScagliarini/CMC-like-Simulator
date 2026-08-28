"""Validation-only rollout driver for the A_iso39_v3 ablation (F1).

Runs the *unchanged* production rollout loop ``rollout_eval.run(args)`` and
installs two hooks **in the ``rollout_eval`` module namespace of this
process only** (no production file is edited, no env/reward attribute is
mutated):

1. ``rollout_eval.RLModule`` is replaced by a loader whose ``from_checkpoint``
   returns an :class:`AIsoAdapterModule` wrapping the real RLModule loaded
   by the real ``RLModule.from_checkpoint`` (called exactly once, as in
   production, so the torch RNG consumption before the first sample is
   unchanged for the same module architecture);
2. ``rollout_eval.env_factory`` is replaced by a proxy whose
   ``make_cmc_env`` calls the real factory and hands the env to the adapter
   (read-only access to ``env.unwrapped.t`` and to the prescribed data).

Per step the adapter inserts the four prescribed targets at indices 2:6 of
the actor block (and therefore of the full 84-wide observation, which becomes
88-wide like the V26 training observation), asserts
``project39to35(obs39) == obs35`` exactly, and delegates to the original V26
module.  In ``passthrough`` mode nothing is inserted and the run must be
bit-identical to ``rollout_eval.py`` (G1 control).

The prescribed targets are rebuilt read-only with the env's own classes
(``GaitPhaseClock`` + ``PhaseBasedImitationTarget``) from the env's own
prescribed data and config values, exactly as the env does when
``gait_clock_enable`` is true; the env's own (disabled) clock/target objects
are left untouched.  Heel strikes are taken from the env's pure read-only
method ``_load_sound_heel_strikes(side)``.

Both the supervisor and the worker process are this script; CLI = the full
``rollout_eval.py`` CLI plus ``--f1-adapter {aiso4,aiso6clk,passthrough}``.
Side-car outputs (in the job output dir): ``f1_adapter_summary.json`` and
``f1_adapter_trace.json``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
BASELINE_DIR = HERE.parents[1]
for entry in (str(HERE), str(BASELINE_DIR)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

import _bootstrap  # noqa: E402

_bootstrap.ensure_sim_paths()

import numpy as np  # noqa: E402

import f1_common as F1  # noqa: E402  (puts the F0 directory on sys.path)
import f0_common as C  # noqa: E402  (F0 library, immutable)
import f1_obs_adapter as OA  # noqa: E402

F1_FLAG = "--f1-adapter"
ADAPTER_MODES = F1.ADAPTER_MODES
ADAPTER_SUMMARY_FILE = "f1_adapter_summary.json"
ADAPTER_TRACE_FILE = "f1_adapter_trace.json"


def split_f1_argv(argv: list[str]) -> tuple[dict[str, str], list[str]]:
    """Extract the F1-only flags; the rest is the verbatim rollout_eval CLI."""
    opts: dict[str, str] = {}
    rest: list[str] = []
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok == F1_FLAG:
            if i + 1 >= len(argv):
                raise SystemExit(f"{F1_FLAG} requires a value")
            opts["adapter"] = argv[i + 1]
            i += 2
            continue
        if tok.startswith(F1_FLAG + "="):
            raise SystemExit(f"{F1_FLAG}=value form is ambiguous for the receipt binding; use '{F1_FLAG} value'")
        rest.append(tok)
        i += 1
    if opts.get("adapter") not in ADAPTER_MODES:
        raise SystemExit(f"{F1_FLAG} must be one of {'|'.join(ADAPTER_MODES)} (got {opts.get('adapter')!r})")
    return opts, rest


# --- reconstructor from the live env (read-only) ----------------------------------


def build_reconstructor_from_env(base_env: Any) -> OA.PrescribedTargetReconstructor:
    """Build clock + phase-based target with the env's own classes and the env's
    own parameters/data, without touching the env's disabled objects."""
    import osim_trj_cmc_like as ENV  # the module the env instance comes from

    env_cfg = base_env.env_cfg
    cfg = base_env.cfg
    if bool(getattr(env_cfg, "gait_clock_enable", True)):
        raise OA.AdapterError("A_iso requires B's runtime with gait_clock_enable=false (the env clock must stay disabled)")
    own_clock = getattr(base_env, "_gait_clock", None)
    if own_clock is not None and bool(getattr(own_clock, "available", False)):
        raise OA.AdapterError("env clock unexpectedly available under the v3 runtime")
    side = str(env_cfg.gait_clock_side).strip().lower()
    heel_strikes = base_env._load_sound_heel_strikes(side)  # pure read of ctx/cfg
    clock = ENV.GaitPhaseClock(heel_strikes, phase_offset=float(env_cfg.gait_clock_phase_offset))
    target = ENV.PhaseBasedImitationTarget(
        base_env.base_kin,
        clock,
        env_cfg.imitation_sound_coords,
        env_cfg.imitation_phase_shifts,
        fallback_phase_shift=float(env_cfg.imitation_phase_shift),
        phase_samples=int(env_cfg.imitation_phase_samples),
        time_window=(float(cfg.t_start), float(cfg.t_end)),
    )
    provenance = {
        "builder": "f1_rollout_aiso.build_reconstructor_from_env",
        "env_classes": {"clock": "osim_trj_cmc_like.GaitPhaseClock", "target": "osim_trj_cmc_like.PhaseBasedImitationTarget"},
        "gait_clock_side": side,
        "gait_clock_phase_offset": float(env_cfg.gait_clock_phase_offset),
        "imitation_sound_coords": dict(env_cfg.imitation_sound_coords),
        "imitation_phase_shifts": {k: float(v) for k, v in dict(env_cfg.imitation_phase_shifts).items()},
        "imitation_phase_shift": float(env_cfg.imitation_phase_shift),
        "imitation_phase_samples": int(env_cfg.imitation_phase_samples),
        "time_window": [float(cfg.t_start), float(cfg.t_end)],
        "grf_data_file": C.rel(getattr(base_env.ctx, "grf_data_file", "")),
        "grf_vertical_force_columns": dict(getattr(base_env.ctx, "grf_vertical_force_columns", {}) or {}),
        "grf_contact_threshold_n": float(cfg.grf_contact_threshold_n),
        "grf_min_contact_duration_s": float(getattr(cfg, "grf_min_contact_duration_s", 0.0)),
        "grf_min_cycle_duration_s": float(getattr(cfg, "grf_min_cycle_duration_s", 0.0)),
        "env_own_clock_available": bool(getattr(own_clock, "available", False)) if own_clock is not None else None,
        "env_own_imitation_target_available": bool(getattr(getattr(base_env, "_imitation_target", None), "available", False)),
    }
    return OA.PrescribedTargetReconstructor(clock, target, provenance=provenance)


# --- adapter RLModule ---------------------------------------------------------------


class AIsoAdapterModule:
    """Duck-typed RLModule facade consumed by rollout_eval's policy helpers."""

    def __init__(self, real: Any, spec: OA.InsertionSpec, mode: str, *, env_width: int, env_full_width: int) -> None:
        self._real = real
        self._spec = spec
        self._mode = mode
        self._n_actor = int(env_width)
        self._n_full = int(env_full_width)
        self._adapter: OA.ObservationAdapter | None = None
        self._base_env: Any = None
        real_n_actor = getattr(real, "_n_actor", None)
        real_n_full = getattr(real, "_n_full", None)
        if mode == "passthrough":
            if real_n_actor is not None and int(real_n_actor) != self._n_actor:
                raise OA.AdapterError(f"passthrough requires a {self._n_actor}-wide module, got {real_n_actor}")
        else:
            if real_n_actor is None or int(real_n_actor) != len(spec.names39):
                raise OA.AdapterError(f"wrapped module width {real_n_actor} != {len(spec.names39)}")
            if real_n_full is not None and int(real_n_full) != self._n_full + spec.count:
                raise OA.AdapterError(f"wrapped module full width {real_n_full} != {self._n_full + spec.count}")

    # -- binding ------------------------------------------------------------------
    def bind_env(self, env: Any) -> None:
        base = env.unwrapped
        names = tuple(str(n) for n in getattr(base, "actor_feature_names", ()))
        full = tuple(str(n) for n in getattr(base, "observation_feature_names", ()))
        if names != self._spec.names35:
            raise OA.AdapterError("env actor feature names != pinned 35D manifest")
        if len(full) != self._n_full or full[: len(names)] != names:
            raise OA.AdapterError(f"env full observation width {len(full)} != {self._n_full} or actor prefix mismatch")
        diag = set(C.CONTROLLER_DIAGNOSTIC_FEATURES)
        if diag & set(names):
            raise OA.AdapterError("controller diagnostics must not sit in the actor block")
        names_out = names[: self._spec.index] + self._spec.inserted_names + names[self._spec.index:]
        if self._mode != "passthrough" and names_out != self._spec.names39:
            raise OA.AdapterError("inserted actor names != pinned 39D manifest")
        self._base_env = base
        recon = None if self._mode == "passthrough" else build_reconstructor_from_env(base)
        self._adapter = OA.ObservationAdapter(self._spec, self._mode, recon)

    # -- RLModule facade --------------------------------------------------------------
    def _adapt_batch(self, batch: dict[str, Any]) -> dict[str, Any]:
        import torch

        if self._adapter is None or self._base_env is None:
            raise OA.AdapterError("adapter used before bind_env")
        obs_t = batch["obs"]
        if obs_t.ndim != 2 or obs_t.shape[0] != 1:
            raise OA.AdapterError(f"adapter expects a (1, n) batch, got {tuple(obs_t.shape)}")
        obs_np = obs_t.detach().cpu().numpy()[0]
        t_pre = float(self._base_env.t)
        out_np = self._adapter.adapt(obs_np, t_pre)
        if self._mode == "passthrough":
            return batch
        out_t = torch.as_tensor(np.ascontiguousarray(out_np), dtype=obs_t.dtype, device=obs_t.device).reshape(1, -1)
        return {**batch, "obs": out_t}

    def forward_inference(self, batch: dict[str, Any]) -> Any:
        return self._real.forward_inference(self._adapt_batch(batch))

    def forward_exploration(self, batch: dict[str, Any]) -> Any:
        return self._real.forward_exploration(self._adapt_batch(batch))

    def get_inference_action_dist_cls(self) -> Any:
        return self._real.get_inference_action_dist_cls()

    def get_exploration_action_dist_cls(self) -> Any:
        return self._real.get_exploration_action_dist_cls()

    def __getattr__(self, name: str) -> Any:  # any other attribute: delegate
        return getattr(self._real, name)

    def adapter_summary(self) -> dict[str, Any]:
        return self._adapter.summary() if self._adapter is not None else {"mode": self._mode, "bound": False}

    def adapter_trace(self) -> list[dict[str, Any]]:
        return self._adapter.trace() if self._adapter is not None else []


class _EnvFactoryProxy:
    def __init__(self, real: Any, on_env: Any) -> None:
        self._real = real
        self._on_env = on_env

    def make_cmc_env(self, env_config: Any) -> Any:
        env = self._real.make_cmc_env(env_config)
        self._on_env(env)
        return env

    def __getattr__(self, name: str) -> Any:
        return getattr(self._real, name)


def install_hooks(RE: Any, mode: str, spec: OA.InsertionSpec, holder: dict[str, Any], *, env_width: int, env_full_width: int) -> None:
    """Install the loader and env-factory hooks into the rollout_eval namespace."""
    RE._load_inference_stack()  # guarded: run() will not re-import / overwrite
    real_rlmodule = RE.RLModule
    real_factory = RE.env_factory

    class _Loader:
        @staticmethod
        def from_checkpoint(path: Any) -> AIsoAdapterModule:
            real = real_rlmodule.from_checkpoint(path)
            module = AIsoAdapterModule(real, spec, mode, env_width=env_width, env_full_width=env_full_width)
            holder["module"] = module
            return module

    def on_env(env: Any) -> None:
        module = holder.get("module")
        if module is None:
            raise OA.AdapterError("env created before the module was loaded (hook order)")
        module.bind_env(env)
        holder["env"] = env

    RE.RLModule = _Loader
    RE.env_factory = _EnvFactoryProxy(real_factory, on_env)
    holder["hooks"] = {"RLModule": "f1_rollout_aiso._Loader", "env_factory": "f1_rollout_aiso._EnvFactoryProxy", "real_RLModule": repr(real_rlmodule), "real_env_factory": getattr(real_factory, "__file__", repr(real_factory))}


def load_insertion_spec() -> OA.InsertionSpec:
    names35, sha35 = OA.read_manifest_names(C.ACTOR_MANIFEST_35, expected_sha256=C.ACTOR_MANIFEST_35_SHA256, sha256_fn=C.sha256_file)
    names39, sha39 = OA.read_manifest_names(C.ACTOR_MANIFEST_39, expected_sha256=C.ACTOR_MANIFEST_39_SHA256, sha256_fn=C.sha256_file)
    return OA.derive_insertion(names35, names39, manifest35_sha256=sha35, manifest39_sha256=sha39)


def main() -> None:
    opts, rest = split_f1_argv(sys.argv[1:])
    mode = opts["adapter"]
    sys.argv = [sys.argv[0], *rest]
    import rollout_eval as RE
    import process_watchdog

    args = RE.parse_args()
    if not args.worker_process:
        output_dir = RE._resolve_output_dir(args.output_dir, "rollout_eval")
        output_dir.mkdir(parents=True, exist_ok=True)
        child_args = [v for v in rest if v != "--worker-process"]
        child_args.extend(["--checkpoint", str(args.checkpoint)])
        child_args.extend(["--output-dir", str(output_dir)])
        child_args.append("--worker-process")
        child_args.extend([F1_FLAG, mode])
        result = process_watchdog.supervise_process(
            [sys.executable, str(Path(__file__).resolve()), *child_args],
            heartbeat_file=output_dir / RE._WATCHDOG_FILENAME,
            summary_file=output_dir / "watchdog_summary.json",
            startup_timeout_s=args.startup_timeout_s,
            stall_timeout_s=args.stall_timeout_s,
            run_timeout_s=args.run_timeout_s,
            label="f1 aiso rollout",
        )
        if result["timeout_reason"] is not None:
            raise SystemExit(124 if result["timeout_reason"] != "user_interrupt" else 130)
        raise SystemExit(int(result["returncode"]))

    spec = load_insertion_spec()
    holder: dict[str, Any] = {}
    install_hooks(RE, mode, spec, holder, env_width=F1.ENV_ACTOR_WIDTH, env_full_width=F1.FULL_OBS_WIDTH_35)
    summary = RE.run(args)
    module = holder.get("module")
    out = Path(args.output_dir)
    adapter_summary = {
        "schema_version": 1,
        "driver": "f1_rollout_aiso",
        "driver_sha256": C.sha256_file(Path(__file__).resolve()),
        "rollout_eval_sha256": C.sha256_file(Path(RE.__file__).resolve()),
        "adapter_mode": mode,
        "hooks": holder.get("hooks"),
        "checkpoint": str(args.checkpoint),
        "steps_recorded_by_rollout": int(summary.get("steps", 0)),
        **(module.adapter_summary() if module is not None else {"bound": False}),
    }
    adapter_summary["steps_match_rollout"] = bool(adapter_summary.get("steps") == adapter_summary["steps_recorded_by_rollout"])
    (out / ADAPTER_SUMMARY_FILE).write_text(json.dumps(adapter_summary, indent=2), encoding="utf-8")
    (out / ADAPTER_TRACE_FILE).write_text(json.dumps(module.adapter_trace() if module is not None else [], indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
