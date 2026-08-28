"""Self-test of f1_sigma_variant on a synthetic module directory (temp only)."""

from __future__ import annotations

import json
import math
import pickle
import sys
import tempfile
from collections import OrderedDict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import f1_common as F1  # noqa: E402
import f1_sigma_variant as SV  # noqa: E402

CHECKS = 0


def check(cond: bool, label: str) -> None:
    global CHECKS
    CHECKS += 1
    if not cond:
        raise AssertionError(f"CHECK FAILED: {label}")


def synthetic_state(width: int, hidden: int = 8, seed: int = 0) -> OrderedDict:
    rng = np.random.default_rng(seed)
    w1 = rng.standard_normal((hidden, width)).astype(np.float32)
    b1 = rng.standard_normal(hidden).astype(np.float32)
    w2 = rng.standard_normal((hidden, hidden)).astype(np.float32)
    b2 = rng.standard_normal(hidden).astype(np.float32)
    w3 = rng.standard_normal((4, hidden)).astype(np.float32)
    b3 = rng.standard_normal(4).astype(np.float32)
    return OrderedDict([("pi_encoder.0.weight", w1), ("pi_encoder.0.bias", b1), ("pi_encoder.2.weight", w2), ("pi_encoder.2.bias", b2), ("pi.0.0.weight", w1.copy()), ("pi.0.0.bias", b1.copy()), ("pi.0.2.weight", w2.copy()), ("pi.0.2.bias", b2.copy()), ("pi.1.weight", w3), ("pi.1.bias", b3)])


def write_module(dir_: Path, state: OrderedDict, names: list[str] | None = None) -> None:
    dir_.mkdir(parents=True)
    with (dir_ / "module_state.pkl").open("wb") as h:
        pickle.dump(state, h, protocol=pickle.HIGHEST_PROTOCOL)
    with (dir_ / "class_and_ctor_args.pkl").open("wb") as h:
        pickle.dump({"class": "synthetic", "ctor_args_and_kwargs": ((), {"model_config": {"n_actor": len(state["pi.0.0.weight"][0])}})}, h)
    (dir_ / "metadata.json").write_text(json.dumps({"checkpoint_version": "2.1"}), encoding="utf-8")
    if names:
        (dir_ / "actor_feature_manifest.json").write_text(json.dumps({"actor_feature_names": names, "actor_feature_count": len(names)}), encoding="utf-8")


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="f1_sigma_"))
    src = tmp / "src" / "rl_module"
    state = synthetic_state(35)
    write_module(src, state, [f"f{i}" for i in range(35)])
    import warm_start as W  # via f1_sigma_variant's sys.path

    src_digest = W.actor_state_digest(state)
    # dry run writes nothing
    rep = SV.derive_constant_sigma_module(src, tmp / "out_dry", sigma=0.005)
    check(rep["mode"] == "dry_run" and rep["materialized"] is False and not (tmp / "out_dry").exists(), "dry-run writes nothing")
    check(rep["source_actor_digest"] == src_digest, "source digest recorded")
    head = rep["derived_head"]
    check(head["logstd_weight_rows_all_zero"] and head["logstd_bias_exact"], "in-memory head invariants")
    check(abs(head["entropy_nat"] - (-7.758758)) <= 1e-5, f"entropy {head['entropy_nat']} ~ -7.758758")
    check(head["logstd_bias"] == [float(np.float32(math.log(0.005)))] * 2, "bias is float32(ln 0.005)")
    check(rep["source_head"]["logstd_weight_rows_all_zero"] is False, "source head state-dependent")
    check(rep["derived_actor_digest"] != src_digest, "derived digest differs from source")
    # materialize
    rep2 = SV.derive_constant_sigma_module(src, tmp / "out", sigma=0.005, expected_source_actor_digest=src_digest)
    rep2 = SV.derive_constant_sigma_module(src, tmp / "out", sigma=0.005, materialize=True, expected_source_actor_digest=src_digest)
    out = tmp / "out" / "rl_module"
    check(out.is_dir() and (out / "module_state.pkl").is_file() and (out / "class_and_ctor_args.pkl").is_file() and (out / "metadata.json").is_file(), "derived module files present")
    check((tmp / "out" / "f1_sigma_variant_receipt.json").is_file(), "receipt written")
    reloaded = W.load_module_state(out)
    ok, head2 = SV.constant_sigma_invariants(reloaded, sigma=0.005)
    check(ok and rep2["save_reload_exact"] and rep2["materialized"], "reload invariants")
    for k in state:
        if k not in ("pi.1.weight", "pi.1.bias"):
            check(np.array_equal(reloaded[k], state[k]) and reloaded[k].dtype == np.float32, f"{k} bit-exact float32")
    check(np.array_equal(reloaded["pi.1.weight"][:2], state["pi.1.weight"][:2]) and np.array_equal(reloaded["pi.1.bias"][:2], state["pi.1.bias"][:2]), "mean rows bit-exact")
    check(np.all(reloaded["pi.1.weight"][2:] == 0.0) and np.array_equal(reloaded["pi.1.bias"][2:], np.asarray(np.log([0.005, 0.005]), dtype=np.float32)), "logstd rows zero / bias ln 0.005")
    manifest = json.loads((out / "actor_feature_manifest.json").read_text())
    check(manifest["actor_digest"] == rep2["derived_actor_digest"] == W.actor_state_digest(reloaded) and manifest["exploration_sigma"] == [0.005, 0.005], "manifest digests")
    # no-clobber
    try:
        SV.derive_constant_sigma_module(src, tmp / "out", sigma=0.005, materialize=True)
        raise AssertionError("expected FileExistsError")
    except FileExistsError:
        check(True, "refuses to overwrite an existing derived dir")
    # wrong expected digest
    try:
        SV.derive_constant_sigma_module(src, tmp / "out2", sigma=0.005, expected_source_actor_digest="0" * 64)
        raise AssertionError("expected SigmaVariantError")
    except SV.SigmaVariantError:
        check(True, "wrong expected source digest refused")
    # module missing file
    bad = tmp / "bad" / "rl_module"
    write_module(bad, synthetic_state(35, seed=1))
    (bad / "metadata.json").unlink()
    try:
        SV.derive_constant_sigma_module(bad, tmp / "out3", sigma=0.005)
        raise AssertionError("expected SigmaVariantError")
    except SV.SigmaVariantError:
        check(True, "missing module file refused")
    # entropy helper
    check(abs(F1.entropy_diag_gauss([math.log(0.005)] * 2) - (-7.758757798)) < 1e-6, "entropy helper")
    print(f"SELFTEST PASS ({CHECKS} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
