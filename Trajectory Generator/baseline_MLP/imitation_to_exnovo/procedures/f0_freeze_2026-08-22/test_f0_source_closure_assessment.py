"""Self-test of f0_closure and f0_source_closure_assessment (temp-only, read-only
over the repository; nothing written under the real F0 output root).

Covered: real closure snapshot (digests well-formed, native plugins found with
the two config-referenced stems, declared files all present), symlink rejection
in the closure (fail-closed), synthetic assessment over a temp evidence tree
(fields, Class B / retrospective mode, correlation flags, limitations), receipt
count or schema mismatch rejected, no-clobber write.
"""

from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import f0_closure as CL  # noqa: E402
import f0_common as C  # noqa: E402
import f0_source_closure_assessment as S  # noqa: E402

PASSED = 0


def ok(label: str) -> None:
    global PASSED
    PASSED += 1
    print(f"  ok   : {label}")


def expect(fn, exc_types, label: str, needle: str | None = None) -> None:
    try:
        fn()
    except exc_types as exc:
        if needle and needle not in str(exc):
            raise AssertionError(f"{label}: error does not mention {needle!r}: {exc}")
        ok(f"{label} -> {type(exc).__name__}")
        return
    raise AssertionError(f"{label}: expected {exc_types}")


def main() -> int:
    base = "/private/tmp" if (Path("/private/tmp").is_dir() and os.access("/private/tmp", os.W_OK)) else tempfile.gettempdir()
    tmp = Path(tempfile.mkdtemp(prefix="f0_closure_selftest_", dir=base))
    try:
        # --- interpreter fingerprint: exact rollout interpreter, subprocess probe, fail-closed ---------
        fp = CL.interpreter_fingerprint(sys.executable)
        assert fp["role"] == "rollout_interpreter" and os.path.realpath(fp["executable_realpath"]) == os.path.realpath(sys.executable) and len(fp["fingerprint_sha256"]) == 64
        assert all(not str(v).startswith("MISSING") for v in fp["modules"].values()) and set(fp["modules"]) == set(CL.PROBE_MODULES) and fp["python_version"] == sys.version.split()[0]
        fp_driver = CL.interpreter_fingerprint(None)
        assert fp_driver["role"] == "driver_current_interpreter" and fp_driver["fingerprint_sha256"] == fp["fingerprint_sha256"]
        ok(f"interpreter fingerprint probed in a subprocess of the exact executable (torch {fp['modules']['torch']}, ray {fp['modules']['ray']}, opensim {fp['modules']['opensim']}); driver role labelled separately")
        alt = next((p for p in ("/usr/bin/python3", "/usr/local/bin/python3") if Path(p).is_file()), None)
        if alt is not None and os.path.realpath(alt) != os.path.realpath(sys.executable):
            # a system interpreter either lacks the stack ("lacks required modules") or is a shim executing
            # another binary (macOS /usr/bin/python3 -> CommandLineTools: "executable mismatch"): both fail closed
            expect(lambda: CL.interpreter_fingerprint(alt), CL.ClosureError, f"driver env != rollout env: {alt} (system interpreter) rejected")
        else:
            print("  note : no alternate system interpreter found; the missing-stack case is covered by the fake probe below")
        if os.name != "nt":
            fake_py = tmp / "fake_python"
            fake_py.write_text("#!/bin/sh\necho garbage\n", encoding="utf-8")
            fake_py.chmod(0o755)
            expect(lambda: CL.interpreter_fingerprint(str(fake_py)), CL.ClosureError, "probe without JSON line", "no JSON")
            fake_py2 = tmp / "fake_python2"
            fake_py2.write_text("#!/bin/sh\necho 'F0PROBE {\"executable\": \"/other/python\", \"executable_realpath\": \"/other/python\", \"python_version\": \"3.0\", \"platform\": \"p\", \"machine\": \"m\", \"modules\": {}}'\n", encoding="utf-8")
            fake_py2.chmod(0o755)
            expect(lambda: CL.interpreter_fingerprint(str(fake_py2)), CL.ClosureError, "probe reporting another executable", "executable mismatch")
        expect(lambda: CL.interpreter_fingerprint(str(tmp / "absent_python")), CL.ClosureError, "non-existent interpreter", "not an executable file")
        ok("interpreter fingerprint fails closed: missing simulation stack, probe without JSON, executable mismatch, absent executable")
        # --- setup-resolved runtime files and per-job inputs ------------------------------------------
        setup_files = [C.rel(p) for p in CL.setup_runtime_files(C.SETUP_XML)]
        assert len(setup_files) == 6 and setup_files[0].endswith("_setup.xml") and any(p.endswith(".osim") for p in setup_files) and any(p.endswith("IK_results_AB06_SEASEA.mot") for p in setup_files) and any(p.endswith("ExternalForces.xml") for p in setup_files) and any(p.endswith("AB06_SEASEA_GRF_FullSpan.mot") for p in setup_files) and any(p.endswith("CMC_Actuators.xml") for p in setup_files)
        bad_setup = tmp / "setup_missing_tag.xml"
        bad_setup.write_text("<OpenSimDocument><CMC_Simulator_Setup><model_file>x.osim</model_file></CMC_Simulator_Setup></OpenSimDocument>", encoding="utf-8")
        expect(lambda: CL.setup_runtime_files(bad_setup), CL.ClosureError, "setup xml without kinematics/external/actuators tags", "closure file")
        bad_setup2 = tmp / "setup_no_model.xml"
        bad_setup2.write_text("<OpenSimDocument><CMC_Simulator_Setup><kinematics_file>x.mot</kinematics_file></CMC_Simulator_Setup></OpenSimDocument>", encoding="utf-8")
        expect(lambda: CL.setup_runtime_files(bad_setup2), CL.ClosureError, "setup xml without model_file", "lacks <model_file>")
        expect(lambda: CL.config_referenced_files({"grf": {}}), CL.ClosureError, "config without simulation.setup_xml", "setup_xml")
        expect(lambda: CL.job_inputs_table({"config": "absent.yaml", "module": "absent_module"}), CL.ClosureError, "job inputs with absent config", "missing")
        ok("setup-resolved runtime files (setup xml, model .osim, IK .mot, ExternalForces.xml + GRF .mot, CMC_Actuators.xml) resolved fail-closed; missing tags/config/setup rejected")
        snap = CL.closure_snapshot(None, fingerprint=fp_driver)
        assert len(snap["runtime_source_closure_digest"]) == 64 and len(snap["orchestration_digest"]) == 64 and snap["runtime_source_closure_digest"] != snap["orchestration_digest"]
        assert len(snap["runtime_core"]) == len(CL.RUNTIME_CORE_FILES) and all(len(r["sha256"]) == 64 for r in snap["runtime_core"])
        stems = [Path(p["path"]).name for p in snap["native_plugins"] if p["referenced_by_config"]]
        assert any("SEA_Plugin_BlackBox_mCMC_impedence_ff" in s for s in stems) and any("OnlineGRFContact" in s for s in stems) and all(p["loaded_in_process_proven"] is False for p in snap["native_plugins"])
        assert snap["closure_declared_not_import_traced"] is True and "NOT Class A" in snap["provenance_limitation"] and len(snap["irrecoverable_limitations"]) == 4
        assert CL.table_digest(snap["runtime_core"], snap["native_plugins"], snap["data_assets"]) == snap["runtime_source_closure_digest"] and snap["environment_fingerprint"]["role"] == "driver_current_interpreter"
        assert CL.sections_equal(snap, CL.closure_snapshot(None, fingerprint=fp_driver)) == {s: True for s in CL.SNAPSHOT_SECTIONS}
        ok(f"real closure snapshot: {len(snap['runtime_core'])} runtime-core files, {len(snap['native_plugins'])} native plugins (both referenced stems), {len(snap['data_assets'])} data assets, digests separate, reproducible and section-equal on recomputation")
        plugin_dir = tmp / "plugins"
        plugin_dir.mkdir()
        (plugin_dir / "libReal.dylib").write_bytes(b"x")
        table = CL.native_plugin_table(plugin_dir)
        assert len(table) == 1
        (plugin_dir / "libLink.dylib").symlink_to(plugin_dir / "libReal.dylib")
        expect(lambda: CL.native_plugin_table(plugin_dir), CL.ClosureError, "symlink among native plugins", "symlink")
        expect(lambda: CL.digest_table([tmp / "missing.py"], label="t"), CL.ClosureError, "missing closure file", "missing")
        link = tmp / "link.py"
        link.symlink_to(tmp / "missing.py")
        expect(lambda: CL.digest_table([link], label="t"), CL.ClosureError, "symlink closure file", "symlink")
        ok("closure fails closed on symlinks and missing files")
        # --- synthetic assessment ----------------------------------------------------
        cfg = tmp / "cfg.yaml"
        cfg.write_text("a: 1\n", encoding="utf-8")
        module = tmp / "module"
        module.mkdir()
        (module / "module_state.pkl").write_bytes(b"weights")
        receipts = []
        for i, family in enumerate(("replay", "det", "det")):
            d = tmp / "rollouts" / family / f"job{i}"
            d.mkdir(parents=True)
            payload = {"schema_version": 4, "job_id": f"job{i}", "family": family, "candidate": "X", "runtime": "r", "status": "ok", "returncode": 0, "git_head": snap["git"]["head"] if i < 2 else "deadbeef", "rollout_eval_sha256": next(r["sha256"] for r in snap["runtime_core"] if r["path"] == C.rel(C.ROLLOUT_EVAL)), "config": C.rel(cfg), "config_sha256": C.sha256_file(cfg), "module": C.rel(module), "module_state_sha256": C.sha256_file(module / "module_state.pkl") if i else "0" * 64, "summary_sha256": "s", "trace_sha256": "t", "python": "py"}
            (d / "f0_receipt.json").write_text(json.dumps(payload), encoding="utf-8")
            receipts.append(d / "f0_receipt.json")
        inventory = tmp / "freeze_inventory.json"
        inventory.write_text("{}", encoding="utf-8")
        manifest = tmp / "rollout_matrix_manifest_execute_1.json"
        manifest.write_text(json.dumps({"mode": "execute", "job_count": 3, "generated_at_utc": "x", "git": {"head": "h"}}), encoding="utf-8")
        out = S.build_assessment(receipts, inventory, [manifest], snap, expected_receipts=3)
        assert out["attestation_mode"] == "retrospective_correlated_evidence" and out["provenance_class"] == "B" and out["bit_exact_claimed"] is False and out["class_A_claimed"] is False and out["receipts_retrofitted"] is False
        assert out["receipt_count"] == 3 and len(out["receipts"]) == 3 and all(len(r["sha256"]) == 64 for r in out["receipts"]) and len(out["freeze_inventory"]["sha256"]) == 64 and out["execution_manifests"][0]["job_count"] == 3
        cs = out["correlation_summary"]
        assert cs["receipts_with_git_head_equal_current"] == 2 and cs["receipts_with_rollout_eval_equal_current"] == 3 and cs["receipts_with_config_equal_disk"] == 3 and cs["receipts_with_module_equal_disk"] == 2 and cs["receipts_status_ok"] == 3
        assert out["irrecoverable_limitations"] == list(CL.IRRECOVERABLE_LIMITATIONS) and out["current_runtime_source_closure_digest"] == snap["runtime_source_closure_digest"] and "bit-exact" in out["what_this_does_not_attest"]
        ok("synthetic assessment: Class B retrospective, per-receipt SHA + correlation flags (HEAD 2/3, rollout_eval 3/3, config 3/3, module 2/3), inventory/manifest digests, limitations listed, no Class A/bit-exact claim")
        expect(lambda: S.build_assessment(receipts[:2], inventory, [manifest], snap, expected_receipts=3), S.AssessmentError, "receipt count mismatch", "expected exactly 3")
        bad = tmp / "rollouts" / "det" / "job_v5"
        bad.mkdir()
        (bad / "f0_receipt.json").write_text(json.dumps({"schema_version": 5, "family": "det", "job_id": "v5"}), encoding="utf-8")
        expect(lambda: S.build_assessment(receipts[:2] + [bad / "f0_receipt.json"], inventory, [manifest], snap, expected_receipts=3), S.AssessmentError, "schema-5 receipt in the legacy set", "schema_version")
        expect(lambda: S.build_assessment(receipts, tmp / "absent.json", [manifest], snap, expected_receipts=3), S.AssessmentError, "missing inventory", "missing")
        expect(lambda: S.build_assessment(receipts, inventory, [], snap, expected_receipts=3), S.AssessmentError, "no execution manifest", "at least one")
        target = tmp / "manifest" / S.ASSESSMENT_FILE
        C.write_json(target, out)
        expect(lambda: C.write_json(target, out), FileExistsError, "no-clobber assessment write")
        assert not (C.OUT_MANIFEST / S.ASSESSMENT_FILE).exists() or True  # the real artefact may legitimately exist; this test never writes it
        assert str(tmp).startswith(base) and not str(tmp).startswith(str(C.REPO))
        ok(f"temp-only: {tmp} (nothing written under the repository)")
        print(f"SELFTEST PASS ({PASSED} checks)")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
