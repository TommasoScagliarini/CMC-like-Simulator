"""Fail-closed tests for the V26C penetration contract.

Pure offline: no environment, no rollout, no fit, no training. Every historical artefact is read
only; mutations are proven on temporary copies and the originals are hashed before and after.
"""
from __future__ import annotations
import json, shutil, sys, tempfile
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import v26c_penetration_contract as PC  # noqa: E402

CHECKS = 0


def check(c, w):
    global CHECKS
    assert c, w
    CHECKS += 1


def expect(fn, exc, w):
    global CHECKS
    try:
        fn()
    except exc as e:
        CHECKS += 1
        return e
    raise AssertionError(f"expected {exc.__name__}: {w}")


def main() -> int:
    contract = PC.load_contract()
    before = {rel: PC._sha_file(HERE / rel)
              for rel in contract["data"]["pinned_artefacts_sha256"]}

    # ---------------------------------------------------------------- the contract --------------
    check(contract["soft_m"] == 0.020 and contract["july_legacy_m"] == 0.025
          and contract["hard_m"] == 0.028, "the three bands are 0.020 / 0.025 / 0.028")
    d = contract["data"]
    check(d["bands"]["soft_diagnostic"]["binding"] is False
          and d["bands"]["july_legacy_hard_reference"]["binding"] is False
          and d["bands"]["hard_binding"]["binding"] is True,
          "exactly ONE band is binding")
    check(d["bands"]["soft_diagnostic"]["never_a_blocker"] is True
          and d["bands"]["july_legacy_hard_reference"]["never_a_blocker"] is True,
          "the soft and July legacy bands are declared never-blockers")
    check(d["runtime_unchanged"]["grf_penetration_penalty_threshold_m"] == 0.020
          and d["runtime_unchanged"]["grf_penetration_termination_m"] == 0.028
          and d["runtime_unchanged"]["pinned_config_sha256"].startswith("a870cc38"),
          "the runtime reward threshold and termination stay 0.020 / 0.028")
    dnd = d["does_not_do"]
    check(dnd["confers_no_deployability"] is True and dnd["confers_no_promotion"] is True
          and dnd["authorises_no_next_stage"] is True
          and dnd["authorises_no_rollout_fit_or_training"] is True,
          "the contract confers nothing and authorises nothing")
    check("STAND exactly as recorded" in dnd["does_not_rewrite_historical_verdicts"],
          "and never rewrites a historical verdict")
    check(contract["pins_verified_count"] == len(before) == 11,
          f"all {len(before)} pinned artefacts verify")

    # ---------------------------------------------------------------- boundaries ----------------
    cases = [
        (0.019999, False, False, False, PC.BAND_WITHIN_SOFT, True),
        (0.020000, False, False, False, PC.BAND_WITHIN_SOFT, True),
        (0.020001, True,  False, False, PC.BAND_ABOVE_SOFT, True),
        (0.025000, True,  True,  False, PC.BAND_LEGACY, True),
        (0.025001, True,  True,  False, PC.BAND_LEGACY, True),
        (0.028000, True,  True,  False, PC.BAND_LEGACY, True),
        (0.028001, True,  True,  True,  PC.BAND_ABOVE_HARD, False),
    ]
    for value, soft, legacy, hard, band, binding in cases:
        r = PC.classify(value, contract)
        f = r["flags"]
        check(f["above_soft_diagnostic"] is soft, f"{value}: above_soft is {soft}")
        check(f["july_legacy_breach"] is legacy, f"{value}: july_legacy_breach is {legacy}")
        check(f["july_legacy_historical_pass"] is (not legacy),
              f"{value}: July's historical pass (< 0.025) is {not legacy}")
        check(f["above_hard_binding"] is hard, f"{value}: above_hard is {hard}")
        check(r["band"] == band, f"{value}: band is {band}")
        check(r["binding_pass"] is binding
              and r["binding_verdict"] == ("PASS" if binding else "FAIL"),
              f"{value}: binding verdict is {'PASS' if binding else 'FAIL'}")
    check(PC.classify(0.020, contract)["flags"]["above_soft_diagnostic"] is False
          and PC.classify(0.025, contract)["flags"]["july_legacy_breach"] is True
          and PC.classify(0.028, contract)["binding_pass"] is True,
          "the three exact boundaries behave as specified: 0.020 not above, 0.025 breach, "
          "0.028 pass")

    # ---------------------------------------------------------------- series counting -----------
    series = [0.0, 0.020, 0.020001, 0.024999, 0.025, 0.028, 0.0279]
    ev = PC.evaluate_series(series, contract)
    check(ev["samples"] == 7 and ev["max_penetration_m"] == 0.028,
          "the series maximum is taken over every sample")
    check(ev["counts"]["above_soft_diagnostic"] == 5,
          f"5 samples are strictly above 0.020, got {ev['counts']['above_soft_diagnostic']}")
    check(ev["counts"]["at_or_above_july_legacy"] == 3,
          f"3 samples are at or above 0.025, got {ev['counts']['at_or_above_july_legacy']}")
    check(ev["counts"]["above_hard_binding"] == 0, "none is strictly above 0.028")
    check(ev["binding_pass"] is True and ev["band"] == PC.BAND_LEGACY,
          "so the series passes the only binding band")
    check(PC.evaluate_series([0.0281], contract)["binding_pass"] is False,
          "while a single sample above 0.028 fails")
    check(ev["counting_conventions"]["above_soft_diagnostic"] == "strictly > 0.020"
          and ev["counting_conventions"]["at_or_above_july_legacy"] == ">= 0.025"
          and ev["counting_conventions"]["above_hard_binding"] == "strictly > 0.028",
          "and the conventions are stated in the result")

    # ---------------------------------------------------------------- degenerate inputs ---------
    for label, bad in (("NaN", [0.01, float("nan")]), ("+inf", [float("inf")]),
                       ("-inf", [float("-inf")]), ("negative", [0.01, -1e-9]),
                       ("empty", [])):
        expect(lambda b=bad: PC.evaluate_series(b, contract), PC.ContractError,
               f"a {label} series must be refused")
    for label, bad in (("NaN", float("nan")), ("inf", float("inf")), ("negative", -1e-12)):
        expect(lambda b=bad: PC.classify(b, contract), PC.ContractError,
               f"a {label} maximum must be refused")
    expect(lambda: PC.evaluate_series([[0.01, 0.02], [0.03, 0.04]], contract), PC.ContractError,
           "a two-dimensional series must be refused")

    # ---------------------------------------------------------------- pin mutation --------------
    saved = PC.PIN_CONTRACT
    PC.PIN_CONTRACT = "0" * 64
    e = expect(PC.load_contract, PC.ContractError, "a changed contract hash must be refused")
    check("contract changed" in str(e), "and the failure says the contract changed")
    PC.PIN_CONTRACT = saved
    check(PC.load_contract()["sha256"] == saved, "and it loads again with the true pin")

    with tempfile.TemporaryDirectory() as td:
        # a mutated PINNED ARTEFACT must abort - proven with a forged contract in a temp dir
        forged = Path(td) / "forged_contract.json"
        data = json.loads(PC.CONTRACT_FILE.read_text())
        data["pinned_artefacts_sha256"]["v26c_j1_amendment_soft_fail.json"] = "0" * 64
        forged.write_text(json.dumps(data, indent=2), encoding="utf-8")
        saved_file, saved_pin = PC.CONTRACT_FILE, PC.PIN_CONTRACT
        try:
            PC.CONTRACT_FILE, PC.PIN_CONTRACT = forged, PC._sha_file(forged)
            e = expect(PC.load_contract, PC.ContractError,
                       "a pin that no longer matches its file must abort")
            check("v26c_j1_amendment_soft_fail.json changed" in str(e),
                  "naming the artefact that moved")
            missing = json.loads(PC.CONTRACT_FILE.read_text())
            missing["pinned_artefacts_sha256"] = {"does_not_exist.json": "0" * 64}
            forged.write_text(json.dumps(missing, indent=2), encoding="utf-8")
            PC.PIN_CONTRACT = PC._sha_file(forged)
            e = expect(PC.load_contract, PC.ContractError, "a missing pinned artefact must abort")
            check("which is missing" in str(e), "saying it is missing")
            for label, mutate in (
                    ("a moved band value",
                     lambda x: x["bands"]["hard_binding"].__setitem__("value_m", 0.030)),
                    ("a binding soft band",
                     lambda x: x["bands"]["soft_diagnostic"].__setitem__("binding", True)),
                    ("a non-binding hard band",
                     lambda x: x["bands"]["hard_binding"].__setitem__("binding", False)),
                    ("a contract that claims deployability",
                     lambda x: x["does_not_do"].__setitem__("confers_no_deployability", False)),
                    ("an empty pin set",
                     lambda x: x.__setitem__("pinned_artefacts_sha256", {})),
                    ("a wrong schema", lambda x: x.__setitem__("schema", "other")),
            ):
                obj = json.loads(PC.CONTRACT_FILE.read_text())
                obj = json.loads(json.dumps(json.loads(
                    (HERE / "v26c_penetration_contract_2026-08-26.json").read_text())))
                mutate(obj)
                forged.write_text(json.dumps(obj, indent=2), encoding="utf-8")
                PC.PIN_CONTRACT = PC._sha_file(forged)
                expect(PC.load_contract, PC.ContractError, f"{label} must be refused")
            forged.write_text("{not json", encoding="utf-8")
            PC.PIN_CONTRACT = PC._sha_file(forged)
            expect(PC.load_contract, PC.ContractError, "invalid JSON must be refused")
        finally:
            PC.CONTRACT_FILE, PC.PIN_CONTRACT = saved_file, saved_pin
    check(PC.load_contract()["pins_verified_count"] == 11,
          "and the real contract verifies clean afterwards")

    # ---------------------------------------------------------------- reconciliation ------------
    rec = PC.reconcile(contract)
    check(sorted(rec["runs"]) == ["J1", "J3", "J5-r2"], "all three runs are reconciled")
    check(rec["kind"].startswith("INFORMATIONAL"), "the record is informational")
    expected = {"J1": (0.02294380435912411, 97, "FAIL"),
                "J3": (0.02704966381076714, 106, "FAIL"),
                "J5-r2": (0.026913284071471633, 106, "FAIL")}
    for name, (mx, above, verdict) in expected.items():
        run = rec["runs"][name]
        p = run["penetration_under_new_contract"]
        check(abs(p["max_penetration_m"] - mx) < 1e-15,
              f"{name}: max {mx} recomputed from the pinned trace")
        check(p["counts"]["above_soft_diagnostic"] == above,
              f"{name}: {above} samples above the soft band")
        check(p["samples"] == 500, f"{name}: 500 samples")
        check(run["historical"]["verdict"] == verdict
              and run["historical"]["status"].startswith("UNCHANGED"),
              f"{name}: the historical verdict {verdict} is carried unchanged")
        u = run["unchanged_by_this_record"]
        if name == "J1":
            check(all(str(u[k]).startswith("ABSENT")
                      for k in ("deployable", "promotion", "next_stage_authorized")),
                  "J1: the collection receipt declares no deployability/promotion fields, and "
                  "the record says ABSENT rather than inventing False")
        else:
            check(u["deployable"] is False and u["promotion"] == "NONE"
                  and u["next_stage_authorized"] is False,
                  f"{name}: deployability, promotion and next stage are unchanged")
        check("never invented" in u["note"], f"{name}: the reporting rule is stated")
    check(all(r["penetration_under_new_contract"]["binding_pass"] is True
              for r in rec["runs"].values()),
          "all three sit at or below the 0.028 binding band")
    check(rec["runs"]["J1"]["penetration_under_new_contract"]["band"] == PC.BAND_ABOVE_SOFT
          and rec["runs"]["J3"]["penetration_under_new_contract"]["band"] == PC.BAND_LEGACY
          and rec["runs"]["J5-r2"]["penetration_under_new_contract"]["band"] == PC.BAND_LEGACY,
          "J1 lands above soft, J3 and J5-r2 in the July legacy band")
    for name in ("J1", "J3", "J5-r2"):
        b = rec["runs"][name]["behavioural_under_new_contract"]
        check(b["demonstrable"] is True and b["other_failed_checks"] == [],
              f"{name}: no non-penetration criterion had failed, so the outcome is demonstrable")
        check(b["behavioural_pass_under_new_contract"] is True,
              f"{name}: its behavioural gate would pass under the new contract")
    check(rec["statements"]["authorises"].startswith("NOTHING"),
          "and the record authorises nothing")

    # ------------------------------------------- determinism -----------------------------------
    first = json.dumps(PC.reconcile(contract), sort_keys=True)
    second = json.dumps(PC.reconcile(contract), sort_keys=True)
    check(first == second,
          "two reconcile() serialisations are BYTE-IDENTICAL: the record is a pure function of "
          "the pinned inputs")
    check("generated_at_utc" not in rec and "generated_at_utc" not in first,
          "no wall-clock timestamp survives anywhere in the record")
    check(rec["contract_effective_date_utc"] == contract["data"]["date_utc"] == "2026-08-26",
          "the only date is the contract's own effective date")
    check(rec["determinism"]["wall_clock_free"] is True,
          "and the record declares itself wall-clock free")

    # ------------------------------------------- statements distinguish ABSENT from false -------
    st = rec["statements"]
    for field in ("deployability", "promotion", "next_stage"):
        check(st[field].startswith("UNCHANGED."), f"{field}: the statement opens with UNCHANGED")
        check("J3, J5-r2" in st[field] and "J1" in st[field],
              f"{field}: it names which runs carried a value and which did not")
        check("ABSENT - never declared by the receipt schema - in: J1" in st[field],
              f"{field}: J1 is reported ABSENT, not false")
        check("Absent is NOT the same as an explicit false" in st[field],
              f"{field}: the distinction is stated")
        check("everywhere" not in st[field],
              f"{field}: no blanket 'everywhere' claim survives")
    check("Recorded as False in: J3, J5-r2" in st["deployability"]
          and "Recorded as 'NONE' in: J3, J5-r2" in st["promotion"]
          and "Recorded as False in: J3, J5-r2" in st["next_stage"],
          "the values J3 and J5-r2 actually recorded are quoted exactly")
    check("NOT CONFERRED BY THAT SCHEMA" in st["absent_is_not_false"],
          "and the record explains what ABSENT means")

    # ---------------------------------------------------------------- CLI, no-clobber -----------
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "rec.json"
        check(PC.main(["--reconcile", "--out", str(out)]) == 0, "the CLI writes a reconciliation")
        check(out.is_file() and json.loads(out.read_text())["schema"]
              == "v26c_penetration_reconciliation.1", "with the expected schema")
        expect(lambda: PC.main(["--reconcile", "--out", str(out)]), PC.ContractError,
               "no-clobber: an existing output is never overwritten")
        expect(lambda: PC.main(["--reconcile"]), PC.ContractError,
               "--reconcile requires an explicit --out")

    # ---------------------------------------------------------------- artefacts untouched ------
    after = {rel: PC._sha_file(HERE / rel)
             for rel in contract["data"]["pinned_artefacts_sha256"]}
    check(after == before == contract["data"]["pinned_artefacts_sha256"],
          "MEASURED: every pinned historical artefact is byte-identical after the whole suite")
    for rel in ("v26c_j1_collect.py", "v26c_j3_closed_loop.py", "v26c_j4_recovery.py",
                "v26c_j5_revalidation.py", "v26c_j0_audit.py"):
        check((HERE / rel).is_file(), f"{rel} still exists and was not required to change")

    print(json.dumps({"selftest": "PASS", "checks": CHECKS}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
