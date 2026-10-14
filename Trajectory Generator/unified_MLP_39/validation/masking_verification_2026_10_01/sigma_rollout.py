"""Evaluate the exact diagnostic subclass while retaining every structural check."""
import json
from pathlib import Path
import sys
from types import FunctionType

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
sys.path.insert(0,str(PACKAGE.parent))

def install():
    from unified_MLP_39 import actor_transition,rollout_eval
    from unified_MLP_39.validation.masking_verification_2026_10_01.sigma_module import SigmaComparisonModule
    original=actor_transition.validate_module_contract
    # Reuse the validator bytecode with only its exact-class allowlist changed.
    # Nested contract/tensor validators retain their original module globals.
    namespace=dict(original.__globals__)
    namespace['MODULE_CLASS']=SigmaComparisonModule.__module__+'.'+SigmaComparisonModule.__name__
    checked=FunctionType(original.__code__,namespace,original.__name__,original.__defaults__,original.__closure__)
    expected=json.loads((HERE/'preflight_sigma.json').read_text())['modules'][0]['reference_sha256']
    def validate(module,contract):
        if type(module) is not SigmaComparisonModule:
            raise ValueError('This diagnostic evaluator requires SigmaComparisonModule')
        checked(module,contract)
        audit=module.sigma_audit([[0.]*module._n_actor])
        if not audit['reference_ready'] or audit['reference_sha256']!=expected:
            raise ValueError('Diagnostic module lost its original CP90 sigma reference')
        treatment=int(module.model_config['diagnostic_sigma_treatment']=='frozen_cp90')
        if audit['treatment']!=treatment:
            raise ValueError('Sigma treatment differs from exported model configuration')
        print(json.dumps({'diagnostic_contract_audit':audit}),flush=True)
    actor_transition.validate_module_contract=validate
    rollout_eval._SCRIPT_PATH=Path(__file__).resolve()
    return rollout_eval

if __name__=='__main__':install().main()
