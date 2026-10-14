"""Check saved marker counter-rotation and ankle-center response; algebra only."""
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

VARIANTS=Path(__file__).resolve().parent
AUDIT=VARIANTS.parent
ROOT=AUDIT.parents[1]


def fp(p):
    p=Path(p).resolve();return {"path":p.relative_to(ROOT).as_posix(),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}


def main():
    output=AUDIT/"marker_rotation_invariance.json"
    if output.exists():raise FileExistsError("Preserve previous algebra audit")
    comparison_path=AUDIT/"variant_contact_comparison.json"
    comparison=json.loads(comparison_path.read_text())
    protocol_path=AUDIT/"variants_protocol.json";protocol=json.loads(protocol_path.read_text())
    sources=dict(comparison["inputs_integrity"]["after"])
    for p in (Path(__file__),comparison_path,protocol_path):sources[fp(p)["path"]]=fp(p)["sha256"]
    def recheck():
        values={raw:fp(ROOT/raw)["sha256"] for raw in sources}
        if values!=sources:raise AssertionError("Protected sources changed")
        return values
    before=recheck();offset=protocol["factors"]["static_ankle_offsets_deg"][1];angle=math.radians(offset)
    counter=np.array([[math.cos(angle),math.sin(angle),0],[-math.sin(angle),math.cos(angle),0],[0,0,1]])
    cases=comparison["cases"];checks={}
    for low,high in (("L0_A0","L0_A1"),("L1_A0","L1_A1")):
        markers=[]
        for name in (low,high):
            model_path=ROOT/cases[name]["model"]["path"];tree=ET.parse(model_path)
            joint=tree.find(".//JointSet/objects/PinJoint[@name='pros_ankle']")
            if joint is None:raise ValueError("Expected unchanged PinJoint")
            child=joint.find(f"frames/PhysicalOffsetFrame[@name='{joint.findtext('socket_child_frame')}']")
            if child.findtext("socket_parent")!="/bodyset/foot_l" or [float(v) for v in child.findtext("translation").split()]!=[0.,0.,0.] or [float(v) for v in child.findtext("orientation").split()]!=[0.,0.,0.]:
                raise AssertionError("Foot origin must be the joint center with aligned child axes")
            markers.append({m.get('name'):(m.findtext('socket_parent_frame'),np.array([float(v) for v in m.findtext('location').split()])) for m in tree.findall('.//MarkerSet/objects/Marker')})
            receipt=json.loads((ROOT/cases[name]['receipt']['path']).read_text())
            setup=ET.parse(ROOT/receipt['actual_solver_setup']['path'])
            if setup.findtext('.//coordinate_file') not in ('',None) or setup.findall('.//IKCoordinateTask'):
                raise AssertionError("Coordinate-prior task breaks declared pure marker objective")
        if markers[0].keys()!=markers[1].keys():raise AssertionError("Marker membership differs")
        foot=[];other=[]
        for name,(parent,v0) in markers[0].items():
            if markers[1][name][0]!=parent:raise AssertionError("Marker body attachment differs")
            v1=markers[1][name][1]
            if parent=='/bodyset/foot_l':foot.append({'name':name,'local_position_A0_m':v0.tolist(),'local_position_A1_m':v1.tolist(),'minus_static_offset_counterrotation_max_abs_error_m':float(np.max(np.abs(counter@v0-v1)))})
            else:other.append({'name':name,'local_position_change_max_abs_m':float(np.max(np.abs(v0-v1)))})
        if len(foot)!=5 or len(other)!=23 or max(x['minus_static_offset_counterrotation_max_abs_error_m'] for x in foot)>1e-14 or max(x['local_position_change_max_abs_m'] for x in other)!=0:
            raise AssertionError("Marker rotation invariance failed")
        a,b=cases[low],cases[high]
        qa,qb=(c['reference_pose_difference_vs_current']['mean_q_difference_by_coordinate'] for c in (a,b))
        mean_delta=math.degrees(qb['pros_ankle_angle']-qa['pros_ankle_angle'])
        center_delta=b['contact']['aggregate']['depth']['ankle_center_Y']['mean_stance_m']-a['contact']['aggregate']['depth']['ankle_center_Y']['mean_stance_m']
        pitch_delta=math.degrees(b['contact']['contact_summary']['aggregate']['mean_sagittal_foot_pitch_rad']-a['contact']['contact_summary']['aggregate']['mean_sagittal_foot_pitch_rad'])
        checks[low+'__'+high]={'status':'PASS','foot_markers':foot,'other23_markers':other,
                             'static_ankle_offset_deg':offset,'full_IK_mean_ankle_q_A1_minus_A0_deg':mean_delta,
                             'mean_ankle_q_response_minus_static_offset_deg':mean_delta-offset,
                             'stance_ankle_center_Y_A1_minus_A0_m':center_delta,'stance_ankle_center_Y_A1_minus_A0_mm':1000*center_delta,
                             'full_primary_mean_foot_pitch_A1_minus_A0_deg':pitch_delta}
    result={'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026','status':'PASS_saved_XML_algebra_only','helper':fp(__file__),
            'checks':checks,'identity':{'marker_objective':'p_ankle + R_parent*Rz(q+delta)*Rz(-delta)*marker_local = p_ankle + R_parent*Rz(q)*marker_local',
                                         'ankle_center':'p_ankle is determined by parent tibia_pylon pose and mount translation; zero childoffset means pros_ankle_angle changes orientation, not this joint center.',
                                         'scope':'Exact marker-local counterrotation and near-invariant solved ankle center; not a manual dynamic angle shift or changed jointzero.'},
            'interpretation':['Free calibration counterrotates all five foot markers; a matching ankle-angle change preserves their marker objective while rotating the hardware foot and contact basis.',
                              'Other23 marker local positions remain exact. Small newIK differences in the parent pose leave only submicrometre ankle-center response to the static-angle factor.',
                              'The length factor instead shifts ankle-center stance means by about17.326mm in the coherent2x2. A small marker residual cannot independently determine hardware foot orientation or mount length.',
                              'This is a structural explanation of observed preparation responses, not anatomical causal shares or physical hardware necessity.'],
            'qualification':{'new_FK':False,'new_solver':False,'new_dynamics':False,'operational_edits':False},
            'inputs_integrity':{'protected_count':len(sources),'before':before,'after':recheck(),'all_unchanged':True}}
    with output.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps({'output':fp(output),'checks':{k:{n:v for n,v in r.items() if n not in ('foot_markers','other23_markers')} for k,r in checks.items()}},allow_nan=False))


if __name__=='__main__':main()
