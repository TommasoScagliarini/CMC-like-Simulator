from pathlib import Path
import subprocess,json,hashlib,datetime
base=Path(__file__).resolve().parent
cmd=['/Users/tommy/.local/bin/claude','--print','--model','claude-opus-5-5','--effort','max','--restricted','--tools','Read,Grep,Glob','--allowedTools','Read,Grep,Glob','--disable-slash-commands','--strict-mcp-config','--mcp-config','{"mcpServers":{}}','--setting-sources','','--no-session-persistence','--output-format','json']
prompt=base.joinpath('prompt.txt').read_text()
receipt={'requested_model':'claude-opus-5-5','requested_effort':'max','fallback_allowed':False,'command':cmd,'input_sha256':hashlib.sha256(prompt.encode()).hexdigest(),'start_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
base.joinpath('claude_receipt.json').write_text(json.dumps(receipt,indent=2))
with base.joinpath('claude_result.json').open('w') as out, base.joinpath('claude_stderr.txt').open('w') as err:
 p=subprocess.run(cmd,input=prompt,text=True,stdout=out,stderr=err,cwd=str(base.parents[3]))
receipt['returncode']=p.returncode
receipt['end_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
try:
 r=json.loads(base.joinpath('claude_result.json').read_text())
 receipt['modelUsage']=r.get('modelUsage');receipt['is_error']=r.get('is_error');receipt['permission_denials']=r.get('permission_denials')
 receipt['model_verified']= bool(r.get('modelUsage')) and all(v.get('canonicalModel',k)=='claude-opus-5-5' for k,v in r.get('modelUsage',{}).items())
 base.joinpath('claude_response.txt').write_text(r.get('result',''))
except Exception as e: receipt['parse_error']=str(e)
base.joinpath('claude_receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2))
