"""Execute OFFLINE local competition-mode scenarios, without editing engine/config.

The script creates temporary YAML plans OUTSIDE the official repository and
writes result logs to an external directory. It never enters competition code.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path


def parse_output(output):
    exprs={
        'score':r'round score:\s*([\d.]+)%',
        'disposed':r'rescued nb:\s*(\d+)\s*/\s*(\d+)',
        'health':r'health return score:\s*([\d.]+)%',
        'time_score':r'time score:\s*([\d.]+)%',
        'steps':r'elapse timestep:\s*(\d+)\s*/\s*(\d+)',
        'destroyed_percent':r'percentage of drones destroyed:\s*([\d.]+)',
    }
    values={}
    for key,pat in exprs.items():
        found=re.findall(pat,output,flags=re.I)
        if found:
            val=found[-1]
            values[key]=[float(v) for v in val] if isinstance(val,tuple) else float(val)
    return values


def write_yaml(scenario, path):
    zones=scenario.get('zones_config',[])
    rows=['stat_saving_enabled: false','state_recording_enabled: false',
          'video_capture_enabled: false','time_best_ratio: 0.3',
          'evaluation_plan:','  - map_name: '+scenario['map_name'],
          '    team_mode: rescue','    nb_rounds: 1','    config_weight: 1',
          '    number_drones: 10',
          '    zones_config: ['+', '.join(zones)+']']
    path.write_text('\n'.join(rows)+'\n',encoding='utf8')


def run(repo, cases_dir, output_dir, max_cases=None, include=None,
        timeout=600, repeats=1, python_bin=sys.executable):
    output_dir.mkdir(parents=True,exist_ok=True)
    case_files=sorted(cases_dir.glob('Map*.json'))
    if include:
        case_files=[f for f in case_files if any(q in f.stem for q in include)]
    if max_cases:
        case_files=case_files[:max_cases]
    all_results=[]
    for file in case_files:
        case=json.loads(file.read_text(encoding='utf8'))
        for repeat in range(repeats):
            with tempfile.TemporaryDirectory(prefix='swarm_eval_') as temp:
                config=Path(temp)/'plan.yml'
                write_yaml(case,config)
                command=[python_bin,'-m','swarm_rescue.launcher','--competition',
                         '--headless','-c',str(config),'--bombs-file',str(file.resolve())]
                environ=os.environ.copy()
                pkg=str((repo/'src').resolve())
                environ['PYTHONPATH']=pkg+os.pathsep+environ.get('PYTHONPATH','')
                try:
                    result=subprocess.run(command,cwd=repo,env=environ,
                                          capture_output=True,text=True,timeout=timeout)
                    log=(result.stdout or '')+'\n'+(result.stderr or '')
                    ret=result.returncode
                except subprocess.TimeoutExpired as ex:
                    log=(ex.stdout or b'').decode(errors='replace') if isinstance(ex.stdout,bytes) else (ex.stdout or '')
                    ret=124
                logfile=output_dir/f'{file.stem}_r{repeat}.log'
                logfile.write_text(log,encoding='utf8')
                stats=parse_output(log)
                stats.update(case=file.stem,repeat=repeat,exit_code=ret,log=str(logfile))
                a,b=stats.get('disposed',[0,10])
                stats['meets_disposal_goal']=(a==b==10)
                stats['meets_score_goal']=(file.stem.startswith('Map01_') or stats.get('score',0)>=92) and stats['meets_disposal_goal']
                all_results.append(stats)
                print(f"{file.stem} #{repeat}: {a:g}/{b:g}, score={stats.get('score','NA')}, status={ret}",flush=True)
                (output_dir/'results.json').write_text(json.dumps(all_results,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print('RESULTS',sum(r['meets_score_goal'] for r in all_results),'/',len(all_results))
    return all_results


if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo',type=Path,default=Path('.'))
    ap.add_argument('--cases',type=Path,default=Path('bench_cases'))
    ap.add_argument('--output',type=Path,default=Path('bench_results'))
    ap.add_argument('--include',nargs='*',help='Partial filename filters, e.g. Map01_fixed Map06')
    ap.add_argument('--max-cases',type=int)
    ap.add_argument('--timeout',type=int,default=600)
    ap.add_argument('--repeats',type=int,default=1)
    ap.add_argument('--python',default=sys.executable)
    args=ap.parse_args()
    run(args.repo.resolve(),args.cases.resolve(),args.output.resolve(),args.max_cases,
        args.include,args.timeout,args.repeats,args.python)
