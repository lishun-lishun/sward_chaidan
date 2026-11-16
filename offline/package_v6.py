"""Build developer archives and a FLAT official-format candidate submission."""
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
import hashlib
root=Path(__file__).resolve().parents[1]
out=root.parent
sol=root/'src'/'swarm_rescue'/'solutions'
files=[sol/n for n in ('blue_nav.py','blue_policy.py','my_drone_eval.py')]
files += [root/'README.md',root/'docs'/'V6_CODE_AGENT_PROMPT.md',root/'docs'/'V6_REPORT_AND_FORMULAS.md']
files += [x for x in sorted((root/'offline').glob('*.py')) if x.name!='patch_v6.py']

def make(name, pairs):
    path=out/name
    with ZipFile(path,'w',compression=ZIP_DEFLATED,compresslevel=9) as z:
        for src,dst in pairs:z.write(src,str(dst).replace('\\','/'))
    with ZipFile(path) as z:
        assert z.testzip() is None,path
        print(name,'files',len(z.namelist()),'bytes',path.stat().st_size)
    return path

# 1) Copy over src/swarm_rescue/solutions/ in a clone.
make('BLUE_V6_solutions_only.zip',[(f,f.relative_to(root)) for f in files[:3]])
# 2) Full bundle with docs and offline programs, excludes pycache and temp artifacts.
make('BLUE_V6_full_bundle.zip',[(f,f.relative_to(root)) for f in files])
# 3) Docs in flat zip so Markdown links are not essential.
make('BLUE_V6_docs_and_prompt.zip',[(f,f.name) for f in files[3:6]])
# 4) Official ZIP uses flat root paths; team metadata copied from current repo.
team=root/'team_info.yml';team.write_text('# Information about your team\nteam_number: 6\nteam_name: SkySaviors\nteam_members: tom, alice, bob, lea\n',encoding='utf8')
req=root/'req.txt';req.write_text('',encoding='utf8')
submission=make('team006_final.zip',[(f,f.name) for f in files[:3]]+
               [(team,'team_info.yml'),(req,'req.txt')])
with ZipFile(submission) as z:
    assert set(z.namelist()) == {'blue_nav.py','blue_policy.py','my_drone_eval.py','team_info.yml','req.txt'}
# Source-hashes are important for V5-V6 A/B reproducibility.
manifest=[]
for f in files[:3]:
    b=f.read_bytes()
    manifest.append(f'{f.name} SHA256 {hashlib.sha256(b).hexdigest()} bytes {len(b)}')
(out/'BLUE_V6_source_hashes.txt').write_text('\n'.join(manifest)+'\n',encoding='utf8')
print('\n'.join(manifest))
