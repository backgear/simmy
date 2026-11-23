#!/usr/bin/env python3
"""Screen recorded servo motion for prolonged no-response symptoms. Read-only.
This cannot recover an unrecorded external drive alarm or diagnose its cause.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

def suspicious_windows(position,effort,reference):
    condition=(np.abs(effort)>=1638)&(np.abs(reference-position)>=64)
    rolling=np.r_[0,np.cumsum(condition)]
    candidates=np.flatnonzero(rolling[100:]-rolling[:-100]==100)
    return [int(i) for i in candidates if np.ptp(position[i:i+100])<=2]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('runs',type=Path,nargs='+');ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);rows=[]
    for root in args.runs:
        for entry in json.loads((root/'history.json').read_text()):
            case=root/'transport'/f"trial-{entry['trial']:04d}"
            report=json.loads((case/'report.json').read_text())
            for sid,name in [('0','stream-0.i64le'),('2','stream-2.i32le')]:
                if hashlib.sha256((case/name).read_bytes()).hexdigest()!=report['streams'][sid]['sha256']:
                    raise ValueError('corrupt raw stream: '+str(case/name))
            p=np.fromfile(case/'stream-0.i64le',dtype='<i8')[::2];p=p-p[0]
            q=np.fromfile(case/'stream-2.i32le',dtype='<i4').reshape(-1,2)
            if len(p)!=len(q):raise ValueError('unaligned telemetry')
            rows.append(dict(run=root.name,trial=entry['trial'],split=entry['split'],role=entry.get('role','search'),
                suspect_window_starts_ms=suspicious_windows(p,q[:,0],q[:,1])))
    script=args.output/Path(__file__).name;shutil.copy2(Path(__file__),script)
    result=dict(hardware_recordings=len(rows),flagged_recordings=sum(bool(r['suspect_window_starts_ms']) for r in rows),
        criterion='100ms with >=5% commanded effort, >=64 count following error, <=2 count encoder excursion',rows=rows,
        source_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
        limitation='A symptom screen, not an external drive-alarm record or proof of continuous drive health. Exact fault trip time and cause remain unknown.')
    (args.output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Audited',len(rows),'hardware records; symptom-flagged:',result['flagged_recordings'])

if __name__=='__main__':main()
