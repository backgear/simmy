#!/usr/bin/env python3
"""Deterministic immutable capture archives with compressed AND file checksums.

Includes raw RX/TX, frame arrival logs, plans, metadata, raw samples and derived
outputs/source snapshots. Omits only native shared libraries and Python caches;
model kernels rebuild from the versioned C source. Does not stage/commit Git.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import tarfile


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source',type=Path);ap.add_argument('output',type=Path);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    groups={d.name:sorted(p for p in d.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.so')
            for d in sorted(args.source.iterdir()) if d.is_dir()}
    groups['session-metadata']=sorted(p for p in args.source.iterdir() if p.is_file())
    archives=[]
    for name,files in groups.items():
        target=args.output/(name+'.tar.gz');inventory=[]
        with target.open('wb') as raw,gzip.GzipFile(filename='',mode='wb',fileobj=raw,mtime=0,compresslevel=6) as gz:
            with tarfile.open(fileobj=gz,mode='w',format=tarfile.PAX_FORMAT) as tar:
                for p in files:
                    arc=(str(Path(name)/p.relative_to(args.source/name)) if name!='session-metadata'
                         else str(p.relative_to(args.source)))
                    info=tar.gettarinfo(str(p),arcname=arc)
                    info.mtime=0;info.uid=info.gid=0;info.uname=info.gname='';info.pax_headers={}
                    info.mode=0o644
                    with p.open('rb') as f:tar.addfile(info,f)
                    inventory.append(dict(path=arc,bytes=p.stat().st_size,sha256=sha(p)))
        archives.append(dict(archive=target.name,bytes=target.stat().st_size,sha256=sha(target),
                             unpacked_bytes=sum(p['bytes'] for p in inventory),files=inventory))
    manifest=dict(schema='axis-bench-capture-archives-v1',source=str(args.source),
                  omitted=['native .so binaries (rebuildable)','__pycache__'],archives=archives,
                  total_compressed_bytes=sum(a['bytes'] for a in archives),
                  total_unpacked_bytes=sum(a['unpacked_bytes'] for a in archives))
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (args.output/'SHA256SUMS').write_text(''.join(f"{a['sha256']}  {a['archive']}\n" for a in archives))
    print(json.dumps({k:v for k,v in manifest.items() if k!='archives'},indent=2))


if __name__=='__main__':main()
