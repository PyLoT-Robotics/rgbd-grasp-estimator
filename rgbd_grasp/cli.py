import argparse
import json
from pathlib import Path
import numpy as np
from .core import Intrinsics,GraspConfig,estimate


def main(argv=None):
    p=argparse.ArgumentParser(description='Depth ROI geometry: optical-frame metres, no motion')
    sub=p.add_subparsers(dest='command',required=True);sub.add_parser('demo')
    run=sub.add_parser('estimate');run.add_argument('depth',type=Path);run.add_argument('--unit',choices=['m','mm'],required=True)
    run.add_argument('--roi',nargs=4,type=int,required=True);run.add_argument('--intrinsics',nargs=4,type=float,metavar=('FX','FY','CX','CY'),required=True)
    run.add_argument('--output',type=Path)
    a=p.parse_args(argv)
    try:
        if a.command=='demo':
            depth=np.ones((40,40),dtype=float);k=Intrinsics(200.,200.,19.5,19.5)
            result=estimate(depth,k,[0,0,40,40]);assert result is not None
            assert abs(result['position_m'][2]-.97)<1e-9
        else:result=estimate(np.load(a.depth,allow_pickle=False),Intrinsics(*a.intrinsics),a.roi,unit=a.unit)
        text=json.dumps({'valid':result is not None,'candidate':result},indent=2,allow_nan=False)
        if getattr(a,'output',None):a.output.write_text(text+'\n')
        else:print(text)
        return 0 if result is not None else 1
    except (ValueError,TypeError,OSError) as e:p.exit(2,str(e)+'\n')


if __name__=='__main__':raise SystemExit(main())
