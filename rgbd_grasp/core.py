"""Depth ROI -> optical-frame points -> clusters -> geometric grasp proposal.

Adapted from PyLoT's bottle/bowl cluster grasp nodes. No robot arm or detector
is required. This estimates geometry; it does not prove a grasp is executable.
"""
from collections import defaultdict,deque
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    fx:float
    fy:float
    cx:float
    cy:float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.fx,self.fy,self.cx,self.cy)) or self.fx<=0 or self.fy<=0:
            raise ValueError('Finite intrinsics with positive fx/fy are required')


@dataclass(frozen=True)
class GraspConfig:
    min_depth_m:float=.05
    max_depth_m:float=3.
    tolerance_m:float=.03
    min_cluster_points:int=12
    max_points:int=2000
    sample_step:int=2
    offset_m:float=.03
    max_planarity_ratio:float=.2

    def __post_init__(self):
        for key in ['min_depth_m','max_depth_m','tolerance_m','offset_m','max_planarity_ratio']:
            if not math.isfinite(getattr(self,key)):raise ValueError('Configuration must be finite')
        if not 0<=self.min_depth_m<self.max_depth_m or self.tolerance_m<=0 or self.offset_m<0 or not 0<self.max_planarity_ratio<=1:
            raise ValueError('Invalid depth/clustering configuration')
        for key in ['min_cluster_points','max_points','sample_step']:
            if not isinstance(getattr(self,key),int) or isinstance(getattr(self,key),bool) or getattr(self,key)<1:raise ValueError('Positive integer required: '+key)
        if self.min_cluster_points<6 or self.max_points<self.min_cluster_points:raise ValueError('Need at least 6 points within max_points')


def roi_points(depth,intrinsics,roi,config=GraspConfig(),unit='m',mask=None):
    depth=np.asarray(depth)
    if depth.ndim!=2 or not np.issubdtype(depth.dtype,np.number) or np.iscomplexobj(depth):raise ValueError('Expected a numeric 2D depth image')
    if unit not in ['m','mm']:raise ValueError('Depth unit must be m or mm')
    values=np.asarray(roi)
    if values.shape!=(4,) or not np.isfinite(values.astype(float)).all() or np.any(values!=values.astype(int)):raise ValueError('ROI must contain four finite integer xyxy coordinates')
    x1,y1,x2,y2=map(int,values);h,w=depth.shape
    if not 0<=x1<x2<=w or not 0<=y1<y2<=h:raise ValueError('ROI lies outside image')
    if mask is not None:
        mask=np.asarray(mask)
        if mask.shape!=depth.shape or mask.dtype!=np.bool_:raise ValueError('Mask must be an image-sized boolean array')
    u,v=np.meshgrid(np.arange(x1,x2,config.sample_step),np.arange(y1,y2,config.sample_step))
    z=depth[v,u].astype(float)*(1. if unit=='m' else .001)
    valid=np.isfinite(z)&(z>config.min_depth_m)&(z<config.max_depth_m)
    if mask is not None:valid&=mask[v,u]
    u,v,z=u[valid],v[valid],z[valid]
    points=np.column_stack(((u-intrinsics.cx)*z/intrinsics.fx,(v-intrinsics.cy)*z/intrinsics.fy,z))
    if len(points)>config.max_points:
        # Deterministic spatially distributed subsampling, not global RNG state.
        points=points[np.linspace(0,len(points)-1,config.max_points,dtype=int)]
    return points


def cluster_points(points,tolerance,min_points):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('Expected finite Nx3 points')
    if not math.isfinite(tolerance) or tolerance<=0 or min_points<1:raise ValueError('Invalid cluster parameters')
    grid=defaultdict(list);keys=np.floor(points/tolerance).astype(np.int64)
    for i,k in enumerate(keys):grid[tuple(k)].append(i)
    visited=np.zeros(len(points),dtype=bool);clusters=[]
    for seed in range(len(points)):
        if visited[seed]:continue
        queue=deque([seed]);visited[seed]=True;members=[]
        while queue:
            i=queue.popleft();members.append(i);cx,cy,cz=keys[i]
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    for dz in [-1,0,1]:
                        for j in grid.get((cx+dx,cy+dy,cz+dz),[]):
                            if not visited[j] and np.sum((points[i]-points[j])**2)<=tolerance*tolerance:
                                visited[j]=True;queue.append(j)
        if len(members)>=min_points:clusters.append(points[members])
    return clusters


def approach_quaternion(normal):
    """Quaternion xyzw whose local +Z approach points opposite surface normal."""
    z=-np.asarray(normal,dtype=float);norm=np.linalg.norm(z)
    if z.shape!=(3,) or not np.isfinite(z).all() or norm<1e-9:raise ValueError('Invalid normal')
    z/=norm
    # Shortest rotation taking the optical +Z basis vector to approach z.
    dot=float(z[2])
    if dot<-1+1e-9:return [1.,0.,0.,0.]
    q=np.array([-z[1],z[0],0.,1+dot]);q/=np.linalg.norm(q)
    return q.tolist()


def estimate(depth,intrinsics,roi,config=GraspConfig(),unit='m',mask=None):
    points=roi_points(depth,intrinsics,roi,config,unit,mask)
    clusters=cluster_points(points,config.tolerance_m,config.min_cluster_points)
    if not clusters:return None
    cluster=max(clusters,key=len);centroid=np.mean(cluster,axis=0)
    values,vectors=np.linalg.eigh(np.cov((cluster-centroid).T))
    if values[1]<=1e-12 or values[2]<=1e-12:return None
    ratio=float(max(0.,values[0])/values[2])
    if ratio>config.max_planarity_ratio:return None
    normal=vectors[:,0]
    if np.dot(normal,-centroid)<0:normal=-normal
    grasp=centroid+normal*config.offset_m
    return {'centroid_m':centroid.tolist(),'normal':normal.tolist(),'position_m':grasp.tolist(),
            'orientation_xyzw':approach_quaternion(normal),'cluster_points':len(cluster),
            'planarity_ratio':ratio}
