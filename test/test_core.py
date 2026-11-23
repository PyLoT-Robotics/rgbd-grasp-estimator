import numpy as np
import pytest
from rgbd_grasp.core import Intrinsics,GraspConfig,roi_points,cluster_points,estimate,approach_quaternion
K=Intrinsics(200.,200.,19.5,19.5)


def test_plane_units_normal_and_approach():
    depth=np.ones((40,40),dtype=float)
    a=estimate(depth,K,[0,0,40,40]);b=estimate((depth*1000).astype(np.uint16),K,[0,0,40,40],unit='mm')
    assert a==b
    assert a['normal']==pytest.approx([0,0,-1])
    assert a['position_m'][2]==pytest.approx(.97)
    assert np.linalg.norm(a['orientation_xyzw'])==pytest.approx(1.)


def test_invalid_depth_does_not_produce_a_grasp():
    depth=np.full((40,40),np.nan)
    assert estimate(depth,K,[0,0,40,40]) is None
    depth[:]=0;assert estimate(depth,K,[0,0,40,40]) is None


def test_mask_and_deterministic_sampling():
    depth=np.ones((40,40));mask=np.zeros_like(depth,dtype=bool);mask[:20,:20]=True
    c=GraspConfig(max_points=20)
    a=roi_points(depth,K,[0,0,40,40],c,mask=mask)
    assert len(a)==20 and np.all(a[:,0]<0)
    np.testing.assert_array_equal(a,roi_points(depth,K,[0,0,40,40],c,mask=mask))


def test_separate_clusters_do_not_merge():
    points=np.array([[0,0,1],[.01,0,1],[2,0,1],[2.01,0,1]])
    assert sorted(map(len,cluster_points(points,.03,2)))==[2,2]


def test_collinear_cloud_is_rejected():
    depth=np.zeros((40,40));depth[20,:]=1.
    assert estimate(depth,K,[0,0,40,40]) is None


@pytest.mark.parametrize('roi',[[-1,0,10,10],[0,0,100,40],[0,0,0,40],[0.,0.,1.5,40]])
def test_bad_roi(roi):
    with pytest.raises(ValueError):estimate(np.ones((40,40)),K,roi)


def test_bad_intrinsics_and_configuration():
    with pytest.raises(ValueError):Intrinsics(0.,1.,0.,0.)
    with pytest.raises(ValueError):GraspConfig(tolerance_m=0.)
    with pytest.raises(ValueError):GraspConfig(max_points=2)


def test_quaternion_rotates_z_to_negative_normal():
    normal=np.array([1.,0.,0.]);x,y,z,w=approach_quaternion(normal)
    rotated_z=np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
    np.testing.assert_allclose(rotated_z,-normal,atol=1e-9)
