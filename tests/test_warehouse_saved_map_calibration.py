import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from calibrate_warehouse_saved_map import fit_se2, image_pixel_to_saved_map


def test_rigid_registration_fit_and_independent_holdout():
    src=np.array([[0,0],[4,0],[0,3],[3,4]],float);a=.2
    R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    dst=src@R.T+[2,-1]
    got=fit_se2(src,dst)
    assert got['accepted'] and got['fit_pair_count']==3 and got['holdout_pair_count']==1
    assert got['transform']['x_m']==pytest.approx(2)
    assert got['transform']['y_m']==pytest.approx(-1)
    assert got['transform']['yaw_rad']==pytest.approx(a)
    assert got['holdout_error_m']<1e-10


def test_rejects_scale_reflection_and_bad_holdout():
    src=np.array([[0,0],[4,0],[0,3],[3,4]],float)
    with pytest.raises(ValueError,match='scale mismatch'):
        fit_se2(src,src*1.2)
    with pytest.raises(ValueError,match='reflection'):
        fit_se2(src,src*np.array([-1,1]))
    dst=src.copy();dst[-1]+=[1,1]
    with pytest.raises(ValueError,match='holdout'):
        fit_se2(src,dst)


def test_saved_map_yaml_pixel_axes_use_top_row_as_high_y():
    assert image_pixel_to_saved_map((0,0),800,800,.05,[-20,-20,0])==pytest.approx([-19.975,19.975])
