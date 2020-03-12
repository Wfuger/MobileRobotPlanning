import pytest
from math import pi
from kinematics import * 

grid = [(1,2), (1,3), (2,4), (3,4)]
wspace = [(0, 1, 0), (1.4, 1.4, 0.7), (1, 0, -0.7)]

def test_grid_path_to_workspace():
    k = Kinematics(0.1)
    workspace = k.grid_path_to_workspace(grid)
    assert workspace == wspace


@pytest.mark.rotation
def test_get_rotation():
    k = Kinematics(0.1)
    a = (1,1)
    b = (2,2)
    o = (3*pi)/2
    assert k.get_rotation(a, b, o) == pytest.approx((3*pi)/4)
    a = (2,2)
    b = (1,3)
    # o = -pi/2
    assert k.get_rotation(a, b, o) == pytest.approx(-(3*pi)/4)

def test_workspace_to_cspace():
    pass
