import pytest
from math import pi, sqrt
from kinematics import * 

grid = [(1,2), (1,3), (2,4), (3,4)]
wspace = [(.1, pi/2), (sqrt(2)/10, -pi/4), (.1, -pi/4)]

@pytest.mark.grid_to_cspace
def test_grid_path_to_cspace():
    k = Kinematics(0.1, 0.15, 0.05)
    workspace = k.grid_path_to_cspace(grid)
    print(workspace)
    for i, move in enumerate(workspace):
        expected = wspace[i]
        assert move[0] == pytest.approx(expected[0])
        assert move[1] == pytest.approx(expected[1])


@pytest.mark.rot_wheel_rotations
def test_rotation_to_wheel_rotation():
    axle_length = 0.15
    wheel_radius = 0.05
    k = Kinematics(0.1, axle_length, wheel_radius)
    theta_axle = pi/2
    theta_l, theta_r = k.rotation_to_wheel_rotation(theta_axle)
    print(f"theta l {theta_l}")
    print(f"theta r {theta_r}")
    aboot = (theta_axle*(axle_length/2))/wheel_radius
    assert theta_r == pytest.approx(aboot)
    assert theta_l == pytest.approx(-aboot)


@pytest.mark.trans_wheel_rotations
def test_translation_to_wheel_rotation():
    k = Kinematics(0.1, 0.15, 0.05)
    theta_l, theta_r = k.translation_to_wheel_rotation(5)
    print(f"theta l {theta_l}")
    print(f"theta r {theta_r}")
    assert theta_r == 100
    assert theta_l == 100

# orientations
s = (3*pi)/2
n = pi/2
w = pi
e = 0
nw = 3*pi/4
ne = pi/4
sw = (5*pi)/4
se = (7*pi)/4

# direction vectors
v_s = [(2,2), (2,1)]
v_n = [(2,2), (2,3)]
v_w = [(2,2), (1,2)]
v_e = [(2,2), (3,2)]
v_ne = [(2,2), (3,3)]
v_nw = [(2,2), (1,3)]
v_se = [(2,2), (3,1)]
v_sw = [(2,2), (1,1)]

# expects
around = [0, pi/4, pi/2, (3*pi)/4, pi, -(3*pi)/4, -pi/2, -pi/4]

circle_test = {
    s: [v_s, v_se, v_e, v_ne, v_n, v_nw, v_w, v_sw],
    se: [v_se, v_e, v_ne, v_n, v_nw, v_w, v_sw, v_s],
    e: [v_e, v_ne, v_n, v_nw, v_w, v_sw, v_s, v_se],
    ne: [v_ne, v_n, v_nw, v_w, v_sw, v_s, v_se, v_e],
    n: [v_n, v_nw, v_w, v_sw, v_s, v_se, v_e, v_ne],
    nw: [v_nw, v_w, v_sw, v_s, v_se, v_e, v_ne, v_n],
    w: [v_w, v_sw, v_s, v_se, v_e, v_ne, v_n, v_nw],
    sw:  [v_sw, v_s, v_se, v_e, v_ne, v_n, v_nw, v_w],
}

@pytest.mark.rotation
def test_get_rotation():
    k = Kinematics(0.1, 0.15, 0.05)
    for o, vecs in circle_test.items():
        for i, vec in enumerate(vecs):
            a = vec[0]
            b = vec[1]
            if around[i] == pi:
                assert abs(k.get_rotation(a, b, o)) == pytest.approx(around[i])
            else:
                assert k.get_rotation(a, b, o) == pytest.approx(around[i])


