import pytest
from math import pi
from kinematics import * 

grid = [(1,2), (1,3), (2,4), (3,4)]
wspace = [(0, 1, 0), (1.4, 1.4, 0.7), (1, 0, -0.7)]

def test_grid_path_to_workspace():
    k = Kinematics(0.1)
    workspace = k.grid_path_to_workspace(grid)
    assert workspace == wspace



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
    k = Kinematics(0.1)
    for o, vecs in circle_test.items():
        print(f"vecs {vecs}")
        for i, vec in enumerate(vecs):
            a = vec[0]
            b = vec[1]
            print(f"i {i}")
            print(f"expect {around[i]}")
            if around[i] == pi:
                assert abs(k.get_rotation(a, b, o)) == pytest.approx(around[i])
            else:
                assert k.get_rotation(a, b, o) == pytest.approx(around[i])


    # # go from s to ne
    # a = v_ne[0]
    # b = v_ne[1]
    # o = s
    # assert k.get_rotation(a, b, o) == pytest.approx((3*pi)/4)
    # # go from s to nw
    # a = v_nw[0]
    # b = v_nw[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(-(3*pi)/4)
    # # go from s to n
    # a = v_n[0]
    # b = v_n[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(-pi)
    # # go from s to e
    # a = v_e[0]
    # b = v_e[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(pi/2)
    # # go from s to w
    # a = v_w[0]
    # b = v_w[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(-pi/2)
    # # go from s to sw
    # a = v_sw[0]
    # b = v_sw[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(-pi/4)
    # # go from s to se
    # a = v_se[0]
    # b = v_se[1]
    # assert k.get_rotation(a, b, o) == pytest.approx(pi/4)

def test_workspace_to_cspace():
    pass
