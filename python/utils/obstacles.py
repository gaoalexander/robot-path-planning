import argparse
import json
import numpy as np


def orientation_test(p, q, r):
    matrix = np.array([[1, p[0], p[1]],
                    [1, q[0], q[1]],
                    [1, r[0], r[1]]])

    determinant = np.linalg.det(matrix)

    if determinant == 0:
        return 0
    elif determinant > 0:
        return 1
    else:
        return -1


def minkowski_difference(P1, P2):
    result = []
    for p_1 in P1:
        for p_2 in P2:
            result.append(p_1 - p_2)
    return np.array(result)


def get_perpendicular_toward_origin(two_simplex):
    o = np.array([0, 0])
    p = two_simplex[0]
    q = two_simplex[1]

    qp = p - q
    qo = o - q

    qp = np.append(qp, 0)
    qo = np.append(qo, 0)

    triple_product = np.cross(np.cross(qp, qo), qp)
    return triple_product[:2]


def contains_origin(simplex):
    o = np.array([0, 0])

    if len(simplex) == 2:
        d = get_perpendicular_toward_origin(simplex)
        return False, d, simplex
    else:
        point_to_idx = {
            "a": 0,
            "b": 1,
            "c": 2,
        }
        a = simplex[0]
        b = simplex[1]
        c = simplex[2]

        orientation_abo = orientation_test(a, b, o)

        if orientation_test(b, c, o) != orientation_abo:
            simplex = np.delete(simplex, point_to_idx["a"], axis=0)
            d = get_perpendicular_toward_origin(simplex)
            return False, d, simplex

        elif orientation_test(c, a, o) != orientation_abo:
            simplex = np.delete(simplex, point_to_idx["b"], axis=0)
            d = get_perpendicular_toward_origin(simplex)
            return False, d, simplex

        return True, None, simplex


def get_support(points, direction):
    dot_products = np.dot(points, direction)
    idx_max = np.argmax(dot_products)

    extreme = points[idx_max]
    return points[idx_max], dot_products[idx_max]


def GJK_solve(p, origin):
    d = np.array([0, 1])
    initial_extreme, _ = get_support(p, d)
    simplex = [initial_extreme]

    d = origin - initial_extreme
    i = 0
    while(True):
        new_extreme, dot_product = get_support(p, d)
        if dot_product < 0:
            return False
        simplex.append(new_extreme)
        bool_contains_origin, d, simplex = contains_origin(simplex)
        if bool_contains_origin:
            return True
        i += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--P', type = json.loads, help='--> \'[[p1_x, p1_y], ... , [pn_x, pn_y]]\' - (2D point set representing convex polygon)', required=True)
    parser.add_argument('--Q', type = json.loads, help='--> \'[[q1_x, q1_y], ... , [qm_x, qm_y]]\' - (2D point set representing convex polygon)', required=True)
    args = parser.parse_args()

    P = np.array(args.P)
    Q = np.array(args.Q)
    origin = np.array([0, 0])

    cso = minkowski_difference(P, Q)
    collide = GJK_solve(cso, origin)
    print("P and Q collide? ", collide)


"""
TEST CASES:

    origin = np.array([0, 0])

    P1 = np.array([[1, 1], [3, 1], [2, 2]])
    P2 = np.array([[4, 3], [6, 4], [5, 5]])
    cso = minkowski_difference(P1, P2)
    collide = GJK_solve(cso, origin)
    print("Polygon A: ", list(P1))
    print("Polygon B: ", list(P2))
    print("A and B collide? ", collide, "\n")

    P1 = np.array([(1, 1), (3, 1), (2, 3)])
    P2 = np.array([(0, 0), (4, 1), (2, 4)])
    cso = minkowski_difference(P1, P2)
    collide = GJK_solve(cso, origin)
    print("Polygon A: ", list(P1))
    print("Polygon B: ", list(P2))
    print("A and B collide? ", collide, "\n")

    P1 = np.array([(-2, 1), (2, 1), (0, 5)])
    P2 = np.array([(1, 2), (4, 2), (4, 5), (1, 5)])
    cso = minkowski_difference(P1, P2)
    collide = GJK_solve(cso, origin)
    print("Polygon A: ", list(P1))
    print("Polygon B: ", list(P2))
    print("A and B collide? ", collide, "\n")

    P1 = np.array([(1, 0), (0, 5), (-2, -1)])
    P2 = np.array([(1, 1), (4, 1), (4, 4), (1, 4)])
    cso = minkowski_difference(P1, P2)
    collide = GJK_solve(cso, origin)
    print("Polygon A: ", list(P1))
    print("Polygon B: ", list(P2))
    print("A and B collide? ", collide, "\n")
"""