# PART 1 - LINE COLLISION

def detect_line_collision(p1, p2, p3, p4):
    """"
    Given two lines:

    y = a1 * x + b1
    y = a2 * x + b2

    by simple algebra, we solve for their intersection point as:
    x = (b2 - b1) / (a1 - a2)

    (plug the solved x-coordinate into either line equation to compute the corresponding y-coordinate).

    NOTE: let us assume GENERAL POSITION, i.e. no line contains two points w/ identical x-coordinates (vertical line).
    """

    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    p3_x = p3[0]
    p3_y = p3[1]
    p4_x = p4[0]
    p4_y = p4[1]

    # compute line slopes
    a1 = (p2_y - p1_y) / (p2_x - p1_x)
    a2 = (p4_y - p3_y) / (p4_x - p3_x)

    if a1 == a2: return

    # compute y-intercepts
    b1 = p1_y - a1 * p1_x
    b2 = p3_y - a2 * p3_x

    collision_x = (b2 - b1) / (a1 - a2)
    collision_y = a1 * collision_x + b1

    return (collision_x, collision_y)


# PART 2 - SEGMENT COLLISION

def detect_segment_collision(p1, p2, p3, p4):
    line_collision = detect_line_collision(p1, p2, p3, p4)

    if line_collision is not None:
        x = line_collision[0]
        s1_min_x = min(p1[0], p2[0])
        s1_max_x = max(p1[0], p2[0])
        s2_min_x = min(p3[0], p4[0])
        s2_max_x = max(p3[0], p4[0])

        if x >= s1_min_x and x <= s1_max_x and x >= s2_min_x and x <= s2_max_x:
            return line_collision

    return None


def print_line_collision_result(collision, p1, p2, p3, p4):
    print(f"\nGiven the following points: [a={p1}, b={p2}, c={p3}, d={p4}]")
    if collision is None:
        print("The lines are parallel and therefore never intersect.")
    else:
        print(f"the intersection between line-ab and line-cd is: {(collision[0], collision[1])}\n")


def print_segment_collision_result(collision, p1, p2, p3, p4):
    print(f"\nGiven the following points: [a={p1}, b={p2}, c={p3}, d={p4}]")
    if collision is None:
        print("The segments never intersect.")
    else:
        print(f"the intersection between segment-ab and segment-cd is: {(collision[0], collision[1])}\n")


if __name__ == "__main__":

    #test cases

    a1 = (-1, 1)
    b1 = (3, 5)
    c1 = (-5, 3)
    d1 = (5, 3)

    line_collision1 = detect_line_collision(a1, b1, c1, d1)
    print_line_collision_result(line_collision1, a1, b1, c1, d1)
    segment_collision1 = detect_segment_collision(a1, b1, c1, d1)
    print_segment_collision_result(segment_collision1, a1, b1, c1, d1)

    a2 = (1, 2)
    b2 = (5, 4)
    c2 = (0, 4)
    d2 = (-2, 8)

    line_collision2 = detect_line_collision(a2, b2, c2, d2)
    print_line_collision_result(line_collision2, a2, b2, c2, d2)
    segment_collision2 = detect_segment_collision(a2, b2, c2, d2)
    print_segment_collision_result(segment_collision2, a2, b2, c2, d2)


    a3 = (0, 0)
    b3 = (5, 5)
    c3 = (0, 1)
    d3 = (5, 6)

    line_collision3 = detect_line_collision(a3, b3, c3, d3)
    print_line_collision_result(line_collision3, a3, b3, c3, d3)
    segment_collision3 = detect_line_collision(a3, b3, c3, d3)
    print_segment_collision_result(segment_collision3, a3, b3, c3, d3)
