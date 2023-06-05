"""
Clothoid Path Planner
Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (AtsushiSakai)
Reference paper: Fast and accurate G1 fitting of clothoid curves
https://www.researchgate.net/publication/237062806
"""

from collections import namedtuple
import matplotlib.pyplot as plt 
import numpy as np
import scipy.integrate as integrate
from scipy.optimize import fsolve
from math import atan2, cos, hypot, pi, sin

Point = namedtuple("Point", ["x", "y"])

show_animation = True


def generate_clothoid_paths(start_point, start_yaw_list,
                            goal_point, goal_yaw_list,
                            n_path_points):
    """
    Generate clothoid path list. This function generate multiple clothoid paths
    from multiple orientations(yaw) at start points to multiple orientations
    (yaw) at goal point.

    :param start_point: Start point of the path
    :param start_yaw_list: Orientation list at start point in radian
    :param goal_point: Goal point of the path
    :param goal_yaw_list: Orientation list at goal point in radian
    :param n_path_points: number of path points
    :return: clothoid path list
    """
    clothoids = []
    for start_yaw in start_yaw_list:
        for goal_yaw in goal_yaw_list:
            clothoid = generate_clothoid_path(start_point, start_yaw,
                                              goal_point, goal_yaw,
                                              n_path_points)
            clothoids.append(clothoid)
    return clothoids


def generate_clothoid_path(start_point, start_yaw,
                           goal_point, goal_yaw, n_path_points):
    """
    Generate a clothoid path list.

    :param start_point: Start point of the path
    :param start_yaw: Orientation at start point in radian
    :param goal_point: Goal point of the path
    :param goal_yaw: Orientation at goal point in radian
    :param n_path_points: number of path points
    :return: a clothoid path
    """
    dx = goal_point.x - start_point.x
    dy = goal_point.y - start_point.y
    r = hypot(dx, dy)

    phi = atan2(dy, dx)
    phi1 = normalize_angle(start_yaw - phi)
    phi2 = normalize_angle(goal_yaw - phi)
    delta = phi2 - phi1

    try:
        # Step1: Solve g function
        A = solve_g_for_root(phi1, phi2, delta)

        # Step2: Calculate path parameters
        L = compute_path_length(r, phi1, delta, A)
        curvature = compute_curvature(delta, A, L)
        curvature_rate = compute_curvature_rate(A, L)
    except Exception as e:
        print(f"Failed to generate clothoid points: {e}")
        return None

    # Step3: Construct a path with Fresnel integral
    xpts = []
    ypts = []
    for s in np.linspace(0, L, n_path_points):
        try:
            x = start_point.x + s * X(curvature_rate * s ** 2, curvature * s,
                                      start_yaw)
            y = start_point.y + s * Y(curvature_rate * s ** 2, curvature * s,
                                      start_yaw)
            xpts.append(x)
            ypts.append(y)
        except Exception as e:
            print(f"Skipping failed clothoid point: {e}")

    return xpts, ypts

def X(a, b, c):
    return integrate.quad(lambda t: cos((a/2)*t**2 + b*t + c), 0, 1)[0]


def Y(a, b, c):
    return integrate.quad(lambda t: sin((a/2)*t**2 + b*t + c), 0, 1)[0]


def solve_g_for_root(theta1, theta2, delta):
    initial_guess = 3*(theta1 + theta2)
    return fsolve(lambda A: Y(2*A, delta - A, theta1), [initial_guess])


def compute_path_length(r, theta1, delta, A):
    return r / X(2*A, delta - A, theta1)


def compute_curvature(delta, A, L):
    return (delta - A) / L


def compute_curvature_rate(A, L):
    return 2 * A / (L**2)


def normalize_angle(angle_rad):
    return (angle_rad + pi) % (2 * pi) - pi


def get_axes_limits(clothoids):
    x_vals = [p.x for clothoid in clothoids for p in clothoid]
    y_vals = [p.y for clothoid in clothoids for p in clothoid]

    x_min = min(x_vals)
    x_max = max(x_vals)
    y_min = min(y_vals)
    y_max = max(y_vals)

    x_offset = 0.1*(x_max - x_min)
    y_offset = 0.1*(y_max - y_min)

    x_min = x_min - x_offset
    x_max = x_max + x_offset
    y_min = y_min - y_offset
    y_max = y_max + y_offset

    return x_min, x_max, y_min, y_max


def draw_clothoids(start, goal, num_steps, clothoidal_paths,
                   save_animation=False):

    fig = plt.figure(figsize=(10, 10))
    x_min, x_max, y_min, y_max = get_axes_limits(clothoidal_paths)
    axes = plt.axes(xlim=(x_min, x_max), ylim=(y_min, y_max))

    axes.plot(start.x, start.y, 'ro')
    axes.plot(goal.x, goal.y, 'ro')
    lines = [axes.plot([], [], 'b-')[0] for _ in range(len(clothoidal_paths))]

    # def animate(i):
    #     for line, clothoid_path in zip(lines, clothoidal_paths):
    #         x = [p.x for p in clothoid_path[:i]]
    #         y = [p.y for p in clothoid_path[:i]]
    #         line.set_data(x, y)

    #     return lines

    # anim = animation.FuncAnimation(
    #     fig,
    #     animate,
    #     frames=num_steps,
    #     interval=25,
    #     blit=True
    # )
    # if save_animation:
    #     anim.save('clothoid.gif', fps=30, writer="imagemagick")
    # plt.show()


def main():

    start_point1 = Point((19.25/2), 0) 
    start_orientation1 = np.pi / 2
    goal_point1 = Point((19.25/2), 114) #assume in ft
    goal_orientation1 = np.pi / 2 
    num_path_points1 = 100

    start_point2 = Point((19.25/2), 114) 
    start_orientation2 = np.pi / 2
    goal_point2 = Point((19.25/2 + 22 + 9), 114 + 22) #assume in ft
    goal_orientation2 = 0
    num_path_points2 = 100




    clothoid_path1 = generate_clothoid_path(
    start_point1, start_orientation1,
    goal_point1, goal_orientation1,
    num_path_points1)

    clothoid_path2 = generate_clothoid_path(
    start_point2, start_orientation2,
    goal_point2, goal_orientation2,
    num_path_points2)
    

    plt.plot(start_point1[0],start_point1[1], 'o', color = "red")
    plt.plot(goal_point2[0],goal_point2[1], 'o', color = "red")
    plt.plot(clothoid_path1[0],clothoid_path1[1])
    plt.plot(clothoid_path2[0],clothoid_path2[1])

    plt.show()
    # if show_animation:
    #     draw_clothoids(start_point, goal_point,
    #                    num_path_points, clothoid_path,
    #                    save_animation=False)

    


if __name__ == "__main__":
    main()

# 3 points is the most for what we have to do for segment one 

