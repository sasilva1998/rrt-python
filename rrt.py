import pygame
from rrt_base import RRTGraph, RRTMap
import time


def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obs_dim = 50
    obs_num = 50

    iteration = 0
    t1 = time.time()

    pygame.init()
    map = RRTMap(start, goal, dimensions, obs_dim, obs_num)
    graph = RRTGraph(start, goal, dimensions, obs_dim, obs_num)

    obstacles = graph.make_obs()

    map.draw_map(obstacles)

    elapsed = 0

    while not graph.path_to_goal():
        elapsed = elapsed + (time.time() - t1)
        t1 = time.time()

        if elapsed > 0.2:
            print("=====No solution found========")
            print("ELAPSED TIME: ", elapsed)
            raise

        if iteration % 10 == 0:
            X, Y, PARENT = graph.bias(goal)

            pygame.draw.circle(
                map.map, map.grey_color, (X[-1], Y[-1]), map.node_rad + 2, 0
            )
            pygame.draw.line(
                map.map,
                map.blue_color,
                (X[-1], Y[-1]),
                (X[PARENT[-1]], Y[PARENT[-1]]),
                map.edge_thickness,
            )
        else:
            X, Y, PARENT = graph.expand()
            pygame.draw.circle(
                map.map, map.grey_color, (X[-1], Y[-1]), map.node_rad + 2, 0
            )
            pygame.draw.line(
                map.map,
                map.blue_color,
                (X[-1], Y[-1]),
                (X[PARENT[-1]], Y[PARENT[-1]]),
                map.edge_thickness,
            )

        pygame.display.update()
        iteration += 1
        # time.sleep(1)
    map.draw_path(graph.get_path_coords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
