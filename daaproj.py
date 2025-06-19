from pyamaze import maze, agent, textLabel
import time
import tkinter as tk
import heapq
import matplotlib.pyplot as plt


def BFS_enhanced(m):
    start = (m.rows, m.cols)
    frontier = [(start, [start])]
    explored = {start}
    nodes_explored = 0

    while frontier:
        currCell, path = frontier.pop(0)
        nodes_explored += 1

        if currCell == (1, 1):
            return constructPath_from_list(path), nodes_explored

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                if childCell not in explored:
                    explored.add(childCell)
                    frontier.append((childCell, path + [childCell]))

    return {}, nodes_explored


def DFS_enhanced(m):
    start = (m.rows, m.cols)
    frontier = [(start, [start])]
    explored = {start}
    nodes_explored = 0

    while frontier:
        currCell, path = frontier.pop()
        nodes_explored += 1

        if currCell == (1, 1):
            return constructPath_from_list(path), nodes_explored

        neighbors = []
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                if childCell not in explored:
                    neighbors.append((childCell, path + [childCell]))

        for neighbor, new_path in reversed(neighbors):
            explored.add(neighbor)
            frontier.append((neighbor, new_path))

    return {}, nodes_explored


def Dijkstra_enhanced(m):
    start = (m.rows, m.cols)
    goal = (1, 1)
    frontier = [(0, start, [start])]
    costSoFar = {start: 0}
    nodes_explored = 0

    while frontier:
        currCost, currCell, path = heapq.heappop(frontier)
        nodes_explored += 1

        if currCell == goal:
            return constructPath_from_list(path), nodes_explored

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                newCost = currCost + 1
                if childCell not in costSoFar or newCost < costSoFar[childCell]:
                    costSoFar[childCell] = newCost
                    heapq.heappush(frontier, (newCost, childCell, path + [childCell]))

    return {}, nodes_explored


def AStar_enhanced(m):
    start = (m.rows, m.cols)
    goal = (1, 1)

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    frontier = [(heuristic(start, goal), 0, start, [start])]
    costSoFar = {start: 0}
    nodes_explored = 0

    while frontier:
        f_score, currCost, currCell, path = heapq.heappop(frontier)
        nodes_explored += 1

        if currCell == goal:
            return constructPath_from_list(path), nodes_explored

        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1])
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                newCost = currCost + 1
                if childCell not in costSoFar or newCost < costSoFar[childCell]:
                    costSoFar[childCell] = newCost
                    priority = newCost + heuristic(childCell, goal)
                    heapq.heappush(frontier, (priority, newCost, childCell, path + [childCell]))

    return {}, nodes_explored


def constructPath_from_list(path_list):
    path = {}
    for i in range(len(path_list) - 1):
        path[path_list[i]] = path_list[i + 1]
    return path


def getScreenSizeMaze():
    root = tk.Tk()
    width, height = root.winfo_screenwidth(), root.winfo_screenheight()
    root.destroy()
    cell_size = 25
    return (height - 100) // cell_size, (width - 100) // cell_size


def visualize_algorithm(name, algo_func, m, delay, result_data):
    print(f"\nRunning {name}...")

    start_time = time.time()
    path, explored = algo_func(m)
    elapsed = round(time.time() - start_time, 4)

    agent_color = {
        'BFS': 'green',
        'DFS': 'yellow',
        'Dijkstra': 'red',
        'A*': 'blue'
    }

    a = agent(m, filled=True, footprints=True, color=agent_color[name])
    m.tracePath({a: path}, delay=delay)

    result_data[name] = {
        'time': elapsed,
        'length': len(path),
        'explored': explored
    }

    label_text = f"{name} | Time: {elapsed}s | Length: {len(path)} | Explored: {explored}"
    textLabel(m, name, label_text)
    m.run()


def plot_comparison(result_data):
    algorithms = list(result_data.keys())
    times = [result_data[algo]['time'] for algo in algorithms]
    lengths = [result_data[algo]['length'] for algo in algorithms]
    explored = [result_data[algo]['explored'] for algo in algorithms]

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    axs[0].bar(algorithms, times, color='skyblue')
    axs[0].set_title('Time Taken (s)')
    axs[0].set_ylabel('Seconds')

    axs[1].bar(algorithms, lengths, color='lightgreen')
    axs[1].set_title('Path Length')
    axs[1].set_ylabel('Steps')

    axs[2].bar(algorithms, explored, color='salmon')
    axs[2].set_title('Nodes Explored')
    axs[2].set_ylabel('Nodes')

    plt.suptitle('Pathfinding Algorithm Comparison')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    import random

    rows, cols = getScreenSizeMaze()
    print(f"Screen Maze Size: {rows}x{cols}")

    # Generate a seed to reuse the exact same maze
    maze_seed = random.randint(1, 100000)
    print(f"Using Maze Seed: {maze_seed}")

    result_data = {}
    delay = 20

    algo_funcs = {
        'BFS': BFS_enhanced,
        'DFS': DFS_enhanced,
        'Dijkstra': Dijkstra_enhanced,
        'A*': AStar_enhanced
    }

    for algo in algo_funcs:
        random.seed(maze_seed)
        m = maze(rows, cols)
        m.CreateMaze(loopPercent=30)
        visualize_algorithm(algo, algo_funcs[algo], m, delay, result_data)

    plot_comparison(result_data)
