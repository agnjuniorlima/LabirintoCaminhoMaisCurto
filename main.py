
from hashlib import algorithms_available
import time
from src.problems import MazeProblem
from src.viewer import MazeViewer
from src.search import bfs
from src.search import dfs
from src.search import dfs2
from src.search import uniform_cost_search
from src.search import a_star_search
from src.search import iterative_depth_first_search


def main():
    maze_problem = MazeProblem(30, 30, 42)
    viewer = MazeViewer(maze_problem, step_time_miliseconds=20, zoom=5)
    max_depth = 90000
    report_csv  = "\tTabela de comparação\n" # criando a tabela de reporte
    report_csv += "Algoritmo, Cost, Steps, Expanded, Generated, Runtime\n"

    algorithms = [bfs, dfs, uniform_cost_search, a_star_search, iterative_depth_first_search]

    for algorithm in algorithms:
        print(algorithm.__name__)
        start_time = time.time()
        if algorithm.__name__ == 'iterative_depth_first_search':
            path, cost, expanded_nodes, gen_nodes = algorithm(maze_problem, viewer, max_depth)
        else:
            path, cost, expanded_nodes, gen_nodes = algorithm(maze_problem, viewer)
        end_time = time.time() - start_time
        
        # Mostrando resultados #
        report_csv += report_search_results(algorithm.__name__, path, cost, expanded_nodes, gen_nodes, end_time)

        #viewer.update(path=path)
        #viewer.pause()

        print("OK!")
    
    print(report_csv)
    gravArq(report_csv, "report.csv")


def report_search_results(algorithm_name, path, cost, expanded_nodes, gen_nodes, end_time):

    # report_csv += "Algoritmo, Cost, Steps, Expanded, Generated, Runtime\n"
    report_csv = algorithm_name + "," + str(cost) + "," + str(len(path)-1) + "," + str(expanded_nodes) + "," + str(gen_nodes) + "," + str(end_time) + "\n"
    return report_csv



if __name__ == "__main__":
    main()