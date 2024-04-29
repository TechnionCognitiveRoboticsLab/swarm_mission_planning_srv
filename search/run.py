import concurrent.futures
import multiprocessing
import sys
import time
import tracemalloc
from multiprocessing import Process

import pandas as pd
import psutil
from . import Solver



def write_data(r, name, ):
    fin_res, t, res, states, inst, algo = r[0], r[1], r[2], r[3], r[4], r[5]
    df = pd.DataFrame({'inst_name': [str(inst.name)], 'num_agents': [len(inst.agents)], 'map_size': [len(inst.map)],
                       'source': [str(inst.source)], 'type': [str(inst.type)], 'horizon': [int(inst.horizon)],
                       'algo': [str(algo)],
                       'final_result': [float(fin_res)], 'time': [float(t)] if t != '-' else '-', 'states': int(states),
                       'result': [str(res)]})
    print({'inst_name': inst.name, 'num_agents': len(inst.agents), 'map_size': len(inst.map),
           'source': inst.source, 'type': inst.type, 'horizon': inst.horizon, 'algo': algo,
           'final_result': fin_res, 'time': t, 'states': states, })
    try:
        df.to_csv('data/' + name + '.csv', mode='a', index=False,
                  header=False)
    except:
        print("Save failed, create \'data\' folder to save.")


def run_solver(inst, algo, timeout=1800, default='-', return_path=False):
    print("Start " + inst.name, algo)
    solver = Solver.Solver(inst)
    solver.dup_det = True
    solver.timeout = timeout
    solver.return_path = return_path
    results = None
    if algo == 'MCTS_E':
        results = solver.emp_mcts()
    if algo == 'MCTS_V':
        results = solver.vector_mcts()
    if algo == 'MCTS_S':
        results = solver.semi_emp_mcts()
    if algo == 'BFS':
        solver.type = 'U1S'
        results = solver.bfs()
    if algo == 'BNBL':
        solver.type = 'U1S'
        results = solver.branch_and_bound(solver.upper_bound_base_plus_utility,
                                          solver.greedy_lower_bound)
    if algo == 'BNB':
        solver.type = 'U1S'
        results = solver.branch_and_bound(solver.upper_bound_base_plus_utility)
    if algo == 'GBNB':
        solver.type = 'U1S'
        results = solver.branch_and_bound(solver.upper_bound_base_plus_utility,
                                          solver.greedy_lower_bound, is_greedy=True)
    if algo == 'ASTAR':
        solver.type = 'U1S'
        results = solver.branch_and_bound(solver.upper_bound_base_plus_utility,
                                          solver.greedy_lower_bound, astar=True)
    if algo == 'DFS':
        solver.type = 'U1S'
        results = solver.branch_and_bound(depth_first=True)
    if return_path:
        return results
    fin_res = results[-1][0] if len(results) > 0 else default
    fin_time = results[-1][-1] if len(results) > 0 and results[-1][-1] < timeout else default
    fin_states = results[-1][1] if len(results) > 0 else default
    return fin_res, fin_time, results, fin_states, inst, algo



