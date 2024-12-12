import rclpy
from rclpy.node import Node
import numpy as np
import time
import matplotlib.pyplot as plt

# Simulação de diferentes algoritmos
class NavigationAlgorithm:
    def __init__(self, name):
        self.name = name
        self.start_time = None
        self.end_time = None
        self.path = []
        self.energy_consumed = 0.0

    def start(self):
        self.start_time = time.time()

    def stop(self):
        self.end_time = time.time()

    def record_path(self, position):
        self.path.append(position)

    def calculate_metrics(self, ideal_path):
        time_taken = self.end_time - self.start_time if self.start_time and self.end_time else 0
        path_error = np.sum([np.linalg.norm(np.array(p) - np.array(ip)) for p, ip in zip(self.path, ideal_path)])
        return time_taken, path_error, self.energy_consumed

class DroneNavigator(Node):
    def __init__(self):
        super().__init__('drone_navigator')
        self.algorithms = [
            NavigationAlgorithm('A*'),
            NavigationAlgorithm('Dijkstra'),
            NavigationAlgorithm('RRT'),
            NavigationAlgorithm('PRM'),
            NavigationAlgorithm('SLAM'),
        ]
        self.current_algorithm = None
        self.ideal_path = self.generate_ideal_path()
        self.current_position = [0, 0, 0]
        self.goal = [10, 10, 5]

    def generate_ideal_path(self):
        return [[x, x, x/2] for x in np.linspace(0, 10, 100)]

    def navigate(self, algorithm):
        self.current_algorithm = algorithm
        self.current_algorithm.start()
        for point in self.ideal_path:
            self.current_position = self.simulate_motion(point)
            self.current_algorithm.record_path(self.current_position)
            self.current_algorithm.energy_consumed += self.simulate_energy_consumption()
        self.current_algorithm.stop()

    def simulate_motion(self, target_position):
        # Simula o movimento do drone em direção ao ponto alvo
        return [tp + np.random.uniform(-0.1, 0.1) for tp in target_position]

    def simulate_energy_consumption(self):
        # Calcula consumo energético simulado
        return np.random.uniform(0.1, 0.2)

    def compare_algorithms(self):
        metrics = {}
        for algo in self.algorithms:
            self.navigate(algo)
            time_taken, path_error, energy = algo.calculate_metrics(self.ideal_path)
            metrics[algo.name] = {'time': time_taken, 'error': path_error, 'energy': energy}
        return metrics

    def plot_results(self, metrics):
        names = metrics.keys()
        times = [metrics[name]['time'] for name in names]
        errors = [metrics[name]['error'] for name in names]
        energy = [metrics[name]['energy'] for name in names]

        fig, ax = plt.subplots(1, 3, figsize=(15, 5))
        ax[0].bar(names, times, color='blue')
        ax[0].set_title('Time Taken')
        ax[0].set_ylabel('Time (s)')
        
        ax[1].bar(names, errors, color='orange')
        ax[1].set_title('Path Error')
        ax[1].set_ylabel('Error (m)')

        ax[2].bar(names, energy, color='green')
        ax[2].set_title('Energy Consumed')
        ax[2].set_ylabel('Energy (arbitrary units)')

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    navigator = DroneNavigator()
    metrics = navigator.compare_algorithms()
    navigator.plot_results(metrics)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
