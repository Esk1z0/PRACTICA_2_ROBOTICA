#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import os
import random
import math
from PIL import Image

class TreeNode:
    """Clase auxiliar para nodos del árbol RRT."""
    def __init__(self, r, c):
        self.r = r  # Fila (y)
        self.c = c  # Columna (x)
        self.parent = None

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.get_logger().info('RRT Planner Initialized')

    def load_map(self, map_path):
        """Carga el mapa y convierte a rejilla binaria (0: libre, 1: obstáculo)."""
        try:
            img = Image.open(map_path).convert('L') # Escala de grises
            grid_data = np.array(img)
            
            binary_grid = (grid_data < 200).astype(int) 
            return binary_grid
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return None

    def euclidean_distance(self, node_a, point_b):
        """Distancia euclidiana entre un nodo y un punto (r, c)."""
        return np.sqrt((node_a.r - point_b[0])**2 + (node_a.c - point_b[1])**2)

    def line_of_sight(self, start, end, grid):
        """
        Algoritmo de Bresenham mejorado para comprobar colisiones pixel a pixel.
        Evita que el robot atraviese paredes o esquinas diagonales.
        """
        x0, y0 = int(start[0]), int(start[1])
        x1, y1 = int(end[0]), int(end[1])
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        rows, cols = grid.shape

        while True:
            # 1. Verificar límites
            if not (0 <= x0 < rows and 0 <= y0 < cols):
                return False 

            # 2. Verificar obstáculo directo
            if grid[x0, y0] == 1:
                return False
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            
            # Variables para detectar movimiento diagonal
            moved_x = False
            moved_y = False

            if e2 > -dy:
                err -= dy
                x0 += sx
                moved_x = True
            
            if e2 < dx:
                err += dx
                y0 += sy
                moved_y = True

            # 3. Verificación de esquinas diagonales (Evitar 'fugas')
            if moved_x and moved_y:
                check_x1, check_y1 = x0, y0 - sy
                check_x2, check_y2 = x0 - sx, y0
                
                obs1 = False
                obs2 = False

                if 0 <= check_x1 < rows and 0 <= check_y1 < cols:
                    if grid[check_x1, check_y1] == 1: obs1 = True
                
                if 0 <= check_x2 < rows and 0 <= check_y2 < cols:
                    if grid[check_x2, check_y2] == 1: obs2 = True

                if obs1 or obs2:
                    return False

        return True

    def get_nearest_node(self, tree, point):
        """Encuentra el nodo más cercano en el árbol a un punto dado."""
        nearest_node = tree[0]
        min_dist = float('inf')
        
        for node in tree:
            dist = self.euclidean_distance(node, point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def steer(self, from_node, to_point, step_size):
        """Calcula un nuevo punto en la dirección deseada, limitado por step_size."""
        dist = self.euclidean_distance(from_node, to_point)
        
        if dist < step_size:
            return to_point
        
        theta = math.atan2(to_point[0] - from_node.r, to_point[1] - from_node.c)
        
        new_r = from_node.r + step_size * math.sin(theta)
        new_c = from_node.c + step_size * math.cos(theta)
        
        return (int(new_r), int(new_c))

    def plan(self, grid, start, goal, max_iter=50000, step_size=15, goal_bias=0.1):
        """
        Algoritmo RRT (Rapidly-exploring Random Tree).
        Devuelve (camino, árbol completo, iteraciones)
        """
        rows, cols = grid.shape
        
        # Validaciones iniciales
        if not (0 <= start[0] < rows and 0 <= start[1] < cols) or grid[start] == 1:
            self.get_logger().error(f"Start {start} invalid/obstacle.")
            return None, [], 0
        if not (0 <= goal[0] < rows and 0 <= goal[1] < cols) or grid[goal] == 1:
            self.get_logger().error(f"Goal {goal} invalid/obstacle.")
            return None, [], 0

        start_node = TreeNode(start[0], start[1])
        tree = [start_node]
        
        self.get_logger().info(f"Starting RRT... (Max Iter: {max_iter}, Step: {step_size})")

        # --- CONTADOR ---
        for i in range(max_iter):
            # 1. Sampling
            if random.random() < goal_bias:
                rand_point = goal
            else:
                rand_point = (random.randint(0, rows - 1), random.randint(0, cols - 1))
            
            # 2. Nearest
            nearest_node = self.get_nearest_node(tree, rand_point)
            
            # 3. Steer
            new_point = self.steer(nearest_node, rand_point, step_size)
            
            # Límites
            if not (0 <= new_point[0] < rows and 0 <= new_point[1] < cols):
                continue

            # 4. Collision Check
            if self.line_of_sight((nearest_node.r, nearest_node.c), new_point, grid):
                new_node = TreeNode(new_point[0], new_point[1])
                new_node.parent = nearest_node
                tree.append(new_node)
                
                # 5. Check Goal
                dist_to_goal = self.euclidean_distance(new_node, goal)
                if dist_to_goal < step_size:
                    if self.line_of_sight((new_node.r, new_node.c), goal, grid):
                        final_node = TreeNode(goal[0], goal[1])
                        final_node.parent = new_node
                        tree.append(final_node)
                        
                        # --- MODIFICADO: Devuelve también 'i + 1' ---
                        self.get_logger().info(f"¡META ALCANZADA! >> Iteraciones: {i+1} | Nodos en Árbol: {len(tree)}")
                        return self.reconstruct_path(final_node), tree, i+1

        self.get_logger().warn(f"RRT limit ({max_iter}) reached without finding goal.")
        return None, tree, max_iter

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append((current_node.r, current_node.c))
            current_node = current_node.parent
        path.reverse()
        return path

    def plot_path(self, grid, path, tree, start, goal, total_iters, title="RRT Path"):
        plt.figure(figsize=(10, 10))
        plt.imshow(grid, cmap='Greys', origin='upper') 
        
        # Dibujar árbol (Ramas verdes)
        if tree:
            for node in tree:
                if node.parent:
                    plt.plot([node.c, node.parent.c], [node.r, node.parent.r], 'g-', alpha=0.3, linewidth=0.5)

        # Dibujar camino (Rojo)
        if path:
            ys = [p[0] for p in path]
            xs = [p[1] for p in path]
            plt.plot(xs, ys, color='red', linewidth=2, label='Path')

        plt.scatter(start[1], start[0], color='lime', s=100, edgecolors='black', label='Start')
        plt.scatter(goal[1], goal[0], color='blue', s=100, edgecolors='black', label='Goal')

        # --- MODIFICADO: Título con ambas estadísticas ---
        plt.title(f"{title} (RRT)\nIterations: {total_iters} | Tree Nodes: {len(tree)}")
        plt.legend()
        
        output_file = os.path.join("maps", f"rrt_result_{title.replace(' ', '_')}.png")
        plt.savefig(output_file)
        self.get_logger().info(f"Plot saved to {os.path.abspath(output_file)}")
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    planner = RRTPlanner()

    scenarios = [
        ("Scenario 1 Practice Map", "practica2.png", (50, 100), (380, 630)),
        ("Scenario 2 Maze01", "Maze01.png", (15, 5), (80, 150)),
        ("Scenario 3 Maze02", "Maze02.png", (870, 410), (420, 420)),
        ("Scenario 4 Casa", "casa.png", (50, 100), (800, 1100)),
        ("Scenario 5 Prado", "prado.png", (50, 30), (100, 400))
    ]

    for name, filename, start, goal in scenarios:
        planner.get_logger().info(f"--- Escenario: {name} ---")
        
        if os.path.exists(os.path.join("maps", filename)):
            map_path = os.path.join("maps", filename)
        else:
            map_path = filename 
        
        grid = planner.load_map(map_path)
             
        if grid is not None:
             rows, cols = grid.shape
             s = (max(0, min(start[0], rows-1)), max(0, min(start[1], cols-1)))
             g = (max(0, min(goal[0], rows-1)), max(0, min(goal[1], cols-1)))
             
             path, tree, iters = planner.plan(grid, s, g, max_iter=100000, step_size=20)
             planner.plot_path(grid, path, tree, s, g, iters, title=name)
        else:
            planner.get_logger().error(f"Could not load map: {filename}")

    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()