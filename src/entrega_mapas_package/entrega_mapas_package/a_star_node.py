#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import heapq
import os
from PIL import Image

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_node')
        self.get_logger().info('A* Planner Initialized')

    def load_map(self, map_path):
        """Loads a map image and converts it to a binary grid (0: free, 1: obstacle)."""
        try:
            img = Image.open(map_path).convert('L') # Grayscale
            # Resize for performance if needed, or keep original resolution
            # img = img.resize((100, 100)) 
            grid_data = np.array(img)
            
            # Simple thresholding: Dark pixels are obstacles
            # Assuming 255 is free space (white) and 0 is obstacle (black)
            # You might need to invert this depending on your map
            binary_grid = (grid_data < 128).astype(int) 
            return binary_grid
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return None

    def heuristic(self, a, b):
        """Euclidean distance heuristic for Theta* (Any-angle)."""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def euclidean_distance(self, a, b):
        """Euclidean distance between two points."""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def line_of_sight(self, start, end, grid):
        """Bresenham's Line Algorithm to check visibility."""
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        while True:
            if grid[x0, y0] == 1:
                return False
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

    def get_neighbors(self, node, grid):
        """Returns valid neighbors (8-connected) and move costs."""
        rows, cols = grid.shape
        r, c = node
        # (dr, dc, cost)
        moves = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), # Straight
            (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), 
            (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2)) # Diagonal
        ]
        valid_neighbors = []
        for dr, dc, cost in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr, nc] == 0: # 0 is free space
                    valid_neighbors.append(((nr, nc), cost))
        return valid_neighbors

    def plan(self, grid, start, goal):
        """A* Algorithm implementation."""
        rows, cols = grid.shape
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            self.get_logger().error("Start point out of bounds")
            return None
        if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
            self.get_logger().error("Goal point out of bounds")
            return None
        if grid[start] == 1:
            self.get_logger().error("Start point is inside an obstacle")
            return None
        if grid[goal] == 1:
            self.get_logger().error("Goal point is inside an obstacle")
            return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        count = 0

        while open_set:
            count += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                self.get_logger().info(f"¡Meta encontrada! Nodos explorados: {count}")
                return self.reconstruct_path(came_from, current), count

            # Theta* Logic
            parent_node = came_from.get(current)

            for neighbor, cost in self.get_neighbors(current, grid):
                # Check for line of sight from parent to neighbor
                if parent_node and self.line_of_sight(parent_node, neighbor, grid):
                    # Path 2: parent -> neighbor
                    new_g_score = g_score[parent_node] + self.euclidean_distance(parent_node, neighbor)
                    if neighbor not in g_score or new_g_score < g_score[neighbor]:
                        came_from[neighbor] = parent_node
                        g_score[neighbor] = new_g_score
                        f = new_g_score + self.heuristic(neighbor, goal)
                        f_score[neighbor] = f
                        heapq.heappush(open_set, (f, neighbor))
                else:
                    # Path 1: current -> neighbor (Standard A*)
                    tentative_g_score = g_score[current] + cost
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f = tentative_g_score + self.heuristic(neighbor, goal)
                        f_score[neighbor] = f
                        heapq.heappush(open_set, (f, neighbor))
        
        self.get_logger().warn("No path found!")
        return None, count

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plot_path(self, grid, path, start, goal, count, title="A* Path"):
        plt.figure(figsize=(6, 6))
        plt.imshow(grid, cmap='Greys', origin='upper') # Display grid (0:White, 1:Black)
        
        if path:
            # Extract Y and X coordinates (row, col) -> (y, x) for plotting? 
            # imshow uses (row, col) but plot uses (x, y) usually.
            # Matplotlib imshow: axis 0 is y (rows), axis 1 is x (cols).
            ys = [p[0] for p in path]
            xs = [p[1] for p in path]
            plt.plot(xs, ys, color='red', linewidth=2, label='Path')

        plt.scatter(start[1], start[0], color='green', s=100, marker='o', label='Start') # x=col, y=row
        plt.scatter(goal[1], goal[0], color='blue', s=100, marker='x', label='Goal')

        plt.title(f"{title} (A*)\nNodes: {count}")
        plt.legend()
        plt.grid(False) # Grid lines might be confusing on top of pixel grid
        
        # Save plot to a file instead of showing (good for docker/headless) 
        output_file = os.path.join("maps", f"astar_result_{title.replace(' ', '_')}.png")
        plt.savefig(output_file)
        print(f"Plot saved to {os.path.abspath(output_file)}")
        # plt.show() 

    def save_debug_map(self, grid, start, goal, title="Debug Map"):
            """Guarda el mapa con una rejilla y los puntos de inicio/fin marcados."""
            # Configuración de la figura
            plt.figure(figsize=(12, 12)) # Hacemos la imagen grande para ver bien
            plt.imshow(grid, cmap='Greys', origin='upper')
            
            # Marcar inicio (Verde) y fin (Rojo)
            # IMPORTANTE: En plot es (Columna, Fila) -> (Y, X) del array
            plt.scatter(start[1], start[0], color='green', s=150, label=f'Start {start}', edgecolors='black') 
            plt.scatter(goal[1], goal[0], color='red', s=150, marker='X', label=f'Goal {goal}', edgecolors='black')
            
            # Activar rejilla y ejes
            plt.grid(True, which='both', color='red', linestyle='--', linewidth=0.5, alpha=0.3)
            plt.minorticks_on() 
            
            # Títulos y ejes
            plt.title(f"{title}\nEje X (Abajo) = Columnas | Eje Y (Izquierda) = Filas\nBusca coordenadas blancas para Start/Goal")
            plt.xlabel("Columna (X)")
            plt.ylabel("Fila (Y)")
            plt.legend()
            
            # Guardar
            output_file = os.path.join("maps", f"debug_coords_{title.replace(' ', '_')}.png")
            plt.savefig(output_file, dpi=150) # dpi alto para ver mejor los números
            self.get_logger().info(f"--> MAPA DE COORDENADAS GUARDADO EN: {output_file}")
            plt.close()

def create_dummy_map(filename, size=(50, 50)):
    # Create a white background
    img = Image.new('L', size, 255)
    pixels = img.load()
    
    # Draw some walls (black)
    for i in range(10, 40):
        pixels[20, i] = 0 # Vertical wall
    for i in range(20, 40):
        pixels[i, 10] = 0 # Horizontal wall
        
    img.save(filename)
    print(f"Created map: {filename}")

def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()

    # Define scenarios
    scenarios = [
        # (Name, MapFilename, Start, Goal)
        ("Scenario 1 Practice Map", "practica2.png", (50, 100), (380, 630)),
        ("Scenario 2 Maze01", "Maze01.png", (15, 5), (80, 150)),
        ("Scenario 3 Maze02", "Maze02.png", (870, 410), (420, 420)),
        ("Scenario 4 Casa", "casa.png", (50, 100), (800, 1100)),
        ("Scenario 5 Prado", "prado.png", (50, 30), (100, 400))
    ]

    create_dummy_map("map_simple.png")
    # Create trapped map
    img2 = Image.new('L', (20, 20), 255)
    pixels = img2.load()
    for i in range(20):
        pixels[i, 10] = 0
    img2.save("map_trapped.png")

    for name, filename, start, goal in scenarios:
        planner.get_logger().info(f"Running {name}...")
        map_path = os.path.join("maps", filename) if filename not in ["map_simple.png", "map_trapped.png"] else filename
        
        # Quick hack for simple/trapped to be found if not in maps/ (though we save them in cur dir)
        # Actually my previous code saved simple/trapped in CWD, but referenced them directly.
        # User asked to check images in maps/, let's assume external images are in maps/
        
        grid = planner.load_map(map_path)
        if grid is None:
             # Try simple/trapped in current dir if failed
             grid = planner.load_map(filename)
             
        if grid is not None:
             rows, cols = grid.shape
             
             # Handle dynamic coordinates
             current_goal = goal
             if current_goal == (None, None):
                 current_goal = (rows - 20, cols - 20)
            
             # Validate coordinates
             s = start
             g = current_goal
             
             # Ensure within bounds
             s = (max(0, min(s[0], rows-1)), max(0, min(s[1], cols-1)))
             g = (max(0, min(g[0], rows-1)), max(0, min(g[1], cols-1)))
             
             # planner.save_debug_map(grid, s, g, title=name)
             planner.get_logger().info(f"Planning {name} from {s} to {g}")
             path, count = planner.plan(grid, s, g)
             if path:
                 planner.plot_path(grid, path, s, g, count, title=name)
             else:
                 planner.get_logger().warn(f"No path found for {name}. Start/Goal might be in obstacle.")


    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
