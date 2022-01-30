import tkinter as tk
import tkinter.ttk as ttk
from tkinter import *

import numpy as np

from q1_rrt import rrt_connect, rrt_response


class Game:
    def __init__(self):
        self.game_window = None
        self.canvas = None
        self.game_mode = 0

        self.grid_width = 30
        self.grid_height = 17
        self.pixels_per_grid_unit = 25
        self.border = 25

        self.canvas_width = self.grid_width * self.pixels_per_grid_unit + 2 * self.border
        self.canvas_height = self.grid_height * self.pixels_per_grid_unit + 2 * self.border

        self.rrt_K = 750

        self.game_state = {
            "start": None,
            "end": None,
            "obstacles": np.zeros((self.grid_height, self.grid_width))
        }

        self.init_game_window()
        self.reset_game()

    def init_game_window(self):
        self.game_window = Tk()
        self.game_window.geometry('+250+0')
        self.game_window.title('RRT-Connect Motion Planning Algorithm')
        self.build_menu()

        self.canvas = Canvas(
            self.game_window,
            width=self.canvas_width,
            height=self.canvas_height)

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.pack()

    def build_menu(self):
        bottom = tk.Frame(self.canvas).pack(side="bottom")
        self.game_mode = tk.StringVar()
        self.game_mode_combobox = ttk.Combobox(width=18,
                                               textvariable=self.game_mode,
                                               state="readonly",
                                               justify='center',
                                               values=['Training Mode', 'Evaluation Mode'])
        self.start_mode = tk.Button(bottom, text="Set Start Location", fg="black", command=lambda x=1: self.set_game_mode(x)).pack()
        self.end_mode = tk.Button(bottom, text="Set Goal Location", fg="black", command=lambda x=2: self.set_game_mode(x)).pack()
        self.obstacle_mode = tk.Button(bottom, text="Set Obstacles", fg="black", command=lambda x=3: self.set_game_mode(x)).pack()
        self.run_rrt = tk.Button(bottom, text="Run RRT Algorithm", fg="black", command=self.run_rrt).pack()
        self.reset = tk.Button(bottom, text="Reset", fg="black", command=self.reset_game).pack()

    def reset_game(self):
        self.game_state = {
            "start": None,
            "end": None,
            "obstacles": np.zeros((self.grid_height, self.grid_width))
        }
        self.draw_current_game_state()

    def set_game_mode(self, game_mode):
        self.game_mode = game_mode

    def on_click(self, event):
        i = (event.y - self.border) // self.pixels_per_grid_unit
        j = (event.x - self.border) // self.pixels_per_grid_unit

        if self.game_mode == 1:
            self.game_state["start"] = (i, j)
            print(self.game_state["start"])

        elif self.game_mode == 2:
            self.game_state["end"] = (i, j)

        elif self.game_mode == 3:
            self.game_state["obstacles"][i, j] = not self.game_state["obstacles"][i, j]

        self.draw_current_game_state()

    def run_rrt(self):
        if self.game_state["start"] is not None and self.game_state["end"] is not None:
            response, path, tree_init, tree_goal, count = rrt_connect(self.game_state["start"],
                                                               self.game_state["end"],
                                                               self.game_state["obstacles"],
                                                               self.rrt_K)

            if count % 2 == 0:
                tree_init, tree_goal = tree_goal, tree_init

            self.draw_rrt_results(tree_init, tree_goal, path, response)

    def draw_current_game_state(self):
        self.canvas.create_rectangle(self.border,
                                     self.border,
                                     self.border + self.grid_width * self.pixels_per_grid_unit,
                                     self.border + self.grid_height * self.pixels_per_grid_unit,
                                     outline='black',
                                     fill='white')

        for i in range(1, self.grid_height):
            self.canvas.create_line(self.border,
                                    self.border + self.pixels_per_grid_unit * i,
                                    self.border + self.grid_width * self.pixels_per_grid_unit,
                                    self.border + self.pixels_per_grid_unit * i,
                                    fill='gray40')

        for i in range(1, self.grid_width):
            self.canvas.create_line(self.border + self.pixels_per_grid_unit * i,
                                    self.border,
                                    self.border + self.pixels_per_grid_unit * i,
                                    self.border + self.grid_height * self.pixels_per_grid_unit,
                                    fill='gray40')
        self.draw_obstacles()
        self.draw_landmark(self.game_state["end"], 'blue')
        self.draw_landmark(self.game_state["start"], 'red')

    def draw_obstacles(self):
        for i in range(len(self.game_state["obstacles"])):
            for j in range(len(self.game_state["obstacles"][0])):
                if self.game_state["obstacles"][i, j] != 0:
                    self.canvas.create_rectangle(self.border + j * self.pixels_per_grid_unit,
                                                 self.border + i * self.pixels_per_grid_unit,
                                                 self.border + j * self.pixels_per_grid_unit + self.pixels_per_grid_unit,
                                                 self.border + i * self.pixels_per_grid_unit + self.pixels_per_grid_unit,
                                                 outline='gray75',
                                                 fill='gray50')

    def draw_landmark(self, location, color):
        if location is None:
            return

        i = location[0]
        j = location[1]

        self.canvas.create_oval(self.border + j * self.pixels_per_grid_unit - 8,
                                self.border + i * self.pixels_per_grid_unit - 8,
                                self.border + j * self.pixels_per_grid_unit + 8,
                                self.border + i * self.pixels_per_grid_unit + 8,
                                outline=color,
                                fill=color)

    def draw_rrt_results(self, tree_init, tree_goal, path, response):
        vertices_init = np.array([[v[0], v[1]] for v, _ in tree_init.tree.items()])
        vertices_goal = np.array([[v[0], v[1]] for v, _ in tree_goal.tree.items()])

        edges_init = []
        for v, adj in tree_init.tree.items():
            for each in adj:
                edges_init.append([[v[1], each[1]], [v[0], each[0]]])

        edges_goal = []
        for v, adj in tree_goal.tree.items():
            for each in adj:
                edges_goal.append([[v[1], each[1]], [v[0], each[0]]])

        for e in edges_init:
            self.canvas.create_line(self.border + e[0][0] * self.pixels_per_grid_unit,
                                    self.border + e[1][0] * self.pixels_per_grid_unit,
                                    self.border + e[0][1] * self.pixels_per_grid_unit,
                                    self.border + e[1][1] * self.pixels_per_grid_unit,
                                    fill='red')
        for e in edges_goal:
            self.canvas.create_line(self.border + e[0][0] * self.pixels_per_grid_unit,
                                    self.border + e[1][0] * self.pixels_per_grid_unit,
                                    self.border + e[0][1] * self.pixels_per_grid_unit,
                                    self.border + e[1][1] * self.pixels_per_grid_unit,
                                    fill='blue')

        if response == rrt_response.SUCCESS:
            for i in range(len(path) - 1):
                self.canvas.create_line(self.border + path[i][1] * self.pixels_per_grid_unit,
                                        self.border + path[i][0] * self.pixels_per_grid_unit,
                                        self.border + path[i + 1][1] * self.pixels_per_grid_unit,
                                        self.border + path[i + 1][0] * self.pixels_per_grid_unit,
                                        fill='lime')

        for i in range(len(vertices_init)):
            self.canvas.create_oval(self.border + vertices_init[i, 1] * self.pixels_per_grid_unit - 3,
                                    self.border + vertices_init[i, 0] * self.pixels_per_grid_unit - 3,
                                    self.border + vertices_init[i, 1] * self.pixels_per_grid_unit + 3,
                                    self.border + vertices_init[i, 0] * self.pixels_per_grid_unit + 3,
                                    outline='red',
                                    fill='red')

        for i in range(len(vertices_goal)):
            self.canvas.create_oval(self.border + vertices_goal[i, 1] * self.pixels_per_grid_unit - 3,
                                    self.border + vertices_goal[i, 0] * self.pixels_per_grid_unit - 3,
                                    self.border + vertices_goal[i, 1] * self.pixels_per_grid_unit + 3,
                                    self.border + vertices_goal[i, 0] * self.pixels_per_grid_unit + 3,
                                    outline='blue',
                                    fill='blue')


def main():
    gui = Game()
    gui.game_window.mainloop()

if __name__ == "__main__":
    main()