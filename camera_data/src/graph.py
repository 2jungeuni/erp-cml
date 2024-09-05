import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

class PolynomialVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Polynomial Visualizer")

        # 초기 다항식 계수 (두 개의 다항식)
        self.coeffs1 = [0.0004588, -0.5604, 250.6]
        self.coeffs2 = [0.0003, -0.4, 100.0]

        # 그래프 초기 설정
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # 입력 상자 설정
        self.input_frame1 = tk.Frame(self.root)
        self.input_frame1.pack(side=tk.TOP)

        self.entries1 = []
        self.entries2 = []
        labels = ["Coefficient a (y^2)", "Coefficient b (y)", "Coefficient c (constant)"]

        tk.Label(self.input_frame1, text="Polynomial 1").pack()
        for i, (coeff, label) in enumerate(zip(self.coeffs1, labels)):
            tk.Label(self.input_frame1, text=label).pack(side=tk.LEFT)
            entry = tk.Entry(self.input_frame1, width=10)
            entry.insert(0, str(coeff))
            entry.pack(side=tk.LEFT)
            entry.bind("<Return>", self.update_plot)
            self.entries1.append(entry)

        self.input_frame2 = tk.Frame(self.root)
        self.input_frame2.pack(side=tk.TOP)

        tk.Label(self.input_frame2, text="Polynomial 2").pack()
        for i, (coeff, label) in enumerate(zip(self.coeffs2, labels)):
            tk.Label(self.input_frame2, text=label).pack(side=tk.LEFT)
            entry = tk.Entry(self.input_frame2, width=10)
            entry.insert(0, str(coeff))
            entry.pack(side=tk.LEFT)
            entry.bind("<Return>", self.update_plot)
            self.entries2.append(entry)

        self.update_plot()

    def update_plot(self, event=None):
        # 입력된 계수 값 업데이트
        try:
            self.coeffs1 = [float(entry.get()) for entry in self.entries1]
            self.coeffs2 = [float(entry.get()) for entry in self.entries2]
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            return

        # 그래프 업데이트
        self.ax.clear()
        y = np.linspace(0, 500, 400)  # 이미지 높이에 맞게 y축 범위 설정
        x1 = np.polyval(self.coeffs1, y)
        x2 = np.polyval(self.coeffs2, y)
        
        # 이미지 좌표계에 맞게 y축을 뒤집고, 범위를 설정
        self.ax.plot(x1, y, label="Polynomial 1", color='blue')
        self.ax.plot(x2, y, label="Polynomial 2", color='red')
        self.ax.set_xlim(0, 300)  # x축을 이미지 좌표계에 맞게 설정
        self.ax.set_ylim(500, 0)  # y축을 이미지 좌표계에 맞게 역으로 설정
        self.ax.grid(True)
        self.ax.legend()

        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    visualizer = PolynomialVisualizer(root)
    root.mainloop()
