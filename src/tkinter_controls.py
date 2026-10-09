import tkinter as tk
from tkinter import ttk

import numpy as np


class ControlPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("2-Link Robot Controls")
        self.root.geometry("480x420")
        self.root.resizable(False, False)

        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.theta1 = tk.DoubleVar(value=0.0)
        self.theta2 = tk.DoubleVar(value=0.0)
        self.kp = tk.DoubleVar(value=30.0)
        self.kd = tk.DoubleVar(value=30.0)

        self.theta1_text = tk.StringVar()
        self.theta2_text = tk.StringVar()
        self.kp_text = tk.StringVar()
        self.kd_text = tk.StringVar()

        style = ttk.Style()
        style.theme_use("clam")

        style.configure(
            "TFrame",
            background="#1e1e1e",
        )

        style.configure(
            "TLabel",
            background="#1e1e1e",
            foreground="#eeeeee",
            font=("Ubuntu", 11),
        )

        style.configure(
            "Title.TLabel",
            background="#1e1e1e",
            foreground="#ffffff",
            font=("Ubuntu", 18, "bold"),
        )

        style.configure(
            "Subtitle.TLabel",
            background="#1e1e1e",
            foreground="#999999",
            font=("Ubuntu", 10),
        )

        style.configure(
            "Section.TLabel",
            background="#1e1e1e",
            foreground="#aaaaaa",
            font=("Ubuntu", 10, "bold"),
        )

        style.configure(
            "Value.TLabel",
            background="#1e1e1e",
            foreground="#ffffff",
            font=("Ubuntu Mono", 11),
        )

        style.configure(
            "TButton",
            font=("Ubuntu", 10),
            padding=(12, 6),
        )

        style.configure(
            "Horizontal.TScale",
            background="#1e1e1e",
        )

        main = ttk.Frame(self.root, padding=20)
        main.pack(fill="both", expand=True)

        main.columnconfigure(1, weight=1)

        ttk.Label(
            main,
            text="2-LINK ROBOT",
            style="Title.TLabel",
        ).grid(
            row=0,
            column=0,
            columnspan=3,
            sticky="w",
        )

        ttk.Label(
            main,
            text="Joint & Controller Parameters",
            style="Subtitle.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(2, 20),
        )

        ttk.Label(
            main,
            text="JOINT CONTROL",
            style="Section.TLabel",
        ).grid(
            row=2,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(0, 10),
        )

        ttk.Label(
            main,
            text="θ₁",
        ).grid(
            row=3,
            column=0,
            sticky="w",
            padx=(0, 15),
        )

        ttk.Label(
            main,
            textvariable=self.theta1_text,
            style="Value.TLabel",
            width=12,
            anchor="e",
        ).grid(
            row=3,
            column=1,
            sticky="e",
        )

        self.theta1_slider = ttk.Scale(
            main,
            from_=-np.pi,
            to=np.pi,
            orient="horizontal",
            variable=self.theta1,
        )

        self.theta1_slider.grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(2, 12),
        )

        ttk.Label(
            main,
            text="θ₂",
        ).grid(
            row=5,
            column=0,
            sticky="w",
            padx=(0, 15),
        )

        ttk.Label(
            main,
            textvariable=self.theta2_text,
            style="Value.TLabel",
            width=12,
            anchor="e",
        ).grid(
            row=5,
            column=1,
            sticky="e",
        )

        self.theta2_slider = ttk.Scale(
            main,
            from_=-np.pi,
            to=np.pi,
            orient="horizontal",
            variable=self.theta2,
        )

        self.theta2_slider.grid(
            row=6,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(2, 20),
        )

        ttk.Label(
            main,
            text="PD CONTROLLER",
            style="Section.TLabel",
        ).grid(
            row=7,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(0, 10),
        )

        ttk.Label(
            main,
            text="Kp",
        ).grid(
            row=8,
            column=0,
            sticky="w",
        )

        ttk.Label(
            main,
            textvariable=self.kp_text,
            style="Value.TLabel",
            width=12,
            anchor="e",
        ).grid(
            row=8,
            column=1,
            sticky="e",
        )

        self.kp_slider = ttk.Scale(
            main,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.kp,
        )

        self.kp_slider.grid(
            row=9,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(2, 12),
        )

        ttk.Label(
            main,
            text="Kd",
        ).grid(
            row=10,
            column=0,
            sticky="w",
        )

        ttk.Label(
            main,
            textvariable=self.kd_text,
            style="Value.TLabel",
            width=12,
            anchor="e",
        ).grid(
            row=10,
            column=1,
            sticky="e",
        )

        self.kd_slider = ttk.Scale(
            main,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.kd,
        )

        self.kd_slider.grid(
            row=11,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(2, 15),
        )

        ttk.Button(
            main,
            text="Reset",
            command=self.reset,
        ).grid(
            row=12,
            column=0,
            columnspan=3,
            pady=(5, 0),
        )

        self.update_display()

        self.theta1.trace_add("write", self.update_display)
        self.theta2.trace_add("write", self.update_display)
        self.kp.trace_add("write", self.update_display)
        self.kd.trace_add("write", self.update_display)

    def update_display(self, *args):
        self.theta1_text.set(f"{self.theta1.get():.2f} rad")
        self.theta2_text.set(f"{self.theta2.get():.2f} rad")
        self.kp_text.set(f"{self.kp.get():.0f}")
        self.kd_text.set(f"{self.kd.get():.0f}")

    def reset(self):
        self.theta1.set(0.0)
        self.theta2.set(0.0)
        self.kp.set(30.0)
        self.kd.set(30.0)

    def show(self):
        self.root.deiconify()

    def hide(self):
        self.root.withdraw()

    def close(self):
        self.root.destroy()


if __name__ == "__main__":
    panel = ControlPanel()
    panel.root.mainloop()
