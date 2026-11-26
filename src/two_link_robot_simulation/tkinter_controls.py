import tkinter as tk
from tkinter import ttk

import numpy as np

from .trajectory import MAX_JOINT_VELOCITY


class ControlPanel:
    def __init__(self):
        self.running = True

        self.root = tk.Tk()
        self.root.title("2-Link Robot Controls")
        self.root.geometry("480x470")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.theta1 = tk.DoubleVar(value=0.0)
        self.theta2 = tk.DoubleVar(value=0.0)
        self.kp = tk.DoubleVar(value=30.0)
        self.kd = tk.DoubleVar(value=30.0)
        self.max_velocity = tk.DoubleVar(value=MAX_JOINT_VELOCITY)

        self.theta1_text = tk.StringVar()
        self.theta2_text = tk.StringVar()
        self.kp_text = tk.StringVar()
        self.kd_text = tk.StringVar()
        self.max_velocity_text = tk.StringVar()

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
            text="2-Link Robot",
            style="Title.TLabel",
        ).grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(
            main,
            text="Value-based control",
            style="Subtitle.TLabel",
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 20))

        ttk.Label(
            main,
            text="JOINT ANGLES",
            style="Section.TLabel",
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Label(main, text="θ₁").grid(row=3, column=0, sticky="w")

        ttk.Scale(
            main,
            from_=-np.pi,
            to=np.pi,
            variable=self.theta1,
            orient="horizontal",
        ).grid(row=3, column=1, sticky="ew", padx=10)

        ttk.Label(
            main,
            textvariable=self.theta1_text,
            style="Value.TLabel",
            width=8,
        ).grid(row=3, column=2)

        ttk.Label(main, text="θ₂").grid(row=4, column=0, sticky="w")

        ttk.Scale(
            main,
            from_=-np.pi,
            to=np.pi,
            variable=self.theta2,
            orient="horizontal",
        ).grid(row=4, column=1, sticky="ew", padx=10)

        ttk.Label(
            main,
            textvariable=self.theta2_text,
            style="Value.TLabel",
            width=8,
        ).grid(row=4, column=2)

        ttk.Label(
            main,
            text="CONTROLLER",
            style="Section.TLabel",
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(20, 8))

        ttk.Label(main, text="Kp").grid(row=6, column=0, sticky="w")

        ttk.Scale(
            main,
            from_=0,
            to=100,
            variable=self.kp,
            orient="horizontal",
        ).grid(row=6, column=1, sticky="ew", padx=10)

        ttk.Label(
            main,
            textvariable=self.kp_text,
            style="Value.TLabel",
            width=8,
        ).grid(row=6, column=2)

        ttk.Label(main, text="Kd").grid(row=7, column=0, sticky="w")

        ttk.Scale(
            main,
            from_=0,
            to=100,
            variable=self.kd,
            orient="horizontal",
        ).grid(row=7, column=1, sticky="ew", padx=10)

        ttk.Label(
            main,
            textvariable=self.kd_text,
            style="Value.TLabel",
            width=8,
        ).grid(row=7, column=2)

        ttk.Label(main, text="ω max").grid(row=8, column=0, sticky="w")

        ttk.Scale(
            main,
            from_=0.5,
            to=4.0,
            variable=self.max_velocity,
            orient="horizontal",
        ).grid(row=8, column=1, sticky="ew", padx=10)

        ttk.Label(
            main,
            textvariable=self.max_velocity_text,
            style="Value.TLabel",
            width=8,
        ).grid(row=8, column=2)

        ttk.Button(
            main,
            text="Reset",
            command=self.reset,
        ).grid(row=9, column=0, columnspan=3, pady=25)

        self.update_values()

    def update_values(self):
        self.theta1_text.set(f"{self.theta1.get():.2f}")
        self.theta2_text.set(f"{self.theta2.get():.2f}")
        self.kp_text.set(f"{self.kp.get():.0f}")
        self.kd_text.set(f"{self.kd.get():.0f}")
        self.max_velocity_text.set(f"{self.max_velocity.get():.1f} r/s")
        self.root.after(50, self.update_values)

    def reset(self):
        self.theta1.set(0.0)
        self.theta2.set(0.0)
        self.kp.set(30.0)
        self.kd.set(30.0)
        self.max_velocity.set(MAX_JOINT_VELOCITY)

    def show(self):
        self.root.deiconify()

    def hide(self):
        self.root.withdraw()

    def set_angles(self, theta1: float, theta2: float):
        """Move the angle sliders, e.g. to the current goal on entering mode 2."""
        self.theta1.set(float(np.clip(theta1, -np.pi, np.pi)))
        self.theta2.set(float(np.clip(theta2, -np.pi, np.pi)))

    def close(self):
        # Called by the window's close button and again on shutdown;
        # destroying an already destroyed Tk root raises TclError.
        if not self.running:
            return

        self.running = False
        self.root.destroy()
