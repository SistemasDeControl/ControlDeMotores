import numpy as np
import matplotlib
matplotlib.use("TkAgg", force=True)  # Backend Tkinter
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle, FancyBboxPatch, Polygon
import random
import time
import tkinter as tk
from tkinter import ttk

# ==========================================================
# ===============   MOTOR BUENO (Modelo)   =================
# ==========================================================
YMAX_DATA  = 15000.0   # tope físico de los motores
GRAPH_YMAX = 20000.0   # tope del eje para mostrar hasta 20k

def motor_step(rpm_prev, u, K, tau, Ts):
    rpm_new = rpm_prev + (Ts/tau) * (-rpm_prev + K * u)
    return max(0.0, min(rpm_new, YMAX_DATA))

class MotorBueno:
    def __init__(self, K=15000.0, tau=0.8, Ts=0.1):
        self.K = K
        self.tau = tau
        self.Ts = Ts
        self.rpm = 0.0
        self.running = False
        self.start_time = time.time()
        self.time = []
        self.rpm_history = []
        self.u_history = []

    def reset(self):
        self.rpm = 0.0
        self.running = False
        self.start_time = time.time()
        self.time.clear()
        self.rpm_history.clear()
        self.u_history.clear()

    def step(self):
        u = 1.0 if self.running else 0.0
        self.rpm = motor_step(self.rpm, u, self.K, self.tau, self.Ts)
        t = time.time() - self.start_time
        self.time.append(t)
        self.rpm_history.append(min(self.rpm, YMAX_DATA))
        self.u_history.append(u)

# ==========================================================
# ===============   MOTOR MALO (Aleatorio)   ===============
# ==========================================================
def generate_malo_rpm_and_u():
    u = 1.0 if random.random() < 0.7 else 0.0
    rpm = random.randint(8000, int(YMAX_DATA)) if u == 1.0 else random.randint(0, 1500)
    return rpm, u

# ==========================================================
# ===============   Botón redondeado Canvas   ==============
# ==========================================================
class RoundedButton(tk.Canvas):
    """
    Botón de Canvas con estética cuidada:
    - Fondo blanco, texto negro
    - Borde #d1d5db, sombra sutil
    - Hover y estado presionado
    - Cursor mano
    """
    def __init__(self, master, text, command=None, width=132, height=40, radius=16,
                 bg="#ffffff", fg="#111111",
                 border="#d1d5db", hover_bg="#f5f5f5", active_bg="#eeeeee",
                 shadow="#e5e7eb", font=("Segoe UI", 10, "bold"), **kwargs):
        super().__init__(master, width=width, height=height, highlightthickness=0,
                         bg=master.cget("bg") if hasattr(master, "cget") else "#ffffff", **kwargs)
        self.command = command
        self.radius = radius
        self.fill = bg
        self.hover_fill = hover_bg
        self.active_fill = active_bg
        self.fg = fg
        self.border = border
        self.shadow = shadow
        self.font = font
        self.state_active = False

        self.configure(cursor="hand2")

        w, h, r = width, height, radius
        # Sombra (ligeramente desplazada)
        self.shadow_id = self._round_rect(3, 3, w-1, h-1, r, fill=self.shadow, outline="")
        # Cuerpo
        self.body_id   = self._round_rect(1, 1, w-3, h-3, r, fill=self.fill, outline=self.border)
        # Texto
        self.text_id   = self.create_text(w//2, h//2, text=text, fill=self.fg, font=self.font)

        # Eventos
        for tag in (self.shadow_id, self.body_id, self.text_id):
            self.tag_bind(tag, "<Enter>", self._on_enter)
            self.tag_bind(tag, "<Leave>", self._on_leave)
            self.tag_bind(tag, "<Button-1>", self._on_press)
            self.tag_bind(tag, "<ButtonRelease-1>", self._on_release)

        # También sobre el canvas
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _round_rect(self, x1, y1, x2, y2, r, **kwargs):
        """
        Dibuja un rectángulo redondeado “limpio” con 4 arcos y 4 bordes rectos.
        """
        points = [
            x1+r, y1,
            x2-r, y1,
            x2,   y1,
            x2,   y1+r,
            x2,   y2-r,
            x2,   y2,
            x2-r, y2,
            x1+r, y2,
            x1,   y2,
            x1,   y2-r,
            x1,   y1+r,
            x1,   y1
        ]
        # Polígono suavizado + borde
        return self.create_polygon(points, smooth=True, splinesteps=36, **kwargs)

    def _on_enter(self, _):
        if not self.state_active:
            self.itemconfig(self.body_id, fill=self.hover_fill)

    def _on_leave(self, _):
        if not self.state_active:
            self.itemconfig(self.body_id, fill=self.fill)

    def _on_press(self, _):
        self.state_active = True
        self.itemconfig(self.body_id, fill=self.active_fill)

    def _on_release(self, _):
        self.state_active = False
        self.itemconfig(self.body_id, fill=self.hover_fill)
        if callable(self.command):
            self.command()

# ==========================================================
# ===============   APP TKINTER + MATPLOTLIB   =============
# ==========================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Motores — 15k RPM + pista estética y autos vectoriales (Start/Pause/Reset)")
        self.geometry("1200x860")

        # ---- Parámetros
        self.XWINDOW = 40.0
        self.UPDATE_MS = 50  # bucle con after

        # ---- Estado general
        self.running = False  # controla BUENO y MALO

        # ---- Estado MALO
        self.tiempo_malo = 0.0
        self.hist_time_malo, self.hist_rpm_malo, self.hist_u_malo = [], [], []

        # ---- Estado BUENO
        self.sim_bueno = MotorBueno()

        # ---- Pista (autos vectoriales)
        self.track_good_x = 0.07
        self.track_bad_x  = 0.07
        self.track_speed_scale = 0.008  # lento para apreciar

        # ---- UI
        self._init_styles()
        self._build_ui()
        self._build_figure()
        self._style_track(self.ax_track)     # FONDO BLANCO de la simulación
        self._setup_track_artists()

        # Atajos (no cambian la lógica)
        self.bind("<space>", lambda e: self._toggle())
        self.bind("<Key-r>", lambda e: self._reset())

        # ---- Bucle (after)
        self._tick()

    # ---------------- Estilos ttk ----------------
    def _init_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except:
            pass
        style.configure("Title.TLabel", font=("Segoe UI", 12, "bold"))

    # ---------------- UI ----------------
    def _build_ui(self):
        # NAV blanco
        top = tk.Frame(self, bg="#ffffff", padx=12, pady=12)
        top.pack(side="top", fill="x")

        tk.Label(top, text="Controles", bg="#ffffff", fg="#0f172a",
                 font=("Segoe UI", 12, "bold")).pack(side="left")

        btns = tk.Frame(top, bg="#ffffff")
        btns.pack(side="right")

        RoundedButton(btns, text="▶ Iniciar",  command=self._start).grid(row=0, column=0, padx=6)
        RoundedButton(btns, text="⏸ Pausar",   command=self._pause).grid(row=0, column=1, padx=6)
        RoundedButton(btns, text="⟲ Reiniciar",command=self._reset).grid(row=0, column=2, padx=6)

        # Canvas Matplotlib
        self.fig = Figure(figsize=(11.5, 8.2), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

    # ------------- Figure & Axes -------------
    def _build_figure(self):
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.2], hspace=0.6, wspace=0.35)

        self.ax_track      = self.fig.add_subplot(gs[0, :])
        self.ax_table_malo = self.fig.add_subplot(gs[1, 0])
        self.ax_table_buen = self.fig.add_subplot(gs[1, 1])
        self.ax_graph_malo = self.fig.add_subplot(gs[2, 0])
        self.ax_graph_buen = self.fig.add_subplot(gs[2, 1])

        # ---- Tablas
        column_labels = ["Métrica", "Valor"]
        self.table_malo  = self._make_table(self.ax_table_malo, column_labels)
        self.table_bueno = self._make_table(self.ax_table_buen, column_labels)

        # ---- Gráficas
        self.line_rpm_malo, = self.ax_graph_malo.plot([], [], label="RPM", color="#ef4444", linewidth=2.2)
        self.line_u_malo,   = self.ax_graph_malo.plot([], [], label="Entrada (u)*escala",
                                                      linestyle="--", color="#6b7280")
        self.ax_graph_malo.set_ylim(0, GRAPH_YMAX)
        self.ax_graph_malo.set_xlim(0, self.XWINDOW)
        self.ax_graph_malo.set_title("Motor Malo", fontsize=12)
        self.ax_graph_malo.set_xlabel("Tiempo [s]")
        self.ax_graph_malo.set_ylabel("RPM")
        self.ax_graph_malo.set_yticks([5000, 10000, 15000, 20000])
        self.ax_graph_malo.grid(True, color="#e5e7eb")
        for s in self.ax_graph_malo.spines.values(): s.set_color("#e5e7eb")
        self.ax_graph_malo.legend(loc="upper right")

        self.line_rpm_bueno, = self.ax_graph_buen.plot([], [], label="RPM", color="#2563eb", linewidth=2.2)
        self.line_u_bueno,   = self.ax_graph_buen.plot([], [], label="Entrada (u)*escala",
                                                       linestyle="--", color="#6b7280")
        self.ax_graph_buen.set_ylim(0, GRAPH_YMAX)
        self.ax_graph_buen.set_xlim(0, self.XWINDOW)
        self.ax_graph_buen.set_title("Motor Bueno", fontsize=12)
        self.ax_graph_buen.set_xlabel("Tiempo [s]")
        self.ax_graph_buen.set_ylabel("RPM")
        self.ax_graph_buen.set_yticks([5000, 10000, 15000, 20000])
        self.ax_graph_buen.grid(True, color="#e5e7eb")
        for s in self.ax_graph_buen.spines.values(): s.set_color("#e5e7eb")
        self.ax_graph_buen.legend(loc="upper right")

    def _make_table(self, ax, column_labels):
        rows = [
            ["Tiempo (s)", "0.00"],
            ["Entrada (u)", "0.00"],
            ["RPM actuales", "0.00"],
            ["Sobreimpulso (%)", "0.00"],
            ["Tiempo subida (s)", "0.00"],
            ["Tiempo asentamiento (s)", "0.00"],
            ["Error estacionario", "0.00"],
        ]
        tbl = ax.table(cellText=rows, colLabels=column_labels, loc="center", cellLoc="center")
        tbl.scale(1.3, 1.5)
        ax.axis("off")
        return tbl

    # --------- Estética de la PISTA (fondo blanco) ---------
    def _style_track(self, ax):
        ax.set_facecolor("#ffffff")
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")

        # Carretera en gris claro
        track = FancyBboxPatch((0.04, 0.38), 0.92, 0.24,
                               boxstyle="round,pad=0.01,rounding_size=0.03",
                               linewidth=0, facecolor="#e5e7eb")
        ax.add_patch(track)

        # línea central
        ax.plot([0.06, 0.94], [0.5, 0.5], lw=7.0, color="#9ca3af", solid_capstyle="round")

        # marcas verticales
        for i in range(12):
            x = 0.06 + i * (0.88 / 11)
            ax.plot([x, x], [0.46, 0.54], lw=2.4, color="#94a3b8", solid_capstyle="round")

    # ------------- Autos vectoriales -------------
    def _setup_track_artists(self):
        self.car_w, self.car_h = 0.06, 0.07

        # BUENO (azul)
        xb, yb = self.track_good_x, 0.57
        self.good_body = FancyBboxPatch((xb - self.car_w/2, yb - self.car_h/2), self.car_w, self.car_h,
                                        boxstyle="round,pad=0.01,rounding_size=0.015",
                                        fc="#2563eb", ec="none")
        self.good_glass = Polygon([[xb - 0.018, yb + 0.01],
                                   [xb + 0.018, yb + 0.01],
                                   [xb,          yb + 0.032]],
                                  closed=True, fc="#93c5fd", ec="none", alpha=0.9)
        self.good_l1 = Circle((xb - 0.022, yb - 0.028), 0.007, fc="#fde68a", ec="none")
        self.good_l2 = Circle((xb + 0.022, yb - 0.028), 0.007, fc="#fde68a", ec="none")
        self.good_w1 = Circle((xb - 0.025, yb - 0.035), 0.012, fc="#0b0d10", ec="#111", lw=0.3)
        self.good_w2 = Circle((xb + 0.025, yb - 0.035), 0.012, fc="#0b0d10", ec="#111", lw=0.3)
        self.good_shadow = Circle((xb, yb + 0.002), 0.035, fc="#000000", ec="none", alpha=0.06)
        self.good_label = self.ax_track.text(xb, yb + 0.055, "BUENO: 0 RPM", ha="center", va="bottom",
                                             color="#111111", fontsize=10, weight="bold")

        # MALO (naranja)
        xm, ym = self.track_bad_x, 0.43
        self.bad_body = FancyBboxPatch((xm - self.car_w/2, ym - self.car_h/2), self.car_w, self.car_h,
                                       boxstyle="round,pad=0.01,rounding_size=0.015",
                                       fc="#f97316", ec="none")
        self.bad_glass = Polygon([[xm - 0.018, ym + 0.01],
                                  [xm + 0.018, ym + 0.01],
                                  [xm,          ym + 0.032]],
                                 closed=True, fc="#fed7aa", ec="none", alpha=0.9)
        self.bad_l1 = Circle((xm - 0.022, ym - 0.028), 0.007, fc="#fde68a", ec="none")
        self.bad_l2 = Circle((xm + 0.022, ym - 0.028), 0.007, fc="#fde68a", ec="none")
        self.bad_w1 = Circle((xm - 0.025, ym - 0.035), 0.012, fc="#0b0d10", ec="#111", lw=0.3)
        self.bad_w2 = Circle((xm + 0.025, ym - 0.035), 0.012, fc="#0b0d10", ec="#111", lw=0.3)
        self.bad_shadow = Circle((xm, ym + 0.002), 0.035, fc="#000000", ec="none", alpha=0.06)
        self.bad_label = self.ax_track.text(xm, ym - 0.055, "MALO: 0 RPM", ha="center", va="top",
                                            color="#111111", fontsize=10, weight="bold")

        for a in [self.good_shadow, self.good_body, self.good_glass, self.good_l1, self.good_l2,
                  self.good_w1, self.good_w2,
                  self.bad_shadow, self.bad_body, self.bad_glass, self.bad_l1, self.bad_l2,
                  self.bad_w1, self.bad_w2]:
            self.ax_track.add_patch(a)

    def _track_move(self, which: str, x_center: float, rpm: float):
        if which == "good":
            body, glass, l1, l2, w1, w2, label, y = (
                self.good_body, self.good_glass, self.good_l1, self.good_l2,
                self.good_w1, self.good_w2, self.good_label, 0.57
            )
        else:
            body, glass, l1, l2, w1, w2, label, y = (
                self.bad_body, self.bad_glass, self.bad_l1, self.bad_l2,
                self.bad_w1, self.bad_w2, self.bad_label, 0.43
            )

        body.set_bounds(x_center - 0.03, y - 0.035, 0.06, 0.07)
        glass.set_xy(np.array([[x_center - 0.018, y + 0.01],
                               [x_center + 0.018, y + 0.01],
                               [x_center,          y + 0.032]]))
        l1.center = (x_center - 0.022, y - 0.028)
        l2.center = (x_center + 0.022, y - 0.028)
        w1.center = (x_center - 0.025, y - 0.035)
        w2.center = (x_center + 0.025, y - 0.035)
        label.set_position((x_center, y + (0.055 if which == "good" else -0.055)))
        label.set_text(("BUENO" if which == "good" else "MALO") + f": {int(rpm)} RPM")

    # ---------------- Controles ----------------
    def _toggle(self):
        if self.running:
            self._pause()
        else:
            self._start()

    def _start(self):
        self.running = True
        self.sim_bueno.running = True

    def _pause(self):
        self.running = False
        self.sim_bueno.running = False

    def _reset(self, *_):
        self.running = False
        self.sim_bueno.reset()
        # Malo
        self.tiempo_malo = 0.0
        self.hist_time_malo.clear(); self.hist_rpm_malo.clear(); self.hist_u_malo.clear()
        # Pista
        self.track_good_x = 0.07
        self.track_bad_x  = 0.07
        # Curvas
        self.line_rpm_malo.set_data([], []); self.line_u_malo.set_data([], [])
        self.line_rpm_bueno.set_data([], []); self.line_u_bueno.set_data([], [])
        # Tablas -> 0.00
        for i in range(1,8):
            self.table_malo._cells[(i,1)].get_text().set_text("0.00")
            self.table_bueno._cells[(i,1)].get_text().set_text("0.00")
        # Etiquetas/autos
        self._track_move("good", self.track_good_x, 0.0)
        self._track_move("bad",  self.track_bad_x,  0.0)
        self.canvas.draw_idle()

    # ---------------- Bucle (after) ----------------
    def _tick(self):
        if self.running:
            # ---- MALO ----
            rpm_m, u_m = generate_malo_rpm_and_u()
            self.tiempo_malo += 0.2
            t_now = time.time()
            if not self.hist_time_malo:
                self._t0_malo = t_now
            self.hist_time_malo.append(t_now)
            self.hist_rpm_malo.append(min(rpm_m, YMAX_DATA))
            self.hist_u_malo.append(u_m)

            # ventana 40s
            while len(self.hist_time_malo) > 0 and (self.hist_time_malo[-1] - self.hist_time_malo[0]) > self.XWINDOW:
                self.hist_time_malo.pop(0); self.hist_rpm_malo.pop(0); self.hist_u_malo.pop(0)

            times_rel_malo = [tt - self.hist_time_malo[0] for tt in self.hist_time_malo]
            self.line_rpm_malo.set_data(times_rel_malo, self.hist_rpm_malo)
            self.line_u_malo.set_data(times_rel_malo, [uu * GRAPH_YMAX for uu in self.hist_u_malo])

            # tabla MALO
            last_t_m = times_rel_malo[-1] if times_rel_malo else 0
            last_u_m = self.hist_u_malo[-1] if self.hist_u_malo else 0.0
            last_rpm_m = self.hist_rpm_malo[-1] if self.hist_rpm_malo else 0.0
            vals_m = [last_t_m, last_u_m, last_rpm_m,
                      random.uniform(5,20), random.uniform(0.5,2),
                      random.uniform(2,6),  random.uniform(0,0.2)]
            for i, v in enumerate(vals_m):
                self.table_malo._cells[(i+1,1)].get_text().set_text(f"{v:.2f}")

            # avance auto MALO
            v_bad = (last_rpm_m / YMAX_DATA) * self.track_speed_scale
            self.track_bad_x = 0.06 + ((self.track_bad_x - 0.06 + v_bad) % 0.88)

            # ---- BUENO ----
            self.sim_bueno.step()
            if self.sim_bueno.rpm_history:
                self.sim_bueno.rpm_history[-1] = min(self.sim_bueno.rpm_history[-1], YMAX_DATA)

            if self.sim_bueno.time:
                times = np.array(self.sim_bueno.time)
                t0 = times[0]
                times_rel_b = times - t0
                mask = (times_rel_b >= max(0, times_rel_b[-1] - self.XWINDOW))
                times_rel_win = times_rel_b[mask]
                rpm_win = np.array(self.sim_bueno.rpm_history)[mask]
                u_win   = np.array(self.sim_bueno.u_history)[mask]

                self.line_rpm_bueno.set_data(times_rel_win, rpm_win)
                self.line_u_bueno.set_data(times_rel_win, u_win * GRAPH_YMAX)

                # tabla BUENO
                last_t_b = times_rel_win[-1] if len(times_rel_win) else 0.0
                last_u_b = float(u_win[-1]) if len(u_win) else 0.0
                last_rpm_b = float(rpm_win[-1]) if len(rpm_win) else 0.0
                vals_b = [last_t_b, last_u_b, last_rpm_b,
                          random.uniform(5,15), random.uniform(0.5,1.5),
                          random.uniform(2,4),  random.uniform(0,0.1)]
                for i, v in enumerate(vals_b):
                    self.table_bueno._cells[(i+1,1)].get_text().set_text(f"{v:.2f}")

                # avance auto BUENO
                v_good = (last_rpm_b / YMAX_DATA) * self.track_speed_scale
                self.track_good_x = 0.06 + ((self.track_good_x - 0.06 + v_good) % 0.88)

        # --------- siempre reubica autos y ajusta ejes ----------
        rpm_bad_now = self.hist_rpm_malo[-1] if self.hist_rpm_malo else 0.0
        rpm_good_now = self.sim_bueno.rpm if hasattr(self.sim_bueno, "rpm") else 0.0
        self._track_move("good", self.track_good_x, rpm_good_now)
        self._track_move("bad",  self.track_bad_x,  rpm_bad_now)

        # xlim malo
        if self.hist_time_malo:
            tspan_m = self.hist_time_malo[-1] - self.hist_time_malo[0]
            self.ax_graph_malo.set_xlim(0, max(1, tspan_m))
        # xlim bueno
        if self.sim_bueno.time:
            times_rel_b = np.array(self.sim_bueno.time) - self.sim_bueno.time[0]
            if len(times_rel_b):
                self.ax_graph_buen.set_xlim(0, max(1, times_rel_b[-1] - max(0, times_rel_b[-1] - self.XWINDOW)))

        self.canvas.draw_idle()
        self.after(self.UPDATE_MS, self._tick)

# ----------------- Entry -----------------
if __name__ == "__main__":
    app = App()
    app.mainloop()
