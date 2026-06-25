#!/usr/bin/env python3
"""
digital_twin_app.py
Real-Time Dual-Sensor Operational Modal Analysis
Optimized with Thread-Safe Shared Memory & Live 3D OpenGL Beam Visualization
═══════════════════════════════════════════════════════════════════════
"""

import sys
import time
import numpy as np
from collections import deque
import pyqtgraph as pg
from PyQt5.QtCore import QThread, Qt, QTimer, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QComboBox, QPushButton, QLabel, QToolBar, QStatusBar, QMessageBox)

# 3D Graphics Engine
try:
    import pyqtgraph.opengl as gl
    GL_AVAILABLE = True
except ImportError:
    GL_AVAILABLE = False

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ─── HARDWARE & PHYSICS CONFIGURATION ────────────────────────────────
BAUD_RATE = 500000         
SAMPLE_RATE_HZ = 1000.0    
WINDOW_SIZE = 1024         
HP_ALPHA = 0.98            
LSB_PER_G = 16384.0        
LSB_PER_DEG = 65.5      # Gyroscope scale factor (500 deg/s config)

# ─── 3D MODEL CONFIGURATION ──────────────────────────────────────────
BEAM_LENGTH_CM = 22.5
NODE_POS_CM = 11.4      # Mid sensor placement
Z_VISUAL_SCALE = 3.0    # Visual multiplier so g-forces look like physical bending
ANGLE_VISUAL_SCALE = 0.5 # Visual multiplier for gyro tilt

# ─── MODERN UI PALETTE (Catppuccin Macchiato Inspired) ───────────────
CLR_BG = "#181825"         
CLR_PANEL = "#1E1E2E"      
CLR_TEXT = "#CDD6F4"       
CLR_SUBTEXT = "#A6ADC8"    
CLR_ACCENT = "#CBA6F7"     
CLR_ACCENT_HOVER = "#B4BEFE" 
CLR_GRID = "#313244"       
CLR_TIP = "#89B4FA"        
CLR_MID = "#F38BA8"        

class SerialWorker(QThread):
    """
    Background thread that reads serial data and pushes it to shared memory.
    """
    connection_status = pyqtSignal(bool)

    def __init__(self, port, buf_tip, buf_mid, buf_gy_tip, buf_gy_mid):
        super().__init__()
        self._port = port
        self._buf_tip = buf_tip
        self._buf_mid = buf_mid
        self._buf_gy_tip = buf_gy_tip
        self._buf_gy_mid = buf_gy_mid
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        ser = None
        try:
            ser = serial.Serial()
            ser.port = self._port
            ser.baudrate = BAUD_RATE
            ser.timeout = 1
            ser.setDTR(False)
            ser.setRTS(False)
            ser.open()
            
            time.sleep(1)  
            ser.reset_input_buffer() 
            self.connection_status.emit(True)
            
            prev_raw_t = np.zeros(3); prev_hp_t = np.zeros(3)
            prev_raw_m = np.zeros(3); prev_hp_m = np.zeros(3)
            prev_gy_t = 0.0; prev_hp_gy_t = 0.0
            prev_gy_m = 0.0; prev_hp_gy_m = 0.0
            
            while self._running:
                if ser.in_waiting == 0:
                    time.sleep(0.001)
                    continue

                raw_line = ser.readline()
                try:
                    decoded = raw_line.decode("ascii", errors="replace").strip()
                    if not decoded or decoded.startswith("time"): continue
                    
                    p = decoded.split(",")
                    if len(p) < 13: continue # Now expecting 13 columns (6-DoF * 2)
                    
                    # Extract Tip Data (Accel & Y-Axis Gyro)
                    raw_t = np.array([float(p[1])/LSB_PER_G, float(p[2])/LSB_PER_G, float(p[3])/LSB_PER_G])
                    gy_t_raw = float(p[5])/LSB_PER_DEG  
                    
                    # Extract Mid Data (Accel & Y-Axis Gyro)
                    raw_m = np.array([float(p[7])/LSB_PER_G, float(p[8])/LSB_PER_G, float(p[9])/LSB_PER_G])
                    gy_m_raw = float(p[11])/LSB_PER_DEG 
                    
                    # High-Pass Filter Math (Accel for displacement proxy)
                    hp_t = HP_ALPHA * (prev_hp_t + raw_t - prev_raw_t)
                    hp_m = HP_ALPHA * (prev_hp_m + raw_m - prev_raw_m)
                    
                    # High-Pass Filter Math (Gyro for angle proxy)
                    hp_gy_t = HP_ALPHA * (prev_hp_gy_t + gy_t_raw - prev_gy_t)
                    hp_gy_m = HP_ALPHA * (prev_hp_gy_m + gy_m_raw - prev_gy_m)
                    
                    prev_raw_t, prev_hp_t = raw_t, hp_t
                    prev_raw_m, prev_hp_m = raw_m, hp_m
                    prev_gy_t, prev_hp_gy_t = gy_t_raw, hp_gy_t
                    prev_gy_m, prev_hp_gy_m = gy_m_raw, hp_gy_m
                    
                    self._buf_tip.append(hp_t[2]) 
                    self._buf_mid.append(hp_m[2])
                    self._buf_gy_tip.append(hp_gy_t)
                    self._buf_gy_mid.append(hp_gy_m)
                        
                except Exception:
                    continue
        finally:
            if ser and ser.is_open: ser.close()
            self.connection_status.emit(False)

class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚙ Dual-Sensor Modal Analysis Engine & 3D Twin")
        self.resize(1600, 850) # Made wider to fit the 3D model beautifully
        
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {CLR_BG}; }}
            QWidget {{ font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px; color: {CLR_TEXT}; }}
            QToolBar {{ background-color: {CLR_PANEL}; border-bottom: 2px solid {CLR_GRID}; padding: 12px; spacing: 20px; }}
            QPushButton {{ background-color: {CLR_ACCENT}; color: #11111B; font-weight: bold; border-radius: 6px; padding: 8px 24px; font-size: 14px; }}
            QPushButton:hover {{ background-color: {CLR_ACCENT_HOVER}; }}
            QComboBox {{ background-color: {CLR_BG}; color: {CLR_TEXT}; border-radius: 6px; padding: 8px 16px; border: 1px solid {CLR_GRID}; min-width: 150px; font-weight: bold; }}
            QComboBox QAbstractItemView {{ background-color: {CLR_BG}; color: {CLR_TEXT}; selection-background-color: {CLR_ACCENT}; selection-color: #11111B; border: 1px solid {CLR_GRID}; }}
            QLabel {{ color: {CLR_SUBTEXT}; font-weight: bold; }}
            QStatusBar {{ background-color: {CLR_PANEL}; color: {CLR_SUBTEXT}; border-top: 1px solid {CLR_GRID}; font-family: 'Consolas', monospace; }}
        """)

        if not GL_AVAILABLE:
            QMessageBox.critical(self, "Missing Dependency", "PyOpenGL is required for the 3D visualization.\nPlease run: pip install PyOpenGL PyOpenGL_accelerate")
            sys.exit(1)

        self._worker = None
        self._buf_z_tip = deque([0.0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self._buf_z_mid = deque([0.0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self._buf_gy_tip = deque([0.0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self._buf_gy_mid = deque([0.0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

        self._build_ui()
        
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_plots)
        self._timer.start(33) 

    def _build_ui(self):
        toolbar = QToolBar()
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        self._port_combo = QComboBox()
        if SERIAL_AVAILABLE:
            for p in serial.tools.list_ports.comports(): 
                self._port_combo.addItem(p.device)
        
        toolbar.addWidget(QLabel(" COM PORT  "))
        toolbar.addWidget(self._port_combo)
        
        spacer = QWidget()
        spacer.setFixedWidth(20)
        toolbar.addWidget(spacer)
        
        self._btn_connect = QPushButton("CONNECT DATALINK")
        self._btn_connect.clicked.connect(self._toggle_conn)
        toolbar.addWidget(self._btn_connect)
        
        central = QWidget()
        self.setCentralWidget(central)
        
        # ─── MASTER HORIZONTAL SPLIT ─────────────────────────────────────────
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # ─── LEFT COLUMN: 2D ANALYTICS ───────────────────────────────────────
        left_layout = QVBoxLayout()
        pg.setConfigOptions(antialias=False) 
        font = QFont('Segoe UI', 10)

        self._plot_time = pg.PlotWidget()
        self._plot_time.setBackground(CLR_PANEL)
        self._plot_time.setTitle("Z-Axis Phase Comparison", color=CLR_TEXT, size="12pt")
        self._plot_time.showGrid(x=True, y=True, alpha=0.2)
        self._plot_time.getAxis('left').setPen(CLR_GRID)
        self._plot_time.getAxis('bottom').setPen(CLR_GRID)
        self._plot_time.getAxis('left').setTextPen(CLR_SUBTEXT)
        self._plot_time.getAxis('bottom').setTextPen(CLR_SUBTEXT)
        
        legend = self._plot_time.addLegend(offset=(20, 20))
        legend.setLabelTextColor(CLR_TEXT)
        self._curve_tip_t = self._plot_time.plot(pen=pg.mkPen(CLR_TIP, width=1.5), name=" Tip Sensor")
        self._curve_mid_t = self._plot_time.plot(pen=pg.mkPen(CLR_MID, width=1.5, style=Qt.DashLine), name=" Mid Sensor")
        left_layout.addWidget(self._plot_time)

        self._plot_fft = pg.PlotWidget()
        self._plot_fft.setBackground(CLR_PANEL)
        self._plot_fft.setTitle("Dual FFT Spectrum (0-500 Hz)", color=CLR_TEXT, size="12pt")
        self._plot_fft.showGrid(x=True, y=True, alpha=0.2)
        self._plot_fft.setXRange(0, SAMPLE_RATE_HZ / 2.0)
        self._plot_fft.getAxis('left').setPen(CLR_GRID)
        self._plot_fft.getAxis('bottom').setPen(CLR_GRID)
        self._plot_fft.getAxis('left').setTextPen(CLR_SUBTEXT)
        self._plot_fft.getAxis('bottom').setTextPen(CLR_SUBTEXT)
        
        legend_fft = self._plot_fft.addLegend(offset=(20, 20))
        legend_fft.setLabelTextColor(CLR_TEXT)
        self._curve_tip_f = self._plot_fft.plot(pen=pg.mkPen(CLR_TIP, width=1.5), name=" Tip Power")
        self._curve_mid_f = self._plot_fft.plot(pen=pg.mkPen(CLR_MID, width=1.5), name=" Mid Power")
        left_layout.addWidget(self._plot_fft)

        main_layout.addLayout(left_layout, stretch=5)

        # ─── RIGHT COLUMN: 3D DIGITAL TWIN ───────────────────────────────────
        right_layout = QVBoxLayout()
        header_3d = QLabel("LIVE 3D STRUCTURAL DEFORMATION")
        header_3d.setAlignment(Qt.AlignCenter)
        header_3d.setStyleSheet(f"font-weight: bold; color: {CLR_ACCENT}; font-size: 14px; letter-spacing: 1px;")
        right_layout.addWidget(header_3d)

        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor((30, 30, 46, 255)) # Matches CLR_PANEL
        self.gl_view.setCameraPosition(distance=45, elevation=15, azimuth=-45)
        
        # 1. 3D Grid Floor
        grid = gl.GLGridItem()
        grid.setSize(x=50, y=50, z=50)
        grid.setSpacing(x=5, y=5, z=5)
        grid.translate(10, 0, -10)
        self.gl_view.addItem(grid)

        # 2. Wooden Clamp Block (x = -7.5 to 0)
        self.wood_block = gl.GLBoxItem(size=pg.Vector(7.5, 4.0, 4.0), color=(80, 80, 90, 255))
        self.wood_block.translate(-7.5, -2.0, -2.0)
        self.gl_view.addItem(self.wood_block)

        # 3. The Aluminum Beam (Interpolated Line)
        self.beam_line = gl.GLLinePlotItem(pos=np.zeros((20,3)), color=pg.glColor(CLR_TEXT), width=4, antialias=True)
        self.gl_view.addItem(self.beam_line)

        # 4. Sensor Markers (Now actual 3D boxes that physically tilt!)
        self.mid_sensor_box = gl.GLBoxItem(size=pg.Vector(2, 2, 1), color=pg.glColor(CLR_MID))
        self.gl_view.addItem(self.mid_sensor_box)
        
        self.tip_sensor_box = gl.GLBoxItem(size=pg.Vector(2, 2, 1), color=pg.glColor(CLR_TIP))
        self.gl_view.addItem(self.tip_sensor_box)

        right_layout.addWidget(self.gl_view)
        main_layout.addLayout(right_layout, stretch=4)

        # ─── STATUS BAR ──────────────────────────────────────────────────────
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("SYSTEM IDLE. READY TO CONNECT.")

    @pyqtSlot()
    def _toggle_conn(self):
        if self._worker:
            self._worker.stop()
            self._worker.wait()
            self._worker = None
            self._btn_connect.setText("CONNECT DATALINK")
            self._btn_connect.setStyleSheet(f"background-color: {CLR_ACCENT}; color: #11111B;")
            self.statusBar.showMessage("DATALINK DISCONNECTED.")
            
            # Reset 3D Model to Flat
            self._update_3d_beam(0.0, 0.0, 0.0, 0.0)
        else:
            self._worker = SerialWorker(self._port_combo.currentText(), self._buf_z_tip, self._buf_z_mid, self._buf_gy_tip, self._buf_gy_mid)
            self._worker.start()
            self._btn_connect.setText("DISCONNECT")
            self._btn_connect.setStyleSheet(f"background-color: {CLR_MID}; color: #11111B;")

    def _update_3d_beam(self, z_mid_accel, z_tip_accel, gy_mid_raw, gy_tip_raw):
        """Calculates the physical beam bending curve and updates OpenGL objects"""
        # Scale the raw G-force data so it looks like physical displacement in cm
        z_mid = z_mid_accel * Z_VISUAL_SCALE
        z_tip = z_tip_accel * Z_VISUAL_SCALE
        
        # Scale the gyro data to create a realistic physical tilt
        angle_mid = gy_mid_raw * ANGLE_VISUAL_SCALE
        angle_tip = gy_tip_raw * ANGLE_VISUAL_SCALE

        # Generate 20 points along the beam using a 2nd-degree polynomial curve 
        # to interpolate between the root (0,0), mid sensor, and tip sensor
        x_pts = np.linspace(0, BEAM_LENGTH_CM, 20)
        try:
            # Polyfit to create a smooth physical bending arc
            curve_fit = np.polyfit([0, NODE_POS_CM, BEAM_LENGTH_CM], [0, z_mid, z_tip], 2)
            z_pts = np.polyval(curve_fit, x_pts)
        except:
            z_pts = np.zeros_like(x_pts)

        # Update Beam Line
        beam_coords = np.vstack([x_pts, np.zeros_like(x_pts), z_pts]).T
        self.beam_line.setData(pos=beam_coords)

        # Update Sensor Cubes (Apply both Height AND Pitch Tilt)
        
        # Mid Sensor Box
        self.mid_sensor_box.resetTransform()
        self.mid_sensor_box.translate(-1, -1, -0.5)          # Center box on its origin
        self.mid_sensor_box.rotate(angle_mid, 0, 1, 0)       # Tilt around Y axis (Pitch)
        self.mid_sensor_box.translate(NODE_POS_CM, 0, z_mid) # Move to physical location
        
        # Tip Sensor Box
        self.tip_sensor_box.resetTransform()
        self.tip_sensor_box.translate(-1, -1, -0.5)
        self.tip_sensor_box.rotate(angle_tip, 0, 1, 0)
        self.tip_sensor_box.translate(BEAM_LENGTH_CM, 0, z_tip)

    @pyqtSlot()
    def _update_plots(self):
        zt_arr = np.array(self._buf_z_tip)
        zm_arr = np.array(self._buf_z_mid)
        
        self._curve_tip_t.setData(zt_arr)
        self._curve_mid_t.setData(zm_arr)

        win = np.hanning(WINDOW_SIZE)
        fft_t = np.abs(np.fft.rfft(zt_arr * win)) / WINDOW_SIZE
        fft_m = np.abs(np.fft.rfft(zm_arr * win)) / WINDOW_SIZE
        freqs = np.fft.rfftfreq(WINDOW_SIZE, d=1.0/SAMPLE_RATE_HZ)

        self._curve_tip_f.setData(freqs, fft_t)
        self._curve_mid_f.setData(freqs, fft_m)

        # Push the newest displacement AND gyro data points to the 3D visualizer
        self._update_3d_beam(zm_arr[-1], zt_arr[-1], self._buf_gy_mid[-1], self._buf_gy_tip[-1])

        min_idx = int(5.0 / (SAMPLE_RATE_HZ / WINDOW_SIZE))
        if len(fft_t) > min_idx:
            pk_t = np.argmax(fft_t[min_idx:]) + min_idx
            pk_m = np.argmax(fft_m[min_idx:]) + min_idx
            self.statusBar.showMessage(f"LIVE METRICS   >>>   TIP DOMINANT: {freqs[pk_t]:.2f} Hz   ||   MID DOMINANT: {freqs[pk_m]:.2f} Hz")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Dashboard()
    w.show()
    sys.exit(app.exec_())