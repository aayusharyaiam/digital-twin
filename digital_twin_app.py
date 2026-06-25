#!/usr/bin/env python3
"""
digital_twin_app.py
Real-Time Dual-Sensor Operational Modal Analysis
Reverted to Original Fast-Path Signal Processing + Modern UI
═══════════════════════════════════════════════════════════════════════
"""

import sys
import time
import numpy as np
from collections import deque
import pyqtgraph as pg
from PyQt5.QtCore import QMutex, QThread, Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QComboBox, QPushButton, QLabel, QToolBar, QStatusBar)

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ─── HARDWARE CONFIGURATION ──────────────────────────────────────────
BAUD_RATE = 500000         
SAMPLE_RATE_HZ = 1000.0    
WINDOW_SIZE = 1024         
HP_ALPHA = 0.98            
LSB_PER_G = 16384.0        

# ─── MODERN UI PALETTE (Catppuccin Macchiato Inspired) ───────────────
CLR_BG = "#181825"         
CLR_PANEL = "#1E1E2E"      
CLR_TEXT = "#CDD6F4"       
CLR_SUBTEXT = "#A6ADC8"    
CLR_ACCENT = "#CBA6F7"     
CLR_ACCENT_HOVER = "#B4BEFE" 
CLR_GRID = "#313244"       
CLR_TIP = "#89B4FA"        # Neon Blue for Free End (Tip)
CLR_MID = "#F38BA8"        # Neon Pink for Middle (Node)

class SerialWorker(QThread):
    # Reverted to emitting single frames of floats (highly optimized in PyQt)
    data_ready = pyqtSignal(float, float, float, float, float, float)
    connection_status = pyqtSignal(bool)

    def __init__(self, port):
        super().__init__()
        self._port = port
        self._running = True
        self._mutex = QMutex()

    def stop(self):
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()

    def run(self):
        ser = None
        try:
            ser = serial.Serial(self._port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino auto-reset
            
            # Flush the serial buffer to instantly destroy any laggy/backed-up data
            ser.reset_input_buffer()
            self.connection_status.emit(True)
            
            while True:
                self._mutex.lock()
                if not self._running: 
                    self._mutex.unlock()
                    break
                self._mutex.unlock()

                raw_line = ser.readline()
                if not raw_line: continue
                try:
                    decoded = raw_line.decode("ascii", errors="replace").strip()
                    if not decoded or decoded.startswith("time"): continue
                    
                    p = decoded.split(",")
                    if len(p) < 7: continue
                    
                    # Convert to G's and emit immediately
                    xt, yt, zt = float(p[1])/LSB_PER_G, float(p[2])/LSB_PER_G, float(p[3])/LSB_PER_G
                    xm, ym, zm = float(p[4])/LSB_PER_G, float(p[5])/LSB_PER_G, float(p[6])/LSB_PER_G
                    
                    self.data_ready.emit(xt, yt, zt, xm, ym, zm)
                        
                except Exception:
                    continue
        finally:
            if ser and ser.is_open: ser.close()
            self.connection_status.emit(False)

class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚙ Dual-Sensor Modal Analysis Engine (Ultra-Low Latency)")
        self.resize(1400, 800)
        
        # Apply Modern Stylesheet
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

        self._worker = None
        self._buf_z_tip = deque(maxlen=WINDOW_SIZE)
        self._buf_z_mid = deque(maxlen=WINDOW_SIZE)
        
        self._prev_raw_t = np.zeros(3)
        self._prev_hp_t = np.zeros(3)
        self._prev_raw_m = np.zeros(3)
        self._prev_hp_m = np.zeros(3)

        self._build_ui()
        
        # UI Refresh Timer (33ms = ~30 FPS for smooth graphing)
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
        layout = QVBoxLayout(central)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)

        # Disable Anti-aliasing globally for maximum rendering speed
        pg.setConfigOptions(antialias=False)
        font = QFont('Segoe UI', 10)

        # ─── TIME DOMAIN PLOT ────────────────────────────────────────────────
        self._plot_time = pg.PlotWidget()
        self._plot_time.setBackground(CLR_PANEL)
        self._plot_time.setTitle("Z-Axis Phase Comparison (Mode Shape Visualizer)", color=CLR_TEXT, size="14pt")
        self._plot_time.showGrid(x=True, y=True, alpha=0.2)
        self._plot_time.getAxis('left').setPen(CLR_GRID)
        self._plot_time.getAxis('bottom').setPen(CLR_GRID)
        self._plot_time.getAxis('left').setTextPen(CLR_SUBTEXT)
        self._plot_time.getAxis('bottom').setTextPen(CLR_SUBTEXT)
        self._plot_time.getAxis('left').setTickFont(font)
        self._plot_time.getAxis('bottom').setTickFont(font)
        
        legend = self._plot_time.addLegend(offset=(20, 20))
        legend.setLabelTextColor(CLR_TEXT)
        
        # Thin lines (width=1.5) for less GPU strain
        self._curve_tip_t = self._plot_time.plot(pen=pg.mkPen(CLR_TIP, width=1.5), name=" Tip Sensor (Free End)")
        self._curve_mid_t = self._plot_time.plot(pen=pg.mkPen(CLR_MID, width=1.5, style=Qt.DashLine), name=" Mid Sensor (Belly)")
        layout.addWidget(self._plot_time)

        # ─── FFT DOMAIN PLOT ─────────────────────────────────────────────────
        self._plot_fft = pg.PlotWidget()
        self._plot_fft.setBackground(CLR_PANEL)
        self._plot_fft.setTitle("Dual FFT Spectrum (Resonance Frequency)", color=CLR_TEXT, size="14pt")
        self._plot_fft.showGrid(x=True, y=True, alpha=0.2)
        self._plot_fft.setXRange(0, SAMPLE_RATE_HZ / 2.0)
        self._plot_fft.getAxis('left').setPen(CLR_GRID)
        self._plot_fft.getAxis('bottom').setPen(CLR_GRID)
        self._plot_fft.getAxis('left').setTextPen(CLR_SUBTEXT)
        self._plot_fft.getAxis('bottom').setTextPen(CLR_SUBTEXT)
        self._plot_fft.getAxis('left').setTickFont(font)
        self._plot_fft.getAxis('bottom').setTickFont(font)
        
        legend_fft = self._plot_fft.addLegend(offset=(20, 20))
        legend_fft.setLabelTextColor(CLR_TEXT)
        
        self._curve_tip_f = self._plot_fft.plot(pen=pg.mkPen(CLR_TIP, width=1.5), name=" Tip Power")
        self._curve_mid_f = self._plot_fft.plot(pen=pg.mkPen(CLR_MID, width=1.5), name=" Mid Power")
        layout.addWidget(self._plot_fft)

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
        else:
            self._worker = SerialWorker(self._port_combo.currentText())
            # Connect the single-frame signal back up
            self._worker.data_ready.connect(self._on_data)
            self._worker.start()
            self._btn_connect.setText("DISCONNECT")
            self._btn_connect.setStyleSheet(f"background-color: {CLR_MID}; color: #11111B;")

    @pyqtSlot(float, float, float, float, float, float)
    def _on_data(self, xt, yt, zt, xm, ym, zm):
        raw_t = np.array([xt, yt, zt])
        raw_m = np.array([xm, ym, zm])
        
        # High-Speed Vectorized Math (No Slow Python For-Loops)
        hp_t = HP_ALPHA * (self._prev_hp_t + raw_t - self._prev_raw_t)
        hp_m = HP_ALPHA * (self._prev_hp_m + raw_m - self._prev_raw_m)
        
        self._prev_raw_t, self._prev_hp_t = raw_t, hp_t
        self._prev_raw_m, self._prev_hp_m = raw_m, hp_m
        
        # Focus on the Z-axis (bending up and down)
        self._buf_z_tip.append(hp_t[2]) 
        self._buf_z_mid.append(hp_m[2])

    @pyqtSlot()
    def _update_plots(self):
        n = len(self._buf_z_tip)
        if n < 64: return

        # Render time domain
        zt_arr, zm_arr = np.array(self._buf_z_tip), np.array(self._buf_z_mid)
        self._curve_tip_t.setData(zt_arr)
        self._curve_mid_t.setData(zm_arr)

        # Render FFT domain
        win = np.hanning(n)
        fft_t = np.abs(np.fft.rfft(zt_arr * win)) / n
        fft_m = np.abs(np.fft.rfft(zm_arr * win)) / n
        freqs = np.fft.rfftfreq(n, d=1.0/SAMPLE_RATE_HZ)

        self._curve_tip_f.setData(freqs, fft_t)
        self._curve_mid_f.setData(freqs, fft_m)

        # Ignore < 5 Hz macro shifts for status bar readout
        min_idx = int(5.0 / (SAMPLE_RATE_HZ / n))
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