#!/usr/bin/env python3
"""
digital_twin_app.py
═══════════════════════════════════════════════════════════════════════
Real-Time Vibration Digital Twin — Industrial Dashboard
═══════════════════════════════════════════════════════════════════════
Stack   : PyQt5  ·  pyqtgraph  ·  NumPy  ·  pyserial
Hardware: Arduino Uno + MPU6050 streaming CSV @ 115 200 baud / 100 Hz
Features:
  • Threaded serial reader (QThread) — zero UI blocking
  • Rolling 256-sample window with α = 0.98 high-pass gravity filter
  • Real-time FFT → frequency-domain spectrum up to Nyquist (50 Hz)
  • Dual live graphs: time-domain waveform + FFT power spectrum
  • Structural Health Twin panel with colour-coded state automation
  • Session CSV logging, pause, and tare/calibrate toolbar actions
═══════════════════════════════════════════════════════════════════════
"""

from __future__ import annotations

import csv
import datetime
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import (
    QMutex,
    QThread,
    Qt,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt5.QtGui import QColor, QFont, QIcon
from PyQt5.QtWidgets import (
    QAction,
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenuBar,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

# ─── Try to import pyserial; fall back to simulated data ────────────
try:
    import serial
    import serial.tools.list_ports

    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ════════════════════════════════════════════════════════════════════
# CONSTANTS
# ════════════════════════════════════════════════════════════════════
BAUD_RATE = 115200
SAMPLE_RATE_HZ = 100
WINDOW_SIZE = 256  # must be power of 2 for FFT
NYQUIST_HZ = SAMPLE_RATE_HZ / 2.0
HP_ALPHA = 0.98  # high-pass filter coefficient (gravity removal)
LSB_PER_G = 16384.0  # MPU6050 ±2 g sensitivity
UPDATE_INTERVAL_MS = 33  # ~30 FPS UI refresh

# ── Structural-health thresholds (Hz) ──────────────────────────────
RESONANCE_LOW = 46.0
RESONANCE_HIGH = 50.8

# ── Colour palette ─────────────────────────────────────────────────
CLR_BG = "#1a1a2e"
CLR_PANEL = "#16213e"
CLR_CARD = "#0f3460"
CLR_ACCENT = "#e94560"
CLR_TEXT = "#eaeaea"
CLR_SAFE = "#2ecc71"
CLR_WARN = "#e67e22"
CLR_CRIT = "#e74c3c"
CLR_GRID = "#2a2a4a"


# ════════════════════════════════════════════════════════════════════
# SERIAL WORKER THREAD
# ════════════════════════════════════════════════════════════════════
class SerialWorker(QThread):
    """Continuously reads CSV lines from the Arduino in a background
    thread and emits parsed (ax, ay, az) tuples to the GUI."""

    data_ready = pyqtSignal(float, float, float)  # ax_g, ay_g, az_g
    error_occurred = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    def __init__(self, port: str, baud: int = BAUD_RATE, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._running = True
        self._paused = False
        self._mutex = QMutex()

    # ── control helpers ────────────────────────────────────────────
    def pause(self):
        self._mutex.lock()
        self._paused = True
        self._mutex.unlock()

    def resume(self):
        self._mutex.lock()
        self._paused = False
        self._mutex.unlock()

    def stop(self):
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()

    # ── thread body ────────────────────────────────────────────────
    def run(self):  # noqa: C901 — linear control flow; clarity > metrics
        ser = None
        try:
            ser = serial.Serial(self._port, self._baud, timeout=1)
            time.sleep(2)  # Arduino reset grace period
            self.connection_status.emit(True)

            while True:
                self._mutex.lock()
                running = self._running
                paused = self._paused
                self._mutex.unlock()

                if not running:
                    break
                if paused:
                    time.sleep(0.05)
                    continue

                raw_line = ser.readline()
                if not raw_line:
                    continue

                try:
                    decoded = raw_line.decode("ascii", errors="replace").strip()
                    if not decoded or decoded.startswith("timestamp"):
                        continue  # skip header or empty lines
                    parts = decoded.split(",")
                    if len(parts) < 4:
                        continue
                    # Convert raw int16 → g-force
                    ax_g = float(parts[1]) / LSB_PER_G
                    ay_g = float(parts[2]) / LSB_PER_G
                    az_g = float(parts[3]) / LSB_PER_G
                    self.data_ready.emit(ax_g, ay_g, az_g)
                except (ValueError, IndexError):
                    continue

        except Exception as exc:
            self.error_occurred.emit(str(exc))
            self.connection_status.emit(False)
        finally:
            if ser and ser.is_open:
                ser.close()
            self.connection_status.emit(False)


# ════════════════════════════════════════════════════════════════════
# SIMULATED DATA WORKER (fallback when no serial hardware)
# ════════════════════════════════════════════════════════════════════
class SimulatedWorker(QThread):
    """Generates synthetic vibration data for UI development/testing."""

    data_ready = pyqtSignal(float, float, float)
    error_occurred = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self._paused = False
        self._mutex = QMutex()

    def pause(self):
        self._mutex.lock()
        self._paused = True
        self._mutex.unlock()

    def resume(self):
        self._mutex.lock()
        self._paused = False
        self._mutex.unlock()

    def stop(self):
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()

    def run(self):
        self.connection_status.emit(True)
        t = 0.0
        dt = 1.0 / SAMPLE_RATE_HZ
        while True:
            self._mutex.lock()
            running = self._running
            paused = self._paused
            self._mutex.unlock()

            if not running:
                break
            if paused:
                time.sleep(0.05)
                continue

            # Composite vibration: 12 Hz machine hum + 60 Hz resonance hint + noise
            ax = 0.05 * np.sin(2 * np.pi * 12.0 * t) + 0.008 * np.sin(2 * np.pi * 60.0 * t)
            ay = 0.03 * np.sin(2 * np.pi * 8.0 * t + 0.5)
            az = 1.0 + 0.02 * np.sin(2 * np.pi * 15.0 * t)  # includes 1 g gravity
            # Add sensor noise
            ax += np.random.normal(0, 0.003)
            ay += np.random.normal(0, 0.003)
            az += np.random.normal(0, 0.003)

            self.data_ready.emit(float(ax), float(ay), float(az))
            t += dt
            time.sleep(dt)

        self.connection_status.emit(False)


# ════════════════════════════════════════════════════════════════════
# MAIN WINDOW — DIGITAL TWIN DASHBOARD
# ════════════════════════════════════════════════════════════════════
class DigitalTwinDashboard(QMainWindow):
    """Industrial-grade real-time vibration monitoring dashboard."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚙  Digital Twin — Real-Time Vibration Monitor")
        self.setMinimumSize(1280, 720)

        # ── State ──────────────────────────────────────────────────
        self._worker: SerialWorker | SimulatedWorker | None = None
        self._is_logging = False
        self._is_paused = False
        self._csv_writer = None
        self._csv_file = None
        self._tare_offsets = np.zeros(3)

        # Rolling buffers (filtered)
        self._buf_ax = deque(maxlen=WINDOW_SIZE)
        self._buf_ay = deque(maxlen=WINDOW_SIZE)
        self._buf_az = deque(maxlen=WINDOW_SIZE)

        # High-pass filter state
        self._prev_raw = np.zeros(3)
        self._prev_hp = np.zeros(3)

        self._sample_count = 0
        self._last_status_time = time.time()

        # ── Build UI ───────────────────────────────────────────────
        self._apply_global_stylesheet()
        self._build_menubar()
        self._build_toolbar()
        self._build_central_widget()
        self._build_statusbar()

        # ── Refresh timer ──────────────────────────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(UPDATE_INTERVAL_MS)

    # ────────────────────────────────────────────────────────────────
    # STYLE
    # ────────────────────────────────────────────────────────────────
    def _apply_global_stylesheet(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {CLR_BG};
                color: {CLR_TEXT};
                font-family: 'Segoe UI', 'Roboto', 'Arial', sans-serif;
                font-size: 13px;
            }}
            QMenuBar {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                border-bottom: 1px solid {CLR_GRID};
                padding: 2px;
            }}
            QMenuBar::item:selected {{
                background-color: {CLR_CARD};
            }}
            QMenu {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                border: 1px solid {CLR_GRID};
            }}
            QMenu::item:selected {{
                background-color: {CLR_CARD};
            }}
            QToolBar {{
                background-color: {CLR_PANEL};
                border: none;
                padding: 4px;
                spacing: 6px;
            }}
            QPushButton {{
                background-color: {CLR_CARD};
                color: {CLR_TEXT};
                border: 1px solid {CLR_GRID};
                border-radius: 4px;
                padding: 6px 14px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {CLR_ACCENT};
            }}
            QPushButton:pressed {{
                background-color: #c0392b;
            }}
            QPushButton:checked {{
                background-color: {CLR_SAFE};
                color: #111;
            }}
            QComboBox {{
                background-color: {CLR_CARD};
                color: {CLR_TEXT};
                border: 1px solid {CLR_GRID};
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QComboBox QAbstractItemView {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                selection-background-color: {CLR_CARD};
            }}
            QGroupBox {{
                border: 1px solid {CLR_GRID};
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 18px;
                font-weight: bold;
                color: {CLR_TEXT};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
            }}
            QStatusBar {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                border-top: 1px solid {CLR_GRID};
            }}
            QLabel {{
                color: {CLR_TEXT};
            }}
        """)

    # ────────────────────────────────────────────────────────────────
    # MENU BAR
    # ────────────────────────────────────────────────────────────────
    def _build_menubar(self):
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")
        file_menu.addAction(
            QAction("&Log Session to CSV", self, triggered=self._toggle_logging)
        )
        file_menu.addSeparator()
        file_menu.addAction(QAction("E&xit", self, triggered=self.close))

        # Tools menu
        tools_menu = menubar.addMenu("&Tools")
        tools_menu.addAction(
            QAction("&Pause Live Stream", self, triggered=self._toggle_pause)
        )
        tools_menu.addAction(
            QAction("&Calibrate / Tare Baseline", self, triggered=self._tare_sensor)
        )

        # Help menu
        help_menu = menubar.addMenu("&Help")
        help_menu.addAction(
            QAction("&About", self, triggered=self._show_about)
        )

    # ────────────────────────────────────────────────────────────────
    # TOOLBAR
    # ────────────────────────────────────────────────────────────────
    def _build_toolbar(self):
        toolbar = QToolBar("Main Controls")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        # Port selector
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        self._port_combo.addItem("Simulated Data")
        if SERIAL_AVAILABLE:
            for p in serial.tools.list_ports.comports():
                self._port_combo.addItem(p.device)
        toolbar.addWidget(QLabel("  Port: "))
        toolbar.addWidget(self._port_combo)
        toolbar.addSeparator()

        # Connect button
        self._btn_connect = QPushButton("▶  Connect")
        self._btn_connect.clicked.connect(self._toggle_connection)
        toolbar.addWidget(self._btn_connect)

        toolbar.addSeparator()

        # Quick-action buttons
        self._btn_log = QPushButton("📄 Log CSV")
        self._btn_log.setCheckable(True)
        self._btn_log.clicked.connect(self._toggle_logging)
        toolbar.addWidget(self._btn_log)

        self._btn_pause = QPushButton("⏸  Pause")
        self._btn_pause.setCheckable(True)
        self._btn_pause.clicked.connect(self._toggle_pause)
        toolbar.addWidget(self._btn_pause)

        self._btn_tare = QPushButton("⚖  Tare")
        self._btn_tare.clicked.connect(self._tare_sensor)
        toolbar.addWidget(self._btn_tare)

    # ────────────────────────────────────────────────────────────────
    # CENTRAL WIDGET
    # ────────────────────────────────────────────────────────────────
    def _build_central_widget(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(10)

        # ── Left column: graphs ────────────────────────────────────
        left_col = QVBoxLayout()
        left_col.setSpacing(8)

        #  Time-domain plot
        self._time_plot = pg.PlotWidget(title="Live Vibration Waveform (g)")
        self._time_plot.setBackground(CLR_PANEL)
        self._time_plot.showGrid(x=True, y=True, alpha=0.3)
        self._time_plot.setLabel("left", "Acceleration", units="g")
        self._time_plot.setLabel("bottom", "Sample Index")
        self._time_plot.addLegend(offset=(10, 10))
        self._curve_ax = self._time_plot.plot(pen=pg.mkPen("#e74c3c", width=2), name="X")
        self._curve_ay = self._time_plot.plot(pen=pg.mkPen("#2ecc71", width=2), name="Y")
        self._curve_az = self._time_plot.plot(pen=pg.mkPen("#3498db", width=2), name="Z")
        left_col.addWidget(self._time_plot, stretch=1)

        #  FFT plot
        self._fft_plot = pg.PlotWidget(title="FFT Power Spectrum (0 – 50 Hz)")
        self._fft_plot.setBackground(CLR_PANEL)
        self._fft_plot.showGrid(x=True, y=True, alpha=0.3)
        self._fft_plot.setLabel("left", "Magnitude")
        self._fft_plot.setLabel("bottom", "Frequency", units="Hz")
        self._fft_plot.setXRange(0, NYQUIST_HZ)
        self._curve_fft = self._fft_plot.plot(
            pen=pg.mkPen("#f1c40f", width=2), fillLevel=0,
            brush=pg.mkBrush(241, 196, 15, 50),
        )
        self._fft_peak_label = pg.TextItem(anchor=(0, 1), color="#f1c40f")
        self._fft_plot.addItem(self._fft_peak_label)
        left_col.addWidget(self._fft_plot, stretch=1)

        root_layout.addLayout(left_col, stretch=3)

        # ── Right column: health panel ─────────────────────────────
        right_col = QVBoxLayout()
        right_col.setSpacing(10)

        # Title header
        header = QLabel("STRUCTURAL HEALTH TWIN")
        header.setAlignment(Qt.AlignCenter)
        header.setFont(QFont("Segoe UI", 16, QFont.Bold))
        header.setStyleSheet(f"color: {CLR_ACCENT}; letter-spacing: 2px;")
        right_col.addWidget(header)

        # ── Health state card ──────────────────────────────────────
        self._health_card = QFrame()
        self._health_card.setFrameShape(QFrame.StyledPanel)
        self._health_card.setMinimumHeight(180)
        self._health_card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._health_card.setStyleSheet(f"""
            QFrame {{
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 {CLR_CARD}, stop:1 #1a3a5c
                );
                border: 2px solid {CLR_GRID};
                border-radius: 12px;
            }}
        """)
        card_layout = QVBoxLayout(self._health_card)
        card_layout.setContentsMargins(20, 20, 20, 20)

        self._lbl_state = QLabel("IDLE / SAFE")
        self._lbl_state.setAlignment(Qt.AlignCenter)
        self._lbl_state.setFont(QFont("Segoe UI", 22, QFont.Bold))
        self._lbl_state.setWordWrap(True)
        self._lbl_state.setStyleSheet(f"""
            color: #fff;
            background-color: {CLR_SAFE};
            border-radius: 8px;
            padding: 18px;
        """)
        card_layout.addWidget(self._lbl_state)
        right_col.addWidget(self._health_card)

        # ── Metrics group ──────────────────────────────────────────
        metrics_group = QGroupBox("Live Metrics")
        metrics_layout = QGridLayout(metrics_group)
        metrics_layout.setVerticalSpacing(10)
        metrics_layout.setHorizontalSpacing(16)

        metric_label_style = f"color: {CLR_ACCENT}; font-weight: bold; font-size: 12px;"
        metric_value_style = "font-size: 18px; font-weight: bold; font-family: 'Consolas', monospace;"

        labels = [
            ("Dominant Freq", "Hz"),
            ("Peak Amplitude", "g"),
            ("RMS Vibration", "g"),
            ("Sample Rate", "Hz"),
        ]
        self._metric_values: list[QLabel] = []
        for i, (name, unit) in enumerate(labels):
            lbl_name = QLabel(f"{name}")
            lbl_name.setStyleSheet(metric_label_style)
            lbl_val = QLabel("—")
            lbl_val.setStyleSheet(metric_value_style)
            lbl_unit = QLabel(unit)
            lbl_unit.setStyleSheet(f"color: #888; font-size: 11px;")
            metrics_layout.addWidget(lbl_name, i, 0)
            metrics_layout.addWidget(lbl_val, i, 1)
            metrics_layout.addWidget(lbl_unit, i, 2)
            self._metric_values.append(lbl_val)

        right_col.addWidget(metrics_group)

        # ── Axis instantaneous readout ─────────────────────────────
        readout_group = QGroupBox("Axis Readout (g)")
        readout_layout = QHBoxLayout(readout_group)
        self._readout_labels: dict[str, QLabel] = {}
        for axis, colour in [("X", "#e74c3c"), ("Y", "#2ecc71"), ("Z", "#3498db")]:
            frame = QFrame()
            frame.setStyleSheet(f"""
                QFrame {{
                    background-color: {CLR_CARD};
                    border: 1px solid {colour};
                    border-radius: 6px;
                    padding: 6px;
                }}
            """)
            fl = QVBoxLayout(frame)
            fl.setContentsMargins(8, 4, 8, 4)
            lbl_axis = QLabel(axis)
            lbl_axis.setAlignment(Qt.AlignCenter)
            lbl_axis.setStyleSheet(f"color: {colour}; font-weight: bold; font-size: 13px;")
            lbl_val = QLabel("0.000")
            lbl_val.setAlignment(Qt.AlignCenter)
            lbl_val.setStyleSheet("font-size: 17px; font-family: 'Consolas', monospace;")
            fl.addWidget(lbl_axis)
            fl.addWidget(lbl_val)
            readout_layout.addWidget(frame)
            self._readout_labels[axis] = lbl_val

        right_col.addWidget(readout_group)
        right_col.addStretch()

        root_layout.addLayout(right_col, stretch=1)

    # ────────────────────────────────────────────────────────────────
    # STATUS BAR
    # ────────────────────────────────────────────────────────────────
    def _build_statusbar(self):
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._statusbar.showMessage("Disconnected  ·  Select a port and click Connect")

    # ────────────────────────────────────────────────────────────────
    # CONNECTION MANAGEMENT
    # ────────────────────────────────────────────────────────────────
    @pyqtSlot()
    def _toggle_connection(self):
        if self._worker is not None:
            self._disconnect()
            return

        port_text = self._port_combo.currentText()
        if port_text == "Simulated Data":
            self._worker = SimulatedWorker(self)
        else:
            if not SERIAL_AVAILABLE:
                QMessageBox.warning(
                    self, "Missing Dependency",
                    "pyserial is not installed.\nRun:  pip install pyserial",
                )
                return
            self._worker = SerialWorker(port_text, BAUD_RATE, self)

        self._worker.data_ready.connect(self._on_data_received)
        self._worker.connection_status.connect(self._on_connection_status)
        self._worker.error_occurred.connect(self._on_worker_error)
        self._worker.start()
        self._btn_connect.setText("⏹  Disconnect")

    def _disconnect(self):
        if self._worker:
            self._worker.stop()
            self._worker.wait(2000)
            self._worker = None
        self._btn_connect.setText("▶  Connect")
        self._statusbar.showMessage("Disconnected")

    @pyqtSlot(bool)
    def _on_connection_status(self, connected: bool):
        if connected:
            self._statusbar.showMessage("Connected  ·  Streaming live data")
        else:
            self._statusbar.showMessage("Disconnected")

    @pyqtSlot(str)
    def _on_worker_error(self, msg: str):
        self._statusbar.showMessage(f"Error: {msg}")

    # ────────────────────────────────────────────────────────────────
    # DATA RECEPTION (called from worker thread via signal)
    # ────────────────────────────────────────────────────────────────
    @pyqtSlot(float, float, float)
    def _on_data_received(self, ax: float, ay: float, az: float):
        raw = np.array([ax, ay, az]) - self._tare_offsets

        # ── High-pass filter (removes static gravity) ──────────────
        hp = np.empty(3)
        for i in range(3):
            hp[i] = HP_ALPHA * (self._prev_hp[i] + raw[i] - self._prev_raw[i])
        self._prev_raw = raw.copy()
        self._prev_hp = hp.copy()

        self._buf_ax.append(hp[0])
        self._buf_ay.append(hp[1])
        self._buf_az.append(hp[2])

        self._sample_count += 1

        # ── CSV logging ────────────────────────────────────────────
        if self._is_logging and self._csv_writer:
            self._csv_writer.writerow([
                datetime.datetime.now().isoformat(),
                f"{hp[0]:.6f}", f"{hp[1]:.6f}", f"{hp[2]:.6f}",
            ])

    # ────────────────────────────────────────────────────────────────
    # PLOT / METRICS REFRESH (~30 FPS)
    # ────────────────────────────────────────────────────────────────
    @pyqtSlot()
    def _refresh_plots(self):
        n = len(self._buf_ax)
        if n < 4:
            return

        ax_arr = np.array(self._buf_ax)
        ay_arr = np.array(self._buf_ay)
        az_arr = np.array(self._buf_az)

        # ── Time-domain curves ─────────────────────────────────────
        self._curve_ax.setData(ax_arr)
        self._curve_ay.setData(ay_arr)
        self._curve_az.setData(az_arr)

        # ── FFT on magnitude vector ────────────────────────────────
        magnitude = np.sqrt(ax_arr**2 + ay_arr**2 + az_arr**2)

        # Apply Hanning window to reduce spectral leakage
        windowed = magnitude * np.hanning(n)
        fft_vals = np.abs(np.fft.rfft(windowed)) / n
        freqs = np.fft.rfftfreq(n, d=1.0 / SAMPLE_RATE_HZ)

        self._curve_fft.setData(freqs, fft_vals)

        # ── Dominant frequency detection ───────────────────────────
        # Ignore frequencies below 5 Hz to filter out gravity shifts and slow tilting
        min_freq_hz = 5.0 
        freq_resolution = SAMPLE_RATE_HZ / n
        min_idx = int(min_freq_hz / freq_resolution)
        
        if len(fft_vals) > min_idx:
            peak_idx = np.argmax(fft_vals[min_idx:]) + min_idx
            dominant_freq = freqs[peak_idx]
            peak_amp = fft_vals[peak_idx]
        else:
            dominant_freq = 0.0
            peak_amp = 0.0

        # ── Peak label on FFT ──────────────────────────────────────
        self._fft_peak_label.setText(f" {dominant_freq:.1f} Hz ")
        self._fft_peak_label.setPos(dominant_freq, peak_amp)

        # ── RMS vibration ──────────────────────────────────────────
        rms = float(np.sqrt(np.mean(magnitude**2)))

        # ── Effective sample rate ──────────────────────────────────
        now = time.time()
        elapsed = now - self._last_status_time
        if elapsed >= 1.0:
            effective_rate = self._sample_count / elapsed
            self._sample_count = 0
            self._last_status_time = now
        else:
            effective_rate = self._sample_count / max(elapsed, 0.001)

        # ── Update metric labels ───────────────────────────────────
        self._metric_values[0].setText(f"{dominant_freq:.2f}")
        self._metric_values[1].setText(f"{peak_amp:.5f}")
        self._metric_values[2].setText(f"{rms:.5f}")
        self._metric_values[3].setText(f"{effective_rate:.0f}")

        # ── Axis readout ───────────────────────────────────────────
        self._readout_labels["X"].setText(f"{ax_arr[-1]:.4f}")
        self._readout_labels["Y"].setText(f"{ay_arr[-1]:.4f}")
        self._readout_labels["Z"].setText(f"{az_arr[-1]:.4f}")

        # ── Structural health state machine ────────────────────────
        self._update_health_state(dominant_freq, peak_amp)

    # ────────────────────────────────────────────────────────────────
    # HEALTH STATE MACHINE
    # ────────────────────────────────────────────────────────────────
    # ────────────────────────────────────────────────────────────────
    # HEALTH STATE MACHINE
    # ────────────────────────────────────────────────────────────────
    def _update_health_state(self, freq_hz: float, amplitude: float):
        if amplitude < 0.0005:
            state_text = "IDLE / SAFE"
            bg_colour = CLR_SAFE
        elif freq_hz <= RESONANCE_LOW:
            state_text = f"UNDER LIMIT\n({freq_hz:.1f} Hz)"
            bg_colour = CLR_SAFE
        elif freq_hz <= RESONANCE_HIGH:
            state_text = f"⚠  WARNING: RESONANCE ZONE\n≈ 48.4 Hz"
            bg_colour = CLR_WARN
        else:
            state_text = f"🔴  CRITICAL OVER-LIMIT\n> 50.8 Hz"
            bg_colour = CLR_CRIT

        self._lbl_state.setText(state_text)
        self._lbl_state.setStyleSheet(f"""
            color: #fff;
            background-color: {bg_colour};
            border-radius: 8px;
            padding: 18px;
        """)
    # ────────────────────────────────────────────────────────────────
    # TOOLBAR ACTIONS
    # ────────────────────────────────────────────────────────────────
    @pyqtSlot()
    def _toggle_logging(self):
        if not self._is_logging:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = Path(f"session_log_{ts}.csv")
            self._csv_file = open(filepath, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(["timestamp", "ax_g", "ay_g", "az_g"])
            self._is_logging = True
            self._btn_log.setChecked(True)
            self._statusbar.showMessage(f"Logging to {filepath}")
        else:
            self._is_logging = False
            self._btn_log.setChecked(False)
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
                self._csv_writer = None
            self._statusbar.showMessage("Logging stopped")

    @pyqtSlot()
    def _toggle_pause(self):
        if self._worker is None:
            return
        self._is_paused = not self._is_paused
        if self._is_paused:
            self._worker.pause()
            self._btn_pause.setChecked(True)
            self._statusbar.showMessage("Stream paused")
        else:
            self._worker.resume()
            self._btn_pause.setChecked(False)
            self._statusbar.showMessage("Stream resumed")

    @pyqtSlot()
    def _tare_sensor(self):
        """Capture current mean offset as the zero baseline (removes
        mounting bias and residual gravity leak)."""
        if len(self._buf_ax) < 16:
            self._statusbar.showMessage("Need ≥ 16 samples to tare — keep streaming")
            return
        self._tare_offsets = np.array([
            np.mean(list(self._buf_ax)[-64:]),
            np.mean(list(self._buf_ay)[-64:]),
            np.mean(list(self._buf_az)[-64:]),
        ])
        self._statusbar.showMessage(
            f"Tare applied  ·  offsets = X:{self._tare_offsets[0]:.4f}  "
            f"Y:{self._tare_offsets[1]:.4f}  Z:{self._tare_offsets[2]:.4f}"
        )

    @pyqtSlot()
    def _show_about(self):
        QMessageBox.information(
            self,
            "About",
            "Digital Twin — Real-Time Vibration Monitor\n"
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            "Hardware : Arduino Uno + MPU6050\n"
            "Protocol : Serial CSV @ 115 200 baud\n"
            "Sampling : 100 Hz  ·  FFT window 256\n"
            "Stack    : Python · PyQt5 · pyqtgraph · NumPy\n\n"
            "© 2026 Digital Twin Project",
        )

    # ────────────────────────────────────────────────────────────────
    # CLEANUP
    # ────────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._disconnect()
        if self._csv_file:
            self._csv_file.close()
        event.accept()


# ════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")  # cross-platform consistent look

    # Dark palette baseline (Fusion + stylesheet = industrial feel)
    from PyQt5.QtGui import QPalette
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(CLR_BG))
    dark_palette.setColor(QPalette.WindowText, QColor(CLR_TEXT))
    dark_palette.setColor(QPalette.Base, QColor(CLR_PANEL))
    dark_palette.setColor(QPalette.AlternateBase, QColor(CLR_CARD))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(CLR_TEXT))
    dark_palette.setColor(QPalette.ToolTipText, QColor(CLR_BG))
    dark_palette.setColor(QPalette.Text, QColor(CLR_TEXT))
    dark_palette.setColor(QPalette.Button, QColor(CLR_CARD))
    dark_palette.setColor(QPalette.ButtonText, QColor(CLR_TEXT))
    dark_palette.setColor(QPalette.Highlight, QColor(CLR_ACCENT))
    dark_palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    app.setPalette(dark_palette)

    window = DigitalTwinDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
