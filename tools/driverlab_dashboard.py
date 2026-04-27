#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
DriverLab Dashboard - Jurababa Motor Characterization Interface

Based on UKMARS DriverLab Dashboard by Peter Harrison.
Adapted for Raspberry Pi Pico with Jurababa micromouse firmware.

Usage:
    python3 driverlab_dashboard.py

Requirements:
    pip3 install pyserial pyqtgraph PyQt6 numpy
"""

import sys
import time
import numpy as np

from serial import Serial
import serial.tools.list_ports

from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QThread, pyqtSignal as Signal, pyqtSlot as Slot, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow,
    QStatusBar, QScrollArea,
    QSpacerItem, QMessageBox, QSizePolicy,
    QWidget, QGroupBox, QFrame,
    QDoubleSpinBox, QComboBox,
    QGridLayout, QHBoxLayout, QVBoxLayout,
    QPlainTextEdit, QPushButton, QLineEdit,
    QRadioButton, QLabel, QSplitter, QButtonGroup)

import pyqtgraph as pg

# App versioning
APP_VERSION = '1.0.0'
APP_NAME = 'Jurababa DriverLab'

# Color palette (from UKMARS)
palette = ("#101418", "#c00000", "#c000c0", "#c06000", "#00c000", "#0072c3", "#6fdc8c", "#d2a106")

# Control modes
FULL_CONTROL = 2  # FF + PD
NO_FF = 1         # PD only
ONLY_FF = 0       # FF only

hline_style = 'border: 2px solid gray'


class DataChannel:
    """Encapsulates a single-dimensional numpy array for plot data."""

    def __init__(self, points):
        self.points = points
        self.data_ = np.zeros(shape=self.points, dtype=float)

    def add_new_value(self, value):
        self.data_[:-1] = self.data_[1:]
        self.data_[-1] = value

    def data(self):
        return self.data_


class Dashboard(QMainWindow):
    usb_dis = Signal()
    usb_con = Signal(str, str)
    message = Signal(str, str)
    dialog = Signal(str, str, str)
    ports_updated = Signal(list)

    def __init__(self):
        super().__init__()

        self.processing = None
        self.device = None
        self.serial = None
        self.data = []
        self.parameters = {}
        self.move_mode = ONLY_FF
        self.auto_connect = False
        self.monitor_thread = None
        self.monitoring = False

        self.nChannels = 10
        self.nPoints = 400
        self.telemetry = [DataChannel(self.nPoints) for i in range(self.nChannels)]
        self.csv_headings = []
        self.plot_curves = {'output': [], 'motion': []}
        self.initUI()

    def initUI(self):
        pg.setConfigOption('background', pg.mkColor(25, 50, 75))
        pg.setConfigOption('background', palette[0])
        pg.setConfigOption('foreground', 'y')
        pg.setConfigOptions(antialias=True)
        styles = {'color': 'cyan', 'font-size': '13px', 'bottom_margin': '50px'}

        # Output plot (voltage)
        self.output_plot = pg.PlotWidget()
        self.output_plot.setYRange(-1, 7)
        self.output_plot.setTitle("<span style=\"color:cyan;\">Controller outputs</span>")
        self.output_plot.setLabel('left', 'Volts', **styles)
        self.output_plot.setLabel('bottom', 'time (ms)', **styles)
        self.output_plot.addLegend(offset=(-5, 20))
        self.output_plot.showGrid(x=True, y=True)
        # Enable mouse interaction for zoom/pan
        self.output_plot.setMouseEnabled(x=True, y=True)
        self.output_plot.enableAutoRange(enable=True)
        ax = self.output_plot.getPlotItem().getAxis('left')
        ax.setWidth(60)

        # Motion plot (speed)
        self.motion_plot = pg.PlotWidget()
        self.motion_plot.setYRange(-50, 500)
        self.motion_plot.setTitle("<span style=\"color:cyan;\">Motion</span>")
        self.motion_plot.setLabel('left', 'Speed (mm/s)', **styles)
        self.motion_plot.setLabel('bottom', 'time (ms)', **styles)
        self.motion_plot.showGrid(x=True, y=True)
        # Enable mouse interaction for zoom/pan
        self.motion_plot.setMouseEnabled(x=True, y=True)
        self.motion_plot.enableAutoRange(enable=True)
        ax = self.motion_plot.getPlotItem().getAxis('left')
        ax.setWidth(60)
        self.motion_plot.addLegend(offset=(-5, 20))

        # ========== PORT SELECTION GROUP ==========
        port_group = QGroupBox("Serial Port")
        port_layout = QVBoxLayout()

        # Port combo box (editable for custom ports)
        port_select_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.port_combo.lineEdit().setPlaceholderText("Select or enter port...")
        self.port_combo.setMinimumWidth(150)
        port_select_layout.addWidget(self.port_combo)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_refresh.setFixedWidth(70)
        port_select_layout.addWidget(self.btn_refresh)

        port_layout.addLayout(port_select_layout)

        # Connect/Disconnect button
        port_button_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        port_button_layout.addWidget(self.btn_connect)

        port_layout.addLayout(port_button_layout)

        # Clear button (monitor is always on when connected)
        monitor_layout = QHBoxLayout()
        lbl_monitor = QLabel("Monitor: always on")
        lbl_monitor.setStyleSheet("color: gray; font-style: italic;")
        monitor_layout.addWidget(lbl_monitor)

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self.clear_monitor)
        self.btn_clear.setFixedWidth(60)
        monitor_layout.addWidget(self.btn_clear)

        port_layout.addLayout(monitor_layout)

        port_group.setLayout(port_layout)

        # Text box for serial output
        self.text_box = QPlainTextEdit()
        self.text_box.setFont(QFont('Menlo'))
        self.text_box.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)

        # Command input for sending serial commands
        self.cmd_input = QLineEdit()
        self.cmd_input.setFont(QFont('Menlo'))
        self.cmd_input.setPlaceholderText("Type command... (Enter to send)")
        self.cmd_input.returnPressed.connect(self.send_command)

        # ========== SETTINGS PANEL ==========
        # Stylesheet for grouped sections with color-coded titles
        group_style = """
            QGroupBox {{
                font-weight: bold;
                border: 1px solid {border};
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 4px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: {title};
            }}
        """

        settings_layout = QVBoxLayout()
        settings_layout.setContentsMargins(0, 0, 0, 0)
        settings_layout.setSpacing(6)
        RA = Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter

        # --- Motor Model (read-only, from OL trial) ---
        motor_group = QGroupBox("Motor Model  (from OL)")
        motor_group.setStyleSheet(group_style.format(border="#555", title="#aaa"))
        motor_grid = QGridLayout()
        motor_grid.setContentsMargins(6, 10, 6, 6)

        lbl = QLabel("kM  mm/s per V:")
        lbl.setAlignment(RA)
        lbl.setToolTip("Velocity constant: slope of speed vs voltage.\nFrom OL trial: linear regression of steady-state speed at each voltage step.\nkV = 1/kM, kA = Tm/kM")
        self.lbl_km_val = QLabel("--")
        self.lbl_km_val.setStyleSheet("font-weight: bold; font-size: 13px;")
        motor_grid.addWidget(lbl, 0, 0)
        motor_grid.addWidget(self.lbl_km_val, 0, 1)

        lbl = QLabel("Tm  seconds:")
        lbl.setAlignment(RA)
        lbl.setToolTip("Motor time constant: how fast speed responds to a voltage step.\nFrom STEP trial: time to reach 63% of final speed.\nUsed to derive kA = Tm/kM and default Td = Tm/2")
        self.lbl_tm_val = QLabel("--")
        self.lbl_tm_val.setStyleSheet("font-weight: bold; font-size: 13px;")
        motor_grid.addWidget(lbl, 1, 0)
        motor_grid.addWidget(self.lbl_tm_val, 1, 1)

        motor_group.setLayout(motor_grid)
        settings_layout.addWidget(motor_group)

        # --- Feedforward ---
        ff_group = QGroupBox("Feedforward  (open-loop)")
        ff_group.setStyleSheet(group_style.format(border="#0072c3", title="#6faadc"))
        ff_grid = QGridLayout()
        ff_grid.setContentsMargins(6, 10, 6, 6)

        self.spin_biasff = self.double_spinbox("FF kS", 0.0, 0.4, 0.01, 2)
        self.spin_biasff.setToolTip("Static friction voltage: minimum V to start moving")
        self.spin_speedff = self.double_spinbox("FF kV", 0.0, 0.001, 0.00001, 5)
        self.spin_speedff.setToolTip("Speed feedforward: volts per mm/s (= 1/kM)")
        self.spin_accff = self.double_spinbox("FF kA", 0.0, 0.001, 0.00001, 5)
        self.spin_accff.setToolTip("Accel feedforward: volts per mm/s^2 (= Tm/kM)")

        self.spin_biasff.valueChanged.connect(self.parameter_change)
        self.spin_speedff.valueChanged.connect(self.parameter_change)
        self.spin_accff.valueChanged.connect(self.parameter_change)

        lbl = QLabel("kS  static V:")
        lbl.setAlignment(RA)
        ff_grid.addWidget(lbl, 0, 0)
        ff_grid.addWidget(self.spin_biasff, 0, 1)

        lbl = QLabel("kV  V/(mm/s):")
        lbl.setAlignment(RA)
        ff_grid.addWidget(lbl, 1, 0)
        ff_grid.addWidget(self.spin_speedff, 1, 1)

        lbl = QLabel("kA  V/(mm/s^2):")
        lbl.setAlignment(RA)
        ff_grid.addWidget(lbl, 2, 0)
        ff_grid.addWidget(self.spin_accff, 2, 1)

        ff_group.setLayout(ff_grid)
        settings_layout.addWidget(ff_group)

        # --- Forward PD Controller ---
        fwd_group = QGroupBox("Forward PD  (straight-line)")
        fwd_group.setStyleSheet(group_style.format(border="#00c000", title="#6fdc8c"))
        fwd_grid = QGridLayout()
        fwd_grid.setContentsMargins(6, 10, 6, 6)

        self.spin_zeta = self.double_spinbox("zeta", 0.0, 2.0, 0.005, 3)
        self.spin_zeta.setToolTip("Damping ratio: <1 underdamped (ringing), =1 critical, >1 overdamped (sluggish)")
        self.spin_td = self.double_spinbox("Td", 0.0, 1.00, 0.01, 3)
        self.spin_td.setToolTip("Derivative time: larger = slower response.\nTypically Tm/2.")
        self.spin_kp = self.double_spinbox("kP", 0.0, 8.0, 0.001, 4)
        self.spin_kp.setToolTip("Proportional gain: auto-computed from zeta & Td.\nHigher = stiffer tracking, more oscillation risk.")
        self.spin_kd = self.double_spinbox("kD", 0.0, 2.0, 0.0001, 4)
        self.spin_kd.setToolTip("Derivative gain: auto-computed from zeta & Td.\nHigher = more damping, but amplifies noise.")

        self.spin_zeta.valueChanged.connect(self.parameter_change)
        self.spin_td.valueChanged.connect(self.parameter_change)
        self.spin_kp.valueChanged.connect(self.parameter_change)
        self.spin_kd.valueChanged.connect(self.parameter_change)

        lbl_zeta = QLabel("Damping (zeta):")
        lbl_zeta.setAlignment(RA)
        fwd_grid.addWidget(lbl_zeta, 0, 0)
        fwd_grid.addWidget(self.spin_zeta, 0, 1)

        lbl_td = QLabel("Derivative Time (Td):")
        lbl_td.setAlignment(RA)
        fwd_grid.addWidget(lbl_td, 1, 0)
        fwd_grid.addWidget(self.spin_td, 1, 1)

        # Separator between design params and derived gains
        sep = QLabel("--- derived gains ---")
        sep.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sep.setStyleSheet("color: #666; font-size: 10px;")
        fwd_grid.addWidget(sep, 2, 0, 1, 2)

        lbl = QLabel("kP:")
        lbl.setAlignment(RA)
        fwd_grid.addWidget(lbl, 3, 0)
        fwd_grid.addWidget(self.spin_kp, 3, 1)

        lbl = QLabel("kD:")
        lbl.setAlignment(RA)
        fwd_grid.addWidget(lbl, 4, 0)
        fwd_grid.addWidget(self.spin_kd, 4, 1)

        fwd_group.setLayout(fwd_grid)
        settings_layout.addWidget(fwd_group)

        # --- Rotation PD Controller ---
        rot_group = QGroupBox("Rotation PD  (turns & steering)")
        rot_group.setStyleSheet(group_style.format(border="#c06000", title="#d2a106"))
        rot_grid = QGridLayout()
        rot_grid.setContentsMargins(6, 10, 6, 6)

        self.spin_turn_kp = self.double_spinbox("turnKP", 0.0, 8.0, 0.01, 4)
        self.spin_turn_kp.setToolTip("Turn proportional gain: higher = snappier turns.\nAlso used for OL steering correction.")
        self.spin_turn_kd = self.double_spinbox("turnKD", 0.0, 2.0, 0.001, 4)
        self.spin_turn_kd.setToolTip("Turn derivative gain: damps turn overshoot.\nIncrease if turns ring/oscillate.")

        self.spin_turn_kp.valueChanged.connect(self.parameter_change)
        self.spin_turn_kd.valueChanged.connect(self.parameter_change)

        lbl = QLabel("turnKP:")
        lbl.setAlignment(RA)
        rot_grid.addWidget(lbl, 0, 0)
        rot_grid.addWidget(self.spin_turn_kp, 0, 1)

        lbl = QLabel("turnKD:")
        lbl.setAlignment(RA)
        rot_grid.addWidget(lbl, 1, 0)
        rot_grid.addWidget(self.spin_turn_kd, 1, 1)

        rot_group.setLayout(rot_grid)
        settings_layout.addWidget(rot_group)

        # --- Settings Buttons ---
        btn_layout = QHBoxLayout()

        self.btn_read_settings = QPushButton('READ', self)
        self.btn_read_settings.setToolTip("Read current settings from device")
        self.btn_read_settings.clicked.connect(self.read_settings)

        self.btn_write_settings = QPushButton('WRITE', self)
        self.btn_write_settings.setToolTip("Send these values to the device")
        self.btn_write_settings.clicked.connect(self.write_settings)

        self.btn_reset_settings = QPushButton('RESET', self)
        self.btn_reset_settings.setToolTip("Reset device to factory defaults")
        self.btn_reset_settings.clicked.connect(self.reset_settings)

        btn_layout.addWidget(self.btn_read_settings)
        btn_layout.addWidget(self.btn_write_settings)
        btn_layout.addWidget(self.btn_reset_settings)
        settings_layout.addLayout(btn_layout)

        settings_group = QGroupBox()
        settings_group.setObjectName("SettingsGroup")
        settings_group.setStyleSheet("QGroupBox#SettingsGroup { border: 1px solid gray;}")
        settings_group.setLayout(settings_layout)

        # ========== TRIAL BUTTONS (calibration workflow order) ==========
        trials_group = QGroupBox("Calibration Workflow")
        trials_layout = QVBoxLayout()
        trials_layout.setContentsMargins(4, 10, 4, 4)
        trials_layout.setSpacing(4)

        self.btn_ol = QPushButton('1. OL  —  Measure kM, kS')
        self.btn_ol.setToolTip("Open-loop voltage sweep → finds motor gain (kM) and static friction (kS)\nDefault: OL 6 1 2000  (1V–6V, 2s per step)")
        self.btn_ol.clicked.connect(lambda: self.run_trial("OL 6 1 2000"))
        trials_layout.addWidget(self.btn_ol)

        self.btn_step = QPushButton('2. STEP  —  Measure Tm')
        self.btn_step.setToolTip("Step response → finds motor time constant (Tm)\nDefault: STEP 3 1000")
        self.btn_step.clicked.connect(lambda: self.run_trial("STEP 3 1000"))
        trials_layout.addWidget(self.btn_step)

        self.btn_move = QPushButton('3. MOVE  —  Tune forward PD')
        self.btn_move.setToolTip("Forward motion trial → tune ζ and Td for forward PD controller\nDefault: MOVE 360 500 1000")
        self.btn_move.clicked.connect(self.send_move)
        trials_layout.addWidget(self.btn_move)

        self.btn_turn = QPushButton('4. TURN  —  Tune rotation PD')
        self.btn_turn.setToolTip("Turn-in-place trial → tune rotation kP and kD\nDefault: TURN 90 200 500")
        self.btn_turn.clicked.connect(lambda: self.run_trial("TURN 90 200 500"))
        trials_layout.addWidget(self.btn_turn)

        trials_group.setLayout(trials_layout)

        # ========== MOVE CONTROL MODE ==========
        option_layout = QHBoxLayout()
        self.rb_full = QRadioButton("Full")
        self.rb_full.setToolTip("FF + PD")
        self.rb_full.mode = FULL_CONTROL
        self.rb_full.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_full)

        self.rb_noff = QRadioButton("PD Only")
        self.rb_noff.setToolTip("PD feedback only, no feedforward")
        self.rb_noff.mode = NO_FF
        self.rb_noff.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_noff)

        self.rb_onlyff = QRadioButton("FF Only")
        self.rb_onlyff.setToolTip("Feedforward only, no PD feedback")
        self.rb_onlyff.mode = ONLY_FF
        self.rb_onlyff.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_onlyff)
        self.rb_onlyff.setChecked(True)

        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.rb_full)
        self.mode_button_group.addButton(self.rb_noff)
        self.mode_button_group.addButton(self.rb_onlyff)

        # ========== SIDEBAR ==========
        # Inner widget holds all sidebar content
        side_inner = QWidget()
        side_layout = QVBoxLayout()
        side_layout.addWidget(port_group)
        side_layout.addWidget(self.text_box)
        side_layout.addWidget(self.cmd_input)
        side_layout.addWidget(self.HLine())
        side_layout.addWidget(trials_group)
        side_layout.addLayout(option_layout)
        side_layout.addWidget(settings_group)
        side_inner.setLayout(side_layout)

        # Scroll area wraps the inner widget so it scrolls when too small
        self.side_bar = QScrollArea()
        self.side_bar.setWidget(side_inner)
        self.side_bar.setWidgetResizable(True)
        self.side_bar.setMinimumWidth(270)
        self.side_bar.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # ========== STATUS BAR ==========
        self.status_bar = QStatusBar()
        self.lbl_con_status = QLabel()
        self.status_bar.addPermanentWidget(self.lbl_con_status)
        self.setStatusBar(self.status_bar)
        self.show_connection_message('warning', 'Not connected')

        # ========== DATA PLOTS WITH SPLITTERS ==========
        # Vertical splitter for plots
        plot_splitter = QSplitter(Qt.Orientation.Vertical)
        plot_splitter.addWidget(self.output_plot)
        plot_splitter.addWidget(self.motion_plot)
        plot_splitter.setStretchFactor(0, 1)
        plot_splitter.setStretchFactor(1, 1)

        # Horizontal splitter for plots and sidebar
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.addWidget(plot_splitter)
        main_splitter.addWidget(self.side_bar)
        # Set initial sizes: plots get ~63%, sidebar gets ~37%
        main_splitter.setSizes([750, 450])

        # Vertical app layout
        main_vbox = QVBoxLayout()
        main_vbox.addWidget(main_splitter)

        # Wrap the layout into a widget
        main_widget = QWidget()
        main_widget.setLayout(main_vbox)

        # Main window style, layout and position
        self.setWindowTitle("Jurababa DriverLab Dashboard")
        self.setCentralWidget(main_widget)
        self.setMinimumSize(1200, 1000)
        self.resize(1200, 1050)
        self.show()
        self.center_window()
        self.setFocus()

        # Initial port scan
        self.refresh_ports()

        # Pre-populate with firmware defaults (mirrors DriverLabSettings::initDefaults)
        self.load_firmware_defaults()

    def load_firmware_defaults(self):
        """Parse tuning.h and set spinboxes to match firmware initDefaults()."""
        import os, re
        tuning_path = os.path.join(os.path.dirname(__file__),
                                   '..', 'firmware', 'config', 'tuning.h')
        defines = {}
        try:
            with open(tuning_path) as f:
                for line in f:
                    m = re.match(r'#define\s+(\w+)\s+([\d.eE+-]+)f?\b', line)
                    if m:
                        defines[m.group(1)] = float(m.group(2))
        except FileNotFoundError:
            self.log_message("tuning.h not found — using zero defaults")
            return

        # Mirror DriverLabSettings::initDefaults()
        kM = defines.get('MOTOR_KM', 342.0)
        tm = defines.get('MOTOR_TM', 0.05)
        kV = 1.0 / kM if kM > 1e-6 else 0.003
        kS = (defines.get('FORWARD_KSL', 0) + defines.get('FORWARD_KSR', 0)) / 2.0
        kA = tm / kM if kM > 1e-6 else 0.0

        zeta = defines.get('FWD_ZETA', 0.707)
        td = defines.get('FWD_TD', 0.025)
        kP = defines.get('FWD_KP', 0.5)
        kD = defines.get('FWD_KD', 0.0)

        turnKP = defines.get('ROT_KP', 0.15)
        turnKD = defines.get('ROT_KD', 0.0)

        # Populate widgets
        self.set_safely(self.spin_biasff, kS)
        self.set_safely(self.spin_speedff, kV)
        self.set_safely(self.spin_accff, kA)
        self.lbl_km_val.setText(f"{kM:.2f}")
        self.lbl_tm_val.setText(f"{tm:.5f}")
        self.set_safely(self.spin_kp, kP)
        self.set_safely(self.spin_kd, kD)
        self.set_safely(self.spin_td, td)
        self.set_safely(self.spin_zeta, zeta)
        self.set_safely(self.spin_turn_kp, turnKP)
        self.set_safely(self.spin_turn_kd, turnKD)

        # Store for zeta/td coupling math
        self.parameters['kM'] = kM
        self.parameters['Tm'] = tm

    # ========== PORT MANAGEMENT ==========

    def refresh_ports(self):
        """Scan and update available serial ports."""
        current_text = self.port_combo.currentText()
        self.port_combo.clear()

        # Add detected ports
        for p in serial.tools.list_ports.comports():
            # Format: /dev/cu.usbmodem1234 (Pico)
            desc = p.description if p.description else "Unknown"
            if '2E8A:' in p.hwid:
                desc = "Raspberry Pi Pico"
            elif '1A86:' in p.hwid:
                desc = "CH340 Serial"
            elif '10C4:' in p.hwid:
                desc = "CP210x UART"
            elif '2341:' in p.hwid:
                desc = "Arduino"
            self.port_combo.addItem(f"{p.device} ({desc})", p.device)

        # Restore selection if still available, otherwise default to last port
        idx = self.port_combo.findText(current_text)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
        elif self.port_combo.count() > 0:
            self.port_combo.setCurrentIndex(self.port_combo.count() - 1)

        self.log_message(f"Found {self.port_combo.count()} port(s)")

    def toggle_connection(self):
        """Connect or disconnect from the selected port."""
        if self.device:
            self.disconnect_device()
        else:
            self.connect_to_selected_port()

    def connect_to_selected_port(self):
        """Connect to the port selected in the combo box."""
        # Get the port path from item data or text
        idx = self.port_combo.currentIndex()
        if idx >= 0:
            port = self.port_combo.itemData(idx)
            if not port:
                port = self.port_combo.currentText().strip()
                if ' (' in port:
                    port = port.split(' (')[0]
        else:
            port = self.port_combo.currentText().strip()
            if ' (' in port:
                port = port.split(' (')[0]

        if not port:
            self.log_message("No port selected")
            return

        self.usb_connect(port, "Manual")

    def disconnect_device(self):
        """Disconnect from current device."""
        self.stop_monitoring()
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.device = None
        self.serial = None
        self.btn_connect.setText("Connect")
        self.show_connection_message('warning', 'Disconnected')
        self.log_message("Disconnected")

    # ========== UI HELPERS ==========

    def HLine(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet(hline_style)
        return line

    def double_spinbox(self, name, min_val, max_val, step, decimals=5):
        spin = QDoubleSpinBox()
        spin.setMinimum(min_val)
        spin.setMaximum(max_val)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setObjectName(name)
        return spin

    def set_safely(self, widget, value):
        """Locks a widget while updating its value."""
        widget.blockSignals(True)
        widget.setValue(value)
        widget.blockSignals(False)

    def parameter_change(self):
        spinner = self.sender()
        if spinner.objectName() == "zeta" or spinner.objectName() == "Td":
            if "Tm" in self.parameters and "kM" in self.parameters:
                tm = self.parameters["Tm"]
                km = self.parameters["kM"]
                z = self.spin_zeta.value()
                td = self.spin_td.value()
                td = max(td, 0.005)
                z = max(z, 0.005)
                kp = 16 * tm / km / z / z / td / td
                kd = (8 * tm - td) / td / km
                self.set_safely(self.spin_kp, kp)
                self.set_safely(self.spin_kd, kd)

    def option_select(self):
        radioButton = self.sender()
        if radioButton.isChecked():
            self.move_mode = radioButton.mode

    def center_window(self):
        screen = QApplication.primaryScreen().availableGeometry()
        qr = self.frameGeometry()
        qr.moveCenter(screen.center())
        self.move(qr.topLeft())

    def show_connection_message(self, msg_type, message, timeout=0):
        if msg_type == 'error':
            style = 'color: #E33;'
        elif msg_type == 'warning':
            style = 'color: #e12;'
        elif msg_type == 'info':
            style = 'color: #050;'
        else:
            return
        self.lbl_con_status.setStyleSheet(style)
        self.lbl_con_status.setText(message)

    # ========== SERIAL CONNECTION ==========

    @Slot(str, str)
    def usb_connect(self, port, hwid):
        try:
            self.serial = Serial(port, baudrate=115200, timeout=0.25)
            self.device = port
        except Exception as e:
            sys.stderr.write(f'Could not open port {port}: {e}\n')
            self.log_message(f"Failed to connect: {e}")
            self.device = None
            return

        if self.device:
            self.show_connection_message('info', f'CONNECTED: {self.device}')
            self.btn_connect.setText("Disconnect")
            self.device_init()
            self.read_settings()
            self.start_monitoring()  # Monitor is always on when connected

    @Slot()
    def usb_disconnect(self):
        if self.device:
            self.disconnect_device()

    def closeEvent(self, unused_event):
        self.stop_monitoring()
        if self.serial and self.serial.is_open:
            self.serial.close()

    @Slot(str, str, str)
    def show_dialog(self, title, message, details):
        msg_box = QMessageBox()
        msg_box.setDetailedText(details)
        msg_box.setWindowTitle('Message')
        msg_box.setTextFormat(Qt.TextFormat.RichText)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Close)
        msg_box.setText(title)
        msg_box.setInformativeText('<font face=Arial>' + message + '</font>')
        horizontalSpacer = QSpacerItem(550, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout = msg_box.layout()
        layout.addItem(horizontalSpacer, layout.rowCount(), 0, 1, layout.columnCount())
        msg_box.exec()

    # ========== PLOTTING ==========

    def plot(self, x, y, plot_widget, plotname, color, line_style=Qt.PenStyle.SolidLine):
        pen = pg.mkPen(color=color, width=3, style=line_style)
        plot_widget.plot(x, y, name=plotname, pen=pen)

    def _get_valid_data(self, time_channel_idx, value_channel_idx):
        """Return only data points where time is non-zero (filters out buffer padding)."""
        x_data = self.telemetry[time_channel_idx].data()
        y_data = self.telemetry[value_channel_idx].data()
        mask = x_data > 0
        return x_data[mask], y_data[mask]

    def log_data(self):
        """Batch log all data lines in a single update for performance."""
        if self.data:
            # Filter out empty lines and join into single string
            text = '\n'.join(line for line in self.data if line)
            if text:
                self.text_box.appendPlainText(text)

    def log_message(self, message):
        """Log a single message to the text box."""
        self.text_box.appendPlainText(message)

    def clear_data(self):
        self.data = []

    # ========== SERIAL COMMUNICATION ==========

    def get_response(self, timeout_sec=5.0):
        """Read response lines until '>' prompt or timeout.

        Uses empty-read counting to handle variable data rates without
        truncating data that's still being transmitted.
        """
        data = []
        empty_reads = 0
        max_empty_reads = 50  # Allow ~50 empty reads (each ~10ms) before giving up
        start_time = time.time()

        while True:
            # Check overall timeout
            if time.time() - start_time > timeout_sec:
                break

            # Non-blocking check for available data
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('ascii', errors='ignore').strip()
                empty_reads = 0  # Reset empty counter on successful read

                if line == '>':
                    break  # Normal completion - got prompt
                if line:
                    data.append(line)
            else:
                # No data available - wait a bit
                time.sleep(0.01)
                empty_reads += 1

                # Only exit after getting some data AND seeing empty reads
                if empty_reads >= max_empty_reads and len(data) > 0:
                    break

        return data

    def write(self, message):
        """Send a message and throw away the response."""
        if not self.device:
            return
        self.serial.flushInput()
        self.serial.write(message.encode('ascii'))

    def query(self, message):
        """Send a message and return the response.

        Stops the monitor thread to avoid serial read conflicts,
        then restarts it after the response is collected.
        """
        if not self.device:
            return []

        # Stop monitor thread so it doesn't steal our response bytes
        was_monitoring = self.monitoring
        if was_monitoring:
            self.stop_monitoring()

        self.serial.flushInput()
        self.serial.write(message.encode('ascii'))
        response = self.get_response()

        # Restart monitor
        if was_monitoring:
            self.start_monitoring()

        return response

    # ========== DEVICE OPERATIONS ==========

    def device_init(self):
        self.data = self.get_response()
        self.log_data()
        self.write('ECHO OFF\n')
        self.get_response()

    def update_parameters(self):
        if 'kS' in self.parameters:
            self.set_safely(self.spin_biasff, self.parameters['kS'])
        if 'kV' in self.parameters:
            self.set_safely(self.spin_speedff, self.parameters['kV'])
        if 'kA' in self.parameters:
            self.set_safely(self.spin_accff, self.parameters['kA'])
        if 'zeta' in self.parameters:
            self.set_safely(self.spin_zeta, self.parameters['zeta'])
        if 'Td' in self.parameters:
            self.set_safely(self.spin_td, self.parameters['Td'])
        if 'kP' in self.parameters:
            self.set_safely(self.spin_kp, self.parameters['kP'])
        if 'kD' in self.parameters:
            self.set_safely(self.spin_kd, self.parameters['kD'])
        if 'kM' in self.parameters:
            self.lbl_km_val.setText(f"{self.parameters['kM']:.2f}")
        if 'Tm' in self.parameters:
            self.lbl_tm_val.setText(f"{self.parameters['Tm']:.5f}")
        if 'turnKP' in self.parameters:
            self.set_safely(self.spin_turn_kp, self.parameters['turnKP'])
        if 'turnKD' in self.parameters:
            self.set_safely(self.spin_turn_kd, self.parameters['turnKD'])

    def write_parameters(self):
        """Send all parameter values to device.

        Must be called with the monitor thread stopped — otherwise the monitor
        eats the firmware's response bytes and commands silently fail.
        """
        cmds = []
        if 'kM' in self.parameters:
            cmds.append(f"KM {self.parameters['kM']}")
        if 'Tm' in self.parameters:
            cmds.append(f"TM {self.parameters['Tm']}")
        cmds.append(f"BIAS {self.spin_biasff.value()}")
        cmds.append(f"SPEEDFF {self.spin_speedff.value()}")
        cmds.append(f"ACCFF {self.spin_accff.value()}")
        cmds.append(f"ZETA {self.spin_zeta.value()}")
        cmds.append(f"TD {self.spin_td.value()}")
        cmds.append(f"KP {self.spin_kp.value()}")
        cmds.append(f"KD {self.spin_kd.value()}")
        cmds.append(f"TURN_KP {self.spin_turn_kp.value()}")
        cmds.append(f"TURN_KD {self.spin_turn_kd.value()}")

        for cmd in cmds:
            self.serial.write((cmd + '\n').encode('ascii'))
            # Wait for firmware to process + drain its response
            time.sleep(0.05)
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)

    def target_reset(self):
        self.output_plot.clear()
        self.motion_plot.clear()
        self.log_message('Device Reset')
        if self.serial:
            try:
                # Try DTR reset (works for real USB serial ports)
                self.serial.setDTR(0)
                time.sleep(0.1)
                self.serial.setDTR(1)
            except (OSError, AttributeError):
                # Ignore for virtual ports like /tmp/ttyBLE that don't support DTR
                pass
        self.log_message('----------------')
        self.device_init()
        self.read_settings()

    def read_settings(self):
        if not self.device:
            return
        self.log_message('Read Settings')
        self.data = self.query("SETTINGS\n")
        self.log_data()

        # Mapping from firmware key names to dashboard key names
        key_mapping = {
            'km': 'kM',
            'tm': 'Tm',
            'ks': 'kS',
            'kv': 'kV',
            'ka': 'kA',
            'zeta': 'zeta',
            'td': 'Td',
            'kp': 'kP',
            'kd': 'kD',
            'turnkp': 'turnKP',
            'turnkd': 'turnKD',
        }

        # Parse settings response
        for line in self.data:
            # Handle format: "key = value" or "key (description) = value unit"
            if '=' in line:
                parts = line.split('=')
                if len(parts) >= 2:
                    raw_key = parts[0].strip()
                    # Strip parenthetical descriptions: "Km (velocity const)" -> "Km"
                    if '(' in raw_key:
                        raw_key = raw_key.split('(')[0].strip()
                    # Normalize to lowercase for mapping lookup
                    key_lower = raw_key.lower()
                    # Map to dashboard key name, or use original if not in mapping
                    key = key_mapping.get(key_lower, raw_key)
                    # Get first word after = as the value
                    val_parts = parts[1].strip().split()
                    if not val_parts:
                        continue
                    try:
                        self.parameters[key] = float(val_parts[0])
                    except ValueError:
                        pass
        self.update_parameters()
        self.log_message('----------------')

    def write_settings(self):
        if not self.device:
            return
        self.log_message('Writing settings...')

        # Stop monitor so it doesn't steal response bytes during writes
        was_monitoring = self.monitoring
        if was_monitoring:
            self.stop_monitoring()

        self.serial.flushInput()
        self.write_parameters()

        # Now read back settings to confirm
        self.serial.write(b"SETTINGS\n")
        self.data = self.get_response()
        self.log_data()

        # Parse the response to update dashboard
        for line in self.data:
            if '=' in line:
                parts = line.split('=')
                if len(parts) >= 2:
                    raw_key = parts[0].strip()
                    if '(' in raw_key:
                        raw_key = raw_key.split('(')[0].strip()
                    key_lower = raw_key.lower()
                    key_mapping = {
                        'km': 'kM', 'tm': 'Tm', 'ks': 'kS', 'kv': 'kV', 'ka': 'kA',
                        'zeta': 'zeta', 'td': 'Td', 'kp': 'kP', 'kd': 'kD',
                        'turnkp': 'turnKP', 'turnkd': 'turnKD',
                    }
                    key = key_mapping.get(key_lower, raw_key)
                    val_parts = parts[1].strip().split()
                    if val_parts:
                        try:
                            self.parameters[key] = float(val_parts[0])
                        except ValueError:
                            pass
        self.update_parameters()

        self.log_message('Settings written OK')

        if was_monitoring:
            self.start_monitoring()

    def reset_settings(self):
        if not self.device:
            return
        self.log_message('Default Settings')
        self.data = self.query("INIT\n")
        self.log_message('----------------')
        self.read_settings()

    def send_move(self):
        """Send MOVE command with selected control mode."""
        self.run_trial(f"MOVE 360 500 1000 {self.move_mode}")
        return  # Data comes through monitor, legacy parsing below kept for reference

        self.data = self.query(f'MOVE 360 500 1000 {self.move_mode}\n')
        self.log_data()

        # Parse CSV data
        # Format: time_ms,set_pos,actual_pos,set_speed,actual_speed,ctrl_v,ff_v,total_v
        d = [[] for i in range(self.nChannels)]
        headings = []

        for line in self.data:
            if len(line) == 0:
                continue
            if line.startswith('#'):
                continue
            if line.startswith('time_ms'):
                headings = line.split(',')
                continue

            parts = line.split(',')
            for i in range(min(len(parts), len(d))):
                try:
                    f = float(parts[i])
                    # Clamp voltage values
                    if i >= 5:
                        f = max(min(f, 6.0), -6.0)
                    d[i].append(f)
                except ValueError:
                    pass

        if len(d[0]) > 0:
            self.output_plot.clear()

            # FF Volts
            if self.move_mode == NO_FF:
                style = Qt.PenStyle.DotLine
            else:
                style = Qt.PenStyle.SolidLine
            if len(d) > 6 and len(d[6]) > 0:
                self.plot(d[0], d[6], self.output_plot, "FF Volts", palette[1], style)

            # CTRL Volts
            if self.move_mode == ONLY_FF:
                style = Qt.PenStyle.DotLine
            else:
                style = Qt.PenStyle.SolidLine
            if len(d) > 5 and len(d[5]) > 0:
                self.plot(d[0], d[5], self.output_plot, "CTRL Volts", palette[2], style)

            # Motor Volts (total)
            if len(d) > 7 and len(d[7]) > 0:
                self.plot(d[0], d[7], self.output_plot, "Motor Volts", palette[3], Qt.PenStyle.SolidLine)

            self.output_plot.enableAutoRange()
            self.output_plot.setYRange(-7, 7)

            self.motion_plot.clear()
            # Set Speed (dashed) vs Actual Speed
            if len(d) > 3 and len(d[3]) > 0:
                self.plot(d[0], d[3], self.motion_plot, "Set Speed", palette[4], Qt.PenStyle.DashLine)
            if len(d) > 4 and len(d[4]) > 0:
                self.plot(d[0], d[4], self.motion_plot, "Actual Speed", palette[5])

            self.motion_plot.enableAutoRange()
            self.output_plot.setXLink(self.motion_plot)

    def about(self, event):
        self.show_dialog('App info',
                         '<br>Jurababa DriverLab Dashboard<br>'
                         'Based on UKMARS DriverLab by Peter Harrison<br>',
                         '')

    # ========== SERIAL MONITOR ==========

    def run_trial(self, cmd):
        """Send a trial command via the serial monitor path (button-initiated)."""
        if not self.device:
            self.log_message("Not connected")
            return
        self.clear_monitor()
        self.text_box.appendPlainText(f"> {cmd}")
        self.serial.write((cmd + '\n').encode('ascii'))

    def send_command(self):
        """Send a command from the input field with local echo."""
        cmd = self.cmd_input.text().strip()
        if not cmd or not self.device:
            return

        # Clear graphs for trial commands (fresh plot for each trial)
        cmd_upper = cmd.split()[0].upper() if cmd.split() else ""
        if cmd_upper in ('OL', 'MOVE', 'TURN', 'STEP'):
            self.clear_monitor()

        # Local echo
        self.text_box.appendPlainText(f"> {cmd}")

        # Send with newline
        self.serial.write((cmd + '\n').encode('ascii'))

        # Clear input
        self.cmd_input.clear()

    def start_monitoring(self):
        """Start monitoring serial port in background thread."""
        if self.monitoring or not self.device:
            return

        self.monitoring = True
        self.monitor_thread = SerialMonitor(self.serial)
        self.monitor_thread.data_received.connect(self.on_monitor_data)
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop monitoring serial port."""
        if not self.monitoring:
            return

        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.stop()
            self.monitor_thread.wait()
            self.monitor_thread = None

    @Slot(str)
    def on_monitor_data(self, line):
        """Handle incoming serial monitor data."""
        self.text_box.appendPlainText(line)

        # Skip empty lines and comments
        if not line or line.startswith('#'):
            return

        # Auto-populate settings from trial results
        # STEP trial: "Tm = 0.05000 s  (motor time constant)"
        if line.startswith('Tm ='):
            try:
                val = float(line.split('=')[1].strip().split()[0])
                self.parameters['Tm'] = val
                self.lbl_tm_val.setText(f"{val:.5f}")
            except (ValueError, IndexError):
                pass
        # OL trial: "Combined: kM=342.00 mm/s/V, kS=0.3500 V (12 points)"
        elif line.startswith('Combined:'):
            for part in line.split(','):
                part = part.strip()
                if 'kM=' in part:
                    try:
                        val = float(part.split('kM=')[1].split()[0])
                        self.parameters['kM'] = val
                        self.lbl_km_val.setText(f"{val:.2f}")
                    except (ValueError, IndexError):
                        pass
                elif 'kS=' in part:
                    try:
                        val = float(part.split('kS=')[1].split()[0])
                        self.parameters['kS'] = val
                        self.set_safely(self.spin_biasff, val)
                    except (ValueError, IndexError):
                        pass
        # STEP/OL derived: "  -> kA = 0.0001234 V/(mm/s^2)"
        elif line.strip().startswith('-> kA ='):
            try:
                val = float(line.split('=')[1].strip().split()[0])
                self.parameters['kA'] = val
                self.set_safely(self.spin_accff, val)
            except (ValueError, IndexError):
                pass
        elif line.strip().startswith('-> Td ='):
            try:
                val = float(line.split('=')[1].strip().split()[0])
                self.parameters['Td'] = val
                self.set_safely(self.spin_td, val)
            except (ValueError, IndexError):
                pass
        elif line.strip().startswith('-> kP ='):
            try:
                # "  -> kP = 0.12345, kD = 0.00123  (recomputed)"
                parts = line.split(',')
                kp_val = float(parts[0].split('=')[1].strip())
                self.parameters['kP'] = kp_val
                self.set_safely(self.spin_kp, kp_val)
                if len(parts) > 1 and 'kD' in parts[1]:
                    kd_val = float(parts[1].split('=')[1].strip().split()[0])
                    self.parameters['kD'] = kd_val
                    self.set_safely(self.spin_kd, kd_val)
            except (ValueError, IndexError):
                pass
        # OL/STEP derived: "  -> kV = 0.0029240 V/(mm/s)"
        elif 'kV =' in line and 'V/(mm/s)' in line:
            try:
                val = float(line.split('kV =')[1].strip().split()[0])
                self.parameters['kV'] = val
                self.set_safely(self.spin_speedff, val)
            except (ValueError, IndexError):
                pass

        # Parse CSV header to identify columns
        if line.startswith('time_ms'):
            self.csv_headings = line.split(',')
            # Clear old plot curves when header arrives (new trial starting)
            self.output_plot.clear()
            self.motion_plot.clear()
            self.plot_curves = {'output': [], 'motion': []}
            return
        
        # Parse CSV data line
        try:
            parts = line.split(',')
            if len(parts) < 2:
                return
            
            # Convert all parts to floats
            values = []
            for part in parts:
                try:
                    values.append(float(part))
                except ValueError:
                    return  # Skip malformed lines
            
            if len(values) == 0:
                return
            
            # Add values to telemetry channels
            for i in range(min(len(values), self.nChannels)):
                self.telemetry[i].add_new_value(values[i])
            
            # Determine data type from headings and update plots accordingly
            if self.csv_headings:
                headings_str = ','.join(self.csv_headings).lower()

                if 'steer_v' in headings_str:
                    # Stereo OL: time_ms,cmd_v,left_v,right_v,left_speed,right_speed,steer_v,yaw
                    # Output plot: cmd voltage, steering correction, yaw
                    if len(self.plot_curves['output']) == 0:
                        pen_cmd = pg.mkPen(color=palette[3], width=2)
                        pen_steer = pg.mkPen(color=palette[2], width=2, style=Qt.PenStyle.DashLine)
                        pen_yaw = pg.mkPen(color=palette[6], width=2)
                        self.plot_curves['output'].append(self.output_plot.plot(name='Cmd Voltage', pen=pen_cmd))
                        self.plot_curves['output'].append(self.output_plot.plot(name='Steer Correction', pen=pen_steer))
                        self.plot_curves['output'].append(self.output_plot.plot(name='Yaw (deg)', pen=pen_yaw))

                    x, y = self._get_valid_data(0, 1)  # cmd_v
                    self.plot_curves['output'][0].setData(x, y)
                    x, y = self._get_valid_data(0, 6)  # steer_v
                    if len(self.plot_curves['output']) > 1:
                        self.plot_curves['output'][1].setData(x, y)
                    if len(values) > 7 and len(self.plot_curves['output']) > 2:
                        x, y = self._get_valid_data(0, 7)  # yaw
                        self.plot_curves['output'][2].setData(x, y)
                    self.output_plot.enableAutoRange()

                    # Motion plot: left speed vs right speed
                    if len(self.plot_curves['motion']) == 0:
                        pen_left = pg.mkPen(color=palette[1], width=2)
                        pen_right = pg.mkPen(color=palette[5], width=2)
                        self.plot_curves['motion'].append(self.motion_plot.plot(name='Left Speed', pen=pen_left))
                        self.plot_curves['motion'].append(self.motion_plot.plot(name='Right Speed', pen=pen_right))

                    x, y = self._get_valid_data(0, 4)  # left_speed
                    self.plot_curves['motion'][0].setData(x, y)
                    x, y = self._get_valid_data(0, 5)  # right_speed
                    if len(self.plot_curves['motion']) > 1:
                        self.plot_curves['motion'][1].setData(x, y)
                    self.motion_plot.enableAutoRange()
                    self.output_plot.setXLink(self.motion_plot)

                elif 'set_yaw' in headings_str:
                    # Turn: time_ms,set_yaw,actual_yaw,set_omega,actual_omega,volts
                    # Output plot: control voltage
                    if len(self.plot_curves['output']) == 0:
                        pen_volts = pg.mkPen(color=palette[3], width=2)
                        self.plot_curves['output'].append(self.output_plot.plot(name='Ctrl Volts', pen=pen_volts))

                    x, y = self._get_valid_data(0, 5)  # volts
                    self.plot_curves['output'][0].setData(x, y)
                    self.output_plot.enableAutoRange()

                    # Motion plot: set yaw vs actual yaw
                    if len(self.plot_curves['motion']) == 0:
                        pen_set = pg.mkPen(color=palette[4], width=2, style=Qt.PenStyle.DashLine)
                        pen_actual = pg.mkPen(color=palette[5], width=2)
                        self.plot_curves['motion'].append(self.motion_plot.plot(name='Set Yaw', pen=pen_set))
                        self.plot_curves['motion'].append(self.motion_plot.plot(name='Actual Yaw', pen=pen_actual))

                    x, y = self._get_valid_data(0, 1)  # set_yaw
                    self.plot_curves['motion'][0].setData(x, y)
                    x, y = self._get_valid_data(0, 2)  # actual_yaw
                    if len(self.plot_curves['motion']) > 1:
                        self.plot_curves['motion'][1].setData(x, y)
                    self.motion_plot.setLabel('left', 'Angle (deg)')
                    self.motion_plot.enableAutoRange()
                    self.output_plot.setXLink(self.motion_plot)

                elif 'step_voltage' in headings_str:
                    # Step: time_ms,step_voltage,speed,position
                    if len(self.plot_curves['output']) == 0:
                        pen = pg.mkPen(color=palette[4], width=2)
                        self.plot_curves['output'].append(self.output_plot.plot(name='Step Voltage', pen=pen))

                    x, y = self._get_valid_data(0, 1)
                    self.plot_curves['output'][0].setData(x, y)
                    self.output_plot.enableAutoRange(axis='x')
                    self.output_plot.setYRange(-1, 7)

                    if len(values) > 2:
                        if len(self.plot_curves['motion']) == 0:
                            pen = pg.mkPen(color=palette[5], width=2)
                            self.plot_curves['motion'].append(self.motion_plot.plot(name='Speed', pen=pen))
                        x, y = self._get_valid_data(0, 2)
                        self.plot_curves['motion'][0].setData(x, y)
                        self.motion_plot.setLabel('left', 'Speed (mm/s)')
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

                elif 'ctrl_v' in headings_str or 'ff_v' in headings_str:
                    # Move controller: time_ms,set_pos,actual_pos,set_speed,actual_speed,ctrl_v,ff_v,total_v
                    # Output plot: FF, ctrl, total voltage
                    if 'ff_v' in headings_str and len(values) > 6:
                        if len(self.plot_curves['output']) == 0:
                            self.plot_curves['output'].append(self.output_plot.plot(
                                name='FF Volts', pen=pg.mkPen(color=palette[1], width=2)))
                        x, y = self._get_valid_data(0, 6)
                        self.plot_curves['output'][0].setData(x, y)

                    if 'ctrl_v' in headings_str and len(values) > 5:
                        if len(self.plot_curves['output']) < 2:
                            self.plot_curves['output'].append(self.output_plot.plot(
                                name='Ctrl Volts', pen=pg.mkPen(color=palette[2], width=2)))
                        x, y = self._get_valid_data(0, 5)
                        if len(self.plot_curves['output']) > 1:
                            self.plot_curves['output'][1].setData(x, y)

                    if 'total_v' in headings_str and len(values) > 7:
                        if len(self.plot_curves['output']) < 3:
                            self.plot_curves['output'].append(self.output_plot.plot(
                                name='Total Volts', pen=pg.mkPen(color=palette[7], width=2)))
                        x, y = self._get_valid_data(0, 7)
                        if len(self.plot_curves['output']) > 2:
                            self.plot_curves['output'][2].setData(x, y)

                    self.output_plot.enableAutoRange()

                    # Motion plot: set speed (dashed) vs actual speed
                    if len(values) > 4:
                        if len(self.plot_curves['motion']) == 0:
                            self.plot_curves['motion'].append(self.motion_plot.plot(
                                name='Set Speed', pen=pg.mkPen(color=palette[4], width=2, style=Qt.PenStyle.DashLine)))
                            self.plot_curves['motion'].append(self.motion_plot.plot(
                                name='Actual Speed', pen=pg.mkPen(color=palette[5], width=2)))

                        x, y = self._get_valid_data(0, 3)  # set_speed
                        self.plot_curves['motion'][0].setData(x, y)
                        x, y = self._get_valid_data(0, 4)  # actual_speed
                        if len(self.plot_curves['motion']) > 1:
                            self.plot_curves['motion'][1].setData(x, y)
                        self.motion_plot.setLabel('left', 'Speed (mm/s)')
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

                elif 'motor_volts' in headings_str:
                    # Profile: time_ms,set_pos,actual_pos,set_speed,actual_speed,motor_volts
                    if len(self.plot_curves['output']) == 0:
                        self.plot_curves['output'].append(self.output_plot.plot(
                            name='Motor Volts', pen=pg.mkPen(color=palette[6], width=2)))
                    x, y = self._get_valid_data(0, 5)
                    self.plot_curves['output'][0].setData(x, y)
                    self.output_plot.enableAutoRange()

                    if len(values) > 4:
                        if len(self.plot_curves['motion']) == 0:
                            self.plot_curves['motion'].append(self.motion_plot.plot(
                                name='Actual Speed', pen=pg.mkPen(color=palette[5], width=2)))
                        x, y = self._get_valid_data(0, 4)
                        self.plot_curves['motion'][0].setData(x, y)
                        self.motion_plot.setLabel('left', 'Speed (mm/s)')
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

        except Exception as e:
            # Silently ignore parsing errors to avoid spam
            pass

    def clear_monitor(self):
        """Clear the text box and reset graphs."""
        self.text_box.clear()

        # Reset graphs
        self.output_plot.clear()
        self.motion_plot.clear()
        self.plot_curves = {'output': [], 'motion': []}
        self.csv_headings = []

        # Reset telemetry channels
        for channel in self.telemetry:
            channel.data_ = np.zeros(shape=channel.points, dtype=float)

        # Reset to default ranges and labels
        styles = {'color': 'cyan', 'font-size': '13px', 'bottom_margin': '50px'}
        self.motion_plot.setLabel('left', 'Speed (mm/s)', **styles)
        self.output_plot.setYRange(-1, 7)
        self.motion_plot.setYRange(-50, 500)
        self.output_plot.enableAutoRange(enable=True)
        self.motion_plot.enableAutoRange(enable=True)


class SerialMonitor(QThread):
    """Background thread to monitor serial port."""
    data_received = Signal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.serial = serial_port
        self.running = True

    def run(self):
        """Read serial data continuously."""
        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def stop(self):
        """Stop the monitor thread."""
        self.running = False


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create the app main window
    app_window = Dashboard()

    # Connect signals to slots
    app_window.dialog.connect(app_window.show_dialog)
    app_window.usb_con.connect(app_window.usb_connect)
    app_window.usb_dis.connect(app_window.usb_disconnect)

    # Launch application
    sys.exit(app.exec())
