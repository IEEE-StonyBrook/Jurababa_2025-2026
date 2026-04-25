#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
MotorLab Dashboard - Jurababa Motor Characterization Interface

Based on UKMARS MotorLab Dashboard by Peter Harrison.
Adapted for Raspberry Pi Pico with Jurababa micromouse firmware.

Usage:
    python3 motorlab_dashboard.py

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
    QStatusBar,
    QSpacerItem, QMessageBox, QSizePolicy,
    QWidget, QGroupBox, QFrame,
    QDoubleSpinBox, QComboBox,
    QGridLayout, QHBoxLayout, QVBoxLayout,
    QPlainTextEdit, QPushButton,
    QRadioButton, QLabel, QSplitter, QButtonGroup)

import pyqtgraph as pg

# App versioning
APP_VERSION = '1.0.0'
APP_NAME = 'Jurababa MotorLab'

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

        self.nChannels = 8
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

        # Monitor and Clear buttons
        monitor_layout = QHBoxLayout()
        self.chk_monitor = QRadioButton("Monitor")
        self.chk_monitor.setToolTip("Show live serial data")
        self.chk_monitor.toggled.connect(self.toggle_monitor)
        monitor_layout.addWidget(self.chk_monitor)

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

        # ========== SETTINGS GROUP ==========
        self.settings_grid = QGridLayout()
        self.settings_grid.setContentsMargins(0, 0, 0, 0)

        self.spin_biasff = self.double_spinbox("FF kS", 0.0, 0.4, 0.01, 2)
        self.spin_speedff = self.double_spinbox("FF kV", 0.0, 0.001, 0.00001, 5)
        self.spin_accff = self.double_spinbox("FF kA", 0.0, 0.001, 0.00001, 5)
        self.spin_zeta = self.double_spinbox("ζ", 0.0, 2.0, 0.005, 3)
        self.spin_td = self.double_spinbox("Td", 0.0, 1.00, 0.01, 3)
        self.spin_kp = self.double_spinbox("kP", 0.0, 8.0, 0.001, 4)
        self.spin_kd = self.double_spinbox("kD", 0.0, 2.0, 0.0001, 4)

        self.spin_biasff.valueChanged.connect(self.parameter_change)
        self.spin_speedff.valueChanged.connect(self.parameter_change)
        self.spin_accff.valueChanged.connect(self.parameter_change)
        self.spin_zeta.valueChanged.connect(self.parameter_change)
        self.spin_td.valueChanged.connect(self.parameter_change)
        self.spin_kp.valueChanged.connect(self.parameter_change)
        self.spin_kd.valueChanged.connect(self.parameter_change)

        # Labels
        self.lbl_biasff = QLabel("kS:")
        self.lbl_biasff.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_speedff = QLabel("kV:")
        self.lbl_speedff.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_accff = QLabel("kA:")
        self.lbl_accff.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_zeta = QLabel("Damping Ratio:")
        self.lbl_zeta.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_td = QLabel("Settling Time:")
        self.lbl_td.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_kp = QLabel("kP:")
        self.lbl_kp.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_kd = QLabel("kD:")
        self.lbl_kd.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_km = QLabel("kM:")
        self.lbl_km.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_km_val = QLabel("0.000")
        self.lbl_tm = QLabel("Tm:")
        self.lbl_tm.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTrailing | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_tm_val = QLabel("0.000")

        # Grid layout for settings
        self.settings_grid.addWidget(self.lbl_biasff, 0, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_biasff, 0, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_speedff, 1, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_speedff, 1, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_accff, 2, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_accff, 2, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_zeta, 3, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_zeta, 3, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_td, 4, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_td, 4, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_kp, 5, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_kp, 5, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_kd, 6, 0, 1, 1)
        self.settings_grid.addWidget(self.spin_kd, 6, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_km, 7, 0, 1, 1)
        self.settings_grid.addWidget(self.lbl_km_val, 7, 1, 1, 1)
        self.settings_grid.addWidget(self.lbl_tm, 8, 0, 1, 1)
        self.settings_grid.addWidget(self.lbl_tm_val, 8, 1, 1, 1)

        # Settings buttons
        self.btn_read_settings = QPushButton('READ', self)
        self.btn_read_settings.clicked.connect(self.read_settings)
        self.btn_read_settings.setEnabled(False)

        self.btn_write_settings = QPushButton('WRITE', self)
        self.btn_write_settings.clicked.connect(self.write_settings)
        self.btn_write_settings.setEnabled(False)

        self.btn_reset_settings = QPushButton('RESET', self)
        self.btn_reset_settings.clicked.connect(self.reset_settings)
        self.btn_reset_settings.setEnabled(False)

        settings_button_layout = QHBoxLayout()
        settings_button_layout.addWidget(self.btn_read_settings)
        settings_button_layout.addWidget(self.btn_write_settings)
        settings_button_layout.addWidget(self.btn_reset_settings)

        self.settings_grid.addLayout(settings_button_layout, 9, 0, 1, 2)

        settings_group = QGroupBox()
        settings_group.setObjectName("SettingsGroup")
        settings_group.setStyleSheet("QGroupBox#SettingsGroup { border: 1px solid gray;}")
        settings_group.setLayout(self.settings_grid)

        # ========== BUTTONS LAYOUT ==========
        self.btn_id = QPushButton('ID', self)
        self.btn_id.clicked.connect(self.get_id)
        self.btn_id.setEnabled(False)

        self.btn_move = QPushButton('MOVE', self)
        self.btn_move.clicked.connect(self.send_move)
        self.btn_move.setEnabled(False)

        self.btn_reset = QPushButton('RESET', self)
        self.btn_reset.clicked.connect(self.target_reset)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.btn_id)
        button_layout.addWidget(self.btn_move)
        button_layout.addWidget(self.btn_reset)

        # ========== OPTION LAYOUT (Radio Buttons) ==========
        option_layout = QHBoxLayout()
        self.rb_full = QRadioButton("Full Control")
        self.rb_full.mode = FULL_CONTROL
        self.rb_full.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_full)

        self.rb_noff = QRadioButton("No FF")
        self.rb_noff.mode = NO_FF
        self.rb_noff.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_noff)

        self.rb_onlyff = QRadioButton("Only FF")
        self.rb_onlyff.mode = ONLY_FF
        self.rb_onlyff.toggled.connect(self.option_select)
        option_layout.addWidget(self.rb_onlyff)
        self.rb_onlyff.setChecked(True)

        # Group radio buttons for exclusive selection
        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.addButton(self.rb_full)
        self.mode_button_group.addButton(self.rb_noff)
        self.mode_button_group.addButton(self.rb_onlyff)

        # ========== SIDEBAR ==========
        self.side_bar = QWidget()
        side_layout = QVBoxLayout()
        side_layout.addWidget(port_group)
        side_layout.addWidget(self.text_box)
        side_layout.addWidget(self.HLine())
        side_layout.addLayout(option_layout)
        side_layout.addLayout(button_layout)
        side_layout.addWidget(settings_group)
        self.side_bar.setLayout(side_layout)
        self.side_bar.setMinimumWidth(250)

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
        self.setWindowTitle("Jurababa MotorLab Dashboard")
        self.setCentralWidget(main_widget)
        self.setMinimumSize(1200, 900)
        self.resize(1200, 900)
        self.show()
        self.center_window()
        self.setFocus()

        # Initial port scan
        self.refresh_ports()

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

        # Restore selection if still available
        idx = self.port_combo.findText(current_text)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

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
        self.chk_monitor.setChecked(False)
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.device = None
        self.serial = None
        self.btn_connect.setText("Connect")
        self.btn_read_settings.setEnabled(False)
        self.btn_write_settings.setEnabled(False)
        self.btn_reset_settings.setEnabled(False)
        self.btn_move.setEnabled(False)
        self.btn_id.setEnabled(False)
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
            self.btn_read_settings.setEnabled(True)
            self.btn_write_settings.setEnabled(True)
            self.btn_reset_settings.setEnabled(True)
            self.btn_move.setEnabled(True)
            self.btn_id.setEnabled(True)
            self.chk_monitor.setChecked(True)  # Auto-enable monitoring

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
        """Send a message and return the response."""
        if not self.device:
            return []

        # Temporarily pause monitor to avoid conflicts
        was_monitoring = self.monitoring
        if was_monitoring:
            self.monitoring = False
            time.sleep(0.1)  # Let monitor thread finish current read

        self.serial.flushInput()
        self.serial.write(message.encode('ascii'))
        response = self.get_response()

        # Resume monitor if it was running
        if was_monitoring:
            self.monitoring = True

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

    def write_parameters(self):
        """Send all parameter values to device."""
        # Send kM and Tm if we have them (from OL calculation)
        if 'kM' in self.parameters:
            self.write(f"KM {self.parameters['kM']}\n")
        if 'Tm' in self.parameters:
            self.write(f"TM {self.parameters['Tm']}\n")
        self.write(f"BIAS {self.spin_biasff.value()}\n")
        self.write(f"SPEEDFF {self.spin_speedff.value()}\n")
        self.write(f"ACCFF {self.spin_accff.value()}\n")
        self.write(f"ZETA {self.spin_zeta.value()}\n")
        self.write(f"TD {self.spin_td.value()}\n")
        self.write(f"KP {self.spin_kp.value()}\n")
        self.write(f"KD {self.spin_kd.value()}\n")

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
                    val_str = parts[1].strip().split()[0]
                    try:
                        self.parameters[key] = float(val_str)
                    except ValueError:
                        pass
        self.update_parameters()
        self.log_message('----------------')

    def write_settings(self):
        if not self.device:
            return
        self.log_message('Save Settings')
        self.write_parameters()
        self.data = self.query("SETTINGS\n")
        self.log_data()
        self.log_message('----------------')

    def reset_settings(self):
        if not self.device:
            return
        self.log_message('Default Settings')
        self.data = self.query("INIT\n")
        self.log_message('----------------')
        self.read_settings()

    def get_id(self):
        """Query device ID and run open-loop test."""
        if not self.device:
            self.log_message("ERROR: Not connected to device")
            return

        # Clear graphs and serial monitor before new trial
        self.clear_monitor()

        self.log_message("Sending OL command...")
        self.data = self.query('OL\n')
        self.log_message(f"Received {len(self.data)} lines")
        self.log_data()

        # Parse CSV data
        headings = []
        d = [[] for i in range(8)]

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
                    d[i].append(float(parts[i]))
                except ValueError:
                    pass

        if len(d[0]) > 0:
            self.output_plot.clear()
            self.plot(d[0], d[1], self.output_plot, "Voltage", palette[3], Qt.PenStyle.SolidLine)
            self.output_plot.setYRange(-1, 7)

            self.motion_plot.clear()
            self.plot(d[0], d[2], self.motion_plot, "Speed", palette[5])
            self.motion_plot.enableAutoRange()
            self.output_plot.setXLink(self.motion_plot)

        # Extract calculated kM, kS, Tm from OL output
        for line in self.data:
            if line.startswith('kM ='):
                parts = line.split('=')
                if len(parts) == 2:
                    val_str = parts[1].strip().split()[0]
                    try:
                        kM_val = float(val_str)
                        self.parameters['kM'] = kM_val
                        self.lbl_km_val.setText(f"{kM_val:.2f}")
                    except ValueError:
                        pass
            elif line.startswith('kS ='):
                parts = line.split('=')
                if len(parts) == 2:
                    val_str = parts[1].strip().split()[0]
                    try:
                        kS_val = float(val_str)
                        self.parameters['kS'] = kS_val
                        self.set_safely(self.spin_biasff, kS_val)
                    except ValueError:
                        pass
            elif line.startswith('Tm ='):
                parts = line.split('=')
                if len(parts) == 2:
                    val_str = parts[1].strip().split()[0]
                    try:
                        Tm_val = float(val_str)
                        self.parameters['Tm'] = Tm_val
                        self.lbl_tm_val.setText(f"{Tm_val:.5f}")
                    except ValueError:
                        pass

        # Auto-populate all derived values from kM and Tm
        if 'kM' in self.parameters:
            kM = self.parameters['kM']

            # kV = 1/kM (speed feedforward)
            kV = 1.0 / kM
            self.parameters['kV'] = kV
            self.set_safely(self.spin_speedff, kV)

            # zeta = 0.707 (critical damping)
            self.parameters['zeta'] = 0.707
            self.set_safely(self.spin_zeta, 0.707)

            if 'Tm' in self.parameters:
                Tm = self.parameters['Tm']

                # kA = Tm/kM (acceleration feedforward)
                kA = Tm / kM
                self.parameters['kA'] = kA
                self.set_safely(self.spin_accff, kA)

                # Td = Tm/2 (common choice for derivative time)
                Td = Tm / 2.0
                self.parameters['Td'] = Td
                self.set_safely(self.spin_td, Td)

                # kP = 1/(kM * Td)
                kP = 1.0 / (kM * Td)
                self.parameters['kP'] = kP
                self.set_safely(self.spin_kp, kP)

                # kD = (2*zeta*Td - Tm) / kM
                zeta = 0.707
                kD = (2.0 * zeta * Td - Tm) / kM
                self.parameters['kD'] = kD
                self.set_safely(self.spin_kd, kD)

    def send_move(self):
        """Send MOVE command with selected control mode."""
        if not self.device:
            return

        # Clear graphs and serial monitor before new trial
        self.clear_monitor()

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
            # Set Speed (use DashLine for better visibility)
            if len(d) > 3 and len(d[3]) > 0:
                self.plot(d[0], d[3], self.motion_plot, "Set Speed", palette[4], Qt.PenStyle.DashLine)
            # Robot Speed
            if len(d) > 4 and len(d[4]) > 0:
                self.plot(d[0], d[4], self.motion_plot, "Robot Speed", palette[5])

            self.motion_plot.enableAutoRange()
            self.output_plot.setXLink(self.motion_plot)

    def about(self, event):
        self.show_dialog('App info',
                         '<br>Jurababa MotorLab Dashboard<br>'
                         'Based on UKMARS MotorLab by Peter Harrison<br>',
                         '')

    # ========== SERIAL MONITOR ==========

    def toggle_monitor(self, checked):
        """Enable or disable serial monitoring."""
        if checked and self.device:
            self.start_monitoring()
        else:
            self.stop_monitoring()

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
                
                # Update output plot based on available data
                if 'voltage' in headings_str and len(values) > 1:
                    # OpenLoop format: time_ms, voltage, speed
                    if len(self.plot_curves['output']) == 0:
                        pen = pg.mkPen(color=palette[3], width=2)
                        curve = self.output_plot.plot(name='Voltage', pen=pen)
                        self.plot_curves['output'].append(curve)
                    
                    x_data, y_data = self._get_valid_data(0, 1)
                    self.plot_curves['output'][0].setData(x_data, y_data)
                    self.output_plot.enableAutoRange(axis='x')
                    self.output_plot.setYRange(-1, 7)

                    # Update motion plot
                    if len(values) > 2:
                        if len(self.plot_curves['motion']) == 0:
                            pen = pg.mkPen(color=palette[5], width=2)
                            curve = self.motion_plot.plot(name='Speed', pen=pen)
                            self.plot_curves['motion'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 2)
                        self.plot_curves['motion'][0].setData(x_data, y_data)
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

                elif 'step_voltage' in headings_str and len(values) > 1:
                    # Step format: time_ms, step_voltage, speed, position
                    if len(self.plot_curves['output']) == 0:
                        pen = pg.mkPen(color=palette[4], width=2)
                        curve = self.output_plot.plot(name='Step Voltage', pen=pen)
                        self.plot_curves['output'].append(curve)
                    
                    x_data, y_data = self._get_valid_data(0, 1)
                    self.plot_curves['output'][0].setData(x_data, y_data)
                    self.output_plot.enableAutoRange(axis='x')
                    self.output_plot.setYRange(-1, 7)

                    # Update motion plot with speed
                    if len(values) > 2:
                        if len(self.plot_curves['motion']) == 0:
                            pen = pg.mkPen(color=palette[5], width=2)
                            curve = self.motion_plot.plot(name='Speed', pen=pen)
                            self.plot_curves['motion'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 2)
                        self.plot_curves['motion'][0].setData(x_data, y_data)
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

                elif 'motor_volts' in headings_str and len(values) > 1:
                    # Profile format: time_ms, set_pos, actual_pos, set_speed, actual_speed, motor_volts
                    if len(self.plot_curves['output']) == 0:
                        pen = pg.mkPen(color=palette[6], width=2)
                        curve = self.output_plot.plot(name='Motor Volts', pen=pen)
                        self.plot_curves['output'].append(curve)
                    
                    x_data, y_data = self._get_valid_data(0, 5)
                    self.plot_curves['output'][0].setData(x_data, y_data)
                    self.output_plot.enableAutoRange(axis='x')
                    self.output_plot.setYRange(-1, 7)

                    # Update motion plot with actual speed
                    if len(values) > 4:
                        if len(self.plot_curves['motion']) == 0:
                            pen = pg.mkPen(color=palette[5], width=2)
                            curve = self.motion_plot.plot(name='Actual Speed', pen=pen)
                            self.plot_curves['motion'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 4)
                        self.plot_curves['motion'][0].setData(x_data, y_data)
                        self.motion_plot.enableAutoRange()
                        self.output_plot.setXLink(self.motion_plot)

                elif 'ctrl_v' in headings_str or 'ff_v' in headings_str:
                    # Controller format: time_ms, set_pos, actual_pos, set_speed, actual_speed, ctrl_v, ff_v, total_v
                    # or Profile format: time_ms, set_pos, actual_pos, set_speed, actual_speed, motor_volts

                    # Update FF voltage curve
                    if 'ff_v' in headings_str and len(values) > 6:
                        if len(self.plot_curves['output']) == 0:
                            pen = pg.mkPen(color=palette[1], width=2, style=Qt.PenStyle.SolidLine)
                            curve = self.output_plot.plot(name='FF Volts', pen=pen)
                            self.plot_curves['output'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 6)
                        self.plot_curves['output'][0].setData(x_data, y_data)

                    # Update control voltage curve
                    if 'ctrl_v' in headings_str and len(values) > 5:
                        if len(self.plot_curves['output']) < 2:
                            pen = pg.mkPen(color=palette[2], width=2, style=Qt.PenStyle.SolidLine)
                            curve = self.output_plot.plot(name='Ctrl Volts', pen=pen)
                            self.plot_curves['output'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 5)
                        if len(self.plot_curves['output']) > 1:
                            self.plot_curves['output'][1].setData(x_data, y_data)

                    # Update total voltage curve
                    if 'total_v' in headings_str and len(values) > 7:
                        if len(self.plot_curves['output']) < 3:
                            pen = pg.mkPen(color=palette[7], width=2, style=Qt.PenStyle.SolidLine)
                            curve = self.output_plot.plot(name='Total Volts', pen=pen)
                            self.plot_curves['output'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 7)
                        if len(self.plot_curves['output']) > 2:
                            self.plot_curves['output'][2].setData(x_data, y_data)

                    self.output_plot.enableAutoRange(axis='x')
                    self.output_plot.setYRange(-1, 7)

                    # Update motion plot with actual speed
                    if len(values) > 4:
                        if len(self.plot_curves['motion']) == 0:
                            pen = pg.mkPen(color=palette[5], width=2)
                            curve = self.motion_plot.plot(name='Actual Speed', pen=pen)
                            self.plot_curves['motion'].append(curve)

                        x_data, y_data = self._get_valid_data(0, 4)
                        self.plot_curves['motion'][0].setData(x_data, y_data)
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

        # Reset telemetry channels
        for channel in self.telemetry:
            channel.data_ = np.zeros(shape=channel.points, dtype=float)

        # Reset to default ranges
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
