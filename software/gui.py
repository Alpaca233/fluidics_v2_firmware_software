import sys
import csv
import json
import time
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTableWidget, QTableWidgetItem, 
                             QHeaderView, QCheckBox, QFileDialog, QMessageBox, QComboBox,
                             QStyledItemDelegate, QSpinBox, QLabel, QProgressBar,
                             QGroupBox, QGridLayout, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot, Q_ARG, QMetaObject
from controller import FluidController as FluidController
from syringe_pump import SyringePump as SyringePump
from selector_valve import SelectorValveSystem
from merfish_operations import MERFISHOperations
from _def import CMD_SET
#import pandas as pd


def load_config(config_path='config.json'):
    with open(config_path, 'r') as f:
        return json.load(f)

class SpinBoxDelegate(QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        editor = QSpinBox(parent)
        editor.setMinimum(0)
        editor.setMaximum(10000)  # Set a reasonable maximum
        editor.setSingleStep(1)
        editor.setButtonSymbols(QSpinBox.UpDownArrows)  # Show up/down arrows by default
        return editor

    def setEditorData(self, spinBox, index):
        value = index.model().data(index, Qt.EditRole)
        spinBox.setValue(int(value))

    def setModelData(self, spinBox, model, index):
        spinBox.interpretText()
        value = spinBox.value()
        model.setData(index, value, Qt.EditRole)

    def paint(self, painter, option, index):
        # This ensures the spinbox is always visible
        if not self.parent().indexWidget(index):
            spinBox = QSpinBox(self.parent(), minimum=0, maximum=10000, singleStep=1)
            spinBox.setValue(int(index.data()))
            spinBox.valueChanged.connect(lambda value: self.parent().model().setData(index, value, Qt.EditRole))
            self.parent().setIndexWidget(index, spinBox)

class PortDelegate(QStyledItemDelegate):
    def __init__(self, parent=None, ports=[]):
        super().__init__(parent)
        self.ports = ports

    def createEditor(self, parent, option, index):
        editor = QComboBox(parent)
        editor.addItems(self.ports)
        return editor

    def setEditorData(self, comboBox, index):
        value = index.model().data(index, Qt.EditRole)
        comboBox.setCurrentText(value)

    def setModelData(self, comboBox, model, index):
        value = int(comboBox.currentText())
        model.setData(index, value, Qt.EditRole)

    def paint(self, painter, option, index):
        if not self.parent().indexWidget(index):
            comboBox = QComboBox(self.parent())
            comboBox.addItems(map(str, self.ports))
            comboBox.setCurrentText(str(index.data()))
            comboBox.currentTextChanged.connect(lambda text: self.parent().model().setData(index, text, Qt.EditRole))
            self.parent().setIndexWidget(index, comboBox)

class SequencesWidget(QWidget):
    def __init__(self, config, syringe, selector_valves):
        super().__init__()
        self.config = config
        self.syringePump = syringe
        self.selectorValveSystem = selector_valves
        self.sequences = []
        self.experiment_ops = None  # Will be set based on the selected application
        self.worker = None

        if self.config['application'] == 'MERFISH':
            self.experiment_ops = MERFISHOperations(self.config, self.syringePump, self.selectorValveSystem)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Table for displaying sequences
        self.table = QTableWidget()
        self.setupTable()
        layout.addWidget(self.table)

        # Buttons
        buttonLayout = QHBoxLayout()
        self.loadButton = QPushButton("Load CSV")
        self.loadButton.clicked.connect(self.loadCSV)
        self.saveButton = QPushButton("Save CSV")
        self.saveButton.clicked.connect(self.saveCSV)
        self.selectAllButton = QPushButton("Select All")
        self.selectAllButton.clicked.connect(self.selectAll)
        self.selectNoneButton = QPushButton("Select None")
        self.selectNoneButton.clicked.connect(self.selectNone)
        self.runButton = QPushButton("Run Selected Sequences")
        self.runButton.clicked.connect(self.runSelectedSequences)
        self.abortButton = QPushButton("Abort")
        self.abortButton.clicked.connect(self.abortSequences)
        self.abortButton.setEnabled(False)  # Initially disabled

        buttonLayout.addWidget(self.loadButton)
        buttonLayout.addWidget(self.saveButton)
        buttonLayout.addWidget(self.selectAllButton)
        buttonLayout.addWidget(self.selectNoneButton)
        buttonLayout.addWidget(self.runButton)
        buttonLayout.addWidget(self.abortButton)

        layout.addLayout(buttonLayout)

        # Progress bar
        self.progressBar = QProgressBar()
        layout.addWidget(QLabel("Execution Progress:"))
        layout.addWidget(self.progressBar)

        self.setLayout(layout)

    def setupTable(self):
        if self.config['application'] == "MERFISH":
            self.table.setColumnCount(8)
            self.table.setHorizontalHeaderLabels(["Sequence Name", "Fluidic Port", "Flow Rate (μL/min)", 
                                                  "Volume (μL)", "Incubation Time (min)", "Repeat", "Fill Tubing With", "Include"])
            
            # Set up delegates
            spinBoxDelegate = SpinBoxDelegate(self.table)
            self.table.setItemDelegateForColumn(2, spinBoxDelegate)  # Flow Rate
            self.table.setItemDelegateForColumn(3, spinBoxDelegate)  # Volume
            self.table.setItemDelegateForColumn(4, spinBoxDelegate)  # Incubation Time
            self.table.setItemDelegateForColumn(5, spinBoxDelegate)  # Repeat
            self.table.setItemDelegateForColumn(6, spinBoxDelegate)  # Fill Tubing With

            # Set up port delegate with simplified port numbers
            ports = self.selectorValveSystem.get_port_names()
            self.portDelegate = PortDelegate(self.table, ports)
            self.table.setItemDelegateForColumn(1, self.portDelegate)  # Fluidic Port

        else:
            # Default setup or other applications
            pass
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def loadCSV(self):
        # Todo: use same load/save csv and runSelectedSequences for different applications
        fileName, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if fileName:
            with open(fileName, 'r') as file:
                csvReader = csv.DictReader(file)
                self.table.setRowCount(0)
                for row in csvReader:
                    rowPosition = self.table.rowCount()
                    self.table.insertRow(rowPosition)
                    self.table.setItem(rowPosition, 0, QTableWidgetItem(row['sequence_name']))
                    self.table.setItem(rowPosition, 1, QTableWidgetItem(self.portDelegate.ports[int(row['fluidic_port']) - 1]))
                    self.table.setItem(rowPosition, 2, QTableWidgetItem(row['flow_rate']))
                    self.table.setItem(rowPosition, 3, QTableWidgetItem(row['volume']))
                    self.table.setItem(rowPosition, 4, QTableWidgetItem(row['incubation_time']))
                    self.table.setItem(rowPosition, 5, QTableWidgetItem(row['repeat']))
                    self.table.setItem(rowPosition, 6, QTableWidgetItem(row['fill_tubing_with']))
                    
                    checkbox = QCheckBox()
                    checkbox.setChecked(row['include'] == '1')
                    self.table.setCellWidget(rowPosition, 7, checkbox)

                    # Make sequence name non-editable
                    self.table.item(rowPosition, 0).setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)

    def saveCSV(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if fileName:
            if not fileName.lower().endswith('.csv'):
                fileName += '.csv'
            with open(fileName, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["sequence_name", "fluidic_port", "flow_rate", "volume", "incubation_time", "repeat", "fill_tubing_with", "include"])
                for row in range(self.table.rowCount()):
                    rowData = []
                    for column in range(self.table.columnCount()):
                        if column == 7:  # Include column
                            item = self.table.cellWidget(row, column)
                            rowData.append('1' if item.isChecked() else '0')
                        else:
                            item = self.table.indexWidget(self.table.model().index(row, column))
                            if item:
                                if isinstance(item, QComboBox):
                                    rowData.append(item.currentIndex() + 1)
                                elif isinstance(item, QSpinBox):
                                    rowData.append(str(item.value()))
                            else:
                                rowData.append(self.table.item(row, column).text())
                    writer.writerow(rowData)

    def selectAll(self):
        for row in range(self.table.rowCount()):
            self.table.cellWidget(row, 7).setChecked(True)

    def selectNone(self):
        for row in range(self.table.rowCount()):
            self.table.cellWidget(row, 7).setChecked(False)

    def runSelectedSequences(self):
        # TODO: map speed codes
        selected_sequences = []
        for row in range(self.table.rowCount()):
            if self.table.cellWidget(row, 7).isChecked():
                sequence = {
                    'sequence_name': self.table.item(row, 0).text(),
                    'fluidic_port': (self.table.cellWidget(row, 1).currentIndex() + 1),
                    'flow_rate': self.table.item(row, 2).text(),
                    'volume': self.table.item(row, 3).text(),
                    'incubation_time': self.table.item(row, 4).text(),
                    'repeat': int(self.table.item(row, 5).text()),
                    'fill_tubing_with': int(self.table.item(row, 6).text()),
                }
                selected_sequences.append(sequence)

        if not selected_sequences:
            QMessageBox.warning(self, "No Sequences Selected", "Please select at least one sequence to run.")
            return

        self.runButton.setEnabled(False)
        self.abortButton.setEnabled(True)
        self.worker = ExperimentWorker(self.experiment_ops, selected_sequences)
        self.worker.progress.connect(self.updateProgress)
        self.worker.error.connect(self.showError)
        self.worker.finished.connect(self.onWorkerFinished)
        self.worker.start()

    def abortSequences(self):
        if self.worker and self.experiment_ops:
            self.experiment_ops.abort()
            self.abortButton.setEnabled(False)

    def updateProgress(self, message, percentage=None):
        if percentage is not None:
            self.progressBar.setValue(int(percentage))
        print(message)

    def showError(self, error_message):
        pass

    def onWorkerFinished(self):
        self.runButton.setEnabled(True)
        self.abortButton.setEnabled(False)
        self.progressBar.setValue(0)
        if self.worker:
            self.worker.quit()
            self.worker.wait()
            self.worker = None
        if self.experiment_ops:
            self.experiment_ops.abort_requested = False
        #QMessageBox.information(self, "Complete", "All selected sequences have been executed.")

class ExperimentWorker(QThread):
    progress = pyqtSignal(str, int)
    error = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self, experiment_ops, sequences):
        super().__init__()
        self.experiment_ops = experiment_ops
        self.sequences = sequences

    def run(self):
        self.experiment_ops.set_callbacks(self.progress.emit, self.error.emit)
        total_sequences = sum(seq['repeat'] for seq in self.sequences)
        completed_sequences = 0

        try:
            for sequence in self.sequences:
                for _ in range(sequence['repeat']):
                    try:
                        self.experiment_ops.run_sequence(sequence)
                        completed_sequences += 1
                        progress_percentage = (completed_sequences / total_sequences) * 100
                        self.progress.emit(f"Completed {completed_sequences}/{total_sequences} sequences", progress_percentage)
                    except AbortRequested:
                        self.error.emit("Operation aborted by user")
                        return
        except Exception as e:
            self.error.emit(str(e))
        finally:
            self.finished.emit()

class ManualControlWidget(QWidget):
    def __init__(self, config, syringe, selector_valves):
        super().__init__()
        self.config = config
        self.syringePump = syringe
        self.selectorValveSystem = selector_valves

        # Initialize timers
        self.progress_timer = QTimer(self)
        self.progress_timer.timeout.connect(self.updateProgress)
        
        self.plunger_timer = QTimer(self)
        self.plunger_timer.timeout.connect(self.updatePlungerPosition)
        
        self.operation_start_time = None
        self.operation_duration = None

        self.initUI()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(10)

        # Selector Valve Control
        valveGroupBox = QGroupBox("Selector Valve Control")
        valveLayout = QHBoxLayout()
        valveLayout.setContentsMargins(5, 5, 5, 5)
        valveLayout.addWidget(QLabel("Source port:"))
        self.valveCombo = QComboBox()
        self.valveCombo.addItems(self.selectorValveSystem.get_port_names())
        self.valveCombo.currentIndexChanged.connect(self.openValve)
        valveLayout.addWidget(self.valveCombo)
        valveGroupBox.setLayout(valveLayout)
        mainLayout.addWidget(valveGroupBox)

        # Syringe Pump Control
        syringeGroupBox = QGroupBox("Syringe Pump Control")
        syringeLayout = QVBoxLayout()
        syringeLayout.setContentsMargins(5, 5, 5, 5)
        syringeLayout.setSpacing(5)

        topLayout = QHBoxLayout()

        # Left side controls
        leftWidget = QWidget()
        leftLayout = QGridLayout(leftWidget)
        self.syringePortCombo = QComboBox()
        self.syringePortCombo.addItems(map(str, self.config['syringe_pump']['ports_allowed']))
        leftLayout.addWidget(QLabel("Port:"), 0, 0)
        leftLayout.addWidget(self.syringePortCombo, 0, 1)

        self.speedCombo = QComboBox()
        speed_code_limit = self.config['syringe_pump']['speed_code_limit']
        for code in range(speed_code_limit, len(self.syringePump.SPEED_SEC_MAPPING)):
            rate = self.syringePump.get_flow_rate(code)
            self.speedCombo.addItem(f"{rate} mL/min", code)
        self.speedCombo.setCurrentIndex(40 - self.config['syringe_pump']['speed_code_limit'])  # Set default to code 40
        leftLayout.addWidget(QLabel("Speed:"), 1, 0)        # TODO: default speed, max speed
        leftLayout.addWidget(self.speedCombo, 1, 1)

        self.volumeSpinBox = QSpinBox()
        self.volumeSpinBox.setRange(1, self.config['syringe_pump']['volume_ul'])
        self.volumeSpinBox.setSuffix(" μL")
        leftLayout.addWidget(QLabel("Volume:"), 2, 0)
        leftLayout.addWidget(self.volumeSpinBox, 2, 1)

        actionLayout = QHBoxLayout()
        self.pushButton = QPushButton("Extract")
        self.pushButton.clicked.connect(self.pullSyringe)
        self.pullButton = QPushButton("Dispense")
        self.pullButton.clicked.connect(self.pushSyringe)
        actionLayout.addWidget(self.pushButton)
        actionLayout.addWidget(self.pullButton)
        leftLayout.addLayout(actionLayout, 3, 0, 1, 2)

        topLayout.addWidget(leftWidget, 3)

        # Right side - Plunger position
        # TODO: stop updating position when not on this tab
        rightWidget = QWidget()
        rightLayout = QVBoxLayout(rightWidget)
        self.plungerPositionLabel = QLabel("Plunger Position (μL)")
        rightLayout.addWidget(self.plungerPositionLabel, alignment=Qt.AlignHCenter)
        self.plungerPositionBar = QProgressBar()
        self.plungerPositionBar.setRange(0, self.config['syringe_pump']['volume_ul'])  # XCaliburD has 3000 steps in standard mode
        self.plungerPositionBar.setOrientation(Qt.Vertical)
        self.plungerPositionBar.setTextVisible(False)
        rightLayout.addWidget(self.plungerPositionBar, alignment=Qt.AlignHCenter)

        topLayout.addWidget(rightWidget, 1)

        syringeLayout.addLayout(topLayout)
        
        # TODO: progress bar
        self.syringeProgressBar = QProgressBar()
        self.syringeProgressBar.setRange(0, 100)
        syringeLayout.addWidget(QLabel("Execution Progress:"))
        syringeLayout.addWidget(self.syringeProgressBar)

        syringeGroupBox.setLayout(syringeLayout)
        mainLayout.addWidget(syringeGroupBox)

        self.setLayout(mainLayout)

        # Initialize plunger position
        self.updatePlungerPosition()

    def openValve(self):
        port = self.valveCombo.currentIndex() + 1
        self.selectorValveSystem.open_port(port)

    def pushSyringe(self):
        self._operateSyringe("dispense")

    def pullSyringe(self):
        self._operateSyringe("extract")

    def _operateSyringe(self, action):
        if self.syringePump.is_busy:
            print("Syringe pump is busy.")
            return

        syringe_port = int(self.syringePortCombo.currentText())
        speed_code = self.speedCombo.currentData()
        volume = self.volumeSpinBox.value()
        
        try:
            # Disable control buttons during operation
            self.setControlsEnabled(False)

            # Start operation
            self.syringePump.reset_chain()
            if action == "dispense":
                exec_time = self.syringePump.dispense(syringe_port, volume, speed_code)
            else:
                exec_time = self.syringePump.extract(syringe_port, volume, speed_code)

            # Set up progress tracking
            self.operation_duration = exec_time

            # Start syringe operation in a separate thread
            operation_thread = threading.Thread(target=self._executeSyringeOperation, 
                                             args=(action, syringe_port, volume, speed_code))
            operation_thread.daemon = True
            operation_thread.start()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error operating syringe pump: {str(e)}")
            self.setControlsEnabled(True)

    def _executeSyringeOperation(self, action, syringe_port, volume, speed_code):
        try:
            self.operation_start_time = time.time()

            # Start progress updates
            QMetaObject.invokeMethod(self, "startProgressTimer", Qt.QueuedConnection)

            self.syringePump.execute()

            # Clean up
            QMetaObject.invokeMethod(self, "operationComplete", Qt.QueuedConnection)
            
        except Exception as e:
            QMetaObject.invokeMethod(self, "handleError", 
                                   Qt.QueuedConnection,
                                   Q_ARG(str, str(e)))

    @pyqtSlot()
    def startProgressTimer(self):
        self.syringeProgressBar.setValue(0)
        self.progress_timer.start(100)  # Update progress every 100ms

    @pyqtSlot()
    def operationComplete(self):
        self.progress_timer.stop()
        self.syringeProgressBar.setValue(100)
        self.setControlsEnabled(True)
        self.operation_start_time = None
        self.operation_duration = None

    @pyqtSlot(str)
    def handleError(self, error_message):
        QMessageBox.critical(self, "Error", f"Syringe pump error: {error_message}")
        self.syringePump.wait_for_stop()
        self.setControlsEnabled(True)
        self.progress_timer.stop()
        self.syringeProgressBar.setValue(0)

    def setControlsEnabled(self, enabled):
        self.pushButton.setEnabled(enabled)
        self.pullButton.setEnabled(enabled)
        self.syringePortCombo.setEnabled(enabled)
        self.speedCombo.setEnabled(enabled)
        self.volumeSpinBox.setEnabled(enabled)
        #self.valveCombo.setEnabled(enabled)

    def updateProgress(self):
        if self.operation_start_time is None or self.operation_duration is None:
            return
            
        elapsed = time.time() - self.operation_start_time
        progress = min(100, int((elapsed / self.operation_duration) * 100))
        self.syringeProgressBar.setValue(progress)

    def updatePlungerPosition(self):
        try:
            position = self.syringePump.get_plunger_position() * self.config['syringe_pump']['volume_ul']
            self.plungerPositionBar.setValue(int(position))
        except Exception as e:
            print(f"Error updating plunger position: {str(e)}")

    def showEvent(self, event):
        # Start timer when widget becomes visible
        super().showEvent(event)
        self.plunger_timer.start(500)

    def hideEvent(self, event):
        # Stop timer when widget becomes hidden
        super().hideEvent(event)
        self.plunger_timer.stop()

    def closeEvent(self, event):
        self.progress_timer.stop()
        self.position_timer.stop()
        super().closeEvent(event)

class FluidicsControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = load_config()
        self.controller = FluidController(self.config['microcontroller']['serial_number'])
        self.controller.begin()
        self.controller.send_command(CMD_SET.CLEAR)

        self.syringePump = SyringePump(
                            sn=self.config['syringe_pump']['serial_number'],
                            syringe_ul=self.config['syringe_pump']['volume_ul'], 
                            speed_code_limit=self.config['syringe_pump']['speed_code_limit'],
                            waste_port=3)
        self.selectorValveSystem = SelectorValveSystem(self.controller, self.config)

        self.initUI()

    def initUI(self):
        self.setWindowTitle("Fluidics Control System")
        self.setGeometry(100, 100, 800, 600)

        # Create tab widget
        tabWidget = QTabWidget()
        
        # "Run Experiments" tab
        runExperimentsTab = SequencesWidget(self.config, self.syringePump, self.selectorValveSystem)
        tabWidget.addTab(runExperimentsTab, "Run Experiments")

        # "Settings and Manual Control" tab
        manualControlTab = ManualControlWidget(self.config, self.syringePump, self.selectorValveSystem)
        tabWidget.addTab(manualControlTab, "Settings and Manual Control")

        self.setCentralWidget(tabWidget)

    def closeEvent(self, event):
        self.syringePump.close()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = FluidicsControlGUI()
    gui.show()
    sys.exit(app.exec_())
