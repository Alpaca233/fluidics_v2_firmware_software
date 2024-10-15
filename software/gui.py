import sys
import csv
import json
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTableWidget, QTableWidgetItem, 
                             QHeaderView, QCheckBox, QFileDialog, QMessageBox, QComboBox,
                             QStyledItemDelegate, QSpinBox, QLabel, QProgressBar,
                             QGroupBox, QGridLayout, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
from controller import FluidController
from syringe_pump import SyringePumpSimulation as SyringePump
import utils
import time

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
        editor.addItems(map(str, self.ports))
        return editor

    def setEditorData(self, comboBox, index):
        value = index.model().data(index, Qt.EditRole)
        comboBox.setCurrentText(str(value))

    def setModelData(self, comboBox, model, index):
        value = int(comboBox.currentText())
        model.setData(index, value, Qt.EditRole)

    def paint(self, painter, option, index):
        if not self.parent().indexWidget(index):
            comboBox = QComboBox(self.parent())
            comboBox.addItems(map(str, self.ports))
            comboBox.setCurrentText(str(index.data()))
            comboBox.currentTextChanged.connect(lambda text: self.parent().model().setData(index, int(text), Qt.EditRole))
            self.parent().setIndexWidget(index, comboBox)

class SequencesWidget(QWidget):
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.simplified_to_actual, self.actual_to_simplified = utils.create_port_mapping(config)
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
        self.runButton.clicked.connect(self.runSequences)
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

        self.setLayout(layout)

    def setupTable(self):
        if self.config['application'] == "MERFISH":
            self.table.setColumnCount(7)
            self.table.setHorizontalHeaderLabels(["Sequence Name", "Fluidic Port", "Flow Rate (μL/min)", 
                                                  "Volume (μL)", "Incubation Time (min)", "Repeat", "Include"])
            
            # Set up delegates
            spinBoxDelegate = SpinBoxDelegate(self.table)
            self.table.setItemDelegateForColumn(2, spinBoxDelegate)  # Flow Rate
            self.table.setItemDelegateForColumn(3, spinBoxDelegate)  # Volume
            self.table.setItemDelegateForColumn(4, spinBoxDelegate)  # Incubation Time
            self.table.setItemDelegateForColumn(5, spinBoxDelegate)  # Repeat

            # Set up port delegate with simplified port numbers
            simplified_ports = utils.get_simplified_ports(self.config)
            portDelegate = PortDelegate(self.table, simplified_ports)
            self.table.setItemDelegateForColumn(1, portDelegate)  # Fluidic Port

        else:
            # Default setup or other applications
            pass
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def loadCSV(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if fileName:
            with open(fileName, 'r') as file:
                csvReader = csv.DictReader(file)
                self.table.setRowCount(0)
                for row in csvReader:
                    rowPosition = self.table.rowCount()
                    self.table.insertRow(rowPosition)
                    self.table.setItem(rowPosition, 0, QTableWidgetItem(row['sequence_name']))
                    self.table.setItem(rowPosition, 1, QTableWidgetItem(row['fluidic_port']))
                    self.table.setItem(rowPosition, 2, QTableWidgetItem(row['flow_rate']))
                    self.table.setItem(rowPosition, 3, QTableWidgetItem(row['volume']))
                    self.table.setItem(rowPosition, 4, QTableWidgetItem(row['incubation_time']))
                    self.table.setItem(rowPosition, 5, QTableWidgetItem(row['repeat']))
                    
                    checkbox = QCheckBox()
                    checkbox.setChecked(row['include'] == '1')
                    self.table.setCellWidget(rowPosition, 6, checkbox)

                    # Make sequence name non-editable
                    self.table.item(rowPosition, 0).setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)

    def saveCSV(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if fileName:
            with open(fileName, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["sequence_name", "fluidic_port", "flow_rate", "volume", "incubation_time", "repeat", "include"])
                for row in range(self.table.rowCount()):
                    rowData = []
                    for column in range(self.table.columnCount()):
                        if column == 6:  # Include column
                            item = self.table.cellWidget(row, column)
                            rowData.append('1' if item.isChecked() else '0')
                        else:
                            item = self.table.indexWidget(self.table.model().index(row, column))
                            if item:
                                if isinstance(item, QComboBox):
                                    rowData.append(item.currentText())
                                elif isinstance(item, QSpinBox):
                                    rowData.append(str(item.value()))
                            else:
                                rowData.append(self.table.item(row, column).text())
                    writer.writerow(rowData)

    def selectAll(self):
        for row in range(self.table.rowCount()):
            self.table.cellWidget(row, 6).setChecked(True)

    def selectNone(self):
        for row in range(self.table.rowCount()):
            self.table.cellWidget(row, 6).setChecked(False)

    def runSequences(self):
        # Placeholder for running sequences
        QMessageBox.information(self, "Run Sequences", "Running selected sequences...")
        self.abortButton.setEnabled(True)
        self.runButton.setEnabled(False)

    def abortSequences(self):
        # Placeholder for aborting sequences
        QMessageBox.information(self, "Abort Sequences", "Aborting sequences...")
        self.abortButton.setEnabled(False)
        self.runButton.setEnabled(True)

class ManualControlWidget(QWidget):
    def __init__(self, config, controller, syringe):
        super().__init__()
        self.config = config
        self.controller = controller
        self.syringePump = syringe
        self.simplified_to_actual, self.actual_to_simplified = utils.create_port_mapping(config)
        self.initUI()

    def initUI(self):
        mainLayout = QVBoxLayout()
        mainLayout.setSpacing(10)

        # Selector Valve Control
        valveGroupBox = QGroupBox("Selector Valve Control")
        valveLayout = QHBoxLayout()
        valveLayout.setContentsMargins(5, 5, 5, 5)
        valveLayout.addWidget(QLabel("Open Valve:"))
        self.valveCombo = QComboBox()
        self.valveCombo.addItems(map(str, utils.get_simplified_ports(self.config)))
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
        for code in self.syringePump.SPEED_SEC.keys():
            rate = self.syringePump.get_flow_rate(code)
            self.speedCombo.addItem(f"{rate} μL/min", code)
        self.speedCombo.setCurrentIndex(40)  # Set default to code 40
        leftLayout.addWidget(QLabel("Speed:"), 1, 0)
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
        simplified_port = int(self.valveCombo.currentText())
        utils.open_selector_valve_path(self.controller, self.config, simplified_port, self.simplified_to_actual)
        QMessageBox.information(self, "Valve Opened", f"Opened valve path to simplified port {simplified_port}")

    def pushSyringe(self):
        self._operateSyringe("dispense")

    def pullSyringe(self):
        self._operateSyringe("extract")

    def _operateSyringe(self, action):
        # TODO: check if the syringe pump is ready

        syringe_port = int(self.syringePortCombo.currentText())
        speed_code = self.speedCombo.currentData()
        volume = self.volumeSpinBox.value()
        
        try:
            # Start syringe operation
            if action == "dispense":
                exec_time = self.syringePump.dispense(syringe_port, volume, speed_code)
            else:  # extract
                exec_time = self.syringePump.extract(syringe_port, volume, speed_code)
        except:
            print("Error operating syringe pump")
        # # Simulate progress
        # self.syringeProgressBar.setRange(0, volume)
        # self.syringeProgressBar.setValue(0)
        
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.updateProgress)
        # self.timer.start(100)  # Update every 100 ms

    def updateProgress(self):
        current = self.syringeProgressBar.value()
        if current < self.syringeProgressBar.maximum():
            self.syringeProgressBar.setValue(current + 1)
        else:
            self.timer.stop()

    def updatePlungerPosition(self):
        try:
            position = self.syringePump.get_plunger_position() * self.config['syringe_pump']['volume_ul']
            self.plungerPositionBar.setValue(position)
            #self.plungerPositionLabel.setText(f"Plunger Position: {position}")
        except:
            print(f"Error updating plunger position")

class FluidicsControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = utils.load_config()
        self.controller = FluidController(self.config['microcontroller']['serial_number'])

        self.syringePump = SyringePump(
                            syringe_ul=self.config['syringe_pump']['volume_ul'], 
                            waste_port=3)

        self.initUI()

    def initUI(self):
        self.setWindowTitle("Fluidics Control System")
        self.setGeometry(100, 100, 800, 600)

        # Create tab widget
        tabWidget = QTabWidget()
        
        # "Run Experiments" tab
        runExperimentsTab = SequencesWidget(self.config)
        tabWidget.addTab(runExperimentsTab, "Run Experiments")

        # "Settings and Manual Control" tab
        manualControlTab = ManualControlWidget(self.config, self.controller, self.syringePump)
        tabWidget.addTab(manualControlTab, "Settings and Manual Control")

        self.setCentralWidget(tabWidget)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = FluidicsControlGUI()
    gui.show()
    sys.exit(app.exec_())
