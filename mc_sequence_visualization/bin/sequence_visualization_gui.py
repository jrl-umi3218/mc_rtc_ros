#! /usr/bin/env python
from sensor_msgs.msg import JointState
import rospy
import rosbag

import sys
import signal
from math import degrees
import copy

from PySide import QtCore, QtGui

from eigen import Vector3d
import rbdyn as rbd
import tasks

import mc_rbdyn
from mc_solver import QPSolver, DynamicsConstraint, ContactConstraint

from mc_sequence_visualization.sequence_mainwindow import Ui_Sequence
from mc_sequence_visualization.task_gui import TaskDialog

from mc_rtc import RobotPublisher

class UndoContactEdit(QtGui.QUndoCommand):
  def __init__(self, mainwindow, oldStances, newStances, parent=None):
    super(UndoContactEdit, self).__init__(parent)
    self.mw = mainwindow
    self.oldStances = oldStances
    self.newStances = newStances


  def undo(self):
    self.mw.changeStances(self.oldStances)


  def redo(self):
    self.mw.changeStances(self.newStances)



class Slider(QtGui.QMainWindow):
  def __init__(self, robots, stances, frequency, timeStep, parent=None):
    super(Slider, self).__init__(parent)
    self.ui = Ui_Sequence()
    self.ui.setupUi(self)
    self.robots = robots
    self.robot = robots.robot()
    self.stances = stances
    self.curStance = None

    self.robot.mbc.gravity = Vector3d(0., 0., 9.81)

    self.timeStep = timeStep

    # Create solver, constraints and tasks
    self.dynamicsConstraint = DynamicsConstraint(robots, 0, timeStep, True)
    self.contactConstraint = ContactConstraint(timeStep)
    self.qpsolver = QPSolver(robots, timeStep)
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.addConstraintSet(self.contactConstraint)

    self.postureTask = tasks.qp.PostureTask(robots.mbs(), 0, self.robot.mbc.q, 1., 1.)
    self.qpsolver.addTask(self.postureTask)

    self.robotPublisher = RobotPublisher("", frequency)

    # Timer to publish TF and Joint State
    self.sendTimer = QtCore.QTimer(self)
    self.sendTimer.timeout.connect(self.sendAll)
    self.sendTimer.start(1000./frequency)

    # empty joint states publisher needed because qp_state_publisher need this message
    # to publish
    self.jointStatePub = rospy.Publisher('sequence_joint_states', JointState)

    # Gui part
    self.ui.stanceSlider.setRange(0, len(stances) - 1)
    self.ui.stanceSpinBox.setRange(0, len(stances) - 1)
    self.ui.stanceSlider.valueChanged.connect(self.setStance)
    self.ui.stanceSpinBox.valueChanged.connect(self.setStance)
    self.ui.contactTable.cellDoubleClicked.connect(self.editStance)

    self.ui.stanceButton.setText("Edit")
    self.ui.stanceButton.clicked.connect(self.editPosture)

    self.undoStack = QtGui.QUndoStack()
    self.undoStack.canUndoChanged.connect(self.ui.undo.setEnabled)
    self.undoStack.canRedoChanged.connect(self.ui.redo.setEnabled)

    self.taskDialog = TaskDialog(robots, self.qpsolver,
                                 self.robotPublisher,
                                 self.jointStatePub, self.timeStep, self)

    self.linkDialog = LinkDialog(robots, self.qpsolver,
                                 self.jointStatePub, self)

    self.adjustSize()
    self.readSettings()

  def writeSettings(self):
    s = QtCore.QSettings('IDH', 'Sequence')
    s.beginGroup('Slider')
    s.setValue('size', self.size())
    s.setValue('pos', self.pos())
    s.endGroup()


  def readSettings(self):
    s = QtCore.QSettings('IDH', 'Sequence')
    s.beginGroup('Slider')
    self.resize(s.value('size', self.size()))
    self.move(s.value('pos', self.pos()))
    s.endGroup()


  def closeEvent(self, e):
    self.writeSettings()


  def updateUi(self, stance, index):
    self.ui.stanceSlider.setValue(index)
    self.ui.stanceSpinBox.setValue(index)
    self.ui.contactTable.clear()
    self.ui.contactTable.setRowCount(len(stance.contacts()))
    self.ui.contactTable.setColumnCount(4)
    self.ui.contactTable.setHorizontalHeaderLabels(['Robot', 'Env',
                                                    'Pos', 'State'])
    for i, c in enumerate(stance.contacts()):
      robSurfItem = QtGui.QTableWidgetItem(c.r1Surface().name)
      envSurfItem = QtGui.QTableWidgetItem(c.r2Surface().name)

      posItem = QtGui.QTableWidgetItem()
      pos = c.X_0_r1s(self.robots).translation()
      posItem.setData(QtCore.Qt.ItemDataRole.DisplayRole, ' '.join(map(str, pos)))
      stateItem = QtGui.QTableWidgetItem('Geometric')
      self.ui.contactTable.setItem(i, 0, robSurfItem)
      self.ui.contactTable.setItem(i, 1, envSurfItem)
      self.ui.contactTable.setItem(i, 2, posItem)
      self.ui.contactTable.setItem(i, 3, stateItem)


  def setStance(self, i):
    if self.curStance != i:
      self.curStance = i

      self.updateUi(self.stances[self.curStance], i)
      self.computeStance()


  def computeStance(self):
    s = self.stances[self.curStance]

    # zero the velocity, acceleration and torque fill by the qp
    self.robot.mbc.zero(self.robot.mb)
    self.robot.mbc.q = s.q

    rbd.forwardKinematics(self.robot.mb, self.robot.mbc)
    rbd.forwardVelocity(self.robot.mb, self.robot.mbc)

    self.postureTask.posture(s.q)

    self.qpsolver.setContacts(s.contacts())
    self.qpsolver.run()

    self.sendAll()

  def editPosture(self):
    print "Editing stance"
    self.sendTimer.stop()
    self.qpsolver.removeConstraintSet(self.dynamicsConstraint)
    self.qpsolver.solver.removeTask(self.postureTask)

    stancesCopy = copy.deepcopy(self.stances)
    clickedStance = stancesCopy[self.curStance]

    if self.linkDialog.exec_(clickedStance, self.curStance):
      self.updateUi(clickedStance, self.curStance)
      undoContactEdit = UndoContactEdit(self, self.stances, stancesCopy)
      self.undoStack.push(undoContactEdit)

    # restart the time and add constraint and task back
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.solver.addTask(self.postureTask)
    self.computeStance()
    self.sendTimer.start(1000./frequency)

  def editStance(self, row, col):
    # stop the time and clear the qpsolver since taskDialog will use it
    self.sendTimer.stop()

    self.qpsolver.removeConstraintSet(self.dynamicsConstraint)
    self.qpsolver.removeTask(self.postureTask)


    # task dialog will modify the stance so we give him a copy

    stancesCopy = copy.copy(self.stances)
    clickedStance = stancesCopy[self.curStance]

    if self.taskDialog.exec_(clickedStance, self.curStance, row):
      contact = self.taskDialog.contact
      stances = list(self.stanceWithContact(stancesCopy, self.curStance, contact))
      progDial = QtGui.QProgressDialog('Update stances', 'Abort', 0, len(stances), self)
      progDial.setCancelButton(None)
      progDial.setWindowModality(QtCore.Qt.WindowModal)
      progDial.setValue(0)
      for i, s in enumerate(stances):
        progDial.setValue(i)
        self.taskDialog.applyOnStance(s)

      progDial.setValue(len(stances))
      self.updateUi(clickedStance, self.curStance)

      # stances copy containt the modified stances
      undoContactEdit = UndoContactEdit(self, self.stances, stancesCopy)
      self.undoStack.push(undoContactEdit)


    # restart the time and add constraint and task back
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.addTask(self.postureTask)
    self.computeStance()
    self.sendTimer.start(1000./frequency)


  def stanceWithContact(self, stances, stanceIndex, contact):
    # forward
    for i in range(stanceIndex+1, len(stances)):
      if contact in stances[i].contacts():
        yield stances[i]
      else:
        break
    # backward
    for i in range(stanceIndex):
      if contact in stances[i].contacts():
        yield stances[i]
      else:
        break


  def changeStances(self, stances):
    """Change the stances list, update the ui and run the qp."""
    self.stances = stances
    self.updateUi(stances[self.curStance], self.curStance)
    self.computeStance()


  @QtCore.Slot()
  def on_undo_triggered(self):
    self.undoStack.undo()


  @QtCore.Slot()
  def on_redo_triggered(self):
    self.undoStack.redo()

  @QtCore.Slot()
  def on_jointState_triggered(self):
    filename, ext = QtGui.QFileDialog.getSaveFileName(self, "Bag file", "joint_states.bag", "Bag (*.bag)")
    if len(filename) > 0:
      bag = rosbag.Bag(filename, 'w')
      js = JointState()
      js.name = [j.name() for j in self.robot.mb.joints() if j.dof() == 1]
      for i, s in enumerate(self.stances):
        js.header.seq = i
        js.header.stamp = rospy.Time(i*10)
        q = rbd.paramToVector(self.robot.mb, s.q)
        js.position = q[self.robot.mb.joint(0).params():].flatten().tolist()[0]
        bag.write('/robot/joint_states', js, js.header.stamp)
      bag.close()



  def sendAll(self):
    curTime = rospy.Time.now()
    js = JointState()
    js.header.stamp = curTime
    self.robotPublisher.update(self.timeStep, self.robot)
    self.jointStatePub.publish(js)


if __name__ == '__main__':

  stances_file = rospy.get_param('/stances_file')

  polyhedrons = rospy.get_param('/polyhedrons_file', None)

  frequency = rospy.get_param('/publish_frequency', 25)

  timeStep = 0.005

  robot_module = mc_rbdyn.get_robot_module(*rospy.get_param('robot_module'))
  env_module = mc_rbdyn.get_robot_module(*rospy.get_param('env/robot_module'))

  # load the robot and the environment
  robots = mc_rbdyn.loadRobotAndEnv(robot_module, robot_module.rsdf_dir,
                                    env_module, env_module.rsdf_dir)
  # load stances
  stances, actions = mc_rbdyn.loadStances(robots, stances_file)

  rospy.init_node('sequence_visualization_gui')

  app = QtGui.QApplication(sys.argv)
  slider = Slider(robots, stances, frequency, timeStep)
  slider.show()


  # close the windows on ctrl+c
  def sigint_handler(*args):
    slider.close()
  signal.signal(signal.SIGINT, sigint_handler)


  # we must create a dummy timer to be able to receive SIGINT
  timer = QtCore.QTimer()
  timer.start(500)
  timer.timeout.connect(lambda: None)

  slider.setStance(0)

  sys.exit(app.exec_())

