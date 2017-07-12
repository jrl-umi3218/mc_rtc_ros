from PySide import QtCore, QtGui

import rospy
from sensor_msgs.msg import JointState

import copy

from eigen import Vector3d
import rbdyn as rbd
import tasks

from mc_solver import KinematicsConstraint

from .contact_marker import ContactControlMarker
from task_dialog import Ui_TaskDialog


class TaskDialog(QtGui.QDialog):
  def __init__(self, robots, qpsolver, robotPublisher, jointStatePub, timeStep, parent=None):
    super(TaskDialog, self).__init__(parent)
    self.ui = Ui_TaskDialog()
    self.ui.setupUi(self)

    self.robots = robots
    self.robot = robots.robot()
    self.env = robots.robots()[1]
    self.qpsolver = qpsolver
    self.jointStatePub = jointStatePub
    self.robotPublisher = robotPublisher
    self.timeStep = timeStep
    self.tasks = {}
    self.setterByItemPos = {}
    self.contact = None
    self.newContact = None
    self.bodyIndexToMove = None

    # create qp constraint
    self.kinematicsConstraint = KinematicsConstraint(robots, 0, qpsolver.timeStep)

    # setup control marker
    self.im = ContactControlMarker('contact', self.targetPosition)

    # Ui setting
    self.ui.taskTable.itemChanged.connect(self.taskChanged)

    # Timer to run the qp
    self.sendTimer = QtCore.QTimer(self)
    self.sendTimer.timeout.connect(self.runQP)

  def init(self, stance, stanceIndex, contactIndex):
    self.stance = stance

    # compute contact body value
    surf = stance.contacts()[contactIndex].r1Surface()
    bodyIndex = self.robot.bodyIndexByName(surf.bodyName)

    self.bodyIndexToMove = bodyIndex
    self.contact = stance.contacts()[contactIndex]
    self.newContact = None

    # reset mbc velocity, acceleration and torque and set initial q value
    self.robot.mbc.zero(self.robot.mb)
    self.robot.mbc.q = stance.q

    rbd.forwardKinematics(self.robot.mb, self.robot.mbc)
    rbd.forwardVelocity(self.robot.mb, self.robot.mbc)
    bodyTf = self.robot.mbc.bodyPosW[bodyIndex]

    def spTask(task, stiffness, weight):
      return tasks.qp.SetPointTask(self.robots.mbs(), 0, task, stiffness, weight)

    # create qp tasks
    self.postureTask = tasks.qp.PostureTask(self.robots.mbs(), 0, stance.q, 1., 5.)
    self.contactTask = tasks.qp.PositionTask(self.robots.mbs(), 0, surf.bodyName, bodyTf.translation())
    self.contactTaskSp = spTask(self.contactTask, 10., 10.)
    self.orientationTask = tasks.qp.OrientationTask(self.robots.mbs(), 0, surf.bodyName, bodyTf.rotation())
    self.orientationTaskSp = spTask(self.orientationTask, 1., 1000.)
    self.comTask = tasks.qp.CoMTask(self.robots.mbs(), 0, rbd.computeCoM(self.robot.mb, self.robot.mbc))
    self.comTaskSp = spTask(self.comTask, 1., 100.)
    self.tasks = {'Posture':self.postureTask, 'Position':self.contactTaskSp,
                  'Orientation':self.orientationTaskSp, 'CoM':self.comTaskSp}

    # add tasks to the qp
    for t in self.tasks.itervalues():
      self.qpsolver.addTask(t)

    # add constraint to the qp
    self.qpsolver.addConstraintSet(self.kinematicsConstraint)

    contacts = []
    for i in range(len(stance.contacts())):
      if i != contactIndex:
        contacts.append(stance.contacts()[i])
    self.qpsolver.setContacts(contacts)

    self.updateTaskTable()

    self.im.init(bodyTf.translation())

    self.sendTimer.start(self.qpsolver.timeStep)


  def updateTaskTable(self):
    self.ui.taskTable.clear()
    self.ui.taskTable.setRowCount(len(self.tasks))
    self.ui.taskTable.setColumnCount(3)
    self.ui.taskTable.setHorizontalHeaderLabels(['Task', 'Stiffness', 'Weight'])
    self.setterByItemPos = {}
    for i, (tn,t) in enumerate(self.tasks.iteritems()):
      taskItem = QtGui.QTableWidgetItem(tn)
      taskItem.setFlags(taskItem.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
      stiffItem = QtGui.QTableWidgetItem()
      stiffItem.setData(QtCore.Qt.ItemDataRole.DisplayRole, t.stiffness())
      weightItem = QtGui.QTableWidgetItem()
      weightItem.setData(QtCore.Qt.ItemDataRole.DisplayRole, t.weight())
      self.ui.taskTable.setItem(i, 0, taskItem)
      self.ui.taskTable.setItem(i, 1, stiffItem)
      self.ui.taskTable.setItem(i, 2, weightItem)

      self.setterByItemPos[(i, 1)] = t.stiffness
      self.setterByItemPos[(i, 2)] = t.weight


  def taskChanged(self, item):
    data = item.data(QtCore.Qt.ItemDataRole.DisplayRole)
    if data < 0.:
      item.setData(QtCore.Qt.ItemDataRole.DisplayRole, 0.)
    else:
      key = (item.row(), item.column())
      if self.setterByItemPos.has_key(key):
        self.setterByItemPos[key](data)


  def runQP(self):
    # run solver
    # if the qp fail we restart the initial configuration
    if not self.qpsolver.run():
      self.robot.mbc.zero(self.robot.mb)
      self.robot.mbc.q = self.stance.q

    self.sendAll()


  def targetPosition(self, feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
    self.contactTask.position(Vector3d(p.x, p.y, p.z))


  def exec_(self, stance, stanceIndex, contactIndex):
    self.init(stance, stanceIndex, contactIndex)
    return super(TaskDialog, self).exec_()


  def done(self, result):
    # stop the time and remove tasks and constraints
    self.sendTimer.stop()

    if result == QtGui.QDialog.DialogCode.Accepted:
      self.stance.q = self.robot.mbc.q
      self.newContact = copy.copy(self.contact)
      self.newContact.X_r2s_r1s(self.newContact.compute_X_r2s_r1s(self.robots))
      self.stance.updateContact(self.contact, self.newContact)

    for t in self.tasks.itervalues():
      self.qpsolver.removeTask(t)
    self.tasks = {}
    self.qpsolver.removeConstraintSet(self.kinematicsConstraint)

    self.im.clear()

    super(TaskDialog, self).done(result)


  def sendAll(self):
    curTime = rospy.Time.now()
    js = JointState()
    js.header.stamp = curTime
    self.robotPublisher.update(self.timeStep, self.robot)
    self.jointStatePub.publish(js)


  def applyOnStance(self, stance):
    """
    Apply the transform specify by the user to a stance.
    Param:
      stance: Stance to modify. q and contact position will be modify.
    """
    tasks = [self.postureTask, self.contactTaskSp, self.orientationTaskSp, self.comTaskSp]

    # we setup back all the task
    # this allow to use the user defined stiffness and weight
    for t in tasks:
      self.qpsolver.addTask(t)
    self.qpsolver.addConstraintSet(self.kinematicsConstraint)

    # reset robot velocity, torque and acceleration
    self.robot.mbc.zero(self.robot.mb)
    self.robot.mbc.q = stance.q

    rbd.forwardKinematics(self.robot.mb, self.robot.mbc)
    rbd.forwardVelocity(self.robot.mb, self.robot.mbc)

    # set the posture and com to keep
    self.postureTask.posture(stance.q)
    self.comTask.com(rbd.computeCoM(self.robot.mb, self.robot.mbc))

    # remove the contact set in init since it's must move
    new_contacts = [c for c in stance.contacts() if c != self.contact]
    self.qpsolver.setContacts(new_contacts)

    run = True
    i = 0
    maxIter = 10000
    stopSpeed = 1e-5
    while run:
      if self.qpsolver.run():
        run = list(self.robot.mbc.bodyVelW)[self.bodyIndexToMove].vector().norm() > stopSpeed and i < maxIter
      else:
        run = False
        self.robot.mbc.q = stance.q
      self.sendAll()
      i += 1
      # we loop a lot, so let Qt work a little avoid gui to freeze
      QtGui.qApp.processEvents()

    # set the new stance value
    stance.q = self.robot.mbc.q
    stance.updateContact(self.contact, self.newContact)

    self.qpsolver.removeConstraintSet(self.kinematicsConstraint)
    for t in tasks:
      self.qpsolver.removeTask(t)

