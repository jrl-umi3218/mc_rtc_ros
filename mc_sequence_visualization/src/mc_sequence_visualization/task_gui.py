# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

import rospy
from sensor_msgs.msg import JointState

import copy

from eigen import Vector3d, Quaterniond, MatrixXd
import numpy as np
import rbdyn as rbd
import tasks

from mc_rbdyn import PlanarSurface, GripperSurface, CylindricalSurface
from mc_solver import KinematicsConstraint, DynamicsConstraint

from .contact_marker import ContactControlMarker
from task_dialog import Ui_TaskDialog
from link_dialog import Ui_LinkDialog

from math import pi, ceil, floor

class TaskDialog(QtGui.QDialog):
  def __init__(self, robots, qpsolver, contCstr, robotPublisher, jointStatePub, timeStep, parent=None):
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
    self.contactConstraint = contCstr

    # create qp constraint
    self.kinematicsConstraint = KinematicsConstraint(robots, 0,
                                                     qpsolver.timeStep,
                                                     damper=(0.1, 0.01, 0.5),
                                                     velocityPercent=0.5)

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
    self.tasks = {'Posture': self.postureTask, 'Position': self.contactTaskSp,
                  'Orientation': self.orientationTaskSp, 'CoM': self.comTaskSp}

    # add tasks to the qp
    for t in self.tasks.itervalues():
      self.qpsolver.addTask(t)

    # add constraint to the qp
    self.qpsolver.addConstraintSet(self.kinematicsConstraint)

    contacts = copy.copy(stance.contacts())
    c = contacts[contactIndex]
    #del contacts[contactIndex]
    cId = contacts[contactIndex].contactId(self.robots)
    dof = np.eye(6)
    if isinstance(c.r1Surface(), GripperSurface)\
       and isinstance(c.r2Surface(), CylindricalSurface):
      #Free x and rotX
      dof[0, 0] = 0
      dof[3, 3] = 0
      #Add all dofs for precise contact repostioning
      dof[1, 1] = 0
      dof[2, 3] = 0
      dof[4, 4] = 0
      dof[5, 5] = 0
    elif isinstance(c.r1Surface(), PlanarSurface)\
          and isinstance(c.r2Surface(), PlanarSurface):
      #Free rotZ, x, y
      dof[2, 2] = 0
      dof[3, 3] = 0
      dof[4, 4] = 0
      #Add all dofs for precise contact repostioning
      dof[0, 0] = 0
      dof[1, 1] = 0
      dof[5, 5] = 0
    else:
      print type(c.robotSurface), type(c.envSurface)
      raise Exception("Contact is neither gripper/cylindrical nor planar/planar")
    self.contactConstraint.contactConstr.addDofContact(cId, MatrixXd(dof))
    self.contactConstraint.contactConstr.updateDofContacts()
    self.qpsolver.setContacts(contacts)

    self.updateTaskTable()

    self.im.init(bodyTf.translation(), bodyTf.rotation(), dof.diagonal().tolist())

    self.sendTimer.start(self.qpsolver.timeStep)

  def updateTaskTable(self):
    self.ui.taskTable.clear()
    self.ui.taskTable.setRowCount(len(self.tasks))
    self.ui.taskTable.setColumnCount(3)
    self.ui.taskTable.setHorizontalHeaderLabels(['Task', 'Stiffness', 'Weight'])
    self.setterByItemPos = {}
    for i, (tn, t) in enumerate(self.tasks.iteritems()):
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
      if key in self.setterByItemPos:
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
    q = feedback.pose.orientation
    ori = Quaterniond(q.w, q.x, q.y, q.z).inverse()
    print feedback.marker_name + " is now at {}, {}, {}".format(p.x, p.y, p.z)
    self.contactTask.position(Vector3d(p.x, p.y, p.z))
    self.orientationTask.orientation(ori)

  def exec_(self, stance, stanceIndex, contactIndex):
    self.init(stance, stanceIndex, contactIndex)
    return super(TaskDialog, self).exec_()

  def done(self, result):
    self.contactConstraint.contactConstr.resetDofContacts()
    self.contactConstraint.contactConstr.updateDofContacts()
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

class LinkDialog(QtGui.QDialog):
  def __init__(self, robots, qpsolver, jointStatePub, parent=None):
    super(LinkDialog, self).__init__(parent)
    self.ui = Ui_LinkDialog()
    self.ui.setupUi(self)

    self.robots = robots
    self.robot = self.robots.robot()
    self.qpsolver = qpsolver
    self.jointStatePub = jointStatePub
    self.tasks = {}
    self.setterByItemPos = {}
    self.contact = None
    self.newContact = None
    self.bodyIndexToMove = None

    # create qp constraint
    self.kinematicsConstraint = KinematicsConstraint(robots, 0,
                                                     qpsolver.timeStep,
                                                     damper=(0.1, 0.01, 0.5),
                                                     velocityPercent=0.5)

    self.dynamicsConstraint = DynamicsConstraint(robots, 0,
                                                 qpsolver.timeStep,
                                                 damper=(0.1, 0.01, 0.5),
                                                 velocityPercent=0.5)

    # setup control marker
    self.link_im = ContactControlMarker('link', self.targetPosition)
    self.com_im = ContactControlMarker('com', self.targetCoM)

    # Ui setting
    self.ui.taskTable.itemChanged.connect(self.taskChanged)

    self.body_names = [b.name() for b in self.robot.mb.bodies()]
    self.ui.linkSelectBox.clear()
    self.ui.linkSelectBox.addItems(self.body_names)
    self.ui.linkSelectBox.currentIndexChanged.connect(self.changeEf)

    self.joint_names = [j.name() for j in self.robot.mb.joints()]
    self.ui.jointSelectBox.clear()
    self.ui.jointSelectBox.addItems(self.joint_names)
    self.ui.jointSelectBox.currentIndexChanged.connect(self.changeCurrentJoint)
    self.ui.jointSelectBox.setCurrentIndex(1)
    self.changeCurrentJoint(1)

    self.ui.jointSlider.sliderMoved.connect(self.changeSliderAngle)
    self.ui.jointLineEdit.textEdited.connect(self.changeTextAngle)

    # Timer to run the qp
    self.sendTimer = QtCore.QTimer(self)
    self.sendTimer.timeout.connect(self.runQP)

  def createEfTasks(self, bodyId, bodyTf):
    def spTask(task, stiffness, weight):
      return tasks.qp.SetPointTask(self.robots.mbs, 0, task, stiffness, weight)

    self.efTask = tasks.qp.PositionTask(self.robots.mbs, 0,
                                        bodyId, bodyTf.translation())
    self.efTaskSp = spTask(self.efTask, 10., 10.)
    self.orientationTask = tasks.qp.OrientationTask(self.robots.mbs, 0,
                                                    bodyId, bodyTf.rotation())
    self.orientationTaskSp = spTask(self.orientationTask, 1., 1000.)

  def init(self, stance, stanceIndex, bodyIndex=0):
    self.stance = stance

    # compute contact body value
    self.bodyIndexToMove = bodyIndex
    bodyId = self.robot.mb.body(bodyIndex).id()

    # reset mbc velocity, acceleration and torque and set initial q value
    self.robot.mbc.zero(self.robot.mb)
    self.robot.mbc.q = stance.q

    rbd.forwardKinematics(self.robot.mb, self.robot.mbc)
    rbd.forwardVelocity(self.robot.mb, self.robot.mbc)
    bodyTf = list(self.robot.mbc.bodyPosW)[bodyIndex]

    def spTask(task, stiffness, weight):
      return tasks.qp.SetPointTask(self.robots.mbs, 0, task, stiffness, weight)

    # create qp tasks
    self.postureTask = tasks.qp.PostureTask(self.robots.mbs, 0,
                                            stance.q, 1., 5.)
    self.createEfTasks(bodyId, bodyTf)
    self.comTask = tasks.qp.CoMTask(self.robots.mbs, 0,
                                    rbd.computeCoM(self.robot.mb,
                                                   self.robot.mbc))
    self.comTaskSp = spTask(self.comTask, 1., 100.)
    self.tasks = {'Posture': self.postureTask, 'Position': self.efTaskSp,
                  'Orientation': self.orientationTaskSp, 'CoM': self.comTaskSp}

    # Avoid crash by making sure we have no tasks in the QP
    self.qpsolver.solver.resetTasks()
    self.qpsolver.updateNrVars()
    # add tasks to the qp
    self.qpsolver.solver.addTask(self.postureTask)
    self.qpsolver.solver.addTask(self.comTaskSp)

    # add constraint to the qp
    self.qpsolver.addConstraintSet(self.kinematicsConstraint)

    self.qpsolver.setContacts(stance.contacts)
    self.qpsolver.update()

    self.updateTaskTable()

    self.link_im.init(bodyTf.translation(), bodyTf.rotation())
    self.com_im.init(rbd.computeCoM(self.robot.mb, self.robot.mbc),
                     dof=[1]*3+[0]*3)

    self.sendTimer.start(self.qpsolver.timeStep)

  def updateTaskTable(self):
    self.ui.taskTable.clear()
    self.ui.taskTable.setRowCount(len(self.tasks))
    self.ui.taskTable.setColumnCount(3)
    self.ui.taskTable.setHorizontalHeaderLabels(['Task', 'Stiffness',
                                                 'Weight'])
    self.setterByItemPos = {}
    for i, (tn, t) in enumerate(self.tasks.iteritems()):
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
      if key in self.setterByItemPos:
        self.setterByItemPos[key](data)

  def changeEf(self, index):
    print "Changing to : {} == {}".format(index,
                                          self.robot.mb.body(index).name())
    self.postureTask.posture(self.robot.mbc.q)
    bodyId = self.robot.mb.body(index).id()
    self.qpsolver.solver.removeTask(self.efTaskSp)
    self.qpsolver.solver.removeTask(self.orientationTaskSp)

    bodyTf = list(self.robot.mbc.bodyPosW)[index]

    self.createEfTasks(bodyId, bodyTf)

    self.qpsolver.solver.addTask(self.efTaskSp)
    self.qpsolver.solver.addTask(self.orientationTaskSp)

    self.link_im.clear()
    self.link_im.init(bodyTf.translation(), bodyTf.rotation())

  def changeCurrentJoint(self, index):
    self.current_joint = index
    degrees = lambda x: 180*x/pi
    qu = self.robot.qu[index][0]
    ql = self.robot.ql[index][0]
    q = self.robot.mbc.q[index][0]

    qudeg = floor(degrees(qu))
    qldeg = ceil(degrees(ql))
    self.ui.jointSlider.setMaximum(qudeg)
    self.ui.jointSlider.setMinimum(qldeg)
    self.ui.jointSlider.setValue(round(degrees(q)))

    self.validator = QtGui.QDoubleValidator(ql, qu, 10)
    self.validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
    self.ui.jointLineEdit.setValidator(self.validator)
    self.ui.jointLineEdit.setText('%.8f' % q)

    self.ui.qlLabel.setText(u"{}\N{DEGREE SIGN}".format(qldeg))
    self.ui.quLabel.setText(u"{}\N{DEGREE SIGN}".format(qudeg))

  def changeSliderAngle(self, value):
    rad = value*pi/180
    txt = '%.8f' % rad
    self.ui.jointLineEdit.setText(txt)
    self.changeJointAngle(rad)

  def changeTextAngle(self, text):
    if text:
      try:
        value = float(text)
      except ValueError:
        print "Invalid value : {}".format(text)
        return
    else:
      return
    deg = value*180/pi
    self.ui.jointSlider.setValue(deg)
    self.changeJointAngle(value)

  def changeJointAngle(self, value):
    p = self.postureTask.posture()
    p[self.current_joint][0] = value
    self.postureTask.posture(p)

  def runQP(self):
    # run solver
    # if the qp fail we restart the initial configuration
    if not self.qpsolver.run():
      self.robot.mbc.zero(self.robot.mb)
      self.robot.mbc.q = self.stance.q

    self.sendAll()

  def targetPosition(self, feedback):
    p = feedback.pose.position
    q = feedback.pose.orientation
    ori = Quaterniond(q.w, q.x, q.y, q.z).inverse()
    print feedback.marker_name + " is now at {}, {}, {}".format(p.x, p.y, p.z)
    self.efTask.position(Vector3d(p.x, p.y, p.z))
    self.orientationTask.orientation(ori)

  def targetCoM(self, feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at {}, {}, {}".format(p.x, p.y, p.z)
    self.comTask.com(Vector3d(p.x, p.y, p.z))

  def exec_(self, stance, stanceIndex):
    self.init(stance, stanceIndex)
    return super(LinkDialog, self).exec_()

  def done(self, result):
    # stop the time
    self.sendTimer.stop()

    #Remove kinematics constraint, keep tasks for checking
    self.qpsolver.removeConstraintSet(self.kinematicsConstraint)

    if result == QtGui.QDialog.DialogCode.Accepted:
      max_iter = 1000
      q = self.robot.mbc.q
      stable = self.check_stability(max_iter, q, self.stance.contacts)

      if not stable:
        msg = QtGui.QMessageBox()
        msg.setText("Posture is not stable")
        msg.setInformativeText("Add it anyway ?")
        msg.setStandardButtons(QtGui.QMessageBox.Cancel | QtGui.QMessageBox.Ok)
        ret = msg.exec_()
        if ret == QtGui.QMessageBox.Ok:
          self.stance.q = q

      else:
        self.stance.q = q

    #Remove tasks
    for t in self.tasks.itervalues():
      self.qpsolver.solver.removeTask(t)
    self.tasks = {}

    self.link_im.clear()
    self.com_im.clear()

    super(LinkDialog, self).done(result)

  def check_stability(self, max_iter, q, contacts):
    #Check if the stance is stable by running a few times dynamic QP
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.robot.mbc.zero(self.robot.mb)
    self.robot.mbc.q = q
    rbd.forwardKinematics(self.robot.mb, self.robot.mbc)
    rbd.forwardVelocity(self.robot.mb, self.robot.mbc)
    self.postureTask.posture(q)

    self.qpsolver.setContacts(contacts)
    self.qpsolver.update()
    nr_iter = 0
    stable = True

    max_iter = 1000
    #Display a progress dialog
    progDial = QtGui.QProgressDialog('Checking stability', 'Abort', 0,
                                     max_iter, self)
    progDial.setCancelButton(None)
    progDial.setWindowModality(QtCore.Qt.WindowModal)
    progDial.setMinimumDuration(0)

    while nr_iter < max_iter:
      progDial.setValue(nr_iter)
      stable = self.qpsolver.run()
      if not stable:
        break
      self.sendAll()
      nr_iter += 1

    progDial.setValue(max_iter)
    self.qpsolver.removeConstraintSet(self.dynamicsConstraint)
    return stable

  def sendAll(self):
    curTime = rospy.Time.now()
    js = JointState()
    js.header.stamp = curTime
    self.qpsolver.send(curTime)
    self.jointStatePub.publish(js)
