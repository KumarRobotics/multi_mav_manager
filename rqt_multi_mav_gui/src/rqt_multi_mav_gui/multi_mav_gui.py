# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospkg

from geometry_msgs.msg import Twist, TwistStamped
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin

import kr_mav_manager.srv
import std_srvs.srv
import std_msgs.msg
import kr_multi_mav_manager.srv

class MultiMavGUI(Plugin):

  #slider_factor = 1000.0

  def __init__(self, context):
    super(MultiMavGUI, self).__init__(context)
    self.setObjectName('MultiMavGUI')

    self._publisher = None

    self.mav_node_name = 'multi_mav_services'

    self._widget = QWidget()
    rp = rospkg.RosPack()
    ui_file = os.path.join(rp.get_path('rqt_multi_mav_gui'), 'resource', 'MultiMavGUI.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('MultiMavGUIUi')
    if context.serial_number() > 1:
      self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._widget)

    self._widget.motors_on_push_button.pressed.connect(self._on_motors_on_pressed)
    self._widget.motors_off_push_button.pressed.connect(self._on_motors_off_pressed)

    self._widget.rotate_on_push_button.pressed.connect(self._on_rotate_on_pressed)
    self._widget.rotate_off_push_button.pressed.connect(self._on_rotate_off_pressed)

    self._widget.land_push_button.pressed.connect(self._on_land_pressed)
    self._widget.estop_push_button.pressed.connect(self._on_estop_pressed)
    self._widget.takeoff_push_button.pressed.connect(self._on_takeoff_pressed)

    self._widget.circle_button.pressed.connect(self._on_circle_pressed)
    self._widget.line_button.pressed.connect(self._on_line_pressed)
    self._widget.rect_button.pressed.connect(self._on_rect_pressed)
    self._widget.angle_button.pressed.connect(self._on_angle_pressed)

    self._widget.circle1_butt.pressed.connect(self._on_circle1_butt_pressed)
    self._widget.circle2_butt.pressed.connect(self._on_circle2_butt_pressed)
    self._widget.circle3_butt.pressed.connect(self._on_circle3_butt_pressed)
    self._widget.christmas1_butt.pressed.connect(self._on_christmas1_butt_pressed)
    self._widget.christmas2_butt.pressed.connect(self._on_christmas2_butt_pressed)
    self._widget.christmas3_butt.pressed.connect(self._on_christmas3_butt_pressed)
    self._widget.flyingv_butt.pressed.connect(self._on_flyingv_butt_pressed)
    self._widget.line1_butt.pressed.connect(self._on_line1_butt_pressed)
    self._widget.rect1_butt.pressed.connect(self._on_rect1_butt_pressed)
    self._widget.rect2_butt.pressed.connect(self._on_rect2_butt_pressed)
    self._widget.stairs_butt.pressed.connect(self._on_stairs_butt_pressed)

    self._widget.demo_buttons.hide()

  def _on_motors_on_pressed(self):
    try:
      motors_topic = '/'+self.mav_node_name+'/motors'
      rospy.wait_for_service(motors_topic, timeout=1.0)
      motors_on = rospy.ServiceProxy(motors_topic, std_srvs.srv.SetBool)
      resp = motors_on(True)
      print 'Motors on ', resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    except(rospy.ROSException), e:
      print "Service call failed: %s"%e

  def _on_motors_off_pressed(self):
    try:
      motors_topic = '/'+self.mav_node_name+'/motors'
      rospy.wait_for_service(motors_topic, timeout=1.0)
      motors_off = rospy.ServiceProxy(motors_topic, std_srvs.srv.SetBool)
      resp = motors_off(False)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    except(rospy.ROSException), e:
      print "Service call failed: %s"%e

  def _on_rotate_on_pressed(self):
    rotate_topic = '/rotate_on'
    rotate_pub = rospy.Publisher(rotate_topic, std_msgs.msg.Bool)
    rotate_pub.publish(True)

  def _on_rotate_off_pressed(self):
    rotate_topic = '/rotate_on'
    rotate_pub = rospy.Publisher(rotate_topic, std_msgs.msg.Bool)
    rotate_pub.publish(False)

  def _on_land_pressed(self):
    try:
      land_topic = '/'+self.mav_node_name+'/land'
      rospy.wait_for_service(land_topic, timeout=1.0)
      land = rospy.ServiceProxy(land_topic, std_srvs.srv.Trigger)
      resp = land()
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    except(rospy.ROSException), e:
      print "Service call failed: %s"%e

  def _on_estop_pressed(self):
    try:
      estop_topic = '/'+self.mav_node_name+'/estop'
      rospy.wait_for_service(estop_topic, timeout=1.0)
      estop = rospy.ServiceProxy(estop_topic, std_srvs.srv.Trigger)
      resp = estop()
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    except(rospy.ROSException), e:
      print "Service call failed: %s"%e

  def _on_takeoff_pressed(self):
    try:
      takeoff_topic = '/'+self.mav_node_name+'/takeoff'
      rospy.wait_for_service(takeoff_topic, timeout=1.0)
      takeoff = rospy.ServiceProxy(takeoff_topic, std_srvs.srv.Trigger)
      resp = takeoff()
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    except(rospy.ROSException), e:
      print "Service call failed: %s"%e

  def _on_circle_pressed(self):

    req = kr_multi_mav_manager.srv.FormationRequest()

    req.center[0] = self._widget.x_doubleSpinBox.value()
    req.center[1] = self._widget.y_doubleSpinBox.value()
    req.center[2] = self._widget.z_doubleSpinBox.value()

    req.roll = self._widget.roll_doubleSpinBox.value()
    req.pitch = self._widget.pitch_doubleSpinBox.value()
    req.yaw = self._widget.yaw_doubleSpinBox.value()

    req.spacing = self._widget.spacing_doubleSpinBox.value()

    print req.center

    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormCircle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
      print resp.message
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_rect_pressed(self):

    req = kr_multi_mav_manager.srv.FormationRequest()

    req.center[0] = self._widget.x_doubleSpinBox.value()
    req.center[1] = self._widget.y_doubleSpinBox.value()
    req.center[2] = self._widget.z_doubleSpinBox.value()

    req.roll = self._widget.roll_doubleSpinBox.value()
    req.pitch = self._widget.pitch_doubleSpinBox.value()
    req.yaw = self._widget.yaw_doubleSpinBox.value()

    req.spacing = self._widget.spacing_doubleSpinBox.value()

    print req.center

    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormRect', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_line_pressed(self):

    req = kr_multi_mav_manager.srv.FormationRequest()

    req.center[0] = self._widget.x_doubleSpinBox.value()
    req.center[1] = self._widget.y_doubleSpinBox.value()
    req.center[2] = self._widget.z_doubleSpinBox.value()

    req.roll = self._widget.roll_doubleSpinBox.value()
    req.pitch = self._widget.pitch_doubleSpinBox.value()
    req.yaw = self._widget.yaw_doubleSpinBox.value()

    req.spacing = self._widget.spacing_doubleSpinBox.value()

    print req.center

    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormLine', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_angle_pressed(self):

    req = kr_multi_mav_manager.srv.FormationRequest()

    req.center[0] = self._widget.x_doubleSpinBox.value()
    req.center[1] = self._widget.y_doubleSpinBox.value()
    req.center[2] = self._widget.z_doubleSpinBox.value()

    req.roll = self._widget.roll_doubleSpinBox.value()
    req.pitch = self._widget.pitch_doubleSpinBox.value()
    req.yaw = self._widget.yaw_doubleSpinBox.value()

    req.spacing = self._widget.spacing_doubleSpinBox.value()

    print req.center

    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormAngle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  ## Predefined shapes
  def _on_circle1_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.0
    req.center[1] = 0
    req.center[2] = 1.25
    req.roll = 0
    req.pitch = 0
    req.yaw = 0
    req.spacing = 0.75
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormCircle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_circle2_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 6
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = .3
    req.yaw = 0
    req.spacing = 0.9
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormCircle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_circle3_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 3.5
    req.center[1] = 0
    req.center[2] = 1
    req.roll = 0
    req.pitch = -.4
    req.yaw = 0
    req.spacing = 0.75
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormCircle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_christmas1_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 1.59
    req.yaw = 0
    req.spacing = 0.75
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormAngle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_christmas2_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 1.59
    req.yaw = 1.0
    req.spacing = 0.75
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormAngle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_christmas3_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 1.59
    req.yaw = 2
    req.spacing = 0.75
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormAngle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_flyingv_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 0
    req.yaw = 0
    req.spacing = 0.6
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormAngle', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_line1_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 0
    req.yaw = 0
    req.spacing = 0.7
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormLine', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_stairs_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.25
    req.roll = 0
    req.pitch = 0.6
    req.yaw = 0
    req.spacing = 0.7
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormLine', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_rect1_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 0
    req.yaw = 0
    req.spacing = 0.7
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormRect', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def _on_rect2_butt_pressed(self):
    req = kr_multi_mav_manager.srv.FormationRequest()
    req.center[0] = 4.5
    req.center[1] = 0
    req.center[2] = 1.5
    req.roll = 0
    req.pitch = 0
    req.yaw = 1.59
    req.spacing = 0.7
    try:
      form_topic = rospy.ServiceProxy('/'+self.mav_node_name+'/goFormRect', kr_multi_mav_manager.srv.Formation())
      resp = form_topic(req)
      print resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  #  def _unregister_publisher(self):
  #    if self._publisher is not None:
  #     self._publisher.unregister()
  #   self._publisher = None

  # def shutdown_plugin(self):
  #   self._unregister_publisher()

  # def save_settings(self, plugin_settings, instance_settings):
  #   instance_settings.set_value('robot_name' , self._widget.robot_name_line_edit.text())
  #   instance_settings.set_value('node_name' , self._widget.node_name_line_edit.text())

  # def restore_settings(self, plugin_settings, instance_settings):
  #   value = instance_settings.value('robot_name', "quadrotor")
  #   #value = rospy.get_param("~robot_name", value)
  #   self.robot_name = value
  #   self._widget.robot_name_line_edit.setText(value)

  #   value = instance_settings.value('node_name', "mav_manager_node")
  #   #value = rospy.get_param("~robot_name", value)
  #   self.node_name = value
  #   self._widget.node_name_line_edit.setText(value)
