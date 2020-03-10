import os
import rospy
import rospkg

from prismm_msgs.msg import dam_data, pas_data
from std_msgs.msg import UInt16

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

class StepperWidget(Plugin):

    x_position = 0
    drill_position = 0
    probe_position = 0
    ext_position = 0
    rot_position = 0

    x_position_target = 0
    drill_position_target = 0
    probe_position_target = 0
    ext_position_target = 0
    rot_position_target = 0


    dam_data_sig = QtCore.pyqtSignal(float, float)
    pas_data_sig = QtCore.pyqtSignal(float, float , float)

    def __init__(self, context):
        super(StepperWidget, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StepperWidget')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stepper_widget'), 'resource', 'stepper_widget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('StepperWidgetPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        ####subscriers
        self.dam_sub = rospy.Subscriber("dam_data", dam_data, self.dam_data_cb)
        self.dam_data_sig.connect(self.dam_data_sig_cb)
        self.pas_sub = rospy.Subscriber("pas_data", pas_data, self.pas_data_cb)
        self.pas_data_sig.connect(self.pas_data_sig_cb)

        ###move buttons
        self.x_move_pub = rospy.Publisher("x_axis_target", UInt16, queue_size=10)
        self._widget.xMoveButton.clicked[bool].connect(self.x_move_cb)

        self.drill_move_pub = rospy.Publisher("drill_y_axis_target", UInt16, queue_size=10)
        self._widget.drillMoveButton.clicked[bool].connect(self.drill_move_cb)

        self.probe_move_pub = rospy.Publisher("probe_y_axis_target", UInt16, queue_size=10)
        self._widget.probeMoveButton.clicked[bool].connect(self.probe_move_cb)

        self.rot_move_pub = rospy.Publisher("probe_rot_target", UInt16, queue_size=10)
        self._widget.rotMoveButton.clicked[bool].connect(self.rot_move_cb)

        self.ext_move_pub = rospy.Publisher("probe_ext_target", UInt16, queue_size=10)
        self._widget.extMoveButton.clicked[bool].connect(self.ext_move_cb)

        ###sliders and boxes
        self._widget.xSlider.valueChanged[int].connect(self.x_slider_cb)
        self._widget.xBox.valueChanged[int].connect(self.x_box_cb)

        self._widget.drillSlider.valueChanged[int].connect(self.drill_slider_cb)
        self._widget.drillBox.valueChanged[int].connect(self.drill_box_cb)

        self._widget.probeSlider.valueChanged[int].connect(self.probe_slider_cb)
        self._widget.probeBox.valueChanged[int].connect(self.probe_box_cb)

        self._widget.rotSlider.valueChanged.connect(self.rot_slider_cb)
        self._widget.rotBox.valueChanged.connect(self.rot_box_cb)

        self._widget.extSlider.valueChanged.connect(self.ext_slider_cb)
        self._widget.extBox.valueChanged.connect(self.ext_box_cb)


    ### sliders and boxes ###
    def x_box_cb(self, pos):
        self.x_position = pos
        self._widget.xSlider.setValue(pos)
    def x_slider_cb(self, pos):
        self.x_position = pos
        self._widget.xBox.setValue(pos)

    def drill_box_cb(self, pos):
        self.drill_position = pos
        self._widget.drillSlider.setValue(pos)
    def drill_slider_cb(self, pos):
        self.drill_position = pos
        self._widget.drillBox.setValue(pos)

    def probe_box_cb(self, pos):
        self.probe_position = pos
        self._widget.probeSlider.setValue(pos)
    def probe_slider_cb(self, pos):
        self.probe_position = pos
        self._widget.probeBox.setValue(pos)

    def rot_box_cb(self, pos):
        self.rot_position = pos
        self._widget.rotSlider.setValue(pos)
    def rot_slider_cb(self, pos):
        self.rot_position = pos
        self._widget.rotBox.setValue(pos)

    def ext_box_cb(self, pos):
        self.ext_position = pos
        self._widget.extSlider.setValue(pos)
    def ext_slider_cb(self, pos):
        self.ext_position = pos
        self._widget.extBox.setValue(pos)

    ### move buttons ###

    def x_move_cb(self, var):
        self.x_move_pub.publish(self.x_position)
        self.x_position_target = self.x_position

    def drill_move_cb(self, var):
        self.drill_move_pub.publish(self.drill_position)
        self.drill_position_target = self.drill_position

    def probe_move_cb(self, var):
        self.probe_move_pub.publish(self.probe_position)
        self.probe_position_target = self.probe_position

    def ext_move_cb(self, var):
        self.ext_move_pub.publish(self.ext_position)
        self.ext_position_target = self.ext_position

    def rot_move_cb(self, var):
        self.rot_move_pub.publish(self.rot_position)
        self.rot_position_target = self.rot_position

    ### dam and pas data ###
    def dam_data_cb(self, msg):
        self.dam_data_sig.emit(msg.stp_x1, msg.stp_y)
    def dam_data_sig_cb(self, stp_x, stp_drill):
        self._widget.xPosLabel.setNum(stp_x)
        self._widget.drillPosLabel.setNum(stp_drill)

        self._widget.xBar.setValue(x_position/x_position_target)
        self._widget.drillBar.setValue(drill_position/drill_position_target)

    def pas_data_cb(self, msg):
        self.pas_data_sig.emit(msg.stp_y, msg.stp_rot, msg.stp_ext)
    def pas_data_sig_cb(self, stp_probe, stp_rot, stp_ext):
        self._widget.probePosLabel.setNum(stp_probe)
        self._widget.rotPosLabel.setNum(stp_rot)
        self._widget.extPosLabel.setNum(stp_ext)

        self._widget.probeBar.setValue(probe_position/probe_position_target)
        self._widget.rotBar.setValue(rot_position/rot_position_target)
        self._widget.extBar.setValue(ext_position/ext_position_target)


    ##UI class overrides
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
