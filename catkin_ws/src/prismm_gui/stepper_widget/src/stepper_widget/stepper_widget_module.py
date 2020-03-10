import os
import rospy
import rospkg

from prismm_msgs.msg import dam_data
from std_msgs.msg import UInt16

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

class StepperWidget(Plugin):

    position = 0

    dam_data_sig = QtCore.pyqtSignal(float)

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

        self.dam_sub = rospy.Subscriber("dam_data", dam_data, self.dam_data_cb)
        self.dam_data_sig.connect(self.dam_data_sig_cb)

        self.move_pub = rospy.Publisher("x_axis_target", UInt16)
        self._widget.moveButton.clicked[bool].connect(self.move_cb)

        self._widget.positionSlider.valueChanged[int].connect(self.pos_slider_cb)

    def pos_slider_cb(self, pos):
        self.position = pos
        self._widget.positionBox.setValue(pos)

    def move_cb(self, var):
        self.move_pub.publish(self.position)

    def dam_data_cb(self, msg):
        self.dam_data_sig.emit(msg.stp_x1)

    def dam_data_sig_cb(self, stp_pos):
        self._widget.positionLabel.setNum(stp_pos)


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
