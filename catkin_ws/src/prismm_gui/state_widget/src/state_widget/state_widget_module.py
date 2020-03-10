import os
import rospy
import rospkg

from prismm_msgs.msg import dam_data, pas_data
from std_msgs.msg import Empty

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

class StateWidget(Plugin):

    pas_state = {-1:"E_STOP",
                  0:"DEFAULT_STATE",
                  1:"HOMED",
                  2:"HOMING_PROBE",
                  3:"HOMING",
                  4:"BOWL",
                  5:"ROCKWELL"}

    dam_state = {-1:"E_STOP",
                 0:"DEFAULT_STATE",
                 1:"HOMED",
                 2:"HOMING",
                 3:"HOMING_DRILL",
                 4:"DRILLING"}

    dam_data_sig = QtCore.pyqtSignal(int)
    pas_data_sig = QtCore.pyqtSignal(int)

    def __init__(self, context):
        super(StateWidget, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('StateWidget')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('state_widget'), 'resource', 'state_widget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('StateWidgetPluginUi')
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

        ####buttons
        self.estop_pub = rospy.Publisher("estop", Empty, queue_size=10)
        self._widget.estopButton.clicked[bool].connect(lambda:self.estop_pub.publish(Empty()))

        self.reset_pub = rospy.Publisher("reset", Empty, queue_size=10)
        self._widget.resetButton.clicked[bool].connect(lambda:self.restart_pub.publish(Empty()))

        self.resume_pub = rospy.Publisher("resume", Empty, queue_size=10)
        self._widget.resumeButton.clicked[bool].connect(lambda:self.resume_pub.publish(Empty()))

    ### dam and pas data ###
    def dam_data_cb(self, msg):
        self.dam_data_sig.emit(msg.state)
    def dam_data_sig_cb(self, state):
        self._widget.damLabel.setText(self.dam_state[state])

    def pas_data_cb(self, msg):
        self.pas_data_sig.emit(msg.state)
    def pas_data_sig_cb(self, state):
        self._widget.probePosLabel.setValue(self.pas_state[state])


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
