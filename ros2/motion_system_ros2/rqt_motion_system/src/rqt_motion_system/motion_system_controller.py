from rqt_gui_py.plugin import Plugin

from rqt_motion_system.motor_manager_widget import MotorManagerWidget

class MotionSystemController(Plugin):
    def __init__(self, context):
        super().__init__(context)

        self.setObjectName('MotionSystemController')
        
        self.widget = MotorManagerWidget(context.node)

        serial_number = context.serial_number()
        if serial_number >= 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + f' {serial_number}')
        
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('MotionSystemController shutdown')
        self.widget.shutdown_widget()