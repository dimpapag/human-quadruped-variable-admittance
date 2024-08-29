class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node_teensy')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.serial_port = self.get_parameter('serial_port').value
        os.system('sudo chmod 777 ' + str(self.serial_port))

        self.serial_port = serial.Serial(self.serial_port, 115200)  # Replace with your serial port name and baudrate
        self.publisher = self.create_publisher(String, 'suction_cups_data_str', 10)
        self.timer = self.create_timer(0.0001, self.read_serial)
        self.subscription = self.create_subscription(EcattieControlPanel,'ecattie_panel_data',self.controll_panel_listener_callback,10)
        self.detach_all_prev = False

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            msg_mData = String()
            msg_mData.data = self.serial_port.readline().decode("utf-8").rstrip()
            self.publisher.publish(msg_mData)