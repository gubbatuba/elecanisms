# damper_gui.py
#!/usr/bin/python
import Tkinter
from usb_haptic import USBCommunications


class simpleapp_tk(Tkinter.Tk):

    def __init__(self, parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.initialize()
        self.usbc = None

    def initialize(self):
        self.grid()
        self.connect_status = Tkinter.StringVar(value="No device connected")
        button_connect = Tkinter.Button(self, text=u"Connect to Device",
                                command=self.try_connect)
        button_connect.grid(column=0, row=0, columnspan=2)
        
        self.connection_label = Tkinter.Label(textvariable=self.connect_status, justify='center')
        self.connection_label.grid(column=2, row=0, columnspan=2)
        # self.spring_constant_scale = Tkinter.Scale(
        #     self, from_=0, to=0.5, resolution=.01)
        # self.spring_constant_scale.grid(column=0, row=2)

        self.wall_angle_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.wall_angle_scale.grid(column=1, row=2)

        # self.Ki_constant_scale = Tkinter.Scale(
        #     self, from_=0, to=0.5, resolution=.01)
        # self.Ki_constant_scale.grid(column=2, row=2)

        # self.Kd_constant_scale = Tkinter.Scale(
        #     self, from_=0, to=0.5, resolution=.01)
        # self.Kd_constant_scale.grid(column=3, row=2)

        button_k = Tkinter.Button(self, text=u"Set Wall Angle",
                                command=self.set_wall_ang)
        button_k.grid(column=0, row=4, columnspan=3)

        # button_kp = Tkinter.Button(self, text=u"Set Kp",
        #                         command=self.set_pid_p)
        # button_kp.grid(column=1, row=4)

        # button_ki = Tkinter.Button(self, text=u"Set Ki",
        #                         command=self.set_pid_i)
        # button_ki.grid(column=2, row=4)

        # button_kd = Tkinter.Button(self, text=u"Set Kd",
        #                         command=self.set_pid_d)
        # button_kd.grid(column=3, row=4)

        # self.grid_columnconfigure(0,weight=1)
        self.resizable(True, False)
        self.update()
        self.geometry(self.geometry())
        # self.entry.focus_set()
        # self.entry.selection_range(0, Tkinter.END)

    def try_connect(self):
        if not self.usbc:
            try:
                self.usbc = USBCommunications()
                self.connect_status.set("Connected!")
            except ValueError:
                print("No connection found...")
                self.usbc = None

    def set_wall_ang(self):
        new_constant = self.wall_angle_scale.get()
        self.usbc.set_wall_angle(new_constant)

    # def set_pid_p(self):
    #     new_constant = self.Kp_constant_scale.get()
    #     self.usbc.set_spring_constant(new_constant)

    # def set_pid_i(self):
    #     new_constant = self.Ki_constant_scale.get()
    #     self.usbc.set_spring_constant(new_constant)

    # def set_pid_d(self):
    #     new_constant = self.Kd_constant_scale.get()
    #     self.usbc.set_spring_constant(new_constant)

if __name__ == "__main__":
    app = simpleapp_tk(None)
    app.title('Haptic Wall Control')
    app.mainloop()
