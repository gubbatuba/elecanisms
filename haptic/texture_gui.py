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

        self.light_stick_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.light_stick_scale.grid(column=0, row=2)

        button_ls = Tkinter.Button(self, text=u"Set Light Stick Angle",
                                command=self.set_ls_ang)
        button_ls.grid(column=0, row=3)

        self.heavy_stick_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.heavy_stick_scale.grid(column=1, row=2)

        button_hs = Tkinter.Button(self, text=u"Set Heavy Stick Angle",
                                command=self.set_hs_ang)
        button_hs.grid(column=1, row=3)

        self.light_slip_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.light_slip_scale.grid(column=2, row=2)

        button_lp = Tkinter.Button(self, text=u"Set Light Slip Angle",
                                command=self.set_lp_ang)
        button_lp.grid(column=2, row=3)

        self.heavy_slip_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.heavy_slip_scale.grid(column=3, row=2)

        button_hp= Tkinter.Button(self, text=u"Set Heavy Slip Angle",
                                command=self.set_ls_ang)
        button_hp.grid(column=3, row=3)

        self.speed_bump_scale = Tkinter.Scale(
            self, from_=-40, to=40, resolution=.1)
        self.speed_bump_scale.grid(column=4, row=2)

        button_hp= Tkinter.Button(self, text=u"Set Speed Bump Angle",
                                command=self.set_sb_ang)
        button_hp.grid(column=4, row=3)
       
        self.resizable(True, False)
        self.update()
        self.geometry(self.geometry())


    def try_connect(self):
        if not self.usbc:
            try:
                self.usbc = USBCommunications()
                self.connect_status.set("Connected!")
            except ValueError:
                print("No connection found...")
                self.usbc = None

    def set_wall_ang(self):
        new_constant = self.Damper_Coef_Scale.get()
        self.usbc.set_damper_coef(new_constant)

    # def set_ls_ang(self):
    #     new_constant = self.Damper_Coef_Scale.get()
    #     self.usbc.set_damper_coef(new_constant)
    # def set_hs_ang(self):
    # def set_lp_ang(self):
    # def set_ls_ang(self):
    # def set_sb_ang(self):


if __name__ == "__main__":
    app = simpleapp_tk(None)
    app.title('Haptic Texture Control')
    app.mainloop()
