import asyncio
import tkinter as tk
from SlidersTab import SlidersTabUI

class ControlPannel():
    def __init__(self, frame_cp, arduino_msg, config):
        self.frame_cp = frame_cp
        self.arduino_msg = arduino_msg
        self.config = config
        self.lock_gripper = tk.BooleanVar()
        self.lock_gripper.set(False)
        self.enable_collision = tk.BooleanVar()
        self.enable_collision.set(False)
        self.cmd_txt = tk.StringVar()

        self.createSerialCanvas()
        self.createButton()
        self.sliderCreator()
        self.cmdLine()

    def createSerialCanvas(self):
        self.frame_cp.columnconfigure(1, weight=1)
        self.frame_cp.rowconfigure(0, weight=1)
        self.dataCanvas = tk.Canvas(self.frame_cp, bg='black')
        self.dataCanvas.grid(row=0, column=1, rowspan=3, sticky=tk.NSEW)

        vsb = tk.Scrollbar(self.frame_cp, orient='vertical', command=self.dataCanvas.yview)
        vsb.grid(row=0, column=2, rowspan=3, sticky=tk.NS)

        self.dataCanvas.config(yscrollcommand=vsb.set, scrollregion=self.dataCanvas.bbox('all'))
        self.dataCanvas.yview_moveto('1.0')

        self.dataFrame = tk.Frame(self.dataCanvas, bg='black')
        self.dataCanvas.create_window((10, 0), window=self.dataFrame, anchor='nw')

    async def sliderCreator_(self, interval):
        self.frame_cp.columnconfigure(0, weight=1)
        self.frame_cp.rowconfigure(0, weight=1)
        self.sliders_frame = tk.Frame(self.frame_cp)
        self.sliders_frame.grid(column=0, row=0, sticky=tk.EW)
        self.sliders_tab_ui = SlidersTabUI(self.sliders_frame, self.config, self.arduino_msg)
        self.sliders_tab_ui.send_values_buttom()
        await asyncio.sleep(interval, True)

    def sliderCreator(self):
        self.frame_cp.columnconfigure(0, weight=1)
        self.frame_cp.rowconfigure(0, weight=1)
        self.sliders_frame = tk.Frame(self.frame_cp)
        self.sliders_frame.grid(column=0, row=0, sticky=tk.EW)
        self.sliders_tab_ui = SlidersTabUI(self.sliders_frame, self.config, self.arduino_msg)
        #self.sliders_tab_ui.send_values_buttom()

    def prind_cmd(self, event):
        msg = self.cmd_txt.get()
        print(msg)
        self.arduino_msg.sendToArduino(msg)
        self.cmd_entry.delete(0, 'end')

    def cmdLine(self):
        self.frame_cp.columnconfigure(0, weight=1)
        self.frame_cp.rowconfigure(2, weight=1)
        self.cmd_frame = tk.Frame(self.frame_cp)
        self.cmd_frame.grid(column=0, row=2)
        tk.Label(self.cmd_frame, text='Cmd').grid(row=0, column=0, sticky='ns')
        self.cmd_entry = tk.Entry(self.cmd_frame, width=40, textvariable=self.cmd_txt, takefocus=False)
        self.cmd_entry.bind('<Return>', self.prind_cmd)
        self.cmd_entry.grid(row=0, column=1, sticky='ns')


    async def serialOutput(self, interval):
        while True:
            recentPacketString = self.arduino_msg.checkSerialPort()
            if not (recentPacketString==self.arduino_msg.no_msg):
                tk.Label(self.dataFrame, text=recentPacketString, bg='black', fg='white').pack()
                self.dataCanvas.config(scrollregion=self.dataCanvas.bbox("all"))
                self.dataCanvas.yview_moveto('1.0')
            await asyncio.sleep(interval, True)

    def runRandom(self):
        self.sliders_tab_ui.sliders_set_rand_values()
        # self.arduino_msg.sendToArduino('runrnd:')
        print('runrnd:')

    def runSpan(self):
        self.sliders_tab_ui.sliders_set_span_values()
        # self.arduino_msg.sendToArduino('runs:')
        print('runs:')

    def runHome(self):
        self.sliders_tab_ui.sliders_set_home_values()
        # self.arduino_msg.sendToArduino('runh:')
        print('runh:')

    def runSqueeze(self):
        self.sliders_tab_ui.sliders_set_squeeze_values()
        # self.arduino_msg.sendToArduino('runsq:')
        print('runsq:')

    def lockGripper(self):
        print('lockGripper: ', self.lock_gripper.get())
        self.sliders_tab_ui.sliders_set_lock_gripper(self.lock_gripper.get())

    def enableCollision(self):
        print('enableCollision: ', self.enable_collision.get())
        self.sliders_tab_ui.sliders_set_collision_flg(self.enable_collision.get())

    def runMin(self):
        self.sliders_tab_ui.sliders_set_min_values()
        # self.arduino_msg.sendToArduino('runmin:')
        print('run min:')

    def runMax(self):
        self.sliders_tab_ui.sliders_set_max_values()
        # self.arduino_msg.sendToArduino('runmax:')
        print('run max:')

    def createButton(self):
        self.frame_cp.columnconfigure(0, weight=1)
        self.frame_cp.rowconfigure(1, weight=1)
        self.buttons_frame = tk.Frame(self.frame_cp)
        self.buttons_frame.grid(column=0, row=1)

        rndButton = tk.Button(self.buttons_frame, text='Random', width=20, height=5, bd='10', command=self.runRandom)
        rndButton.grid(row=0, column=0, sticky='ns')
        spanButton = tk.Button(self.buttons_frame, text='Span', width=20, height=5, bd='10', command=self.runSpan)
        spanButton.grid(row=1, column=0, sticky='ns')
        homeButton = tk.Button(self.buttons_frame, text='Home', width=20, height=5, bd='10', command=self.runHome)
        homeButton.grid(row=0, column=1, sticky='ns')
        squeezeButton = tk.Button(self.buttons_frame, text='Squeeze', width=20, height=5, bd='10', command=self.runSqueeze)
        squeezeButton.grid(row=1, column=1, sticky='ns')

        spanButton = tk.Button(self.buttons_frame, text='Min', width=20, height=5, bd='10', command=self.runMin)
        spanButton.grid(row=1, column=2, sticky='ns')
        homeButton = tk.Button(self.buttons_frame, text='Max', width=20, height=5, bd='10', command=self.runMax)
        homeButton.grid(row=0, column=2, sticky='ns')

        collisionButton = tk.Checkbutton(self.buttons_frame, text='Enable Collision', width=20, height=5, bd='10', command=self.enableCollision,
                                         variable=self.enable_collision, onvalue=True, offvalue=False)
        collisionButton.grid(row=1, column=3, sticky='ns')
        gripperButton = tk.Checkbutton(self.buttons_frame, text='Lock Gripper', width=20, height=5, bd='10', command=self.lockGripper,
                                       variable=self.lock_gripper, onvalue=True, offvalue=False)
        gripperButton.grid(row=0, column=3, sticky='ns')
