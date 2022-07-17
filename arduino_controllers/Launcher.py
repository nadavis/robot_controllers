import asyncio
from ArduinoMsg import ArduinoMsg
import tkinter as tk
from tkinter import ttk
from ControlPanel import ControlPannel
from ParametersManager import ParamsManger
import yaml

class App(tk.Tk):

    def __init__(self, loop, interval=1/120):
        super().__init__()
        self.loop = loop
        self.protocol("WM_DELETE_WINDOW", self.close)

        self.config = []
        with open("config.yml", 'r') as conf_file:
            self.config = yaml.load(conf_file, Loader=yaml.FullLoader)

        self.win_width = 1000
        self.win_height = 1000
        self.resizable(True, True)
        self.title('Arm controller')
        notebook = ttk.Notebook(self)
        self.frame_cp = tk.Frame(notebook, height=self.win_height, width=self.win_width)
        self.frame_pm = tk.Frame(notebook, height=self.win_height, width=self.win_width)
        notebook.add(self.frame_cp, text='Cmd')
        notebook.add(self.frame_pm, text='Setup')
        notebook.columnconfigure(0, weight=1)
        notebook.rowconfigure(0, weight=1)
        notebook.pack(expand=1, fill="both")

        time_msg_interval = self.config['video_stream']['time_msg_interval'] #'1.0
        msg_buff = self.config['arduino']['msg_buff'] #'9600
        #port = '/dev/cu.usbmodem1433201'
        port = self.config['arduino']['port'] #'/dev/cu.usbmodem141201'
        sleep_msg = 0.001
        enable_msg = self.config['arduino']['enable_msg']

        self.arduino_msg = None
        if(enable_msg):
            self.arduino_msg = ArduinoMsg(msg_buff, port, time_msg_interval, sleep_msg)
        self.cp = ControlPannel(self.frame_cp, self.arduino_msg, self.config)
        self.pm = ParamsManger(self.frame_pm, self.arduino_msg, self.config)

        self.tasks = []
        #self.tasks.append(loop.create_task(self.cp.sliderCreator(interval)))
        self.tasks.append(loop.create_task(self.cp.serialOutput(interval)))
        self.tasks.append(loop.create_task(self.updater(interval)))

    async def updater(self, interval):
        while True:
            self.update()
            #self.serialOutput()
            await asyncio.sleep(interval)

    def close(self):
        for task in self.tasks:
            task.cancel()
        self.loop.stop()
        self.destroy()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    app = App(loop)
    loop.run_forever()
    loop.close()