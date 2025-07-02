#import ui packages
from nicegui import ui

#import ros packages
import rclpy
from testhandle_node import TestConsole

#import python packages
import threading
import datetime
import numpy as np
import random

#create a lock for trials
trial_lock = threading.Lock()

x = np.linspace(0.0,10.0,50)
y = []
for i in range(50):
    y.append(random.randint(0,15))

#startup ros and all ros nodes
def test_node_startup():
    rclpy.init()
    global TestHandle
    TestHandle = TestConsole()
    rclpy.spin(TestHandle)

#setup gui web interface
def setup_gui():
    trialname = ''
    #wraps gui cards horrizontally
    with ui.row().classes('flex-wrap items-start'):
        #Parameter set widget
        with ui.card():
            ui.label('Testing Parameters')
            linvel = ui.number(label='Linear velocity (cm/s)', value=0.00, format='%i')
            turnrad = ui.number(label='Trial turning radius (cm) (enter >1E6 for straight, 0 for turn in place): ', value=0.00, format='%.2f')
            slope = ui.number(label='Trial regolith slope (deg) ', value=0.00, format='%.2f')
            trialnum = ui.number(label='Trial number ', value=1, format='%i')

            def set_params():
                nonlocal trialname
                ui.notify("set trial parameters")
                trialname = f"Trial_{int(linvel.value)}cm_{turnrad.value}radius_{slope.value}slope_Trial{int(trialnum.value)}_{datetime.datetime.now().strftime("%m%d%Y_%H_%M_%S")}"
                TestHandle.set_params(linvel.value, turnrad.value, trialname)

            def start_trial():
                ui.notify(f"starting trial: {trialname}")
                with trial_lock:
                    threading.Thread(target=TestHandle.start_test, daemon=True).start()

            ui.button('Set parameters', on_click=set_params)
            ui.button("Start trial", on_click=start_trial)

        #Plot widgets
        with ui.grid(rows=2, columns=3):
            #motor encoder counts
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("Motor Encoder Position")
                    ax.plot(x,y,'-')
            #motor encoder velocity
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("Motor Encoder Velocity")
                    ax.plot(x,y,'-')
            #Roboclaw voltage
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("Roboclaw voltage")
                    ax.plot(x,y,'-')
            #Motor current draw
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("Roboclaw voltage")
                    ax.plot(x,y,'-')
            #IMU acceleration
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("IMU acceleration")
                    ax.plot(x,y,'-')
            #MOCAP Data
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax = fig.gca()
                    ax.set_title("MOCAP position")
                    ax.plot(x,y,'-')

def main():
    #threading.Thread(target=test_node_startup, daemon=True).start()
    setup_gui()
    ui.run(host='0.0.0.0', port=5000)

if __name__ in {"__main__", "__mp_main__"}:
    main()