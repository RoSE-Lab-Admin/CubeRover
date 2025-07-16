#import ui packages
from nicegui import ui

#import ros packages
import rclpy
#from testhandle_node import TestConsole
from plot_node import MultiPlot

#import python packages
import threading
import datetime
import numpy as np
import time

#create a lock for trials
trial_lock = threading.Lock()


#startup ros and all ros nodes
# def test_node_startup():
#     global TestHandle
#     TestHandle = TestConsole()
#     try:
#         rclpy.spin(TestHandle)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         TestHandle.destroy_node()

def plot_helper_startup():
    global Plotter
    Plotter = MultiPlot()
    time.sleep(5)
    rclpy.spin(Plotter)

#setup gui web interface
def setup_gui():
    #wraps gui cards horrizontally
    with ui.row().classes('flex-wrap items-start'):
        # #Parameter set widget
        # with ui.card():
        #     trialname = ''
        #     ui.label('Testing Parameters')
        #     linvel = ui.number(label='Linear velocity (cm/s)', value=0.00, format='%i')
        #     turnrad = ui.number(label='Trial turning radius (cm) (enter >1E6 for straight, 0 for turn in place): ', value=0.00, format='%.2f')
        #     slope = ui.number(label='Trial regolith slope (deg) ', value=0.00, format='%.2f')
        #     trialnum = ui.number(label='Trial number ', value=1, format='%i')

        #     def set_params():
        #         ui.notify("set trial parameters")
        #         trialname = f"Trial_{int(linvel.value)}cm_{turnrad.value}radius_{slope.value}slope_Trial{int(trialnum.value)}_{datetime.datetime.now().strftime("%m%d%Y_%H_%M_%S")}"
        #         TestHandle.set_params(linvel.value, turnrad.value, trialname)

        #     def start_trial():
        #         ui.notify(f"starting trial: {trialname}")
        #         with trial_lock:
        #             threading.Thread(target=TestHandle.start_test, daemon=True).start()

        #     ui.button('Set parameters', on_click=set_params)
        #     ui.button("Start trial", on_click=start_trial)

        #Plot widgets (init to null values)
        with ui.grid(rows=2, columns=3):
            #motor encoder counts
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax1 = fig.gca()
                    ax1.set_title("Motor Encoder Position")
                    ax1.plot([],[],'-')
            #motor encoder velocity
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax2 = fig.gca()
                    ax2.set_title("Motor Encoder Velocity")
                    ax2.plot([],[],'-')
            #Roboclaw voltage
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax3 = fig.gca()
                    ax3.set_title("Roboclaw voltage")
                    ax3.plot([],[],'-')
            #Motor current draw
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax4 = fig.gca()
                    ax4.set_title("Roboclaw amp")
                    ax4.plot([],[],'-')
            #IMU acceleration
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax5 = fig.gca()
                    ax5.set_title("IMU acceleration")
                    ax5.plot([],[],'-')
            #MOCAP Data
            with ui.card():
                with ui.matplotlib(figsize=(3,2)).figure as fig:
                    ax6 = fig.gca()
                    ax6.set_title("MOCAP position")
                    ax6.plot([],[],'-')
            
            def update_plots():
                if Plotter.motor_data['time']:
                    x = np.array(Plotter.motor_data['time'])

                    ax1.clear()
                    for i in range(4):     
                        y = np.array(Plotter.motor_data['pos'][i])
                        ax1.plot(x,y,'-', label=f"Enc pos: m{i + 1}")
                    ax1.legend()
                    ax1.figure.canvas.draw()

                    ax2.clear()
                    for i in range(4):
                        y = np.array(Plotter.motor_data['vel'][i])
                        ax2.plot(x,y,'-', label=f"Enc vel: m{i + 1}")
                    ax2.legend()
                    ax2.figure.canvas.draw()

                    ax3.clear()
                    for i in range(4):
                        y = np.array(Plotter.motor_data['volt'][i])
                        ax3.plot(x,y,'-', label=f"Roboclaw voltage: r{i + 1}")
                    ax3.legend()
                    ax3.figure.canvas.draw()

                    ax4.clear()
                    for i in range(4):
                        y = np.array(Plotter.motor_data['amp'][i])
                        ax4.plot(x,y,'-', label=f"Motor current: m{i + 1}")
                    ax4.legend()
                    ax4.figure.canvas.draw()


                if Plotter.imu_data["time"]:
                    x = np.array(Plotter.imu_data['time'])

                    ax5.clear()
                    for i in range(3):
                        y = np.array(Plotter.imu_data['lin_accel'][i])
                        ax5.plot(x,y,'-', label=f"IMU acceleration: axis {chr(ord('X') + i)}")
                    ax5.legend()
                    ax5.figure.canvas.draw()

                mocap = Plotter.get_mocap()
                if mocap["time"]:
                        ax6.clear()
                        x = np.array(mocap.get('time'))
                        # ax6.set_xlim(Plotter.mocap_data.get("time")[0], Plotter.mocap_data.get("time")[-1])
                        for i in range(3):
                            y = np.array(mocap['pose'][i])
                            print(f"{len(y)}; {np.sum(np.isnan(y))}; {y}")
                            ax6.plot(x,y,'-', label=f"Mocap pose: axis {chr(ord('X') + i)}")
                            ui.notify(f"plotted {chr(ord('X') + i)}")
                        else:
                            ui.notify("no data")
                        ax6.legend()
                        ax6.figure.canvas.draw()
                ui.notify("plot update")
            ui.timer(0.5,update_plots)

def main():
    try:
        rclpy.init()
        #threading.Thread(target=test_node_startup, daemon=True).start()
        plotthread = threading.Thread(target=plot_helper_startup, daemon=True)
        plotthread.start()
        setup_gui()
        ui.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        plotthread.close()
        print("why")
        pass
    finally:
        rclpy.shutdown()

if __name__ in {"__main__", "__mp_main__"}:
    main()