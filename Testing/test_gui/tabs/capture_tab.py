import asyncio
import pandas as pd
import io
import time
import os
import traceback
from datetime import datetime
from nicegui import ui

from test_gui import state
from test_gui.ui_registry import UI
from test_gui.components.chart_card import ChartCard
from .analysis_tab import update_time_slider_limits

BATTERY_SCALE = 0.1
CURRENT_SCALE = 0.01
PWM_SCALE = 1 / 327.67

def update_master_stream():
    if state.daq is None or not state.is_running: return
    if state.start_time is None: state.start_time = time.time()
    
    elapsed = round(time.time() - state.start_time, 2)
    row = {'Seconds': elapsed}
    
    snap = state.daq.telemetry 
    row['volt1'] = snap.battery_voltage_1 * BATTERY_SCALE
    row['volt2'] = snap.battery_voltage_2 * BATTERY_SCALE
    
    for m in state.MOTORS:
        m_id = m['id']
        motor_data = getattr(snap, m_id) 
        row[f"{m_id}_pwm"] = motor_data.pwm * PWM_SCALE
        row[f"{m_id}_rpm"] = motor_data.velocity
        row[f"{m_id}_curr"] = motor_data.current * CURRENT_SCALE
        row[f"{m_id}_enc"] = motor_data.encoder_count
        
    state.global_history.append(row)
    display_history = state.global_history[-state.MAX_LIVE_POINTS:]
    
    # --- The JS is gone! Native Python batch updating ---
    if UI.master_chart_card:
        data_updates = {}
        for v_id in ['volt1', 'volt2']:
            data_updates[f'live_{v_id}'] = [[r['Seconds'], r[v_id]] for r in display_history]
        
        for s in state.MOTOR_SENSORS:
            for m in state.MOTORS:
                data_updates[f'live_{s["id"]}_{m["id"]}'] = [[r['Seconds'], r[f"{m['id']}_{s['id']}"]] for r in display_history]
                
        UI.master_chart_card.batch_update_series_data(data_updates, redraw=True)
    
    if state.using_live_for_A:
        update_time_slider_limits()

async def run_test_engine(profile_filename, client):
    # (Unchanged... identical to previous implementation)
    try:
        test_start_seconds = time.time() - state.start_time if state.start_time else 0
        await state.engine.execute_profile(profile_filename)
        df = pd.DataFrame(state.global_history)
        
        if not df.empty:
            df = df[df['Seconds'] >= test_start_seconds]
            final_report = state.engine.evaluate_current_profile(df)
            with client:
                if final_report['passed']: ui.notify("✅ Test Passed!", type='positive')
                else: ui.notify("❌ Test Failed! Check console.", type='negative')
        else:
            with client: ui.notify("Cannot evaluate: No data was captured.", type='warning')

        if state.is_running:
            with client: toggle_master()
    except asyncio.CancelledError:
        await state.engine.stop()
        raise
    except Exception as e:
        traceback.print_exc() 
        if state.is_running:
            with client: toggle_master()
          
def toggle_master():    
    state.is_running = not state.is_running
    if UI.btn_start:
        UI.btn_start.text = 'Stop Capture' if state.is_running else 'Start Capture'
        UI.btn_start.props(f'color={"red" if state.is_running else "green"} icon={"stop" if state.is_running else "play_arrow"}')
    
    if state.is_running:
        if state.global_history: reset_master_live(show_notify=False)
        if hasattr(state.daq, 'reset_clock'): state.daq.reset_clock() # type: ignore
        state.start_time = time.time() 
        client = ui.context.client
        state.engine_task = asyncio.create_task(run_test_engine(UI.capture_mode.value if UI.capture_mode else None, client))
    else:
        if state.engine_task is not None and not state.engine_task.done(): state.engine_task.cancel()

def reset_master_live(show_notify=True):
    state.start_time = None
    state.global_history.clear()
    
    if UI.master_chart_card:
        empty_data = {f'live_{v_id}': [] for v_id in ['volt1', 'volt2']}
        for s in state.MOTOR_SENSORS:
            for m in state.MOTORS: empty_data[f'live_{s["id"]}_{m["id"]}'] = []
        UI.master_chart_card.batch_update_series_data(empty_data, redraw=True)
        
    update_time_slider_limits()
    if show_notify: ui.notify('Live data reset')

def download_master_csv():
    if not state.global_history: return ui.notify('No data to save!', type='warning')
    df = pd.DataFrame(state.global_history)
    cols = ['Seconds', 'volt1', 'volt2'] + [f"{m['id']}_{s['id']}" for m in state.MOTORS for s in state.MOTOR_SENSORS]
    csv_content = df[[c for c in cols if c in df.columns]].to_csv(index=False).encode('utf-8')
    ui.download(csv_content, filename=f'robot_run_{datetime.now().strftime("%H-%M-%S")}.csv')

def update_chart_visibility():
    if not UI.master_chart_card: return
    vis_dict = {}
    
    for v_id in ['volt1', 'volt2']:
        v_vis = bool(state.sensor_switches[v_id].value)
        vis_dict[f'live_{v_id}'] = v_vis
        vis_dict[f'ref_{v_id}'] = v_vis
    
    for s in state.MOTOR_SENSORS:
        for m in state.MOTORS:
            is_vis = bool(state.sensor_switches[s['id']].value and state.motor_switches[m['id']].value)
            vis_dict[f'live_{s["id"]}_{m["id"]}'] = is_vis
            vis_dict[f'ref_{s["id"]}_{m["id"]}'] = is_vis
            
    UI.master_chart_card.batch_set_visibility(vis_dict, redraw=True)

def toggle_log_scale(e):
    is_log = e.value
    if UI.switch_capture_panel and UI.switch_capture_panel.value != is_log: 
        UI.switch_capture_panel.value = is_log
    if UI.master_chart_card and hasattr(UI.master_chart_card, 'log_switch') and UI.master_chart_card.log_switch.value != is_log:
        UI.master_chart_card.log_switch.value = is_log
    if UI.master_chart_card:
        UI.master_chart_card.update_chart(is_log=is_log)

async def load_live_reference(e):
    try:
        content = await e.file.read()
        state.live_reference_df = pd.read_csv(io.BytesIO(content))
        
        if UI.master_chart_card:
            metrics_to_load = [
                ('volt1', 'volt1', '#f59e0b', 'Solid', bool(state.sensor_switches['volt1'].value)),
                ('volt2', 'volt2', '#d97706', 'Solid', bool(state.sensor_switches['volt2'].value))
            ]
            for s in state.MOTOR_SENSORS:
                for m in state.MOTORS:
                    metrics_to_load.append((f"{s['id']}_{m['id']}", f"{m['id']}_{s['id']}", s['color'], m['dash'], bool(state.sensor_switches[s['id']].value and state.motor_switches[m['id']].value)))
                    
            for chart_id, col_name, color, dash, is_visible in metrics_to_load:
                if col_name in state.live_reference_df.columns:
                    UI.master_chart_card.remove_series(f"ref_{chart_id}")
                    UI.master_chart_card.add_series({
                        "id": f"ref_{chart_id}", "name": f"Ref: {col_name}", 
                        "data": state.live_reference_df[['Seconds', col_name]].dropna().values.tolist(),
                        "color": color, "dashStyle": dash, "opacity": 0.4, "visible": is_visible, "marker": {"enabled": False}
                    })
            UI.master_chart_card.redraw() # triggers redraw

        if UI.ref_pill_label: UI.ref_pill_label.set_text(e.file.name)
        if UI.ref_pill: UI.ref_pill.classes(remove='hidden')
        if UI.btn_load_ref: UI.btn_load_ref.classes('hidden')
        e.sender.reset() 
        ui.notify('Reference loaded', type='positive')
    except Exception as ex:
        ui.notify(f'Error: {ex}', type='negative')

def unload_live_reference():
    state.live_reference_df = pd.DataFrame() 
    if UI.master_chart_card:
        ids = ['volt1', 'volt2'] + [f"{s['id']}_{m['id']}" for s in state.MOTOR_SENSORS for m in state.MOTORS]
        for cid in ids: UI.master_chart_card.remove_series(f"ref_{cid}")
        UI.master_chart_card.update_chart()

    if UI.ref_pill: UI.ref_pill.classes('hidden')
    if UI.btn_load_ref: UI.btn_load_ref.classes(remove='hidden')
    ui.notify('Reference unloaded')

def build_capture_tab():
    with ui.card().classes('w-full mb-4 p-4 bg-slate-50 shadow border border-slate-200 flex-nowrap'):
        with ui.row().classes('w-full justify-between items-center'):
            with ui.row().classes('gap-4 items-center'):
                available_tests = state.engine.get_available_tests()
                first_test_key = list(available_tests.keys())[0] if available_tests else None
                with ui.column().classes('gap-1'):
                    UI.capture_mode = ui.select(
                        options={key: value['name'] for key, value in available_tests.items()},
                        value=first_test_key, label='Test Profile',
                        on_change=lambda e: desc_label.set_text(available_tests[e.value]['description'])
                    ).classes('w-48').props('dense outlined bg-color=white')
                    desc_label = ui.label(available_tests[first_test_key]['description']).classes('text-xs text-gray-500 italic max-w-[192px] leading-tight')
                
                UI.btn_start = ui.button('Start Capture', on_click=toggle_master).props('color=green icon=play_arrow')
                ui.button('Reset', on_click=reset_master_live).props('color=grey icon=refresh outline')
            
            with ui.row().classes('gap-4 items-center'):
                ui.button('Save Run CSV', icon='save', on_click=download_master_csv).props('color=blue')
                ref_uploader = ui.upload(auto_upload=True, on_upload=load_live_reference).props('accept=".csv"').classes('hidden')
                UI.btn_load_ref = ui.button('Load Ref Ghost', icon='upload', on_click=lambda: ref_uploader.run_method('pickFiles')).props('color=slate outline')
                
                with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-2 hidden flex-nowrap') as UI.ref_pill:
                    ui.icon('description', size='sm')
                    UI.ref_pill_label = ui.label('filename.csv').classes('text-sm font-semibold truncate max-w-[200px]')
                    ui.button(icon='close', on_click=unload_live_reference).props('flat round size=sm padding=none text-color=slate')
    
    with ui.row().classes('w-full flex-grow min-h-0 gap-6 flex-nowrap min-w-0'):
        with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
            ui.label('Chart Settings').classes('text-lg font-bold text-gray-800 mb-2')
            UI.switch_capture_panel = ui.switch('Logarithmic Y-Axis', on_change=toggle_log_scale).classes('mb-4 font-semibold text-blue-600')
            ui.separator().classes('mb-4')
            
            with ui.card().classes('w-full p-4 bg-white shadow-sm border'):
                ui.label('Active Motors').classes('text-lg font-bold text-gray-800 mb-2')
                with ui.column().classes('w-full gap-2'):
                    for m in state.MOTORS: state.motor_switches[m['id']] = ui.switch(m['name'], value=True, on_change=update_chart_visibility).classes('w-full')

            with ui.card().classes('w-full p-4 bg-white shadow-sm border'):
                ui.label('Active Sensors').classes('text-lg font-bold text-gray-800 mb-2')
                with ui.column().classes('w-full gap-2'):
                    for s in state.MOTOR_SENSORS: state.sensor_switches[s['id']] = ui.switch(s['name'], value=True, on_change=update_chart_visibility).classes('w-full')
                    ui.separator().classes('my-2 w-full')
                    state.sensor_switches['volt1'] = ui.switch("Bus Voltage 1 (V)", value=True, on_change=update_chart_visibility).classes('w-full')
                    state.sensor_switches['volt2'] = ui.switch("Bus Voltage 2 (V)", value=True, on_change=update_chart_visibility).classes('w-full')

        with ui.column().classes('w-3/4 flex-grow h-full min-w-0'):
            series_list = []
            for s in state.MOTOR_SENSORS:
                for m in state.MOTORS: series_list.append({'id': f"live_{s['id']}_{m['id']}", 'name': f"{m['name']} {s['name']}", 'data': [], 'color': s['color'], 'dashStyle': m['dash'], 'marker': {'enabled': False}})
            series_list.append({'id': 'live_volt1', 'name': 'Bus Voltage 1 (V)', 'data': [], 'color': '#f59e0b', 'marker': {'enabled': False}})
            series_list.append({'id': 'live_volt2', 'name': 'Bus Voltage 2 (V)', 'data': [], 'color': '#d97706', 'marker': {'enabled': False}})
            
            # --- Instantiating our custom Component ---
            UI.master_chart_card = ChartCard(
                options={
                    'chart': {'type': 'line', 'animation': False}, 'title': {'text': 'Data Capture'}, 
                    'xAxis': {'title': {'text': 'Seconds'}}, 'yAxis': {'type': 'linear'},
                    'tooltip': {'shared': True, 'crosshairs': True}, 'series': series_list
                },
                height_px=400,
                show_log_toggle=True,
                on_log_toggle=toggle_log_scale
            ).classes('h-full')