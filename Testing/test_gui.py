import io
import time
import random
import pandas as pd
import numpy as np
from datetime import datetime
from nicegui import app, ui
from pathlib import Path
import asyncio
import traceback # Add this to your imports at the top

from test_engine.test_engine import TestEngine
from PySerial.mock_teensy_telemetry import MockTeensyTelemetryReader

# ==========================================
# 1. Global App State & Configuration
# ==========================================
is_running = False
start_time = None
MAX_LIVE_POINTS = 600

global_history = [] 
live_reference_df = pd.DataFrame()
analysis_df_A = pd.DataFrame()
analysis_df_B = pd.DataFrame()
using_live_for_A = False

SENSORS = [
    {"id": "pwm", "name": "Motor PWM (%)", "color": "#ef4444", "base_val": 0.1},
    {"id": "rpm", "name": "Motor Speed (RPM)", "color": "#10b981", "base_val": 0.1},
    {"id": "curr", "name": "Motor Current (A)", "color": "#3b82f6", "base_val": 0.5},
    {"id": "volt", "name": "Bus Voltage (V)", "color": "#f59e0b", "base_val": 24.0}
]
sensor_names = [s['name'] for s in SENSORS]
sensor_switches = {}

current_dir = Path(__file__).parent.resolve()
engine = TestEngine(hardware_interface=None, base_path=current_dir)

# Initialize the DAQ (Using Mock for now)
# (When ready for real hardware, just change the import and the port!)
daq = MockTeensyTelemetryReader("COM3")
daq.start() # Start the background polling thread

def calculate_mape(df_A, df_B, metric):
    try:
        t_A = df_A['Seconds'].values
        val_A = df_A[metric].values
        t_B = df_B['Seconds'].values
        val_B = df_B[metric].values
        
        if len(t_A) == 0 or len(t_B) == 0: return "N/A"
        
        interp_B = np.interp(t_A, t_B, val_B)
        safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A) 
        
        mape = np.mean(np.abs((val_A - interp_B) / safe_A)) * 100
        return f"{mape:.1f}%"
    except Exception:
        return "N/A"

# ==========================================
# 2. Live Capture Handlers
# ==========================================
def update_master_stream():
    global start_time
    if not is_running: return
    if start_time is None: start_time = time.time()
    
    elapsed = round(time.time() - start_time, 2)
    row = {'Seconds': elapsed}
    
    # Grab a thread-safe snapshot from the DAQ
    snap = daq.telemetry
    
    BATTERY_SCALE = 0.1   # Roboclaw reports in 10mV units
    CURRENT_SCALE = 0.01  # Roboclaw reports in 10mA units
    PWM_SCALE = 1 / 327.67 # See wheel.hpp for details
    
    # Map the telemetry to the UI sensors. 
    # Convert hardware integer units (10mA, 10mV) back into real-world Floats.
    row['Motor PWM (%)'] = snap.fl.pwm * PWM_SCALE
    row['Motor Speed (RPM)'] = snap.fl.velocity
    row['Motor Current (A)'] = snap.fl.current * CURRENT_SCALE      # 10mA units -> Amps
    row['Bus Voltage (V)'] = snap.battery_voltage_1 * BATTERY_SCALE # 10mV units -> Volts
    
    global_history.append(row)
    display_history = global_history[-MAX_LIVE_POINTS:]
    
    js_commands = [f'const chart = getElement({master_chart.id}).chart;']
    for s in SENSORS:
        sensor_data = [[r['Seconds'], r[s['name']]] for r in display_history]
        js_commands.append(f'chart.get("live_{s["id"]}").setData({sensor_data}, false, false, false);')
    
    js_commands.append('chart.redraw(false);')
    ui.run_javascript('\n'.join(js_commands))
    
    if using_live_for_A:
        update_time_slider_limits()

async def run_test_engine(profile_filename, client):
    global is_running, engine, start_time, global_history
    
    try:
        test_start_seconds = time.time() - start_time if start_time else 0
        
        # 1. Tell the engine to run the file
        await engine.execute_profile(profile_filename)
        
        # 2. Get the data
        df = pd.DataFrame(global_history)
        
        if not df.empty:
            df = df[df['Seconds'] >= test_start_seconds]
            
            # 3. Ask the engine to evaluate it
            final_report = engine.evaluate_current_profile(df)
            
            # 4. Update the UI
            with client:
                if final_report['passed']:
                    ui.notify("✅ Test Passed!", type='positive', position='top', timeout=5000)
                else:
                    ui.notify("❌ Test Failed! Check console.", type='negative', position='top', timeout=5000)
            
            # Print Report
            print("\n=== TEST REPORT ===")
            for res in final_report.get('results', []):
                status = "PASS" if res['passed'] else "FAIL"
                print(f"[{status}] {res['type'].upper()}: {res['details']}")
            print("===================\n")
        else:
            with client:
                ui.notify("Cannot evaluate: No data was captured.", type='warning')

        if is_running:
            with client:
                toggle_master()
                
    except asyncio.CancelledError:
        print("--- EMERGENCY STOP TRIGGERED ---")
        await engine.stop()
        raise
    except Exception as e:
        print(f"\n❌ FATAL ENGINE ERROR: {e}")
        traceback.print_exc() 
        if is_running:
            with client:
                toggle_master()

engine_task = None            
def toggle_master():
    global is_running, engine_task, start_time, global_history, daq
    
    is_running = not is_running
    btn_start.text = 'Stop Capture' if is_running else 'Start Capture'
    btn_start.props(f'color={"red" if is_running else "green"} icon={"stop" if is_running else "play_arrow"}')
    
    if is_running:
        if global_history:
            reset_master_live(show_notify=False)
        
        if hasattr(daq, 'reset_clock'):
            daq.reset_clock()
        
        selected_profile = capture_mode.value 
        start_time = time.time()  # Ensure start_time is set!
        client = ui.context.client
        engine_task = asyncio.create_task(run_test_engine(selected_profile, client))
    else:
        if engine_task is not None and not engine_task.done():
            engine_task.cancel()

def reset_master_live(show_notify=True):
    global start_time
    start_time = None
    global_history.clear()
    
    js_commands = [f'const chart = getElement({master_chart.id}).chart;']
    for s in SENSORS:
        js_commands.append(f'chart.get("live_{s["id"]}").setData([], false, false, false);')
    js_commands.append('chart.redraw();')
    ui.run_javascript('\n'.join(js_commands))
    update_time_slider_limits()
    
    if show_notify:
        ui.notify('Live data reset')

def download_master_csv():
    if not global_history:
        ui.notify('No data to save!', type='warning')
        return
    df = pd.DataFrame(global_history)
    df = df[['Seconds'] + [s['name'] for s in SENSORS]]
    csv_content = df.to_csv(index=False).encode('utf-8')
    ui.download(csv_content, filename=f'robot_run_{datetime.now().strftime("%H-%M-%S")}.csv')

def update_visibility(sensor_id, is_visible):
    visible_str = str(is_visible).lower()
    ui.run_javascript(f'''
        const chart = getElement({master_chart.id}).chart;
        if (chart.get("live_{sensor_id}")) chart.get("live_{sensor_id}").setVisible({visible_str}, false);
        if (chart.get("ref_{sensor_id}")) chart.get("ref_{sensor_id}").setVisible({visible_str}, false);
        chart.redraw();
    ''')

def toggle_log_scale(e):
    # Sync the two switches so they stay at the same value
    switch_capture_panel.value = e.value
    switch_capture_chart.value = e.value
    
    axis_type = "logarithmic" if e.value else "linear"
    ui.run_javascript(f'getElement({master_chart.id}).chart.yAxis[0].update({{type: "{axis_type}"}});')

async def load_live_reference(e):
    global live_reference_df
    try:
        live_reference_df = pd.read_csv(io.BytesIO(await e.file.read()))
        js_commands = [f'const chart = getElement({master_chart.id}).chart;']
        
        for s in SENSORS:
            if s['name'] in live_reference_df.columns:
                data = live_reference_df[['Seconds', s['name']]].dropna().values.tolist()
                is_visible = str(sensor_switches[s['id']].value).lower()
                var_name = f"old_ref_{s['id']}"
                js_commands.append(f'const {var_name} = chart.get("ref_{s["id"]}"); if ({var_name}) {var_name}.remove(false);')
                
                js_commands.append(f'''
                    chart.addSeries({{
                        id: "ref_{s['id']}", name: "Ref: {s['name']}", data: {data},
                        color: "{s['color']}", dashStyle: 'ShortDash', opacity: 0.5, visible: {is_visible}
                    }}, false);
                ''')
        js_commands.append('chart.redraw();')
        ui.run_javascript('\n'.join(js_commands))
        
        # --- UI TOGGLE LOGIC ---
        ref_pill_label.set_text(e.file.name)
        ref_pill.classes(remove='hidden')
        btn_load_ref.classes('hidden')
        e.sender.reset() 
        ui.notify('Reference trace loaded for Live Capture')
    except Exception as ex:
        ui.notify(f'Error: {ex}', type='negative')
        
def unload_live_reference():
    global live_reference_df
    live_reference_df = pd.DataFrame() # Clear the dataframe
    
    # Remove all reference series from the chart
    js_commands = [f'const chart = getElement({master_chart.id}).chart;']
    for s in SENSORS:
        var_name = f"old_ref_{s['id']}"
        js_commands.append(f'const {var_name} = chart.get("ref_{s["id"]}"); if ({var_name}) {var_name}.remove(false);')
    js_commands.append('chart.redraw();')
    ui.run_javascript('\n'.join(js_commands))
    
    # --- UI TOGGLE LOGIC ---
    ref_pill.classes('hidden')
    btn_load_ref.classes(remove='hidden')
    ui.notify('Reference trace unloaded')

# ==========================================
# 3. Analysis Tab Handlers
# ==========================================
def update_time_slider_limits():
    max_t = 0.0
    if using_live_for_A and global_history: 
        max_t = max(max_t, global_history[-1]['Seconds'])
    elif not using_live_for_A and not analysis_df_A.empty: 
        max_t = max(max_t, analysis_df_A['Seconds'].max())
        
    if not analysis_df_B.empty: 
        max_t = max(max_t, analysis_df_B['Seconds'].max())
    
    max_t = max(10.0, max_t) 
    
    if time_range.max != max_t:
        time_range.max = float(max_t)
        if time_range.value['max'] >= (time_range.max - 1):
            time_range.value = {'min': time_range.value['min'], 'max': float(max_t)}
        time_range.update()

def reset_time_window():
    time_range.value = {'min': 0, 'max': float(time_range.max)}
    update_analysis_view()
    ui.notify('Time window reset to full view')

async def handle_analysis_upload(e, dataset_slot):
    global analysis_df_A, analysis_df_B, using_live_for_A
    try:
        df = pd.read_csv(io.BytesIO(await e.file.read()))
        if dataset_slot == 'A':
            analysis_df_A = df
            using_live_for_A = False
            label_A_status.set_text(e.file.name)
            label_A_status.classes(remove='text-slate-500', add='text-blue-800')
            btn_clear_A.classes(remove='hidden')
        else:
            analysis_df_B = df
            label_B_status.set_text(e.file.name)
            label_B_status.classes(remove='text-slate-500', add='text-slate-800')
            btn_clear_B.classes(remove='hidden')
        
        update_time_slider_limits()
        time_range.value = {'min': 0, 'max': time_range.max}
        update_analysis_view()
        e.sender.reset()
        ui.notify(f'Loaded to Dataset {dataset_slot}', type='positive')
    except Exception as ex:
        ui.notify(f'Error: {ex}', type='negative')

def set_analysis_A_to_live():
    global using_live_for_A
    if not global_history:
        ui.notify('No Live Capture data exists yet!', type='warning')
        return
    
    using_live_for_A = True
    label_A_status.set_text('Current Live Run')
    label_A_status.classes(remove='text-slate-500', add='text-blue-800')
    btn_clear_A.classes(remove='hidden') # Show the 'X' now so it can be cleared!
    
    update_time_slider_limits()
    time_range.value = {'min': 0, 'max': time_range.max}
    
    update_analysis_view()
    ui.notify('Dataset A set to Current Live Run')
    
def set_analysis_B_to_live_ref():
    global analysis_df_B
    if live_reference_df.empty:
        ui.notify('No Reference Ghost loaded in Live Capture!', type='warning')
        return
        
    analysis_df_B = live_reference_df.copy() # Copy to prevent accidental cross-talk
    label_B_status.set_text('Live Capture Reference')
    label_B_status.classes(remove='text-slate-500')
    btn_clear_B.classes(remove='hidden')
    
    update_time_slider_limits()
    time_range.value = {'min': 0, 'max': time_range.max}    
    
    update_analysis_view()
    ui.notify('Dataset B set to Live Capture Reference')
    
def unload_analysis_dataset(slot):
    global analysis_df_A, analysis_df_B, using_live_for_A
    if slot == 'A':
        analysis_df_A = pd.DataFrame() # Clear the dataframe
        using_live_for_A = False       # Disconnect from live history
        label_A_status.set_text('None Loaded')
        label_A_status.classes(remove='text-blue-800', add='text-slate-500')
        btn_clear_A.classes('hidden')
    else:
        analysis_df_B = pd.DataFrame()
        label_B_status.set_text('None Loaded')
        label_B_status.classes(remove='text-slate-800', add='text-slate-500')
        btn_clear_B.classes('hidden')
        
    update_time_slider_limits()
    update_analysis_view()
    ui.notify(f'Dataset {slot} unloaded')

def toggle_analysis_log_scale(e):
    axis_type = "logarithmic" if e.value else "linear"
    ui.run_javascript(f'getElement({analysis_chart.id}).chart.yAxis[0].update({{type: "{axis_type}"}});')

def update_analysis_view():
    selected_metrics = analysis_metric_select.value
    if not selected_metrics: return
    
    df_A_raw = pd.DataFrame(global_history) if using_live_for_A else analysis_df_A
    df_B_raw = analysis_df_B
    
    t_min = time_range.value['min']
    t_max = time_range.value['max']
    
    df_A = df_A_raw[(df_A_raw['Seconds'] >= t_min) & (df_A_raw['Seconds'] <= t_max)] if not df_A_raw.empty else df_A_raw
    df_B = df_B_raw[(df_B_raw['Seconds'] >= t_min) & (df_B_raw['Seconds'] <= t_max)] if not df_B_raw.empty else df_B_raw
    
    main_series = []
    delta_series = []
    dist_series = []
    stats = []
    
    y_type = 'logarithmic' if analysis_log_scale.value else 'linear'
    
    for metric_name in selected_metrics:
        color = next((s['color'] for s in SENSORS if s['name'] == metric_name), '#000000')
        mape_val = calculate_mape(df_A, df_B, metric_name) if not df_A.empty and not df_B.empty else "-"
        
        if not df_A.empty and metric_name in df_A.columns:
            main_series.append({
                'name': f'A: {metric_name}',
                'data': df_A[['Seconds', metric_name]].dropna().values.tolist(),
                'color': color, 
                'lineWidth': 2,
                'type': 'line',
                'dashStyle': 'Solid',
                'opacity': 1.0,
                'id': f'main_A_{metric_name}',
            })
            rms_A = np.sqrt(np.mean(df_A[metric_name].dropna()**2))
            # ADDED 'id' FIELD FOR UNIQUE ROW KEY
            stats.append({'id': f'{metric_name}_A', 'Metric': metric_name, 'Dataset': 'A (Solid)', 'Max': round(df_A[metric_name].max(), 2), 'Mean': round(df_A[metric_name].mean(), 2), 'RMS': round(rms_A, 2), 'Diff_vs_A': '-'})
            
        if not df_B.empty and metric_name in df_B.columns:
            main_series.append({
                'name': f'B: {metric_name}',
                'data': df_B[['Seconds', metric_name]].dropna().values.tolist(),
                'color': color, 
                'lineWidth': 2,
                'dashStyle': 'ShortDash', 
                'opacity': 0.5,
                'id': f'main_B_{metric_name}',
            })
            rms_B = np.sqrt(np.mean(df_B[metric_name].dropna()**2))
            # ADDED 'id' FIELD FOR UNIQUE ROW KEY
            stats.append({'id': f'{metric_name}_B', 'Metric': metric_name, 'Dataset': 'B (Dashed)', 'Max': round(df_B[metric_name].max(), 2), 'Mean': round(df_B[metric_name].mean(), 2), 'RMS': round(rms_B, 2), 'Diff_vs_A': mape_val})
            
        # Look for this specific block inside update_analysis_view() and replace it:
        if not df_A.empty and not df_B.empty and metric_name in df_A.columns and metric_name in df_B.columns:
            t_A = df_A['Seconds'].values
            val_A = df_A[metric_name].values
            t_B = df_B['Seconds'].values
            val_B = df_B[metric_name].values
            
            if len(t_A) > 0 and len(t_B) > 0:
                interp_B = np.interp(t_A, t_B, val_B)
                
                # --- NEW PERCENTAGE MATH ---
                # We use a safe division array to prevent divide-by-zero errors when the motor is stopped
                safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A)
                delta_vals = ((val_A - interp_B) / safe_A) * 100
                
                delta_data = [[float(t), float(d)] for t, d in zip(t_A, delta_vals)]
                delta_series.append({'name': f'Δ {metric_name} (%)', 'data': delta_data, 'color': color, 'lineWidth': 2})

    dist_metric = dist_metric_select.value
    if dist_metric:
        dist_color = next((s['color'] for s in SENSORS if s['name'] == dist_metric), '#000000')
        all_vals = []
        if not df_A.empty and dist_metric in df_A.columns: all_vals.extend(df_A[dist_metric].dropna().tolist())
        if not df_B.empty and dist_metric in df_B.columns: all_vals.extend(df_B[dist_metric].dropna().tolist())
        
        if all_vals:
            min_val, max_val = min(all_vals), max(all_vals)
            if min_val == max_val: max_val += 0.1 
            global_bins = np.linspace(min_val, max_val, 30) 
            
            if not df_A.empty and dist_metric in df_A.columns:
                counts_A, _ = np.histogram(df_A[dist_metric].dropna(), bins=global_bins)
                bin_centers = (global_bins[:-1] + global_bins[1:]) / 2
                data_A = [[float(x), int(y)] for x, y in zip(bin_centers, counts_A)]
                dist_series.append({'name': f'A: {dist_metric}', 'data': data_A, 'color': dist_color, 'opacity': 0.7})
                
            if not df_B.empty and dist_metric in df_B.columns:
                counts_B, _ = np.histogram(df_B[dist_metric].dropna(), bins=global_bins)
                bin_centers = (global_bins[:-1] + global_bins[1:]) / 2
                data_B = [[float(x), int(y)] for x, y in zip(bin_centers, counts_B)]
                dist_series.append({'name': f'B: {dist_metric}', 'data': data_B, 'color': '#64748b', 'opacity': 0.7})

    ui.run_javascript(f'''
        const chartM = getElement({analysis_chart.id}).chart;
        chartM.yAxis[0].update({{ type: "{y_type}" }}, false); 
        chartM.xAxis[0].setExtremes({t_min}, {t_max}, false);
        chartM.update({{series: {main_series}}}, true, true, false);
        
        const chartD = getElement({delta_chart.id}).chart;
        chartD.xAxis[0].setExtremes({t_min}, {t_max}, false);
        chartD.update({{series: {delta_series}}}, true, true, false);
        
        const chartH = getElement({dist_chart.id}).chart;
        chartH.update({{series: {dist_series}}}, true, true, false);
    ''')
    
    # ASSIGN AND FORCE UPDATE
    stats_table.rows = stats
    stats_table.update()

# ==========================================
# 4. Main UI Layout
# ==========================================
ui.page_title('Robotics Telemetry Hub')

# The "fixed inset-0" pins the container to the top, bottom, left, and right of the window.
# It completely bypasses any hidden framework padding.
# The "p-4" provides the inward border, and "mx-auto" keeps it centered on large ultra-wide monitors.
with ui.column().classes('fixed inset-0 p-4 w-full max-w-screen-2xl mx-auto flex-nowrap overflow-hidden'):
    
    # --- HEADER ROW (Centered Tabs) ---
    with ui.row().classes('w-full items-end border-b-2 border-slate-300 pb-1 mb-2 flex-nowrap'):
        
        # Left Section: Title
        with ui.row().classes('flex-1 justify-start'):
            ui.label("Hardware Validation Hub").classes('text-2xl font-extrabold text-gray-800')
        
        # Middle Section: Tabs
        with ui.row().classes('flex-none justify-center'):
            with ui.tabs() as tabs:
                capture_tab = ui.tab('Live Capture', icon='monitor').classes('py-0 min-h-[40px]')
                analysis_tab = ui.tab('Analysis', icon='analytics').classes('py-0 min-h-[40px]')
        
        # Right Section: Empty Spacer
        with ui.row().classes('flex-1'):
            ui.space()

    # --- MAIN CONTENT AREA ---
    with ui.tab_panels(tabs, value=capture_tab).classes('w-full flex-grow min-h-0 min-w-0 bg-transparent p-0'):
        
        # --- LIVE CAPTURE PANEL ---
        with ui.tab_panel(capture_tab).classes('p-0 h-full flex flex-col flex-nowrap'):
            
            with ui.card().classes('w-full mb-4 p-4 bg-slate-50 shadow border border-slate-200 flex-nowrap'):
                with ui.row().classes('w-full justify-between items-center'):
                    
                    # --- LEFT SIDE: Configuration & Execution ---
                    with ui.row().classes('gap-4 items-center'):
                        # Test Selector Dropdown
                        available_tests = engine.get_available_tests()
                        first_test_key = list(available_tests.keys())[0]
                        with ui.column().classes('gap-1'):
                            capture_mode = ui.select(
                                options={key: value['name'] for key, value in available_tests.items()},
                                value=first_test_key, 
                                label='Test Profile',
                                # Update the description label whenever the user picks a new test
                                on_change=lambda e: desc_label.set_text(available_tests[e.value]['description'])
                            ) \
                                .classes('w-48') \
                                .props('dense outlined bg-color=white')
                                
                            # The description sits neatly underneath
                            desc_label = ui.label(available_tests[first_test_key]['description']) \
                                .classes('text-xs text-gray-500 italic max-w-[192px] leading-tight')
                        
                        btn_start = ui.button('Start Capture', on_click=toggle_master).props('color=green icon=play_arrow')
                        ui.button('Reset', on_click=reset_master_live).props('color=grey icon=refresh outline')
                    
                    # --- RIGHT SIDE: Data Management ---
                    with ui.row().classes('gap-4 items-center'):
                        ui.button('Save Run CSV', icon='save', on_click=download_master_csv).props('color=blue')
                        
                        ref_uploader = ui.upload(auto_upload=True, on_upload=load_live_reference).props('accept=".csv"').classes('hidden')
                        btn_load_ref = ui.button('Load Ref Ghost', icon='upload', on_click=lambda: ref_uploader.run_method('pickFiles')).props('color=slate outline')
                        
                        # THE LIVE CAPTURE PILL
                        with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-2 hidden flex-nowrap') as ref_pill:
                            ui.icon('description', size='sm')
                            ref_pill_label = ui.label('filename.csv').classes('text-sm font-semibold truncate max-w-[200px]')
                            ui.button(icon='close', on_click=unload_live_reference).props('flat round size=sm padding=none text-color=slate')
            
            with ui.row().classes('w-full flex-grow min-h-0 gap-6 flex-nowrap min-w-0'):
                with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
                    ui.label('Chart Settings').classes('text-lg font-bold text-gray-800 mb-2')
                    # Assign to a variable so we can sync it
                    switch_capture_panel = ui.switch('Logarithmic Y-Axis', on_change=toggle_log_scale).classes('mb-4 font-semibold text-blue-600')
                    ui.separator().classes('mb-4')
                    
                    ui.label('Active Streams').classes('text-lg font-bold text-gray-800 mb-2')
                    for s in SENSORS:
                        sensor_switches[s['id']] = ui.switch(
                            s['name'], value=True, 
                            on_change=lambda e, sid=s['id']: update_visibility(sid, e.value)
                        ).classes('mb-1')

                with ui.column().classes('w-3/4 flex-grow p-4 bg-white shadow-sm border rounded h-full min-w-0'):
                    with ui.card().classes('w-full h-full p-4 relative overflow-hidden flex flex-col'):
                        
                        # Floating Overlay Toggle (same as Analysis)
                        with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                            ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                            switch_capture_chart = ui.switch(on_change=toggle_log_scale).props('size=sm')

                        # Added inner sizing div (flex-grow so it fills the h-full card)
                        with ui.element('div').classes('relative w-full flex-grow min-h-0'):
                            initial_series = [{'id': f"live_{s['id']}", 'name': s['name'], 'data': [], 'color': s['color']} for s in SENSORS]
                            master_chart = ui.highchart({
                                'chart': {'type': 'line', 'animation': False},
                                'title': {'text': 'Data Capture'}, # The title creates the perfect natural space!
                                'xAxis': {'title': {'text': 'Seconds'}},
                                'yAxis': {'type': 'linear'},
                                'tooltip': {'shared': True, 'crosshairs': True},
                                'series': initial_series
                            }).classes('absolute inset-0 w-full h-full')

        # --- ANALYSIS PANEL ---
        with ui.tab_panel(analysis_tab).classes('p-0 h-full'): 
            
            with ui.row().classes('w-full gap-6 flex-nowrap items-start h-full min-w-0'):
                
                # --- LEFT MENU (25%) ---
                with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
                    ui.label('Analysis Setup').classes('text-lg font-bold text-gray-800 mb-4')
                    
                    ui.label('1. Metrics to Plot').classes('text-xs font-bold text-gray-500 uppercase mb-1')
                    analysis_metric_select = ui.select(sensor_names, multiple=True, value=sensor_names, on_change=update_analysis_view).classes('w-full mb-6')
                    
                    # --- GROUPED DATA SOURCES ---
                    with ui.column().classes('w-full p-3 bg-slate-50 rounded border border-slate-200 gap-2'):
                        ui.label('2. Data Sources').classes('text-xs font-bold text-slate-500 uppercase mb-2')
                        
                        # Dataset A Group
                        ui.label('Dataset A (Solid)').classes('text-[10px] font-bold text-blue-500 uppercase tracking-wider')
                        with ui.row().classes('bg-blue-100 text-blue-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                            ui.icon('timeline', size='xs')
                            label_A_status = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                            btn_clear_A = ui.button(icon='close', on_click=lambda: unload_analysis_dataset('A')).props('flat round size=xs padding=none text-color=blue').classes('hidden')

                        with ui.row().classes('w-full gap-2 mb-2'):
                            ui.button('Sync Live', icon='refresh', on_click=set_analysis_A_to_live).props('size=sm color=slate outline').classes('flex-none')
                            up_A = ui.upload(auto_upload=True, on_upload=lambda e: handle_analysis_upload(e, 'A')).props('accept=".csv"').classes('hidden')
                            ui.button('Load CSV', on_click=lambda: up_A.run_method('pickFiles')).props('size=sm color=blue').classes('flex-grow')
                        
                        ui.separator().classes('my-1')

                        # Dataset B Group
                        ui.label('Dataset B (Dashed)').classes('text-[10px] font-bold text-slate-500 uppercase tracking-wider')
                        with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                            ui.icon('show_chart', size='xs')
                            label_B_status = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                            btn_clear_B = ui.button(icon='close', on_click=lambda: unload_analysis_dataset('B')).props('flat round size=xs padding=none text-color=slate').classes('hidden')

                        with ui.row().classes('w-full gap-2 mb-2'):
                            ui.button('Use Live Ref', icon='move_down', on_click=set_analysis_B_to_live_ref).props('size=sm color=slate outline').classes('flex-none')
                            up_B = ui.upload(auto_upload=True, on_upload=lambda e: handle_analysis_upload(e, 'B')).props('accept=".csv"').classes('hidden')
                            ui.button('Load CSV', on_click=lambda: up_B.run_method('pickFiles')).props('size=sm color=slate outline').classes('flex-grow')

                # --- RIGHT GRAPHS (75%) ---
                with ui.column().classes('w-3/4 flex-grow h-full gap-0 bg-white border shadow-sm rounded min-w-0'):
                    
                    with ui.card().classes('w-full p-4 bg-slate-50 border-b-2 border-blue-400 z-10 shadow-sm rounded-none'):
                        with ui.row().classes('w-full justify-start items-center mb-1 gap-4'):
                            ui.label('Global Time Window (Seconds)').classes('text-sm font-bold text-gray-800')
                            ui.button('Reset', icon='restart_alt', on_click=reset_time_window).props('outline size=sm color=slate padding=xs')
                        
                        time_range = ui.range(min=0, max=10, value={'min': 0, 'max': 10}, on_change=update_analysis_view).props('label-always color=blue').classes('w-full px-4')
                    
                    # overflow-y-auto: Allows vertical scrolling only when needed
                    # overflow-x-hidden: Absolutely forbids horizontal scrollbars from rendering
                    with ui.element('div').classes('w-full flex-grow overflow-y-auto overflow-x-hidden'):
                        with ui.column().classes('w-full items-stretch gap-4 min-w-0 p-4 overflow-x-hidden'):
                            # --- Comparison Overlay Card ---
                            with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0 relative'):
                                # Floating Toggle (Z-index ensures it stays above the chart)
                                with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                                    ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                                    analysis_log_scale = ui.switch(on_change=update_analysis_view).props('size=sm')
                                
                                with ui.element('div').classes('relative w-full h-[400px]'):
                                    analysis_chart = ui.highchart({
                                        'chart': {'type': 'line'},
                                        'title': {'text': 'Comparison Overlay'},
                                        'xAxis': {'title': {'text': 'Seconds'}},
                                        'yAxis': {'type': 'linear'},
                                        'tooltip': {'shared': True, 'crosshairs': True},
                                        'series': []
                                    }).classes('absolute inset-0 w-full h-full')
                            
                            # --- Relative Error Card ---
                            with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0'):
                                with ui.element('div').classes('relative w-full h-[250px]'):
                                    delta_chart = ui.highchart({
                                        'chart': {'type': 'line'},
                                        'title': {'text': 'Relative Error (Dataset A vs B)'},
                                        'xAxis': {'title': {'text': 'Seconds'}},
                                        'yAxis': {'title': {'text': '% Difference'}},
                                        'tooltip': {'shared': True, 'crosshairs': True, 'valueSuffix': '%'},
                                        'series': []
                                    }).classes('absolute inset-0 w-full h-full')
                                
                            # --- Data Distribution Card ---
                            with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0'):
                                with ui.row().classes('w-full justify-between items-center mb-2'):
                                    ui.label('Data Distribution (Windowed)').classes('text-lg font-bold text-gray-800')
                                    dist_metric_select = ui.select(sensor_names, value=sensor_names[0], on_change=update_analysis_view).classes('w-64')
                                    
                                with ui.element('div').classes('relative w-full h-[250px]'):
                                    dist_chart = ui.highchart({
                                        'chart': {'type': 'column'},
                                        'title': {'text': ''}, 
                                        'xAxis': {'title': {'text': 'Metric Value'}},
                                        'yAxis': {'title': {'text': 'Frequency (Points)'}},
                                        'tooltip': {'shared': True},
                                        'plotOptions': {'column': {'pointPadding': 0, 'groupPadding': 0.1, 'borderWidth': 0}},
                                        'series': []
                                    }).classes('absolute inset-0 w-full h-full')
                            
                            # --- Comparison Statistics Card ---
                            with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0'):
                                ui.label('Comparison Statistics (Windowed)').classes('text-lg font-bold text-gray-800 mb-2')
                                stats_table = ui.table(
                                    columns=[
                                        {'name': 'metric', 'label': 'Metric', 'field': 'Metric', 'align': 'left'},
                                        {'name': 'dataset', 'label': 'Dataset', 'field': 'Dataset', 'align': 'left'},
                                        {'name': 'max', 'label': 'Max Value', 'field': 'Max', 'align': 'right'},
                                        {'name': 'mean', 'label': 'Mean', 'field': 'Mean', 'align': 'right'},
                                        {'name': 'rms', 'label': 'RMS', 'field': 'RMS', 'align': 'right'},
                                        {'name': 'diff', 'label': 'Avg % Diff (MAPE)', 'field': 'Diff_vs_A', 'align': 'right'}
                                    ],
                                    rows=[], row_key='id'
                                ).classes('w-full')

# Gracefully shutdown the DAQ when the browser window closes
app.on_shutdown(daq.stop)

ui.timer(0.1, update_master_stream)
ui.run()