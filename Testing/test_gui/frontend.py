import asyncio
from nicegui import ui
import pandas as pd
import numpy as np
import json
import io
from datetime import datetime
import time
import traceback
import os

from test_gui import state

class UI:
    # Use the modern "Type | None" syntax (Python 3.10+)
    master_chart: ui.highchart | None = None
    btn_start: ui.button | None = None
    capture_mode: ui.select | None = None  # Assuming this is a toggle or radio!
    switch_capture_panel: ui.switch | None = None
    switch_capture_chart: ui.switch | None = None
    ref_pill_label: ui.label | None = None
    ref_pill: ui.element | None = None # Fallback to ui.element if you don't know the exact type
    btn_load_ref: ui.button | None = None
    time_range: ui.range | None = None
    label_A_status: ui.label | None = None
    btn_clear_A: ui.button | None = None
    label_B_status: ui.label | None = None
    btn_clear_B: ui.button | None = None
    analysis_sensor_select: ui.select | None = None
    analysis_motor_select: ui.select | None = None
    analysis_log_scale: ui.switch | None = None
    dist_metric_select: ui.select | None = None
    stats_table: ui.table | None = None
    dist_chart: ui.highchart | None = None
    delta_chart: ui.highchart | None = None
    analysis_chart: ui.highchart | None = None

# ==========================================
# 2. Live Capture Handlers
# ==========================================
BATTERY_SCALE = 0.1   # Roboclaw reports in 10mV units
CURRENT_SCALE = 0.01  # Roboclaw reports in 10mA units
PWM_SCALE = 1 / 327.67 # See wheel.hpp for details

def update_master_stream():
    if state.daq is None:
        # We will only print this once per second so it doesn't flood the terminal
        if int(time.time() * 10) % 10 == 0: 
            print(f"⏳ Timer waiting... daq is None. [Process ID: {os.getpid()}]")
        return
    
    if not state.is_running: return
    if state.start_time is None: state.start_time = time.time()
    
    elapsed = round(time.time() - state.start_time, 2)
    row = {'Seconds': elapsed}
    
    snap = state.daq.telemetry 
    
    # Extract Rover-level data
    row['volt1'] = snap.battery_voltage_1 * BATTERY_SCALE
    row['volt2'] = snap.battery_voltage_2 * BATTERY_SCALE
    
    # Extract all 16 Motor-level data points
    for m in state.MOTORS:
        m_id = m['id']
        motor_data = getattr(snap, m_id) 
        
        row[f"{m_id}_pwm"] = motor_data.pwm * PWM_SCALE
        row[f"{m_id}_rpm"] = motor_data.velocity
        row[f"{m_id}_curr"] = motor_data.current * CURRENT_SCALE
        row[f"{m_id}_enc"] = motor_data.encoder_count
        
    state.global_history.append(row)
    display_history = state.global_history[-state.MAX_LIVE_POINTS:]
    
    # Push to Chart
    js_commands = [f'{{ const chart = getElement({UI.master_chart.id}).chart;'] if UI.master_chart else []
    
    # Push Both Voltages
    for v_id in ['volt1', 'volt2']:
        v_data = [[r['Seconds'], r[v_id]] for r in display_history]
        js_commands.append(f'chart.get("live_{v_id}").setData({v_data}, false, false, false);')
    
    # Push Motor Data
    for s in state.MOTOR_SENSORS:
        for m in state.MOTORS:
            data_key = f"{m['id']}_{s['id']}"
            sensor_data = [[r['Seconds'], r[data_key]] for r in display_history]
            js_commands.append(f'chart.get("live_{s["id"]}_{m["id"]}").setData({sensor_data}, false, false, false);')
            
    js_commands.append('chart.redraw(false); }')
    ui.run_javascript('\n'.join(js_commands))
    
    if state.using_live_for_A:
        update_time_slider_limits()

async def run_test_engine(profile_filename, client):
    try:
        test_start_seconds = time.time() - state.start_time if state.start_time else 0
        
        # 1. Tell the engine to run the file
        await state.engine.execute_profile(profile_filename)
        
        # 2. Get the data
        df = pd.DataFrame(state.global_history)
        
        if not df.empty:
            df = df[df['Seconds'] >= test_start_seconds]
            
            # 3. Ask the engine to evaluate it
            final_report = state.engine.evaluate_current_profile(df)
            
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

        if state.is_running:
            with client:
                toggle_master()
                
    except asyncio.CancelledError:
        print("--- EMERGENCY STOP TRIGGERED ---")
        await state.engine.stop()
        raise
    except Exception as e:
        print(f"\n❌ FATAL ENGINE ERROR: {e}")
        traceback.print_exc() 
        if state.is_running:
            with client:
                toggle_master()
          
def toggle_master():    
    state.is_running = not state.is_running
    if UI.btn_start:
        UI.btn_start.text = 'Stop Capture' if state.is_running else 'Start Capture'
        UI.btn_start.props(f'color={"red" if state.is_running else "green"} icon={"stop" if state.is_running else "play_arrow"}')
    
    if state.is_running:
        if state.global_history:
            reset_master_live(show_notify=False)
        
        if hasattr(state.daq, 'reset_clock'):
            state.daq.reset_clock() # type: ignore
        
        selected_profile = UI.capture_mode.value if UI.capture_mode else None
        state.start_time = time.time()  # Ensure start_time is set!
        client = ui.context.client
        state.engine_task = asyncio.create_task(run_test_engine(selected_profile, client))
    else:
        if state.engine_task is not None and not state.engine_task.done():
            state.engine_task.cancel()

def reset_master_live(show_notify=True):
    state.start_time = None
    state.global_history.clear()
    
    js_commands = [f'const chart = getElement({UI.master_chart.id}).chart;'] if UI.master_chart else []
    
    # Reset Voltages
    for v_id in ['volt1', 'volt2']:
        js_commands.append(f'chart.get("live_{v_id}").setData([], false, false, false);')
        
    # Reset Motor Streams
    for s in state.MOTOR_SENSORS:
        for m in state.MOTORS:
            js_commands.append(f'chart.get("live_{s["id"]}_{m["id"]}").setData([], false, false, false);')
            
    js_commands.append('chart.redraw();')
    ui.run_javascript('\n'.join(js_commands))
    update_time_slider_limits()
    
    if show_notify:
        ui.notify('Live data reset')

def download_master_csv():
    if not state.global_history:
        ui.notify('No data to save!', type='warning')
        return
    
    df = pd.DataFrame(state.global_history)
    
    # Define the desired column order: Seconds, Voltages, then Motor Data
    cols = ['Seconds', 'volt1', 'volt2']
    for m in state.MOTORS:
        for s in state.MOTOR_SENSORS:
            cols.append(f"{m['id']}_{s['id']}")
            
    # Only keep columns that actually exist in the dataframe (safeguard)
    valid_cols = [c for c in cols if c in df.columns]
    df = df[valid_cols]
    
    csv_content = df.to_csv(index=False).encode('utf-8')
    ui.download(csv_content, filename=f'robot_run_{datetime.now().strftime("%H-%M-%S")}.csv')

def update_chart_visibility():
    """Calculates visibility based on Motor Toggles AND Sensor Toggles"""
    js_commands = [f'{{ const chart = getElement({UI.master_chart.id}).chart;'] if UI.master_chart else []
    
    # Update Both Voltages
    for v_id in ['volt1', 'volt2']:
        v_vis = str(state.sensor_switches[v_id].value).lower()
        # Add safety checks and toggle both Live AND Ref traces
        js_commands.append(f'if (chart.get("live_{v_id}")) chart.get("live_{v_id}").setVisible({v_vis}, false);')
        js_commands.append(f'if (chart.get("ref_{v_id}")) chart.get("ref_{v_id}").setVisible({v_vis}, false);')
    
    # Update Motor Streams (Depends on BOTH switches)
    for s in state.MOTOR_SENSORS:
        is_sensor_on = state.sensor_switches[s['id']].value
        for m in state.MOTORS:
            is_motor_on = state.motor_switches[m['id']].value
            is_visible = str(is_sensor_on and is_motor_on).lower()
            
            # Add safety checks and toggle both Live AND Ref traces
            js_commands.append(f'if (chart.get("live_{s["id"]}_{m["id"]}")) chart.get("live_{s["id"]}_{m["id"]}").setVisible({is_visible}, false);')
            js_commands.append(f'if (chart.get("ref_{s["id"]}_{m["id"]}")) chart.get("ref_{s["id"]}_{m["id"]}").setVisible({is_visible}, false);')
            
    js_commands.append('chart.redraw(); }')
    ui.run_javascript('\n'.join(js_commands))

def toggle_log_scale(e):
    # Sync the two switches so they stay at the same value
    if UI.switch_capture_panel:
        UI.switch_capture_panel.value = e.value
    
    if UI.switch_capture_chart:
        UI.switch_capture_chart.value = e.value
    
    axis_type = "logarithmic" if e.value else "linear"
    ui.run_javascript(f'getElement({UI.master_chart.id}).chart.yAxis[0].update({{type: "{axis_type}"}});') if UI.master_chart else None

async def load_live_reference(e):
    try:
        content = await e.file.read()
        state.live_reference_df = pd.read_csv(io.BytesIO(content))
        
        # 1. Declare the chart ONCE at the top of the command list
        js_commands = [f'const chart = getElement({UI.master_chart.id}).chart;'] if UI.master_chart else []
        
        metrics_to_load = [
            ('volt1', 'volt1', '#f59e0b', 'Solid', bool(state.sensor_switches['volt1'].value)),
            ('volt2', 'volt2', '#d97706', 'Solid', bool(state.sensor_switches['volt2'].value))
        ]
        
        for s in state.MOTOR_SENSORS:
            for m in state.MOTORS:
                is_vis = bool(state.sensor_switches[s['id']].value and state.motor_switches[m['id']].value)
                metrics_to_load.append((
                    f"{s['id']}_{m['id']}",   
                    f"{m['id']}_{s['id']}",   
                    s['color'], m['dash'], is_vis
                ))
                
        # 2. Add all series to the command list without executing them yet
        for chart_id, col_name, color, dash, is_visible in metrics_to_load:
            if col_name in state.live_reference_df.columns:
                data = state.live_reference_df[['Seconds', col_name]].dropna().values.tolist()
                
                # We keep json.dumps() because it safely translates Python 'False' to Javascript 'false'
                series_config = json.dumps({
                    "id": f"ref_{chart_id}",
                    "name": f"Ref: {col_name}",
                    "data": data,
                    "color": color,
                    "dashStyle": dash,
                    "opacity": 0.4,
                    "visible": is_visible,
                    "marker": {"enabled": False}
                })
                
                js_commands.append(f'if (chart.get("ref_{chart_id}")) chart.get("ref_{chart_id}").remove(false);')
                js_commands.append(f'chart.addSeries({series_config}, false);')
                
        # 3. Redraw once and execute the single Javascript string
        js_commands.append('chart.redraw();') 
        ui.run_javascript('\n'.join(js_commands))
        
        if UI.ref_pill_label:
            UI.ref_pill_label.set_text(e.file.name)
        
        if UI.ref_pill:
            UI.ref_pill.classes(remove='hidden')
        
        if UI.btn_load_ref:
            UI.btn_load_ref.classes('hidden')
        e.sender.reset() 
        ui.notify('Reference trace loaded for Live Capture', type='positive')
    except Exception as ex:
        ui.notify(f'Error: {ex}', type='negative')
        print(f"Live Ref Error: {ex}")

def unload_live_reference():
    state.live_reference_df = pd.DataFrame() 
    
    # Wrapped in { }
    js_commands = [f'{{ const chart = getElement({UI.master_chart.id}).chart;'] if UI.master_chart else []
    
    # Generate all chart IDs to remove
    ids_to_remove = ['volt1', 'volt2'] + [f"{s['id']}_{m['id']}" for s in state.MOTOR_SENSORS for m in state.MOTORS]
    
    for cid in ids_to_remove:
        js_commands.append(f'if (chart.get("ref_{cid}")) chart.get("ref_{cid}").remove(false);')
        
    js_commands.append('chart.redraw(); }') # Closing block
    ui.run_javascript('\n'.join(js_commands))
    
    if UI.ref_pill:
        UI.ref_pill.classes('hidden')
    if UI.btn_load_ref:
        UI.btn_load_ref.classes(remove='hidden')
    ui.notify('Reference trace unloaded')

# ==========================================
# 3. Analysis Tab Handlers
# ==========================================
def update_time_slider_limits():
    max_t = 0.0
    if state.using_live_for_A and state.global_history: 
        max_t = max(max_t, state.global_history[-1]['Seconds'])
    elif not state.using_live_for_A and not state.analysis_df_A.empty: 
        max_t = max(max_t, state.analysis_df_A['Seconds'].max())
        
    if not state.analysis_df_B.empty: 
        max_t = max(max_t, state.analysis_df_B['Seconds'].max())
    
    max_t = max(10.0, max_t) 
    
    if UI.time_range and UI.time_range.max != max_t:
        # 1. Save the old maximum BEFORE we overwrite it
        old_max = UI.time_range.max 
        
        # 2. Update the slider's absolute max bound
        UI.time_range.max = float(max_t)
        
        # 3. Check if the handle was near the OLD max. If so, snap it to the NEW max.
        if UI.time_range.value['max'] >= (old_max - 1): # type: ignore
            UI.time_range.value = {'min': UI.time_range.value['min'], 'max': float(max_t)} # type: ignore
            
        UI.time_range.update()

def reset_time_window():
    if UI.time_range:
        UI.time_range.value = {'min': 0, 'max': float(UI.time_range.max)}
    update_analysis_view()
    ui.notify('Time window reset to full view')

async def handle_analysis_upload(e, dataset_slot):
    try:
        # Back to the testing_cap_analysis approach!
        content = await e.file.read()
        df = pd.read_csv(io.BytesIO(content))
        
        if dataset_slot == 'A':
            state.analysis_df_A = df
            state.using_live_for_A = False
            if UI.label_A_status:
                UI.label_A_status.set_text(e.file.name) # Old approach
                UI.label_A_status.classes(remove='text-slate-500', add='text-blue-800')
            if UI.btn_clear_A:
                UI.btn_clear_A.classes(remove='hidden')
        else:
            state.analysis_df_B = df
            if UI.label_B_status:
                UI.label_B_status.set_text(e.file.name) # Old approach
                UI.label_B_status.classes(remove='text-slate-500', add='text-slate-800')
            if UI.btn_clear_B:
                UI.btn_clear_B.classes(remove='hidden')
        
        update_time_slider_limits()
        if UI.time_range:
            UI.time_range.value = {'min': 0, 'max': UI.time_range.max}
        update_analysis_view()
        e.sender.reset()
        ui.notify(f'Loaded to Dataset {dataset_slot}', type='positive')
    except Exception as ex:
        ui.notify(f'Error: {ex}', type='negative')
        print(f"Analysis Upload Error: {ex}")

# REQUIRED: Explicit wrappers so NiceGUI doesn't lose the async coroutine
async def upload_dataset_A(e):
    await handle_analysis_upload(e, 'A')

async def upload_dataset_B(e):
    await handle_analysis_upload(e, 'B')

def set_analysis_A_to_live():
    if not state.global_history:
        ui.notify('No Live Capture data exists yet!', type='warning')
        return
    
    state.using_live_for_A = True
    if UI.label_A_status:
        UI.label_A_status.set_text('Current Live Run')
        UI.label_A_status.classes(remove='text-slate-500', add='text-blue-800')
    if UI.btn_clear_A:
        UI.btn_clear_A.classes(remove='hidden') # Show the 'X' now so it can be cleared!
    
    update_time_slider_limits()
    if UI.time_range:
        UI.time_range.value = {'min': 0, 'max': UI.time_range.max}
    
    update_analysis_view()
    ui.notify('Dataset A set to Current Live Run')
    
def set_analysis_B_to_live_ref():
    if state.live_reference_df.empty:
        ui.notify('No Reference Ghost loaded in Live Capture!', type='warning')
        return
        
    state.analysis_df_B = state.live_reference_df.copy() # Copy to prevent accidental cross-talk
    if UI.label_B_status:
        UI.label_B_status.set_text('Live Capture Reference')
        UI.label_B_status.classes(remove='text-slate-500')
    if UI.btn_clear_B:
        UI.btn_clear_B.classes(remove='hidden')
    
    update_time_slider_limits()
    if UI.time_range:
        UI.time_range.value = {'min': 0, 'max': UI.time_range.max}    
    
    update_analysis_view()
    ui.notify('Dataset B set to Live Capture Reference')
    
def unload_analysis_dataset(slot):
    if slot == 'A':
        state.analysis_df_A = pd.DataFrame() # Clear the dataframe
        state.using_live_for_A = False       # Disconnect from live history
        if UI.label_A_status:
            UI.label_A_status.set_text('None Loaded')
            UI.label_A_status.classes(remove='text-blue-800', add='text-slate-500')
        if UI.btn_clear_A:
            UI.btn_clear_A.classes('hidden')
    else:
        state.analysis_df_B = pd.DataFrame()
        if UI.label_B_status:
            UI.label_B_status.set_text('None Loaded')
            UI.label_B_status.classes(remove='text-slate-800', add='text-slate-500')
        if UI.btn_clear_B:
            UI.btn_clear_B.classes('hidden')
        
    update_time_slider_limits()
    update_analysis_view()
    ui.notify(f'Dataset {slot} unloaded')

def toggle_analysis_log_scale(e):
    axis_type = "logarithmic" if e.value else "linear"
    ui.run_javascript(f'getElement({UI.analysis_chart.id}).chart.yAxis[0].update({{type: "{axis_type}"}});') if UI.analysis_chart else None

def update_analysis_view():
    if UI.time_range is None or UI.analysis_sensor_select is None:
        return
        
    # --- DYNAMICALLY BUILD THE ACTIVE METRICS LIST ---
    selected_metrics = []
    active_sensors = UI.analysis_sensor_select.value or []
    active_motors = UI.analysis_motor_select.value or [] if UI.analysis_motor_select else []
    
    for s_id in active_sensors:
        if s_id in ['volt1', 'volt2']:
            selected_metrics.append(s_id) # Voltages don't need a motor prefix
        else:
            for m_id in active_motors:
                selected_metrics.append(f"{m_id}_{s_id}") # Combine them (e.g., 'fl' + 'pwm' = 'fl_pwm')
                
    if not selected_metrics: 
        return # If nothing is selected, exit early
    
    df_A_raw = pd.DataFrame(state.global_history) if state.using_live_for_A else state.analysis_df_A
    df_B_raw = state.analysis_df_B
    
    t_min = UI.time_range.value['min'] # type: ignore
    t_max = UI.time_range.value['max'] # type: ignore
    
    df_A = df_A_raw[(df_A_raw['Seconds'] >= t_min) & (df_A_raw['Seconds'] <= t_max)] if not df_A_raw.empty else df_A_raw
    df_B = df_B_raw[(df_B_raw['Seconds'] >= t_min) & (df_B_raw['Seconds'] <= t_max)] if not df_B_raw.empty else df_B_raw
    
    main_series = []
    delta_series = []
    dist_series = []
    stats = []
    
    y_type = 'logarithmic' if UI.analysis_log_scale and UI.analysis_log_scale.value else 'linear'
    
    # --- HELPER: Resolve color from column name ---
    def _get_metric_color(metric_key):
        if metric_key == 'volt1': return '#f59e0b'
        if metric_key == 'volt2': return '#d97706'
        for s in state.MOTOR_SENSORS:
            if metric_key.endswith(f"_{s['id']}"):
                return s['color']
        return '#000000'

    for metric_name in selected_metrics:
        color = _get_metric_color(metric_name)
        mape_val = state.calculate_mape(df_A, df_B, metric_name) if not df_A.empty and not df_B.empty else "-"
        
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
                'marker': {'enabled': False}
            })
            rms_A = np.sqrt(np.mean(df_A[metric_name].dropna()**2))
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
                'marker': {'enabled': False}
            })
            rms_B = np.sqrt(np.mean(df_B[metric_name].dropna()**2))
            stats.append({'id': f'{metric_name}_B', 'Metric': metric_name, 'Dataset': 'B (Dashed)', 'Max': round(df_B[metric_name].max(), 2), 'Mean': round(df_B[metric_name].mean(), 2), 'RMS': round(rms_B, 2), 'Diff_vs_A': mape_val})
            
        if not df_A.empty and not df_B.empty and metric_name in df_A.columns and metric_name in df_B.columns:
            t_A = df_A['Seconds'].values
            val_A = df_A[metric_name].values
            t_B = df_B['Seconds'].values
            val_B = df_B[metric_name].values
            
            if len(t_A) > 0 and len(t_B) > 0:
                interp_B = np.interp(t_A, t_B, val_B) # type: ignore
                safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A)
                delta_vals = ((val_A - interp_B) / safe_A) * 100
                delta_data = [[float(t), float(d)] for t, d in zip(t_A, delta_vals)]
                delta_series.append({'name': f'Δ {metric_name} (%)', 'data': delta_data, 'color': color, 'lineWidth': 2, 'marker': {'enabled': False}})

    dist_metric = UI.dist_metric_select.value if UI.dist_metric_select else None
    if dist_metric:
        dist_color = _get_metric_color(dist_metric)
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

    if UI.analysis_chart and UI.delta_chart and UI.dist_chart:
        ui.run_javascript(f'''
            var chartM = getElement({UI.analysis_chart.id}).chart;
            chartM.yAxis[0].update({{ type: "{y_type}" }}, false);
            chartM.xAxis[0].setExtremes({t_min}, {t_max}, false);
            chartM.update({{series: {json.dumps(main_series)}}}, true, true, false);
            
            var chartD = getElement({UI.delta_chart.id}).chart;
            chartD.xAxis[0].setExtremes({t_min}, {t_max}, false);
            chartD.update({{series: {json.dumps(delta_series)}}}, true, true, false);
            
            var chartH = getElement({UI.dist_chart.id}).chart;
            chartH.update({{series: {json.dumps(dist_series)}}}, true, true, false);
        ''')
    
    if UI.stats_table:
        UI.stats_table.rows = stats
        UI.stats_table.update()

# ==========================================
# 4. Main UI Layout
# ==========================================
# The decorator tells the server: "When a browser connects to the root URL, run this!"
@ui.page('/')
def index():
    # Put your Section 4: Main UI Layout here!
    ui.page_title('Flat Rosey Test UI')

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
                            available_tests = state.engine.get_available_tests()
                            first_test_key = list(available_tests.keys())[0] if available_tests else None
                            with ui.column().classes('gap-1'):
                                UI.capture_mode = ui.select(
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
                            
                            UI.btn_start = ui.button('Start Capture', on_click=toggle_master).props('color=green icon=play_arrow')
                            ui.button('Reset', on_click=reset_master_live).props('color=grey icon=refresh outline')
                        
                        # --- RIGHT SIDE: Data Management ---
                        with ui.row().classes('gap-4 items-center'):
                            ui.button('Save Run CSV', icon='save', on_click=download_master_csv).props('color=blue')
                            
                            ref_uploader = ui.upload(auto_upload=True, on_upload=load_live_reference).props('accept=".csv"').classes('hidden')
                            UI.btn_load_ref = ui.button('Load Ref Ghost', icon='upload', on_click=lambda: ref_uploader.run_method('pickFiles')).props('color=slate outline')
                            
                            # THE LIVE CAPTURE PILL
                            with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-2 hidden flex-nowrap') as UI.ref_pill:
                                ui.icon('description', size='sm')
                                UI.ref_pill_label = ui.label('filename.csv').classes('text-sm font-semibold truncate max-w-[200px]')
                                ui.button(icon='close', on_click=unload_live_reference).props('flat round size=sm padding=none text-color=slate')
                
                with ui.row().classes('w-full flex-grow min-h-0 gap-6 flex-nowrap min-w-0'):
                    with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
                        ui.label('Chart Settings').classes('text-lg font-bold text-gray-800 mb-2')
                        # Assign to a variable so we can sync it
                        UI.switch_capture_panel = ui.switch('Logarithmic Y-Axis', on_change=toggle_log_scale).classes('mb-4 font-semibold text-blue-600')
                        ui.separator().classes('mb-4')
                        
                        with ui.card().classes('w-full p-4 bg-white shadow-sm border'):
                            ui.label('Active Motors').classes('text-lg font-bold text-gray-800 mb-2')
                            with ui.column().classes('w-full gap-2'):
                                for m in state.MOTORS:
                                    # Note: on_change calls our visibility matrix function
                                    state.motor_switches[m['id']] = ui.switch(
                                        m['name'], 
                                        value=True, 
                                        on_change=update_chart_visibility
                                    ).classes('w-full')

                        # --- Sensor Stream Toggles ---
                        with ui.card().classes('w-full p-4 bg-white shadow-sm border'):
                            ui.label('Active Sensors').classes('text-lg font-bold text-gray-800 mb-2')
                            with ui.column().classes('w-full gap-2'):
                                for s in state.MOTOR_SENSORS:
                                    state.sensor_switches[s['id']] = ui.switch(
                                        s['name'], value=True, on_change=update_chart_visibility
                                    ).classes('w-full')
                                
                                ui.separator().classes('my-2 w-full')
                                
                                # Add Both Voltages separately since they don't belong to a specific motor
                                state.sensor_switches['volt1'] = ui.switch("Bus Voltage 1 (V)", value=True, on_change=update_chart_visibility).classes('w-full')
                                state.sensor_switches['volt2'] = ui.switch("Bus Voltage 2 (V)", value=True, on_change=update_chart_visibility).classes('w-full')

                    with ui.column().classes('w-3/4 flex-grow p-4 bg-white shadow-sm border rounded h-full min-w-0'):
                        with ui.card().classes('w-full h-full p-4 relative overflow-hidden flex flex-col'):
                            
                            # Floating Overlay Toggle (same as Analysis)
                            with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                                ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                                UI.switch_capture_chart = ui.switch(on_change=toggle_log_scale).props('size=sm')

                            # Added inner sizing div (flex-grow so it fills the h-full card)
                            with ui.element('div').classes('relative w-full flex-grow min-h-0'):
                                series_list = []
                                # Add the 16 motor streams
                                for s in state.MOTOR_SENSORS:
                                    for m in state.MOTORS:
                                        series_list.append({
                                            'id': f"live_{s['id']}_{m['id']}",
                                            'name': f"{m['name']} {s['name']}",
                                            'data': [],
                                            'color': s['color'],
                                            'dashStyle': m['dash'],
                                            'marker': {'enabled': False}
                                        })

                                # Add the 2 standalone voltage streams
                                series_list.append({'id': 'live_volt1', 'name': 'Bus Voltage 1 (V)', 'data': [], 'color': '#f59e0b', 'marker': {'enabled': False}})
                                series_list.append({'id': 'live_volt2', 'name': 'Bus Voltage 2 (V)', 'data': [], 'color': '#d97706', 'marker': {'enabled': False}}) # Slightly darker orange
                                
                                UI.master_chart = ui.highchart({
                                    'chart': {'type': 'line', 'animation': False},
                                    'title': {'text': 'Data Capture'}, # The title creates the perfect natural space!
                                    'xAxis': {'title': {'text': 'Seconds'}},
                                    'yAxis': {'type': 'linear'},
                                    'tooltip': {'shared': True, 'crosshairs': True},
                                    'series': series_list
                                }).classes('absolute inset-0 w-full h-full')

            # --- ANALYSIS PANEL ---
            with ui.tab_panel(analysis_tab).classes('p-0 h-full'): 
                
                with ui.row().classes('w-full gap-6 flex-nowrap items-start h-full min-w-0'):
                    
                    # --- LEFT MENU (25%) ---
                    with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
                        ui.label('Analysis Setup').classes('text-lg font-bold text-gray-800 mb-4')
                        
                        ui.label('1. Metrics to Plot').classes('text-xs font-bold text-gray-500 uppercase mb-1')
                        # The Main Line Chart Dropdowns
                        with ui.row().classes('w-full items-start gap-4 mb-6 flex-nowrap'):
                            
                            # Dropdown 1: Motors
                            UI.analysis_motor_select = ui.select(
                                options=state.analysis_motor_options,
                                multiple=True,
                                value=list(state.analysis_motor_options.keys()), # Select all by default
                                label='Filter by Motor',
                                on_change=update_analysis_view
                            ).classes('flex-1 min-w-0') # flex-1 forces equal 50/50 width
                            
                            # Dropdown 2: Sensors
                            UI.analysis_sensor_select = ui.select(
                                options=state.analysis_sensor_options,
                                multiple=True,
                                value=list(state.analysis_sensor_options.keys()), # Select all by default
                                label='Filter by Sensor Type',
                                on_change=update_analysis_view
                            ).classes('flex-1 min-w-0') # flex-1 forces equal 50/50 width
                        
                        # --- GROUPED DATA SOURCES ---
                        with ui.column().classes('w-full p-3 bg-slate-50 rounded border border-slate-200 gap-2'):
                            ui.label('2. Data Sources').classes('text-xs font-bold text-slate-500 uppercase mb-2')
                            
                            # Dataset A Group
                            ui.label('Dataset A (Solid)').classes('text-[10px] font-bold text-blue-500 uppercase tracking-wider')
                            with ui.row().classes('bg-blue-100 text-blue-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                                ui.icon('timeline', size='xs')
                                UI.label_A_status = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                                UI.btn_clear_A = ui.button(icon='close', on_click=lambda: unload_analysis_dataset('A')).props('flat round size=xs padding=none text-color=blue').classes('hidden')

                            with ui.row().classes('w-full gap-2 mb-2'):
                                ui.button('Sync Live', icon='refresh', on_click=set_analysis_A_to_live).props('size=sm color=slate outline').classes('flex-none')
                                up_A = ui.upload(auto_upload=True, on_upload=upload_dataset_A).props('accept=".csv"').classes('hidden')
                                ui.button('Load CSV', on_click=lambda: up_A.run_method('pickFiles')).props('size=sm color=blue').classes('flex-grow')
                            
                            ui.separator().classes('my-1')

                            # Dataset B Group
                            ui.label('Dataset B (Dashed)').classes('text-[10px] font-bold text-slate-500 uppercase tracking-wider')
                            with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                                ui.icon('show_chart', size='xs')
                                UI.label_B_status = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                                UI.btn_clear_B = ui.button(icon='close', on_click=lambda: unload_analysis_dataset('B')).props('flat round size=xs padding=none text-color=slate').classes('hidden')

                            with ui.row().classes('w-full gap-2 mb-2'):
                                ui.button('Use Live Ref', icon='move_down', on_click=set_analysis_B_to_live_ref).props('size=sm color=slate outline').classes('flex-none')
                                up_B = ui.upload(auto_upload=True, on_upload=upload_dataset_B).props('accept=".csv"').classes('hidden')
                                ui.button('Load CSV', on_click=lambda: up_B.run_method('pickFiles')).props('size=sm color=slate outline').classes('flex-grow')

                    # --- RIGHT GRAPHS (75%) ---
                    with ui.column().classes('w-3/4 flex-grow h-full gap-0 bg-white border shadow-sm rounded min-w-0'):
                        
                        with ui.card().classes('w-full p-4 bg-slate-50 border-b-2 border-blue-400 z-10 shadow-sm rounded-none'):
                            with ui.row().classes('w-full justify-start items-center mb-1 gap-4'):
                                ui.label('Global Time Window (Seconds)').classes('text-sm font-bold text-gray-800')
                                ui.button('Reset', icon='restart_alt', on_click=reset_time_window).props('outline size=sm color=slate padding=xs')
                            
                            UI.time_range = ui.range(min=0, max=10, value={'min': 0, 'max': 10}, on_change=update_analysis_view).props('label-always color=blue').classes('w-full px-4')
                        
                        # overflow-y-auto: Allows vertical scrolling only when needed
                        # overflow-x-hidden: Absolutely forbids horizontal scrollbars from rendering
                        with ui.element('div').classes('w-full flex-grow overflow-y-auto overflow-x-hidden'):
                            with ui.column().classes('w-full items-stretch gap-4 min-w-0 p-4 overflow-x-hidden'):
                                # --- Comparison Overlay Card ---
                                with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0 relative'):
                                    # Floating Toggle (Z-index ensures it stays above the chart)
                                    with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                                        ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                                        UI.analysis_log_scale = ui.switch(on_change=update_analysis_view).props('size=sm')
                                    
                                    with ui.element('div').classes('relative w-full h-[400px]'):
                                        UI.analysis_chart = ui.highchart({
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
                                        UI.delta_chart = ui.highchart({
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
                                        
                                        UI.dist_metric_select = ui.select(
                                            options=state.analysis_options,
                                            value='fl_rpm', 
                                            on_change=update_analysis_view
                                        ).classes('w-72 min-w-0 mb-6').props('options-dense') # FIXED: Changed w-full to w-72 min-w-0
                                        
                                    with ui.element('div').classes('relative w-full h-[250px]'):
                                        UI.dist_chart = ui.highchart({
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
                                    UI.stats_table = ui.table(
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

    # Start the timer here at the end of the UI build!
    ui.timer(0.1, update_master_stream)