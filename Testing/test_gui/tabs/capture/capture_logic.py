import asyncio
import pandas as pd
import io
import time
import traceback
from datetime import datetime
from typing import Any, List, Dict, Union, Optional, cast
from nicegui import ui
from nicegui.client import Client
from nicegui.events import UploadEventArguments, ValueChangeEventArguments
from nicegui.elements.upload import Upload

from test_gui import state
from test_gui.ui_registry import UI
from test_gui.constants import (
    BATTERY_SCALE, CURRENT_SCALE, PWM_SCALE, COLORS, 
    DEFAULT_CSV_PREFIX, MOTORS, MOTOR_SENSORS, MAX_LIVE_POINTS
)
from test_gui.tabs.analysis.analysis_logic import update_time_slider_limits

def update_master_stream() -> None:
    """
    Polls the hardware data acquisition (DAQ) reader layer, extracts a telemetry snapshot,
    applies physical scaling transformations, and appends the frame to global historical storage.
    """
    if state.daq is None or not state.is_running: 
        return
    if state.start_time is None: 
        state.start_time = time.time()
    
    elapsed = round(time.time() - state.start_time, 2)
    row: Dict[str, Union[float, int]] = {'Seconds': elapsed}
    
    snap = state.daq.telemetry 
    row['volt1'] = snap.battery_voltage_1 * BATTERY_SCALE
    row['volt2'] = snap.battery_voltage_2 * BATTERY_SCALE
    
    for m in MOTORS:
        m_id = str(m['id'])
        motor_data = getattr(snap, m_id) 
        row[f"{m_id}_pwm"] = motor_data.pwm * PWM_SCALE
        row[f"{m_id}_rpm"] = motor_data.velocity
        row[f"{m_id}_curr"] = motor_data.current * CURRENT_SCALE
        row[f"{m_id}_enc"] = motor_data.encoder_count
        
    state.global_history.append(row)
    display_history = state.global_history[-MAX_LIVE_POINTS:]
    
    if UI.master_chart_card:
        data_updates: Dict[str, List[List[Union[float, int]]]] = {}
        
        for v_id in ['volt1', 'volt2']:
            data_updates[f'live_{v_id}'] = [[r['Seconds'], r[v_id]] for r in display_history] # type: ignore
        
        for s in MOTOR_SENSORS:
            for m in MOTORS:
                # Original logic: chart ID uses s_id_m_id, but dataframe column uses m_id_s_id
                data_updates[f'live_{s["id"]}_{m["id"]}'] = [[r['Seconds'], r[f"{m['id']}_{s['id']}"]] for r in display_history] # type: ignore
                
        UI.master_chart_card.batch_update_series_data(data_updates, redraw=True)
    
    if state.using_live_for_A:
        update_time_slider_limits()

async def run_test_engine(profile_filename: Optional[str], client: Client) -> None:
    """
    Background worker that runs automated test profiles, records data, 
    and evaluates pass/fail metrics against the TestEngine.
    """
    try:
        test_start_seconds = time.time() - state.start_time if state.start_time else 0
        if profile_filename:
            await state.engine.execute_profile(profile_filename)
        df = pd.DataFrame(state.global_history)
        
        if not df.empty:
            df = df[df['Seconds'] >= test_start_seconds]
            final_report = state.engine.evaluate_current_profile(df)
            with client:
                if final_report['passed']: 
                    ui.notify("✅ Test Passed!", type=COLORS['positive'])
                else: 
                    ui.notify("❌ Test Failed! Check console.", type=COLORS['negative'])
        else:
            with client: 
                ui.notify("Cannot evaluate: No data was captured.", type=COLORS['warning'])

        if state.is_running:
            with client: 
                toggle_master()
                
    except asyncio.CancelledError:
        await state.engine.stop()
        raise
    except Exception as e:
        traceback.print_exc() 
        if state.is_running:
            with client: 
                toggle_master()
          
def toggle_master() -> None:    
    """
    Toggles the global capture state. Starts background engine tasks, resets timers, 
    and syncs the master UI control button.
    """
    state.is_running = not state.is_running
    
    if UI.btn_start:
        UI.btn_start.text = 'Stop Capture' if state.is_running else 'Start Capture'
        UI.btn_start.props(f'color={"red" if state.is_running else "green"} icon={"stop" if state.is_running else "play_arrow"}')
    
    if state.is_running:
        if state.global_history: 
            reset_master_live(show_notify=False)
            
        if hasattr(state.daq, 'reset_clock'): 
            state.daq.reset_clock() # type: ignore
            
        state.start_time = time.time() 
        client = ui.context.client
        
        profile = UI.capture_mode.value if UI.capture_mode else None
        state.engine_task = asyncio.create_task(run_test_engine(profile, client))
    else:
        if state.engine_task is not None and not state.engine_task.done(): 
            state.engine_task.cancel()

def reset_master_live(show_notify: bool = True) -> None:
    """Clears all live data from memory and empties the master chart."""
    state.start_time = None
    state.global_history.clear()
    
    if UI.master_chart_card:
        empty_data: Dict[str, List[Any]] = {f'live_{v_id}': [] for v_id in ['volt1', 'volt2']}
        for s in MOTOR_SENSORS:
            for m in MOTORS: 
                empty_data[f'live_{s["id"]}_{m["id"]}'] = []
        UI.master_chart_card.batch_update_series_data(empty_data, redraw=True)
        
    update_time_slider_limits()
    if show_notify: 
        ui.notify('Live data reset')

def download_master_csv() -> None:
    """Packs the live capture DataFrame and triggers a native browser file download."""
    if not state.global_history: 
        return ui.notify('No data to save!', type=COLORS['warning'])
        
    df = pd.DataFrame(state.global_history)
    cols = ['Seconds', 'volt1', 'volt2'] + [f"{m['id']}_{s['id']}" for m in MOTORS for s in MOTOR_SENSORS]
    
    # Filter columns to ensure we only export what physically exists in the dataframe
    export_df = df[[c for c in cols if c in df.columns]]
    csv_content = export_df.to_csv(index=False).encode('utf-8')
    
    filename = f'{DEFAULT_CSV_PREFIX}{datetime.now().strftime("%H-%M-%S")}.csv'
    ui.download(csv_content, filename=filename)

def update_chart_visibility() -> None:
    """
    Calculates visibility intersecting motor switches AND sensor switches, 
    and applies changes to the chart.
    """
    if not UI.master_chart_card: 
        return
        
    vis_dict: Dict[str, bool] = {}
    
    for v_id in ['volt1', 'volt2']:
        v_vis = bool(UI.sensor_switches[v_id].value)
        vis_dict[f'live_{v_id}'] = v_vis
        vis_dict[f'ref_{v_id}'] = v_vis
    
    for s in MOTOR_SENSORS:
        for m in MOTORS:
            is_vis = bool(UI.sensor_switches[s['id']].value and UI.motor_switches[m['id']].value)
            vis_dict[f'live_{s["id"]}_{m["id"]}'] = is_vis
            vis_dict[f'ref_{s["id"]}_{m["id"]}'] = is_vis
            
    UI.master_chart_card.batch_set_visibility(vis_dict, redraw=True)

def toggle_log_scale(e: ValueChangeEventArguments) -> None:
    """Synchronizes log scale configurations across all UI panels and chart configs."""
    is_log = bool(e.value)
    
    if UI.switch_capture_panel and UI.switch_capture_panel.value != is_log: 
        UI.switch_capture_panel.value = is_log
        
    if UI.master_chart_card and hasattr(UI.master_chart_card, 'log_switch') and UI.master_chart_card.log_switch.value != is_log:
        UI.master_chart_card.log_switch.value = is_log
        
    if UI.master_chart_card:
        UI.master_chart_card.update_chart(is_log=is_log)

async def load_live_reference(e: UploadEventArguments) -> None:
    """Loads a CSV into memory as a reference 'ghost' dataset."""
    try:
        content = await e.file.read()
        state.live_reference_df = pd.read_csv(io.BytesIO(content))
        
        if UI.master_chart_card:
            metrics_to_load = [
                ('volt1', 'volt1', COLORS['volt1'], 'Solid', bool(UI.sensor_switches['volt1'].value)),
                ('volt2', 'volt2', COLORS['volt2'], 'Solid', bool(UI.sensor_switches['volt2'].value))
            ]
            for s in MOTOR_SENSORS:
                for m in MOTORS:
                    metrics_to_load.append((
                        f"{s['id']}_{m['id']}", 
                        f"{m['id']}_{s['id']}", 
                        s['color'], 
                        m['dash'], 
                        bool(UI.sensor_switches[s['id']].value and UI.motor_switches[m['id']].value)
                    ))
                    
            for chart_id, col_name, color, dash, is_visible in metrics_to_load:
                if col_name in state.live_reference_df.columns:
                    UI.master_chart_card.remove_series(f"ref_{chart_id}")
                    
                    UI.master_chart_card.add_series({
                        "id": f"ref_{chart_id}", 
                        "name": f"Ref: {col_name}", 
                        "data": state.live_reference_df[['Seconds', col_name]].dropna().values.tolist(), 
                        "color": str(color), 
                        "dashStyle": str(dash), 
                        "opacity": 0.4, 
                        "visible": is_visible, 
                        "marker": {"enabled": False}
                    })
            UI.master_chart_card.redraw() 

        if UI.ref_pill_label: UI.ref_pill_label.set_text(e.file.name)
        if UI.ref_pill: UI.ref_pill.classes(remove='hidden')
        if UI.btn_load_ref: UI.btn_load_ref.classes('hidden')
        
        cast(Upload, e.sender).reset() 
        ui.notify('Reference loaded', type=COLORS['positive'])
    except Exception as ex:
        ui.notify(f'Error: {ex}', type=COLORS['negative'])

def unload_live_reference() -> None:
    """Unloads the ghost reference dataset."""
    state.live_reference_df = pd.DataFrame() 
    if UI.master_chart_card:
        ids = ['volt1', 'volt2'] + [f"{s['id']}_{m['id']}" for s in MOTOR_SENSORS for m in MOTORS]
        for cid in ids: 
            UI.master_chart_card.remove_series(f"ref_{cid}")
        UI.master_chart_card.redraw()

    if UI.ref_pill: UI.ref_pill.classes('hidden')
    if UI.btn_load_ref: UI.btn_load_ref.classes(remove='hidden')
    ui.notify('Reference unloaded')