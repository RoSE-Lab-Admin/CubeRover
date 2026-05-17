# test_gui/tabs/analysis/analysis_logic.py
import pandas as pd
import numpy as np
import io
from typing import Dict, List
from nicegui import ui
from nicegui.events import UploadEventArguments, ValueChangeEventArguments

from test_gui import state
from test_gui.ui_registry import UI
from test_gui.constants import COLORS, MOTOR_SENSORS

from typing import List, Dict, Union, Literal, TypedDict, cast

# Structural shape for a standard [timestamp, value] data point coordinate
DataPoint = List[Union[float, int]]

# Shape for the main comparative Highcharts lines
class HighchartsLineSeries(TypedDict, total=False):
    id: str
    name: str
    data: List[DataPoint]
    color: str
    lineWidth: int
    type: str
    dashStyle: Literal['Solid', 'ShortDash', 'Dash', 'Dot', 'DashDot', 'ShortDashDot']
    opacity: float
    marker: Dict[str, bool]

# Shape for the Relative Error Delta Highcharts lines
class HighchartsDeltaSeries(TypedDict):
    name: str
    data: List[DataPoint]
    color: str
    lineWidth: int
    marker: Dict[str, bool]

# Shape for the Data Distribution Column/Histogram bars
class HighchartsDistSeries(TypedDict):
    name: str
    data: List[DataPoint]
    color: str
    opacity: float

# Shape for rows fed straight into UI.stats_table
class AnalysisStatRow(TypedDict):
    id: str
    Metric: str
    Dataset: Literal['A (Solid)', 'B (Dashed)']
    Max: float
    Mean: float
    RMS: float
    Diff_vs_A: Union[float, str]  # Float or string indicator like "-" / "N/A"


def calculate_mape(df_A: pd.DataFrame, df_B: pd.DataFrame, metric: str) -> str:
    """
    Calculates the Mean Absolute Percentage Error (MAPE) between two datasets 
    for a matching metric.
    
    Because Dataset A and Dataset B may be sampled at different rates or times,
    this function aligns them by interpolating Dataset B's values onto Dataset A's 
    timestamps.

    Args:
        df_A (pd.DataFrame): The ground truth / baseline dataframe (must contain 'Seconds').
        df_B (pd.DataFrame): The comparison dataframe to evaluate (must contain 'Seconds').
        metric (str): The column header name of the metric to compare.

    Returns:
        str: The calculated percentage error formatted as a string (e.g., "4.2%"), 
             or "N/A" if calculations cannot be performed.
    """
    try:
        t_A = df_A['Seconds'].values
        val_A = df_A[metric].values
        t_B = df_B['Seconds'].values
        val_B = df_B[metric].values
        
        if len(t_A) == 0 or len(t_B) == 0: 
            return "N/A"
        
        # 1. Map/Align timestamps: Project Dataset B values onto Dataset A's time track
        interp_B = np.interp(t_A, t_B, val_B) # type: ignore
        
        # 2. Prevent division-by-zero errors by substituting absolute zeros with a tiny floor value
        safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A) # type: ignore
        
        # 3. Compute the classic MAPE arithmetic formula
        mape = np.mean(np.abs((val_A - interp_B) / safe_A)) * 100
        return f"{mape:.1f}%"
    except Exception:
        return "N/A"

def update_time_slider_limits() -> None:
    """
    Evaluates the currently active datasets to determine the maximum runtime duration (in seconds),
    then dynamically updates the bounds of the global UI time-range slider.
    
    Ensures that if the user loads a long historic log file, the range scales outward, 
    and handles automatic snapping behaviors if the slider is currently pinned to the maximum limit.
    """
    max_t = 0.0
    
    # Check bounds depending on whether Slot A is actively using live streaming or standard uploaded data
    if state.using_live_for_A and state.global_history: 
        max_t = max(max_t, state.global_history[-1]['Seconds'])
    elif not state.using_live_for_A and not state.analysis_df_A.empty: 
        max_t = max(max_t, state.analysis_df_A['Seconds'].max())
        
    if not state.analysis_df_B.empty: 
        max_t = max(max_t, state.analysis_df_B['Seconds'].max())
    
    # Establish a minimum window floor of 10 seconds for clean UI scaling
    max_t = max(10.0, max_t) 
    
    if UI.time_range and UI.time_range.max != max_t:
        old_max = UI.time_range.max 
        UI.time_range.max = float(max_t)
        
        # If the user's slider handle was sitting at the edge, drag it along with the new expanding limit
        if UI.time_range.value['max'] >= (old_max - 1): # type: ignore
            UI.time_range.value = {'min': UI.time_range.value['min'], 'max': float(max_t)} # type: ignore
        UI.time_range.update()

def reset_time_window() -> None:
    """
    Resets the UI slider handle endpoints back to full-width (0 to Max), 
    and triggers a cascading update to refresh all analysis graphs.
    """
    if UI.time_range: 
        UI.time_range.value = {'min': 0, 'max': float(UI.time_range.max)}
    update_analysis_view()

async def handle_analysis_upload(e: UploadEventArguments, dataset_slot: str) -> None:
    """
    Asynchronously reads an uploaded binary file event, converts the underlying CSV content 
    into a Pandas DataFrame, stores it within the corresponding application state slot, 
    and refreshes the visualization layout.

    Args:
        e (UploadEventArguments): NiceGUI native file upload event arguments object.
        dataset_slot (str): Target slot string identifier: either 'A' or 'B'.
    """
    try:
        # Wrap bytes stream into an IO handler for standard Pandas parsing
        df = pd.read_csv(io.BytesIO(await e.file.read()))
        
        if dataset_slot == 'A':
            state.analysis_df_A, state.using_live_for_A = df, False
            if UI.dataset_a_card: 
                UI.dataset_a_card.set_status(e.file.name, True)
        else:
            state.analysis_df_B = df
            if UI.dataset_b_card: 
                UI.dataset_b_card.set_status(e.file.name, True)
        
        # Reset and align view restrictions for the new dataset
        update_time_slider_limits()
        if UI.time_range: 
            UI.time_range.value = {'min': 0, 'max': UI.time_range.max}
        update_analysis_view()
        e.sender.reset()  # Reset NiceGUI uploader element state
    except Exception as ex:
        ui.notify(f'Error: {ex}', type=COLORS['negative'])

def set_analysis_A_to_live() -> None:
    """
    Redirects Dataset Slot A to read straight from the live telemetry buffer (`state.global_history`)
    instead of an uploaded static file.
    """
    if not state.global_history: 
        return ui.notify('No Live Capture data exists yet!', type=COLORS['warning'])
    
    state.using_live_for_A = True
    if UI.dataset_a_card: 
        UI.dataset_a_card.set_status('Current Live Run', True)
        
    update_time_slider_limits()
    if UI.time_range: 
        UI.time_range.value = {'min': 0, 'max': UI.time_range.max}
    update_analysis_view()
    
def set_analysis_B_to_live_ref() -> None:
    """
    Copies the current reference ghost profile loaded inside the Live Capture tab 
    and explicitly anchors it as static Dataset B for comparison.
    """
    if state.live_reference_df.empty: 
        return ui.notify('No Reference Ghost loaded in Live Capture!', type=COLORS['warning'])
        
    state.analysis_df_B = state.live_reference_df.copy()
    if UI.dataset_b_card: 
        UI.dataset_b_card.set_status('Live Capture Reference', True)
        
    update_time_slider_limits()
    if UI.time_range: 
        UI.time_range.value = {'min': 0, 'max': UI.time_range.max}    
    update_analysis_view()
    
def unload_analysis_dataset(slot: str) -> None:
    """
    Flushes out storage structures and clears memory matrices inside a specified analysis slot.

    Args:
        slot (str): The identifier label to clear ('A' or 'B').
    """
    if slot == 'A':
        state.analysis_df_A, state.using_live_for_A = pd.DataFrame(), False
        if UI.dataset_a_card: 
            UI.dataset_a_card.set_status('None Loaded', False)
    else:
        state.analysis_df_B = pd.DataFrame()
        if UI.dataset_b_card: 
            UI.dataset_b_card.set_status('None Loaded', False)
            
    update_time_slider_limits()
    update_analysis_view()

def toggle_analysis_log_scale(e: ValueChangeEventArguments) -> None:
    """
    Intercepts scale selection switches to transition the comparative chart's Y-axis type.

    Args:
        e (ValueChangeEventArguments): NiceGUI element change event returning True (log) or False (linear).
    """
    if UI.analysis_chart_card: 
        UI.analysis_chart_card.update_chart(is_log=e.value)

def update_analysis_view() -> None:
    """
    The main coordinator function of the Analysis tab. 
    Reads the UI filters, applies time-window slicing via Pandas, computes descriptive analytical 
    statistics, handles histogram bin allocations, and passes high-performance dataset blocks 
    to client-side Highcharts components via direct JS injection.
    """
    if UI.time_range is None or UI.analysis_sensor_select is None: 
        return
        
    # --- Step 1: Construct Metric List based on UI Dropdown Intersects ---
    selected_metrics = []
    active_sensors: List[str] = UI.analysis_sensor_select.value or []
    active_motors: List[str] = UI.analysis_motor_select.value or [] if UI.analysis_motor_select else []
    
    for s_id in active_sensors:
        if s_id in ['volt1', 'volt2']: 
            selected_metrics.append(s_id) # Volts are global, not bound to specific motors
        else:
            for m_id in active_motors: 
                selected_metrics.append(f"{m_id}_{s_id}") # e.g., "fl_rpm"
                
    if not selected_metrics: 
        return
    
    # --- Step 2: Fetch Base Core DataFrames ---
    df_A_raw = pd.DataFrame(state.global_history) if state.using_live_for_A else state.analysis_df_A
    df_B_raw = state.analysis_df_B
    
    # --- Step 3: Apply Slicing Filters via Global Window Constraints ---
    t_min, t_max = UI.time_range.value['min'], UI.time_range.value['max'] # type: ignore
    df_A = df_A_raw[(df_A_raw['Seconds'] >= t_min) & (df_A_raw['Seconds'] <= t_max)] if not df_A_raw.empty else df_A_raw
    df_B = df_B_raw[(df_B_raw['Seconds'] >= t_min) & (df_B_raw['Seconds'] <= t_max)] if not df_B_raw.empty else df_B_raw
    
    # Storage targets matching Highcharts configurations
    main_series: List[HighchartsLineSeries] = []
    delta_series: List[HighchartsDeltaSeries] = []
    dist_series: List[HighchartsDistSeries] = []
    stats: List[AnalysisStatRow] = []
    
    # Extract structural scale properties from the current chart configuration
    y_type = 'logarithmic' if UI.analysis_chart_card and hasattr(UI.analysis_chart_card, 'log_switch') and UI.analysis_chart_card.log_switch.value else 'linear'
    
    def _get_metric_color(metric_key: str) -> str:
        """Helper to safely map metrics to standard global hardware color codes."""
        if metric_key == 'volt1': return COLORS['volt1']
        if metric_key == 'volt2': return COLORS['volt2']
        for s in MOTOR_SENSORS:
            if metric_key.endswith(f"_{s['id']}"): 
                return s['color']
        return '#000000'

    # --- Step 4: Map Time Series Coordinates & Construct Graph Payloads ---
    for metric_name in selected_metrics:
        color = _get_metric_color(metric_name)
        mape_val = calculate_mape(df_A, df_B, metric_name) if not df_A.empty and not df_B.empty else "-"
        
        # Format Dataset A Series (Rendered Solid)
        if not df_A.empty and metric_name in df_A.columns:
            main_series.append({
                'name': f'A: {metric_name}', 
                'data': df_A[['Seconds', metric_name]].dropna().values.tolist(), 
                'color': color, 'lineWidth': 2, 'type': 'line', 'dashStyle': 'Solid', 'opacity': 1.0, 
                'id': f'main_A_{metric_name}', 'marker': {'enabled': False}
            })
            rms_A = np.sqrt(np.mean(df_A[metric_name].dropna()**2))
            stats.append({'id': f'{metric_name}_A', 'Metric': metric_name, 'Dataset': 'A (Solid)', 'Max': round(df_A[metric_name].max(), 2), 'Mean': round(df_A[metric_name].mean(), 2), 'RMS': round(rms_A, 2), 'Diff_vs_A': '-'})
            
        # Format Dataset B Series (Rendered Dashed)
        if not df_B.empty and metric_name in df_B.columns:
            main_series.append({
                'name': f'B: {metric_name}', 
                'data': df_B[['Seconds', metric_name]].dropna().values.tolist(), 
                'color': color, 'lineWidth': 2, 'dashStyle': 'ShortDash', 'opacity': 0.5, 
                'id': f'main_B_{metric_name}', 'marker': {'enabled': False}
            })
            rms_B = np.sqrt(np.mean(df_B[metric_name].dropna()**2))
            stats.append({'id': f'{metric_name}_B', 'Metric': metric_name, 'Dataset': 'B (Dashed)', 'Max': round(df_B[metric_name].max(), 2), 'Mean': round(df_B[metric_name].mean(), 2), 'RMS': round(rms_B, 2), 'Diff_vs_A': mape_val})
            
        # Format Delta Error Series (Dataset A vs Dataset B Relative Error Percentage)
        if not df_A.empty and not df_B.empty and metric_name in df_A.columns and metric_name in df_B.columns:
            t_A, val_A = df_A['Seconds'].values, df_A[metric_name].values
            t_B, val_B = df_B['Seconds'].values, df_B[metric_name].values
            if len(t_A) > 0 and len(t_B) > 0:
                interp_B = np.interp(t_A, t_B, val_B) # type: ignore
                safe_A = np.where(np.abs(val_A) < 0.001, 0.001, val_A)
                delta_vals = ((val_A - interp_B) / safe_A) * 100
                delta_series.append({
                    'name': f'Δ {metric_name} (%)', 
                    'data': [[float(t), float(d)] for t, d in zip(t_A, delta_vals)], 
                    'color': color, 'lineWidth': 2, 'marker': {'enabled': False}
                })

    # --- Step 5: Process Distribution Data Histograms ---
    dist_metric = UI.dist_metric_select.value if UI.dist_metric_select else None
    if dist_metric:
        dist_color = _get_metric_color(dist_metric)
        all_vals = []
        if not df_A.empty and dist_metric in df_A.columns: all_vals.extend(df_A[dist_metric].dropna().tolist())
        if not df_B.empty and dist_metric in df_B.columns: all_vals.extend(df_B[dist_metric].dropna().tolist())
        
        if all_vals:
            min_val, max_val = min(all_vals), max(all_vals)
            if min_val == max_val: max_val += 0.1 
            # Divide metrics spectrum uniformly into 30 static buckets
            global_bins = np.linspace(min_val, max_val, 30) 
            
            if not df_A.empty and dist_metric in df_A.columns:
                counts_A, _ = np.histogram(df_A[dist_metric].dropna(), bins=global_bins)
                # Map frequency buckets to highcharts bar formats: [[midpoint_x, height_y], ...]
                dist_series.append({'name': f'A: {dist_metric}', 'data': [[float(x), int(y)] for x, y in zip((global_bins[:-1] + global_bins[1:]) / 2, counts_A)], 'color': dist_color, 'opacity': 0.7})
                
            if not df_B.empty and dist_metric in df_B.columns:
                counts_B, _ = np.histogram(df_B[dist_metric].dropna(), bins=global_bins)
                dist_series.append({'name': f'B: {dist_metric}', 'data': [[float(x), int(y)] for x, y in zip((global_bins[:-1] + global_bins[1:]) / 2, counts_B)], 'color': '#64748b', 'opacity': 0.7})

    # --- Step 6: Dispatch Batch Datasets Directly to Components ---
    if UI.analysis_chart_card: UI.analysis_chart_card.update_chart(series_data=main_series, x_min=t_min, x_max=t_max, is_log=(y_type=='logarithmic'))
    if UI.delta_chart_card: UI.delta_chart_card.update_chart(series_data=delta_series, x_min=t_min, x_max=t_max)
    if UI.dist_chart_card: UI.dist_chart_card.update_chart(series_data=dist_series)
    
    # Flush analytical computation statistics out to rows in NiceGUI standard tables
    if UI.stats_table:
        # Cast to a generic dict of strings to objects to satisfy the invariant table list
        UI.stats_table.rows = cast(List[Dict[str, object]], stats)
        UI.stats_table.update()