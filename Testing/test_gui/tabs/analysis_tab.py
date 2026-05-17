import pandas as pd
import numpy as np
import io
import json
from nicegui import ui

from test_gui import state
from test_gui.ui_registry import UI

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
        old_max = UI.time_range.max 
        UI.time_range.max = float(max_t)
        
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
        content = await e.file.read()
        df = pd.read_csv(io.BytesIO(content))
        
        if dataset_slot == 'A':
            state.analysis_df_A = df
            state.using_live_for_A = False
            if UI.label_A_status:
                UI.label_A_status.set_text(e.file.name)
                UI.label_A_status.classes(remove='text-slate-500', add='text-blue-800')
            if UI.btn_clear_A:
                UI.btn_clear_A.classes(remove='hidden')
        else:
            state.analysis_df_B = df
            if UI.label_B_status:
                UI.label_B_status.set_text(e.file.name)
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
        UI.btn_clear_A.classes(remove='hidden')
    
    update_time_slider_limits()
    if UI.time_range:
        UI.time_range.value = {'min': 0, 'max': UI.time_range.max}
    
    update_analysis_view()
    ui.notify('Dataset A set to Current Live Run')
    
def set_analysis_B_to_live_ref():
    if state.live_reference_df.empty:
        ui.notify('No Reference Ghost loaded in Live Capture!', type='warning')
        return
        
    state.analysis_df_B = state.live_reference_df.copy()
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
        state.analysis_df_A = pd.DataFrame()
        state.using_live_for_A = False
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
        
    selected_metrics = []
    active_sensors = UI.analysis_sensor_select.value or []
    active_motors = UI.analysis_motor_select.value or [] if UI.analysis_motor_select else []
    
    for s_id in active_sensors:
        if s_id in ['volt1', 'volt2']:
            selected_metrics.append(s_id)
        else:
            for m_id in active_motors:
                selected_metrics.append(f"{m_id}_{s_id}")
                
    if not selected_metrics: 
        return
    
    df_A_raw = pd.DataFrame(state.global_history) if state.using_live_for_A else state.analysis_df_A
    df_B_raw = state.analysis_df_B
    
    t_min = UI.time_range.value['min'] # type: ignore
    t_max = UI.time_range.value['max'] # type: ignore
    
    df_A = df_A_raw[(df_A_raw['Seconds'] >= t_min) & (df_A_raw['Seconds'] <= t_max)] if not df_A_raw.empty else df_A_raw
    df_B = df_B_raw[(df_B_raw['Seconds'] >= t_min) & (df_B_raw['Seconds'] <= t_max)] if not df_B_raw.empty else df_B_raw
    
    main_series, delta_series, dist_series, stats = [], [], [], []
    y_type = 'logarithmic' if UI.analysis_log_scale and UI.analysis_log_scale.value else 'linear'
    
    def _get_metric_color(metric_key):
        if metric_key == 'volt1': return '#f59e0b'
        if metric_key == 'volt2': return '#d97706'
        for s in state.MOTOR_SENSORS:
            if metric_key.endswith(f"_{s['id']}"): return s['color']
        return '#000000'

    for metric_name in selected_metrics:
        color = _get_metric_color(metric_name)
        mape_val = state.calculate_mape(df_A, df_B, metric_name) if not df_A.empty and not df_B.empty else "-"
        
        if not df_A.empty and metric_name in df_A.columns:
            main_series.append({
                'name': f'A: {metric_name}',
                'data': df_A[['Seconds', metric_name]].dropna().values.tolist(),
                'color': color, 'lineWidth': 2, 'type': 'line', 'dashStyle': 'Solid',
                'opacity': 1.0, 'id': f'main_A_{metric_name}', 'marker': {'enabled': False}
            })
            rms_A = np.sqrt(np.mean(df_A[metric_name].dropna()**2))
            stats.append({'id': f'{metric_name}_A', 'Metric': metric_name, 'Dataset': 'A (Solid)', 'Max': round(df_A[metric_name].max(), 2), 'Mean': round(df_A[metric_name].mean(), 2), 'RMS': round(rms_A, 2), 'Diff_vs_A': '-'})
            
        if not df_B.empty and metric_name in df_B.columns:
            main_series.append({
                'name': f'B: {metric_name}',
                'data': df_B[['Seconds', metric_name]].dropna().values.tolist(),
                'color': color, 'lineWidth': 2, 'dashStyle': 'ShortDash', 
                'opacity': 0.5, 'id': f'main_B_{metric_name}', 'marker': {'enabled': False}
            })
            rms_B = np.sqrt(np.mean(df_B[metric_name].dropna()**2))
            stats.append({'id': f'{metric_name}_B', 'Metric': metric_name, 'Dataset': 'B (Dashed)', 'Max': round(df_B[metric_name].max(), 2), 'Mean': round(df_B[metric_name].mean(), 2), 'RMS': round(rms_B, 2), 'Diff_vs_A': mape_val})
            
        if not df_A.empty and not df_B.empty and metric_name in df_A.columns and metric_name in df_B.columns:
            t_A, val_A = df_A['Seconds'].values, df_A[metric_name].values
            t_B, val_B = df_B['Seconds'].values, df_B[metric_name].values
            
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

def build_analysis_tab():
    """Builds the UI elements for the Analysis Tab."""
    with ui.row().classes('w-full gap-6 flex-nowrap items-start h-full min-w-0'):
        # --- LEFT MENU ---
        with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
            ui.label('Analysis Setup').classes('text-lg font-bold text-gray-800 mb-4')
            
            ui.label('1. Metrics to Plot').classes('text-xs font-bold text-gray-500 uppercase mb-1')
            with ui.row().classes('w-full items-start gap-4 mb-6 flex-nowrap'):
                UI.analysis_motor_select = ui.select(
                    options=state.analysis_motor_options, multiple=True,
                    value=list(state.analysis_motor_options.keys()), label='Filter by Motor',
                    on_change=update_analysis_view
                ).classes('flex-1 min-w-0')
                
                UI.analysis_sensor_select = ui.select(
                    options=state.analysis_sensor_options, multiple=True,
                    value=list(state.analysis_sensor_options.keys()), label='Filter by Sensor Type',
                    on_change=update_analysis_view
                ).classes('flex-1 min-w-0')
            
            # --- GROUPED DATA SOURCES ---
            with ui.column().classes('w-full p-3 bg-slate-50 rounded border border-slate-200 gap-2'):
                ui.label('2. Data Sources').classes('text-xs font-bold text-slate-500 uppercase mb-2')
                
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

                ui.label('Dataset B (Dashed)').classes('text-[10px] font-bold text-slate-500 uppercase tracking-wider')
                with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                    ui.icon('show_chart', size='xs')
                    UI.label_B_status = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                    UI.btn_clear_B = ui.button(icon='close', on_click=lambda: unload_analysis_dataset('B')).props('flat round size=xs padding=none text-color=slate').classes('hidden')

                with ui.row().classes('w-full gap-2 mb-2'):
                    ui.button('Use Live Ref', icon='move_down', on_click=set_analysis_B_to_live_ref).props('size=sm color=slate outline').classes('flex-none')
                    up_B = ui.upload(auto_upload=True, on_upload=upload_dataset_B).props('accept=".csv"').classes('hidden')
                    ui.button('Load CSV', on_click=lambda: up_B.run_method('pickFiles')).props('size=sm color=slate outline').classes('flex-grow')

        # --- RIGHT GRAPHS ---
        with ui.column().classes('w-3/4 flex-grow h-full gap-0 bg-white border shadow-sm rounded min-w-0'):
            with ui.card().classes('w-full p-4 bg-slate-50 border-b-2 border-blue-400 z-10 shadow-sm rounded-none'):
                with ui.row().classes('w-full justify-start items-center mb-1 gap-4'):
                    ui.label('Global Time Window (Seconds)').classes('text-sm font-bold text-gray-800')
                    ui.button('Reset', icon='restart_alt', on_click=reset_time_window).props('outline size=sm color=slate padding=xs')
                UI.time_range = ui.range(min=0, max=10, value={'min': 0, 'max': 10}, on_change=update_analysis_view).props('label-always color=blue').classes('w-full px-4')
            
            with ui.element('div').classes('w-full flex-grow overflow-y-auto overflow-x-hidden'):
                with ui.column().classes('w-full items-stretch gap-4 min-w-0 p-4 overflow-x-hidden'):
                    with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0 relative'):
                        with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                            ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                            UI.analysis_log_scale = ui.switch(on_change=update_analysis_view).props('size=sm')
                        with ui.element('div').classes('relative w-full h-[400px]'):
                            UI.analysis_chart = ui.highchart({
                                'chart': {'type': 'line'}, 'title': {'text': 'Comparison Overlay'},
                                'xAxis': {'title': {'text': 'Seconds'}}, 'yAxis': {'type': 'linear'},
                                'tooltip': {'shared': True, 'crosshairs': True}, 'series': []
                            }).classes('absolute inset-0 w-full h-full')
                    
                    with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0'):
                        with ui.element('div').classes('relative w-full h-[250px]'):
                            UI.delta_chart = ui.highchart({
                                'chart': {'type': 'line'}, 'title': {'text': 'Relative Error (Dataset A vs B)'},
                                'xAxis': {'title': {'text': 'Seconds'}}, 'yAxis': {'title': {'text': '% Difference'}},
                                'tooltip': {'shared': True, 'crosshairs': True, 'valueSuffix': '%'}, 'series': []
                            }).classes('absolute inset-0 w-full h-full')
                        
                    with ui.card().classes('w-full p-4 bg-white shadow-sm border min-w-0'):                                
                        with ui.row().classes('w-full justify-between items-center mb-2'):
                            ui.label('Data Distribution (Windowed)').classes('text-lg font-bold text-gray-800')
                            UI.dist_metric_select = ui.select(options=state.analysis_options, value='fl_rpm', on_change=update_analysis_view).classes('w-72 min-w-0 mb-6').props('options-dense')
                        with ui.element('div').classes('relative w-full h-[250px]'):
                            UI.dist_chart = ui.highchart({
                                'chart': {'type': 'column'}, 'title': {'text': ''}, 
                                'xAxis': {'title': {'text': 'Metric Value'}}, 'yAxis': {'title': {'text': 'Frequency (Points)'}},
                                'tooltip': {'shared': True}, 'plotOptions': {'column': {'pointPadding': 0, 'groupPadding': 0.1, 'borderWidth': 0}}, 'series': []
                            }).classes('absolute inset-0 w-full h-full')
                    
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