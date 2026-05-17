# test_gui/tabs/analysis/analysis_view.py
from nicegui import ui

from test_gui import state
from test_gui.ui_registry import UI
from test_gui.components.chart_card import ChartCard
from test_gui.components.data_source import DataSourceCard

# Import the controller logic!
from . import analysis_logic

def build_dist_header():
    with ui.row().classes('w-full justify-between items-center mb-2'):
        ui.label('Data Distribution (Windowed)').classes('text-lg font-bold text-gray-800')
        UI.dist_metric_select = ui.select(
            options=state.analysis_options, 
            value='fl_rpm', 
            on_change=analysis_logic.update_analysis_view
        ).classes('w-72 min-w-0 mb-6').props('options-dense')

def build_analysis_tab():
    with ui.row().classes('w-full gap-6 flex-nowrap items-start h-full min-w-0'):
        
        # --- LEFT MENU (Settings & Datasets) ---
        with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
            ui.label('Analysis Setup').classes('text-lg font-bold text-gray-800 mb-4')
            
            ui.label('1. Metrics to Plot').classes('text-xs font-bold text-gray-500 uppercase mb-1')
            with ui.row().classes('w-full items-start gap-4 mb-6 flex-nowrap'):
                UI.analysis_motor_select = ui.select(
                    options=state.analysis_motor_options, multiple=True, 
                    value=list(state.analysis_motor_options.keys()), label='Filter by Motor', 
                    on_change=analysis_logic.update_analysis_view
                ).classes('flex-1 min-w-0')
                
                UI.analysis_sensor_select = ui.select(
                    options=state.analysis_sensor_options, multiple=True, 
                    value=list(state.analysis_sensor_options.keys()), label='Filter by Sensor Type', 
                    on_change=analysis_logic.update_analysis_view
                ).classes('flex-1 min-w-0')
            
            # --- DATA SOURCES ---
            ui.label('2. Data Sources').classes('text-xs font-bold text-slate-500 uppercase mb-2')
            
            UI.dataset_a_card = DataSourceCard(
                title='Dataset A (Solid)', theme_color='blue', bg_color='blue', icon_name='timeline', 
                live_btn_text='Sync Live', live_btn_icon='refresh', 
                on_live_click=analysis_logic.set_analysis_A_to_live, 
                on_upload=lambda e: analysis_logic.handle_analysis_upload(e, 'A'), 
                on_clear=lambda: analysis_logic.unload_analysis_dataset('A')
            )
            
            UI.dataset_b_card = DataSourceCard(
                title='Dataset B (Dashed)', theme_color='slate', bg_color='slate', icon_name='show_chart', 
                live_btn_text='Use Live Ref', live_btn_icon='move_down', 
                on_live_click=analysis_logic.set_analysis_B_to_live_ref, 
                on_upload=lambda e: analysis_logic.handle_analysis_upload(e, 'B'), 
                on_clear=lambda: analysis_logic.unload_analysis_dataset('B')
            )

        # --- RIGHT AREA (Graphs & Stats) ---
        with ui.column().classes('w-3/4 flex-grow h-full gap-0 bg-white border shadow-sm rounded min-w-0'):
            
            # Global Time Window Card
            with ui.card().classes('w-full p-4 bg-slate-50 border-b-2 border-blue-400 z-10 shadow-sm rounded-none'):
                with ui.row().classes('w-full justify-start items-center mb-1 gap-4'):
                    ui.label('Global Time Window (Seconds)').classes('text-sm font-bold text-gray-800')
                    ui.button('Reset', icon='restart_alt', on_click=analysis_logic.reset_time_window).props('outline size=sm color=slate padding=xs')
                UI.time_range = ui.range(min=0, max=10, value={'min': 0, 'max': 10}, on_change=analysis_logic.update_analysis_view).props('label-always color=blue').classes('w-full px-4')
            
            # Scrollable Graphs Container
            with ui.element('div').classes('w-full flex-grow overflow-y-auto overflow-x-hidden'):
                with ui.column().classes('w-full items-stretch gap-4 min-w-0 p-4 overflow-x-hidden'):
                    
                    UI.analysis_chart_card = ChartCard(
                        options={
                            'chart': {'type': 'line'}, 'title': {'text': 'Comparison Overlay'}, 
                            'xAxis': {'title': {'text': 'Seconds'}}, 'yAxis': {'type': 'linear'}, 
                            'tooltip': {'shared': True, 'crosshairs': True}, 'series': []
                        }, 
                        height_px=400, show_log_toggle=True, 
                        on_log_toggle=analysis_logic.toggle_analysis_log_scale
                    )
                    
                    UI.delta_chart_card = ChartCard(
                        options={
                            'chart': {'type': 'line'}, 'title': {'text': 'Relative Error (Dataset A vs B)'}, 
                            'xAxis': {'title': {'text': 'Seconds'}}, 'yAxis': {'title': {'text': '% Difference'}}, 
                            'tooltip': {'shared': True, 'crosshairs': True, 'valueSuffix': '%'}, 'series': []
                        }, 
                        height_px=250
                    )
                    
                    UI.dist_chart_card = ChartCard(
                        options={
                            'chart': {'type': 'column'}, 'title': {'text': ''}, 
                            'xAxis': {'title': {'text': 'Metric Value'}}, 'yAxis': {'title': {'text': 'Frequency (Points)'}}, 
                            'tooltip': {'shared': True}, 'plotOptions': {'column': {'pointPadding': 0, 'groupPadding': 0.1, 'borderWidth': 0}}, 'series': []
                        }, 
                        height_px=250, header_elements=build_dist_header
                    )
                    
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