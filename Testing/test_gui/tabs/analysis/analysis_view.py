# test_gui/tabs/analysis/analysis_view.py
from nicegui import ui

from test_gui import state
from test_gui.ui_registry import UI
from test_gui.components.chart_card import ChartCard
from test_gui.components.data_source import DataSourceCard
from test_gui.constants import ANALYSIS_OPTIONS, ANALYSIS_MOTOR_OPTIONS, ANALYSIS_SENSOR_OPTIONS

# Import the controller logic to bind UI component triggers
from . import analysis_logic

def build_dist_header() -> None:
    """
    Constructs the dynamic header row for the Data Distribution ChartCard.
    
    This function is intended to be passed as a delayed layout injection callback 
    (lambda) to a ChartCard instance, allowing it to render a selection dropdown 
    directly adjacent to its top title section.
    """
    with ui.row().classes('w-full justify-between items-center mb-2'):
        ui.label('Data Distribution (Windowed)').classes('text-lg font-bold text-gray-800')
        
        # Metric dropdown selection targeting the distribution histogram focus area
        UI.dist_metric_select = ui.select(
            options=ANALYSIS_OPTIONS, 
            value='fl_rpm', 
            on_change=analysis_logic.update_analysis_view
        ).classes('w-72 min-w-0 mb-6').props('options-dense')

def build_analysis_tab() -> None:
    """
    Constructs the core grid layout and initializes all reactive UI nodes 
    housed within the 'Analysis' module tab context.
    
    Layout Architecture:
    - Main container: 2-column full-width horizontal grid layout matrix.
      ├─ Left Column (1/4 Width): Dedicated parameters menu and data import slot controls.
      └─ Right Column (3/4 Width): Visual stack consisting of 3 interactive analytics charts
         and a comparative statistics processing table.
    """
    with ui.row().classes('w-full gap-6 flex-nowrap items-start h-full min-w-0'):
        
        # =====================================================================
        # 1. LEFT COLUMN: PARAMETERS SETUP & DATA IMPORT CONTROLS
        # =====================================================================
        with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
            ui.label('Analysis Setup').classes('text-lg font-bold text-gray-800 mb-4')
            
            # --- Sector A: Motor & Sensor Multiselect Dropdowns ---
            ui.label('1. Metrics to Plot').classes('text-xs font-bold text-slate-400 uppercase tracking-wider mb-1')
            
            # Conditionally render motor filters only if there is more than 1 motor configured
            if len(ANALYSIS_MOTOR_OPTIONS) > 1:
                UI.analysis_motor_select = ui.select(
                    options=ANALYSIS_MOTOR_OPTIONS,
                    multiple=True,
                    label='Select Motors',
                    value=list(ANALYSIS_MOTOR_OPTIONS.keys()),  # Select all motors by default
                    on_change=analysis_logic.update_analysis_view
                ).classes('w-full mb-2').props('multiple use-chips options-dense label-color=slate')
                
            UI.analysis_sensor_select = ui.select(
                options=ANALYSIS_SENSOR_OPTIONS,
                multiple=True,
                label='Select Sensors / Channels',
                value=['rpm'],  # Focus on speed telemetry profiles by default
                on_change=analysis_logic.update_analysis_view
            ).classes('w-full mb-6').props('multiple use-chips options-dense label-color=slate')
            
            # --- Sector B: Time Navigation Bounds Slider ---
            ui.label('2. Time Range Interval').classes('text-xs font-bold text-slate-400 uppercase tracking-wider mb-1')
            with ui.row().classes('w-full items-center justify-between gap-0 mb-1'):
                # Interactive range slider spanning from zero to maximum duration discovered in sources
                UI.time_range = ui.range(
                    min=0, max=10, value={'min': 0, 'max': 10}, 
                    step=0.1, 
                    on_change=analysis_logic.update_analysis_view
                ).classes('flex-grow px-2')
                
                # Snap-back utility resets viewing limits back to default comprehensive boundaries
                ui.button(icon='restart_alt', on_click=analysis_logic.reset_time_window).props('flat round size=sm color=slate')

            ui.separator().classes('my-4')
            
            # --- Sector C: Dataset Target Selection Slots ---
            ui.label('3. Data Sources').classes('text-xs font-bold text-slate-400 uppercase tracking-wider mb-3')
            
            # Primary baseline data source interface component
            UI.dataset_a_card = DataSourceCard(
                title="Dataset A (Solid)", 
                theme_color="blue", 
                bg_color="blue", 
                icon_name="timeline",
                live_btn_text="Sync Live", 
                live_btn_icon="refresh",
                on_live_click=analysis_logic.set_analysis_A_to_live,
                on_upload=lambda e: analysis_logic.handle_analysis_upload(e, 'A'),
                on_clear=lambda: analysis_logic.unload_analysis_dataset('A')
            )
            
            # Secondary comparative data source interface component
            UI.dataset_b_card = DataSourceCard(
                title="Dataset B (Dashed)", 
                theme_color="slate", 
                bg_color="slate", 
                icon_name="history",
                live_btn_text="Use Live Ref", 
                live_btn_icon="difference",
                on_live_click=analysis_logic.set_analysis_B_to_live_ref,
                on_upload=lambda e: analysis_logic.handle_analysis_upload(e, 'B'),
                on_clear=lambda: analysis_logic.unload_analysis_dataset('B')
            )

        # =====================================================================
        # 2. RIGHT COLUMN: GRAPH VISUALIZATION STACK & ANALYTICS TABLES
        # =====================================================================
        with ui.column().classes('w-3/4 flex-grow h-full overflow-y-auto pr-2 gap-4 min-w-0'):
            
            # --- Visualization Slot 1: Primary Comparative Time Series Chart ---
            UI.analysis_chart_card = ChartCard(
                options={
                    'chart': {'type': 'line', 'zoomType': 'x'}, 
                    'title': {'text': 'Time Series Comparison'}, 
                    'xAxis': {'title': {'text': 'Seconds'}}, 
                    'yAxis': {'title': {'text': 'Telemetry Amplitude'}}, 
                    'tooltip': {'shared': True}, 
                    'series': []
                },
                height_px=400, 
                show_log_toggle=True, 
                on_log_toggle=analysis_logic.toggle_analysis_log_scale
            )
            
            # Sub-grid dividing the secondary analytical widgets side-by-side
            with ui.row().classes('w-full gap-4 flex-nowrap items-start min-w-0'):
                
                # --- Visualization Slot 2: Point-to-Point Relative Delta Error ---
                with ui.column().classes('w-1/2 min-w-0'):
                    UI.delta_chart_card = ChartCard(
                        options={
                            'chart': {'type': 'line', 'zoomType': 'x'}, 
                            'title': {'text': 'Relative Error Delta (A vs B)'}, 
                            'xAxis': {'title': {'text': 'Seconds'}}, 
                            'yAxis': {'title': {'text': 'Deviation Error (%)'}}, 
                            'tooltip': {'shared': True}, 
                            'series': []
                        }, 
                        height_px=250
                    )
                
                # --- Visualization Slot 3: Statistical Frequency Distribution Histogram ---
                with ui.column().classes('w-1/2 min-w-0'):
                    UI.dist_chart_card = ChartCard(
                        options={
                            'chart': {'type': 'column'}, 
                            'title': {'text': ''}, 
                            'xAxis': {'title': {'text': 'Metric Value'}}, 
                            'yAxis': {'title': {'text': 'Frequency (Points)'}}, 
                            'tooltip': {'shared': True}, 
                            'plotOptions': {'column': {'pointPadding': 0, 'groupPadding': 0.1, 'borderWidth': 0}}, 
                            'series': []
                        }, 
                        height_px=250, 
                        header_elements=build_dist_header  # Dropdown context injection setup
                    )
                    
                    # --- Analytics Widget 4: Windowed Calculation Summary Data Table ---
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
                            rows=[], 
                            row_key='id'
                        ).classes('w-full')