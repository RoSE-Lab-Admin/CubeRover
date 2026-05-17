from nicegui import ui
from test_gui import state
from test_gui.ui_registry import UI
from test_gui.constants import COLORS
from test_gui.components.chart_card import ChartCard
from test_gui.constants import MOTORS, MOTOR_SENSORS

# Import the controller logic!
from . import capture_logic

def build_capture_tab():
    """Builds the UI elements for the Capture Tab."""
    
    # --- TOP CONTROL BAR ---
    with ui.card().classes('w-full mb-4 p-4 bg-slate-50 shadow border border-slate-200 flex-nowrap'):
        with ui.row().classes('w-full justify-between items-center'):
            
            # Left Group: Test Engine Controls
            with ui.row().classes('gap-4 items-center'):
                available_tests = state.engine.get_available_tests()
                first_test_key = list(available_tests.keys())[0] if available_tests else None
                
                with ui.column().classes('gap-1'):
                    UI.capture_mode = ui.select(
                        options={key: value['name'] for key, value in available_tests.items()},
                        value=first_test_key, label='Test Profile',
                        on_change=lambda e: desc_label.set_text(available_tests[e.value]['description'])
                    ).classes('w-48').props('dense outlined bg-color=white')
                    desc_label = ui.label(available_tests[first_test_key]['description'] if first_test_key else '').classes('text-xs text-gray-500 italic max-w-[192px] leading-tight')
                
                UI.btn_start = ui.button('Start Capture', on_click=capture_logic.toggle_master).props('color=green icon=play_arrow')
                ui.button('Reset', on_click=capture_logic.reset_master_live).props('color=grey icon=refresh outline')
            
            # Right Group: File IO
            with ui.row().classes('gap-4 items-center'):
                ui.button('Save Run CSV', icon='save', on_click=capture_logic.download_master_csv).props('color=blue')
                
                ref_uploader = ui.upload(auto_upload=True, on_upload=capture_logic.load_live_reference).props('accept=".csv"').classes('hidden')
                UI.btn_load_ref = ui.button('Load Ref Ghost', icon='upload', on_click=lambda: ref_uploader.run_method('pickFiles')).props('color=slate outline')
                
                with ui.row().classes('bg-slate-200 text-slate-800 rounded-full pl-3 pr-1 py-1 items-center gap-2 hidden flex-nowrap') as UI.ref_pill:
                    ui.icon('description', size='sm')
                    UI.ref_pill_label = ui.label('filename.csv').classes('text-sm font-semibold truncate max-w-[200px]')
                    ui.button(icon='close', on_click=capture_logic.unload_live_reference).props('flat round size=sm padding=none text-color=slate')
    
    # --- MAIN CONTENT AREA ---
    with ui.row().classes('w-full flex-grow min-h-0 gap-6 flex-nowrap min-w-0'):
        
        # Left Sidebar: Visibility Switches
        with ui.column().classes('w-1/4 min-w-[250px] p-4 bg-white shadow-sm border rounded h-full overflow-y-auto'):
            ui.label('Chart Settings').classes('text-lg font-bold text-gray-800 mb-2')
            
            # Re-added the Left Panel Log Switch
            UI.switch_capture_panel = ui.switch('Logarithmic Y-Axis', on_change=capture_logic.toggle_log_scale).classes('mb-4 font-semibold text-blue-600')
            ui.separator().classes('mb-4')

            with ui.card().classes('w-full p-4 bg-white shadow-sm border mb-4'):
                ui.label('Active Motors').classes('text-lg font-bold text-gray-800 mb-2')
                with ui.column().classes('w-full gap-2'):
                    for m in MOTORS: 
                        UI.motor_switches[m['id']] = ui.switch(m['name'], value=True, on_change=capture_logic.update_chart_visibility).classes('w-full')

            with ui.card().classes('w-full p-4 bg-white shadow-sm border'):
                ui.label('Active Sensors').classes('text-lg font-bold text-gray-800 mb-2')
                with ui.column().classes('w-full gap-2'):
                    for s in MOTOR_SENSORS:
                        UI.sensor_switches[s['id']] = ui.switch(s['name'], value=True, on_change=capture_logic.update_chart_visibility).classes('w-full')
                    ui.separator().classes('my-2 w-full')
                    UI.sensor_switches['volt1'] = ui.switch("Bus Voltage 1 (V)", value=True, on_change=capture_logic.update_chart_visibility).classes('w-full')
                    UI.sensor_switches['volt2'] = ui.switch("Bus Voltage 2 (V)", value=True, on_change=capture_logic.update_chart_visibility).classes('w-full')

        # Right Area: The Highcharts Canvas
        with ui.column().classes('w-3/4 flex-grow h-full min-w-0'):
            
            # Setup Series config (No Y-Axis index necessary for single scale)
            series_list = []
            for s in MOTOR_SENSORS:
                for m in MOTORS: 
                    series_list.append({'id': f"live_{s['id']}_{m['id']}", 'name': f"{m['name']} {s['name']}", 'data': [], 'color': s['color'], 'dashStyle': m['dash'], 'marker': {'enabled': False}})
            
            series_list.append({'id': 'live_volt1', 'name': 'Bus Voltage 1 (V)', 'data': [], 'color': COLORS['volt1'], 'marker': {'enabled': False}})
            series_list.append({'id': 'live_volt2', 'name': 'Bus Voltage 2 (V)', 'data': [], 'color': COLORS['volt2'], 'marker': {'enabled': False}})
            
            # Instantiating our custom Component with Log capabilities
            UI.master_chart_card = ChartCard(
                options={
                    'chart': {'type': 'line', 'animation': False}, 
                    'title': {'text': 'Data Capture'}, 
                    'xAxis': {'title': {'text': 'Seconds'}}, 
                    'yAxis': {'type': 'linear'},
                    'tooltip': {'shared': True, 'crosshairs': True}, 
                    'series': series_list
                },
                height_px=400,
                show_log_toggle=True,
                on_log_toggle=capture_logic.toggle_log_scale
            ).classes('h-full')