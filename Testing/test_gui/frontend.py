from nicegui import ui

from test_gui.tabs.capture.capture_view import build_capture_tab
from test_gui.tabs.capture.capture_logic import update_master_stream
from test_gui.tabs.analysis.analysis_view import build_analysis_tab

from test_gui.constants import GUI_TITLE

@ui.page('/')
def index():
    ui.page_title(GUI_TITLE)

    with ui.column().classes('fixed inset-0 p-4 w-full max-w-screen-2xl mx-auto flex-nowrap overflow-hidden'):
        
        # --- HEADER ROW ---
        with ui.row().classes('w-full items-end border-b-2 border-slate-300 pb-1 mb-2 flex-nowrap'):
            with ui.row().classes('flex-1 justify-start'):
                ui.label(GUI_TITLE).classes('text-2xl font-extrabold text-gray-800')
            
            with ui.row().classes('flex-none justify-center'):
                with ui.tabs() as tabs:
                    capture_tab = ui.tab('Live Capture', icon='monitor').classes('py-0 min-h-[40px]')
                    analysis_tab = ui.tab('Analysis', icon='analytics').classes('py-0 min-h-[40px]')
            
            with ui.row().classes('flex-1'):
                ui.space()

        # --- MAIN CONTENT AREA ---
        with ui.tab_panels(tabs, value=capture_tab).classes('w-full flex-grow min-h-0 min-w-0 bg-transparent p-0'):
            
            with ui.tab_panel(capture_tab).classes('p-0 h-full flex flex-col flex-nowrap'):
                build_capture_tab()

            with ui.tab_panel(analysis_tab).classes('p-0 h-full'): 
                build_analysis_tab()

    # Start the timer here at the end of the UI build!
    ui.timer(0.1, update_master_stream)