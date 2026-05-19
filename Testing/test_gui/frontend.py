from nicegui import ui

from test_gui.tabs.capture.capture_view import build_capture_tab
from test_gui.tabs.capture.capture_logic import update_master_stream
from test_gui.tabs.analysis.analysis_view import build_analysis_tab

from test_gui.constants import GUI_TITLE

@ui.page('/')
def index() -> None:
    """
    Constructs the main index page (`/`) for the application.
    
    Layout Structure:
      - A fixed-screen container to prevent global body scrollbars (internal scrolling 
        is handled dynamically within each specific tab pane).
      - Header Row: Displays the app title and the central tab navigation menu.
      - Main Content Area: A dynamic tab panel container that swaps between the 
        Live Capture and Post-Run Analysis interface layouts.
        
    Additionally, this function launches the high-frequency UI timer responsible 
    for polling the hardware DAQ for live telemetry updates.
    """
    ui.page_title(GUI_TITLE)

    # Global wrapper ensuring the app spans the viewport and handles inner overflow correctly
    with ui.column().classes('fixed inset-0 p-4 w-full max-w-screen-2xl mx-auto flex-nowrap overflow-hidden'):
        
        # --- HEADER ROW ---
        with ui.row().classes('w-full items-end border-b-2 border-slate-300 pb-1 mb-2 flex-nowrap'):
            
            # Left section: Application Title
            with ui.row().classes('flex-1 justify-start'):
                ui.label(GUI_TITLE).classes('text-2xl font-extrabold text-gray-800')
            
            # Center section: Navigation Tabs
            with ui.row().classes('flex-none justify-center'):
                with ui.tabs() as tabs:
                    capture_tab = ui.tab('Live Capture', icon='monitor').classes('py-0 min-h-[40px]')
                    analysis_tab = ui.tab('Analysis', icon='analytics').classes('py-0 min-h-[40px]')
            
            # Right section: Empty spacer flex-box to perfectly center the tabs
            with ui.row().classes('flex-1'):
                ui.space()

        # --- MAIN CONTENT AREA ---
        # Houses the individual view layouts mapped to the navigation tabs above
        with ui.tab_panels(tabs, value=capture_tab).classes('w-full flex-grow min-h-0 min-w-0 bg-transparent p-0'):
            
            # Sub-routing to the Live Data Capture Interface
            with ui.tab_panel(capture_tab).classes('p-0 h-full flex flex-col flex-nowrap'):
                build_capture_tab()

            # Sub-routing to the Post-Run Telemetry Analysis Interface
            with ui.tab_panel(analysis_tab).classes('p-0 h-full'): 
                build_analysis_tab()

    # Start the global data acquisition polling timer at 10Hz (0.1 seconds)
    # This must be started here at the end of the UI build to ensure all chart components exist!
    ui.timer(0.1, update_master_stream)