from typing import Callable
from nicegui import ui
from nicegui.events import UploadEventArguments

class DataSourceCard(ui.column):
    def __init__(
            self, 
            title: str, 
            theme_color: str, 
            bg_color: str, 
            icon_name: str, 
            live_btn_text: str, 
            live_btn_icon: str,
            on_live_click: Callable[[], None], 
            on_upload: Callable[[UploadEventArguments], None], 
            on_clear: Callable[[], None]
        ):
        """
        Initializes the DataSourceCard.

        Args:
            title (str): Header text displayed above the status pill (e.g., 'Dataset A (Solid)').
            theme_color (str): Tailwind color prefix used for buttons and labels (e.g., 'blue', 'slate').
            bg_color (str): Tailwind color prefix used for the status pill background (e.g., 'blue', 'slate').
            icon_name (str): Material icon name displayed inside the status pill (e.g., 'timeline').
            live_btn_text (str): Label for the left action button (e.g., 'Sync Live', 'Use Live Ref').
            live_btn_icon (str): Material icon name for the left action button (e.g., 'refresh').
            on_live_click (callable): Callback executed when the live/reference button is pressed.
            on_upload (callable): Callback executed when a file is chosen. Receives an UploadEventArguments object.
            on_clear (callable): Callback executed when the 'close' button on the status pill is pressed.
        """
        super().__init__()
        self.classes('w-full p-3 bg-slate-50 rounded border border-slate-200 gap-2 mb-2')
        
        with self:
            # Title element
            ui.label(title).classes(f'text-[10px] font-bold text-{theme_color}-500 uppercase tracking-wider')
            
            # Status Pill Tracker
            with ui.row().classes(f'bg-{bg_color}-100 text-{theme_color}-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                ui.icon(icon_name, size='xs')
                self.status_label = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                self.clear_btn = ui.button(icon='close', on_click=on_clear).props(f'flat round size=xs padding=none text-color={theme_color}').classes('hidden')

            # Action Buttons Row
            with ui.row().classes('w-full gap-2'):
                ui.button(live_btn_text, icon=live_btn_icon, on_click=on_live_click).props('size=sm color=slate outline').classes('flex-none')
                uploader = ui.upload(auto_upload=True, on_upload=on_upload).props('accept=".csv"').classes('hidden')
                ui.button('Load CSV', on_click=lambda: uploader.run_method('pickFiles')).props(f'size=sm color={theme_color} {"outline" if theme_color == "slate" else ""}').classes('flex-grow')

    def set_status(self, text: str, is_active: bool):
        """
        Updates the descriptive text inside the status pill and toggles the visibility 
        of the clear ('close') action button.

        Args:
            text (str): The filename or text description to display inside the pill.
            is_active (bool): If True, shows the clear button and colors text. If False, hides it.
        """
        self.status_label.set_text(text)
        if is_active:
            self.status_label.classes(remove='text-slate-500', add='text-gray-800')
            self.clear_btn.classes(remove='hidden')
        else:
            self.status_label.classes(remove='text-gray-800', add='text-slate-500')
            self.clear_btn.classes('hidden')