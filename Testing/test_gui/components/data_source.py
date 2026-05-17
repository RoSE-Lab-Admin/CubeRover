from nicegui import ui

class DataSourceCard(ui.column):
    def __init__(self, title: str, theme_color: str, bg_color: str, icon_name: str, 
                 live_btn_text: str, live_btn_icon: str,
                 on_live_click, on_upload, on_clear):
        super().__init__()
        self.classes('w-full p-3 bg-slate-50 rounded border border-slate-200 gap-2 mb-2')
        
        with self:
            ui.label(title).classes(f'text-[10px] font-bold text-{theme_color}-500 uppercase tracking-wider')
            with ui.row().classes(f'bg-{bg_color}-100 text-{theme_color}-800 rounded-full pl-3 pr-1 py-1 items-center gap-1 w-full flex-nowrap'):
                ui.icon(icon_name, size='xs')
                self.status_label = ui.label('None Loaded').classes('text-[11px] font-bold flex-grow truncate text-slate-500')
                self.clear_btn = ui.button(icon='close', on_click=on_clear).props(f'flat round size=xs padding=none text-color={theme_color}').classes('hidden')

            with ui.row().classes('w-full gap-2'):
                ui.button(live_btn_text, icon=live_btn_icon, on_click=on_live_click).props('size=sm color=slate outline').classes('flex-none')
                uploader = ui.upload(auto_upload=True, on_upload=on_upload).props('accept=".csv"').classes('hidden')
                ui.button('Load CSV', on_click=lambda: uploader.run_method('pickFiles')).props(f'size=sm color={theme_color} {"outline" if theme_color == "slate" else ""}').classes('flex-grow')

    def set_status(self, text: str, is_active: bool):
        self.status_label.set_text(text)
        if is_active:
            self.status_label.classes(remove='text-slate-500', add='text-gray-800')
            self.clear_btn.classes(remove='hidden')
        else:
            self.status_label.classes(remove='text-gray-800', add='text-slate-500')
            self.clear_btn.classes('hidden')