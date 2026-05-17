import json
from nicegui import ui

class ChartCard(ui.card):
    """A reusable card containing a Highchart with built-in helper methods for JS interactions."""
    def __init__(self, options: dict, height_px: int = 400, show_log_toggle: bool = False, on_log_toggle=None, header_elements=None):
        super().__init__()
        self.classes('w-full p-4 bg-white shadow-sm border min-w-0 relative flex flex-col')
        
        with self:
            if header_elements:
                header_elements() # Execute lambda to build dynamic headers (like the Dropdowns in Analysis)
                
            if show_log_toggle:
                with ui.row().classes('absolute top-2 right-4 z-10 items-center bg-white/80 backdrop-blur rounded pl-2 border shadow-sm'):
                    ui.label('Log Y').classes('text-[10px] font-bold text-slate-500 uppercase')
                    self.log_switch = ui.switch(on_change=on_log_toggle).props('size=sm')
            
            with ui.element('div').classes(f'relative w-full flex-grow min-h-[{height_px}px]'):
                self.chart = ui.highchart(options).classes('absolute inset-0 w-full h-full')

    def update_chart(self, series_data=None, x_min=None, x_max=None, is_log=None):
        """Updates chart config and data in a single batch operation."""
        js = [f'var c = getElement({self.chart.id}).chart;']
        if is_log is not None:
            y_type = "logarithmic" if is_log else "linear"
            js.append(f'c.yAxis[0].update({{ type: "{y_type}" }}, false);')
        if x_min is not None and x_max is not None:
            js.append(f'c.xAxis[0].setExtremes({x_min}, {x_max}, false);')
        if series_data is not None:
            js.append(f'c.update({{series: {json.dumps(series_data)}}}, false, true, false);')
            
        if len(js) > 1:
            js.append('c.redraw();')
            ui.run_javascript('\n'.join(js))
    
    def redraw(self):
        """Forces the Highchart instance to physically repaint itself."""
        ui.run_javascript(f'getElement({self.chart.id}).chart.redraw();')
            
    def batch_update_series_data(self, data_dict: dict, redraw: bool = True):
        """Updates multiple series at once to minimize websocket traffic. data_dict is {series_id: data}"""
        js = [f'var c = getElement({self.chart.id}).chart;']
        for sid, data in data_dict.items():
            js.append(f'if(c.get("{sid}")) c.get("{sid}").setData({json.dumps(data)}, false, false, false);')
        if redraw:
            js.append('c.redraw();')
        ui.run_javascript('\n'.join(js))
        
    def batch_set_visibility(self, vis_dict: dict, redraw: bool = True):
        """Toggles visibility of multiple series at once. vis_dict is {series_id: bool}"""
        js = [f'var c = getElement({self.chart.id}).chart;']
        for sid, vis in vis_dict.items():
            js.append(f'if(c.get("{sid}")) c.get("{sid}").setVisible({str(vis).lower()}, false);')
        if redraw:
            js.append('c.redraw();')
        ui.run_javascript('\n'.join(js))
        
    def add_series(self, series_config: dict, redraw: bool = False):
        ui.run_javascript(f'getElement({self.chart.id}).chart.addSeries({json.dumps(series_config)}, {str(redraw).lower()});')
        
    def remove_series(self, series_id: str, redraw: bool = False):
        ui.run_javascript(f'var c = getElement({self.chart.id}).chart; if(c.get("{series_id}")) c.get("{series_id}").remove({str(redraw).lower()});')