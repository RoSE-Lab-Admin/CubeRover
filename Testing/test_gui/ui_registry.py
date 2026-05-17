from nicegui import ui

class UI:
    # Use the modern "Type | None" syntax (Python 3.10+)
    master_chart: ui.highchart | None = None
    btn_start: ui.button | None = None
    capture_mode: ui.select | None = None
    switch_capture_panel: ui.switch | None = None
    switch_capture_chart: ui.switch | None = None
    ref_pill_label: ui.label | None = None
    ref_pill: ui.element | None = None
    btn_load_ref: ui.button | None = None
    time_range: ui.range | None = None
    label_A_status: ui.label | None = None
    btn_clear_A: ui.button | None = None
    label_B_status: ui.label | None = None
    btn_clear_B: ui.button | None = None
    analysis_sensor_select: ui.select | None = None
    analysis_motor_select: ui.select | None = None
    analysis_log_scale: ui.switch | None = None
    dist_metric_select: ui.select | None = None
    stats_table: ui.table | None = None
    dist_chart: ui.highchart | None = None
    delta_chart: ui.highchart | None = None
    analysis_chart: ui.highchart | None = None