# ui_registry.py
from __future__ import annotations  # Must be the first import!
from typing import TYPE_CHECKING
from nicegui import ui

# These imports ONLY happen during static type checking (mypy, IDEs)
# They are completely ignored at runtime, preventing circular import crashes.
if TYPE_CHECKING:
    from test_gui.components.chart_card import ChartCard
    from test_gui.components.data_source import DataSourceCard

class UI:
    btn_start: ui.button | None = None
    capture_mode: ui.select | None = None
    switch_capture_panel: ui.switch | None = None
    ref_pill_label: ui.label | None = None
    ref_pill: ui.element | None = None
    btn_load_ref: ui.button | None = None
    time_range: ui.range | None = None
    
    analysis_sensor_select: ui.select | None = None
    analysis_motor_select: ui.select | None = None
    dist_metric_select: ui.select | None = None
    stats_table: ui.table | None = None
    
    # Custom Components with full Type Safety restored!
    master_chart_card: ChartCard | None = None
    analysis_chart_card: ChartCard | None = None
    delta_chart_card: ChartCard | None = None
    dist_chart_card: ChartCard | None = None
    dataset_a_card: DataSourceCard | None = None
    dataset_b_card: DataSourceCard | None = None