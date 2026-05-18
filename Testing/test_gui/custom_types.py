from typing import List, TypedDict

class Row(TypedDict):
    Seconds: float
    volt1: float 
    volt2: float
    
    fl_pwm: float
    fl_rpm: int
    fl_curr: float
    fl_enc: int
    
    fr_pwm: float
    fr_rpm: int
    fr_curr: float
    fr_enc: int
    
    bl_pwm: float
    bl_rpm: int
    bl_curr: float
    bl_enc: int
    
    br_pwm: float
    br_rpm: int
    br_curr: float
    br_enc: int

GlobalHistory = List[Row]