import pandas as pd
from ..interfaces import BasePassCriteria

class ParametricCriteria(BasePassCriteria):
    def evaluate(self, criteria_config: dict, test_data: pd.DataFrame) -> dict:
        checks = criteria_config.get('checks', {})
        results = []
        all_passed = True
        
        # Example: checking { max: 15.0 } on current_amps
        for metric, limits in checks.items():
            if metric not in test_data.columns:
                continue
                
            max_val = test_data[metric].max()
            allowed_max = limits.get('max', float('inf'))
            
            if max_val > allowed_max:
                all_passed = False
                results.append(f"{metric} exceeded limit: {max_val} > {allowed_max}")
            else:
                results.append(f"{metric} passed (Max: {max_val})")
                
        return {
            'passed': all_passed,
            'details': " | ".join(results)
        }