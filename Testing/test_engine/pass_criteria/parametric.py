import fnmatch
import pandas as pd
from test_engine.interfaces import BasePassCriteria

class ParametricPassCriteria(BasePassCriteria):
    def evaluate(self, criteria_config: dict, test_data: pd.DataFrame) -> dict:
        """
        Evaluates a GlobalHistory DataFrame against min/max parametric bounds using glob matching.
        """
        checks = criteria_config.get("checks", {})
        
        if test_data.empty:
            return {"passed": False, "details": "Evaluation failed: Provided test_data DataFrame is empty."}
            
        if not checks:
            return {"passed": True, "details": "No parametric checks defined."}

        for pattern, limits in checks.items():
            # Find all DataFrame columns that match the YAML pattern (e.g., '*_curr')
            matched_cols = fnmatch.filter(test_data.columns, pattern)
            
            # Strict validation: Ensure the pattern actually caught something
            if not matched_cols:
                return {
                    "passed": False, 
                    "details": f"Configuration Error: Pattern '{pattern}' did not match any telemetry columns."
                }

            # Evaluate the limits against every matched column
            for col in matched_cols:
                # --- Evaluate Maximum Bounds ---
                if "max" in limits:
                    breaches = test_data[test_data[col] > limits["max"]]
                    
                    if not breaches.empty:
                        first_failure = breaches.iloc[0]
                        bad_val = first_failure[col]
                        timestamp = first_failure.get('Seconds', 'Unknown')
                        
                        return {
                            "passed": False,
                            "details": (
                                f"Safety Violation: '{col}' (matched by '{pattern}') exceeded max limit of {limits['max']}. "
                                f"Recorded {bad_val:.2f} at T={timestamp}s."
                            )
                        }

                # --- Evaluate Minimum Bounds ---
                if "min" in limits:
                    breaches = test_data[test_data[col] < limits["min"]]
                    
                    if not breaches.empty:
                        first_failure = breaches.iloc[0]
                        bad_val = first_failure[col]
                        timestamp = first_failure.get('Seconds', 'Unknown')
                        
                        return {
                            "passed": False,
                            "details": (
                                f"Safety Violation: '{col}' (matched by '{pattern}') dropped below min limit of {limits['min']}. "
                                f"Recorded {bad_val:.2f} at T={timestamp}s."
                            )
                        }

        return {
            "passed": True,
            "details": "All physical sensor values remained within safe operating limits."
        }