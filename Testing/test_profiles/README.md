# Test Profiles Information

Test profiles are composed using the following fields: `name`, `description`, `steps`, `pass_criteria`.

## TestEngine `step` / `pass_criteria` Registration

New `step` and `pass_criteria` scripts can be created by adding new class definitions to the following locations (see the `README.md` in each location for details):
* `steps`: [`Testing/test_engine/steps`](../test_engine/steps)
* `pass_criteria`: [`Testing/test_engine/pass_criteria`](../test_engine/pass_criteria)

**Note:** Use a `NAME` attribute at the top of a `step` / `pass_criteria` class to specify a custom name for use with the `TestEngine`. Otherwise, the `TestEngine` defaults to using the filename. Using a custom name enables adding more than one class per file. Each class definition must have a unique name.

```python
# Location: my_module.py --> TestEngine defaults to registering the class as "my_module"
class MyClass():
    NAME = "my_custom_name" # Changes class name to "my_custom_name"
    ...
```

## Example Comprehensive Test

This format combines hardware commands with several types of validation logic.

```yml
name: "Step Response Test"
description: "Commands a sudden jump in RPM to test motor tuning."
steps:
  - type: set_motor_speed
    value: 0
    duration_sec: 2

  - type: set_motor_speed
    value: 1.0         # Linear X velocity
    duration_sec: 5    # Optional, defaults to running until stopped
    pub_rate: 10       # Optional, defaults to 10

pass_criteria:
  - type: parametric
    checks:
      '*_curr': { max: 15.0 }             # Applies to fl_curr, fr_curr, bl_curr, br_curr
      'volt*': { min: 11.5, max: 12.5 }   # Applies to volt1, volt2
      '*_rpm': { max: 5000 }              # Applies to all motor speeds

  - type: signal_metrics
    checks:
      overshoot_percent: { max: 10 }
      settling_time_ms: { max: 50 }
      rise_time_ms: { max: 20 }

  - type: envelope
    reference_profile: "golden_traces/nominal_step_response.csv"
    allowed_deviation_percent: 5.0
```


# Evaluation Types

## Parametric

**Focus:** Safety and basic health.

Validates that physical sensor values (Current, Voltage, Temperature) remained within safe operating limits throughout the test duration.

```yml
pass_criteria:
  - type: parametric
    checks:
      '*_curr': { max: 15.0 }             # Applies to fl_curr, fr_curr, bl_curr, br_curr
      'volt*': { min: 11.5, max: 12.5 }   # Applies to volt1, volt2
      '*_rpm': { max: 5000 }              # Applies to all motor speeds
```

## Signal Metrics

**Focus:** Performance and Tuning.

Calculates high-level behavior characteristics from a time-series signal, such as how quickly a motor reaches its target and how much it oscillates.

```yml
pass_criteria:
  - type: signal_metrics
    checks:
      overshoot_percent: { max: 10 }
      settling_time_ms: { max: 50 }
      rise_time_ms: { max: 20 }
```

## Envelope

**Focus:** Repeatability and "Golden" Comparison.

Compares the entire captured data trace against a pre-recorded "Golden" trace. The test fails if the live data drifts outside of a defined percentage "envelope" around the reference.

```yml
pass_criteria:
  - type: envelope
    reference_profile: "golden_traces/nominal_step_response.csv"
    allowed_deviation_percent: 5.0
```