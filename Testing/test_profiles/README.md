# Test Profiles Information

Test profiles are composed using the following fields: `name`, `description`, `steps`, `pass_criteria`.

The `steps` and `pass_criteria` implementation locations are:
* `steps`: [`Testing/test_engine/steps`](../test_engine/steps)
* `pass_criteria`: [`Testing/test_engine/pass_criteria`](../test_engine/pass_criteria)

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
      current_amps: { max: 15.0 }
      voltage_v: { min: 11.5, max: 12.5 }
      temp_c: { max: 85.0 }

  - type: signal_metrics
    checks:
      overshoot_percent: { max: 10 }
      settling_time_ms: { max: 50 }
      rise_time_ms: { max: 20 }

  - type: envelope
    reference_profile: "golden_traces/nominal_step_response.csv"
    allowed_deviation_percent: 5.0
```

## Example Custom Script Test

For complex logic that cannot be defined by standard steps, use an external Python script.

```yml
name: "My Custom Script Test"
description: "Runs an external test routine."
steps:
  - type: run_script
    target: "src/python/my_test_script.py"
    args: []
    timeout_sec: 60           # Aborts script if it hangs
    capture_output: true      # Logs stdout/stderr to the test report
    expected_exit_code: 0     # Defaults to 0 (Standard Success)
```


# Evaluation Types

## Parametric

**Focus:** Safety and basic health.

Validates that physical sensor values (Current, Voltage, Temperature) remained within safe operating limits throughout the test duration.

```yml
pass_criteria:
  - type: parametric
    checks:
      current_amps: { max: 15.0 }
      voltage_v: { min: 11.5, max: 12.5 }
      temp_c: { max: 85.0 }
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