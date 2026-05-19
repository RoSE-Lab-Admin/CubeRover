## State Variable / NiceGui Interaction Info

IMPORTANT: The gui state variables MUST be kept in a separate file from the NiceGui front-end code!

- When NiceGui starts, it imports the UI code several times. If the gui state variables are in the same file as the front-end code, the states can be overwritten, causing the data variables to be reset.
- Placing the gui state variables in their own module ensures that Python correctly reimports the variables rather than overwriting them.