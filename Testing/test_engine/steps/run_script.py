import asyncio
import sys
from ..interfaces import BaseStep

class RunScriptStep(BaseStep):
    def __init__(self):
        pass

    async def execute(self, step_config: dict):
        print(f"--- STEP: Running Script ---")
        
    async def stop(self):
        print("--- STEP ABORT: Terminating custom script ---")


# class RunScriptStep(BaseStep):
#     def __init__(self):
#         # We need to store the process reference so the stop() method can kill it
#         self._process = None

#     async def execute(self, step_config: dict):
#         # 1. Parse the YAML configuration
#         target = step_config.get("target")
#         args = step_config.get("args", [])
#         timeout_sec = step_config.get("timeout_sec", 60)
#         capture_output = step_config.get("capture_output", True)
#         expected_exit_code = step_config.get("expected_exit_code", 0)

#         if not target:
#             raise ValueError("RunScriptStep requires a 'target' script to execute.")

#         # If it's a python script, run it using the current Python environment
#         cmd = [sys.executable, target] if target.endswith('.py') else [target]
#         cmd.extend(args)

#         print(f"--- STEP: Running Script ---")
#         print(f"Executing: {' '.join(cmd)}")

#         # Configure standard output/error pipes if we are capturing
#         stdout_pipe = asyncio.subprocess.PIPE if capture_output else None
#         stderr_pipe = asyncio.subprocess.PIPE if capture_output else None

#         try:
#             # 2. Spawn the subprocess without blocking the UI Event Loop
#             self._process = await asyncio.create_subprocess_exec(
#                 *cmd,
#                 stdout=stdout_pipe,
#                 stderr=stderr_pipe
#             )

#             # 3. Wait for the script to finish, applying the safety timeout
#             stdout, stderr = await asyncio.wait_for(
#                 self._process.communicate(), 
#                 timeout=timeout_sec
#             )

#             # 4. Handle the captured output
#             if capture_output:
#                 if stdout:
#                     print(f"[SCRIPT STDOUT]\n{stdout.decode().strip()}")
#                 if stderr:
#                     print(f"[SCRIPT STDERR]\n{stderr.decode().strip()}")

#             # 5. Evaluate the Exit Code
#             if self._process.returncode != expected_exit_code:
#                 raise RuntimeError(
#                     f"Script failed! Exited with code {self._process.returncode}, "
#                     f"expected {expected_exit_code}."
#                 )
                
#             print("Script executed successfully.")

#         except asyncio.TimeoutError:
#             # The script ran longer than timeout_sec allowed
#             print(f"ERROR: Script timed out after {timeout_sec} seconds!")
#             await self.stop()  # Ensure the rogue script is killed!
#             raise RuntimeError(f"Script timeout: {target}")

#         except asyncio.CancelledError:
#             # The user clicked "Stop Capture" in the UI
#             print("RunScriptStep interrupted by system abort.")
#             await self.stop()
#             raise  # Re-raise so the TestEngine handles the global abort

#         finally:
#             # Clean up the process reference
#             self._process = None

#     async def stop(self):
#         # 6. Safely and instantly terminate the background script
#         if self._process and self._process.returncode is None:
#             print("--- STEP ABORT: Terminating custom script ---")
#             try:
#                 # Ask politely first (SIGTERM)
#                 self._process.terminate()
                
#                 # Give it 1 second to clean up its own resources
#                 try:
#                     await asyncio.wait_for(self._process.wait(), timeout=1.0)
#                 except asyncio.TimeoutError:
#                     # If it refuses to close, bring down the hammer (SIGKILL)
#                     print("Script did not terminate cleanly. Force killing...")
#                     self._process.kill()
#                     await self._process.wait()
                    
#             except ProcessLookupError:
#                 pass  # The process is already dead
            
#             print("Script terminated.")