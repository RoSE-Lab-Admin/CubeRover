import asyncio
from ..interfaces import BaseStep

class AdHocCaptureStep(BaseStep):
    def __init__(self):
        # State flag to allow graceful stopping
        self._is_capturing = False

    async def execute(self, step_config: dict):
        print("--- STEP: Starting Ad-Hoc Capture ---")
        print("Manual control engaged. Waiting for user to stop...")
        
        self._is_capturing = True
        
        try:
            # Loop continuously until stop() is called or the task is cancelled
            while self._is_capturing:
                # IMPORTANT: await asyncio.sleep() hands control back to the NiceGUI 
                # event loop so your charts and UI can continue to update!
                await asyncio.sleep(0.5)
                
        except asyncio.CancelledError:
            # This triggers if the TestEngine task is forcefully cancelled
            print("Ad-Hoc Capture interrupted by system abort.")
            raise  # Re-raise so the TestEngine handles the global emergency stop
            
        finally:
            self._is_capturing = False
            print("Ad-Hoc Capture finished.")

    async def stop(self):
        print("--- STEP ABORT: Stopping Ad-Hoc Capture ---")
        # Flipping this flag gracefully breaks the while loop in execute()
        self._is_capturing = False
        
        # TODO: If you have hardware, you might want to command 0 RPM here
        # await self.hardware.set_manual_mode(False)
#         # await self.hardware.set_speed(0)