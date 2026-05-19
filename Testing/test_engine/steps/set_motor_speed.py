import asyncio
from ..interfaces import BaseStep

class SetMotorSpeedStep(BaseStep):
    def __init__(self):
        # We need to keep track of the ROS publisher process
        self._process = None
        self._is_holding = False

    async def execute(self, step_config: dict):
        target_speed = step_config.get('value', 0.0)
        duration = step_config.get('duration_sec', 'infinite')
        pub_rate = step_config.get('pub_rate', 10)  # Default to 10Hz

        print(f"STEP: Commanding {target_speed} m/s via ROS 2.")

        # Construct the exact bash command as a list of arguments
        ros_cmd = [
            "ros2", "topic", "pub", "/cmd_vel", "geometry_msgs/msg/TwistStamped",
            f"{{header: {{frame_id: 'base_link'}}, twist: {{linear: {{x: {target_speed}}}}}}}",
            "--rate", str(pub_rate)
        ]

        # 1. Spawn the ROS 2 publisher in the background
        self._process = await asyncio.create_subprocess_exec(
            *ros_cmd,
            stdout=asyncio.subprocess.DEVNULL, # Suppresses the spammy terminal output
            stderr=asyncio.subprocess.PIPE
        )

        try:
            # 2. Wait for the specified duration
            if duration == 'infinite' or duration is None:
                print("Holding speed indefinitely. Waiting for manual stop...")
                self._is_holding = True
                while self._is_holding:
                    await asyncio.sleep(0.5)
            else:
                duration = float(duration)
                print(f"Holding for {duration} seconds.")
                await asyncio.sleep(duration)
                
        finally:
            # 3. Always ensure we stop the motor when leaving this block!
            await self.stop()

    async def stop(self):
        self._is_holding = False
        
        # 1. Kill the continuous publisher
        if self._process and self._process.returncode is None:
            print("STEP ABORT: Terminating continuous ROS publisher...")
            self._process.terminate()
            try:
                await asyncio.wait_for(self._process.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                self._process.kill()
                await self._process.wait()
            self._process = None

        # ROS Safety Guarantee: Explicitly publish a zero-speed command
        print("STEP ABORT: Sending explicit 0.0 m/s command to hardware!")
        stop_cmd = [
            "ros2", "topic", "pub", "/cmd_vel", "geometry_msgs/msg/TwistStamped",
            "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}}}",
            "--once"  # Important: Only send it once, don't spam it!
        ]
        
        await asyncio.create_subprocess_exec(
            *stop_cmd,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL
        )