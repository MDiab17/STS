#!/usr/bin/env python3
"""
ros2 run my_robot_tasks friendly_robot_server

A demo ROS 2 Action **server** that:
  • accepts ExecuteTask.action goals
  • speaks status with GPT‑4o‑mini‑TTS
  • streams feedback until "done"
  • gets task and object from voice input

Dependencies
------------
    sudo apt install ros-humble-desktop  # or your ROS 2 distro
    pip install -U "openai[voice]" sounddevice numpy rclpy speech_recognition pyaudio

Ensure the generated Python module for ExecuteTask.action is discoverable:
    source install/setup.bash
"""

# ── 1. Imports ──────────────────────────────────────────────────────────────
import asyncio, time
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node   import Node

from geometry_msgs.msg import PoseStamped
from your_robot_msgs.action import ExecuteTask   # ⇠ replace with your package

import sounddevice as sd                        # audio sink (default device)
from openai import OpenAI, AsyncOpenAI
from openai.helpers import LocalAudioPlayer
import speech_recognition as sr

# ── 2. API key (hard‑coded – demo only) ─────────────────────────────────────
API_KEY = (
    "sk-proj-jbNlYSK4jexNXhqwgtHLB8-Po9sxa_iirOsibjRaupSujxQFzvQEX0_N-6fgG5"
    "rol1SGFoP1PNT3BlbkFJVj4ogGLQJDddp89ddvB4GXJnd2Nbx8geL_98ThlwzaiAnkCae"
    "JazjkpnZetfBmCxC40Q-OPgwA"
)

# ── 3. GPT‑4o clients ───────────────────────────────────────────────────────
CHAT = OpenAI(api_key=API_KEY, http2=True)       # sync (for short text)
TTS  = AsyncOpenAI(api_key=API_KEY, http2=True)  # async streaming

VOICE = "alloy"
STYLE = (
    "Delivery: warm, upbeat, slightly synthetic. "
    "Voice: friendly service robot. Tone: helpful and positive."
)

# ── 4. Speech input/output helpers ──────────────────────────────────────────
def get_speech_input(timeout=10) -> Optional[str]:
    """
    Get speech input from the user and convert it to text
    Args:
        timeout (int): Number of seconds to wait for speech input before timing out
    """
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print(f"Please speak now... (I'll wait for {timeout} seconds)")
        try:
            audio = recognizer.listen(source, timeout=timeout)
            text = recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text
        except sr.WaitTimeoutError:
            print(f"No speech detected for {timeout} seconds.")
            return None
        except sr.UnknownValueError:
            print("Sorry, I could not understand your speech")
            return None
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
            return None

async def _tts(text: str):
    async with TTS.audio.speech.with_streaming_response.create(
        model="gpt-4o-mini-tts",
        voice=VOICE,
        input=text,
        instructions=STYLE,
        response_format="pcm",
    ) as r:
        await LocalAudioPlayer().play(r)

def speak(text: str):
    """Blocking wrapper so we can call from ROS execute thread."""
    print(f"\n🤖 {text}")
    try:
        asyncio.run(_tts(text))
    except KeyboardInterrupt:
        print("⏭️  Speech skipped.")

def extract_task_and_object(text: str) -> Tuple[Optional[str], Optional[str]]:
    """
    Extract task name and target object from speech text using GPT-4
    """
    prompt = f"""Extract the task name and target object from this text. 
    Return them in this format: task_name: <task>, target_object: <object>
    If you can't identify either, use 'unknown'.
    
    Text: {text}
    
    Format:"""
    
    response = CHAT.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3,
        max_tokens=100
    )
    
    result = response.choices[0].message.content.lower()
    
    # Parse the response
    task_name = "unknown"
    target_object = "unknown"
    
    if "task_name:" in result:
        task_name = result.split("task_name:")[1].split(",")[0].strip()
    if "target_object:" in result:
        target_object = result.split("target_object:")[1].strip()
    
    return task_name, target_object

# ── 5. ROS 2 Action Server Node ─────────────────────────────────────────────
class FriendlyRobotServer(Node):
    def __init__(self):
        super().__init__("friendly_robot_server")

        self._as = ActionServer(
            self,
            ExecuteTask,
            "execute_task",
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        
        speak("Hello! I am your friendly robot assistant. Please tell me what task you'd like me to perform and on which object.")

    # ------- goal reception --------------------------------------------------
    def _goal_cb(self, goal_request: ExecuteTask.Goal):
        self.get_logger().info(f"Received goal: {goal_request.task_name} / {goal_request.target_object}")
        return rclpy.action.server.GoalResponse.ACCEPT

    # ------- cancellation ----------------------------------------------------
    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    # ------- execution -------------------------------------------------------
    async def _execute(self, goal_handle):
        # Get voice input
        speak("Please tell me what task you'd like me to perform and on which object.")
        speech_text = get_speech_input()
        
        if not speech_text:
            speak("I couldn't understand what you said. Please try again.")
            goal_handle.abort()
            return ExecuteTask.Result(
                success=False,
                result_message="No valid speech input received."
            )
        
        # Extract task and object from speech
        task_name, target_object = extract_task_and_object(speech_text)
        
        if task_name == "unknown" or target_object == "unknown":
            speak("I couldn't clearly understand the task or object. Please try again.")
            goal_handle.abort()
            return ExecuteTask.Result(
                success=False,
                result_message="Could not extract task or object from speech."
            )
        
        # Update goal with extracted information
        goal = goal_handle.request
        goal.task_name = task_name
        goal.target_object = target_object
        
        self.get_logger().info(f"Executing task {goal.task_name}…")
        speak(f"Starting task {goal.task_name} on {goal.target_object}.")

        feedback_msg = ExecuteTask.Feedback()
        total_steps  = 100               # simulate work with 100 ticks

        for step in range(total_steps):
            # Check if client requested cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                speak("Task canceled.")
                return ExecuteTask.Result(
                    success=False,
                    result_message="Task canceled by client."
                )

            # Publish feedback
            feedback_msg.progress     = step / total_steps
            feedback_msg.current_step = f"step_{step}"
            goal_handle.publish_feedback(feedback_msg)

            # Simulate work
            await asyncio.sleep(0.05)  # 20 Hz feedback

        # DONE
        speak("Task completed successfully!")
        goal_handle.succeed()
        return ExecuteTask.Result(
            success=True,
            result_message="Finished task."
        )

# ── 6. Main ─────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    server = FriendlyRobotServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
