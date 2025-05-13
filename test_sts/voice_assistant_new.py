import openai
import os
import speech_recognition as sr
import pygame
import time
import threading
from openai import OpenAI
import argparse
import logging
from logging.handlers import RotatingFileHandler
from dotenv import load_dotenv
import shutil
from datetime import datetime
from flask import Flask, render_template_string
import queue
import webbrowser
from rich.console import Console
from rich.panel import Panel
from rich.text import Text
from rich import box
from rich.layout import Layout
from rich.live import Live
from rich.table import Table

# Load environment variables
load_dotenv()

# Initialize Flask app
app = Flask(__name__)
display_queue = queue.Queue()
console = Console()

# HTML template for web visualization
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>HMND Robot</title>
    <style>
        body {
            background-color: #1a1a1a;
            color: #ffffff;
            font-family: 'Courier New', monospace;
            margin: 20px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
            padding: 30px;
            background: linear-gradient(45deg, #00ff00, #00ffff, #00ff00);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            font-size: 72px;
            font-weight: 900;
            letter-spacing: 5px;
            text-transform: uppercase;
            text-shadow: 0 0 20px rgba(0, 255, 0, 0.5);
            animation: glow 2s ease-in-out infinite alternate;
            position: relative;
        }
        .header::after {
            content: '';
            position: absolute;
            bottom: 0;
            left: 50%;
            transform: translateX(-50%);
            width: 80%;
            height: 2px;
            background: linear-gradient(90deg, transparent, #00ff00, transparent);
            animation: lineGlow 2s ease-in-out infinite alternate;
        }
        .description {
            text-align: center;
            margin: -20px auto 30px;
            padding: 20px;
            color: #00ff00;
            font-size: 1.2em;
            line-height: 1.6;
            max-width: 600px;
            text-shadow: 0 0 10px rgba(0, 255, 0, 0.3);
            animation: fadeIn 1s ease-out;
        }
        @keyframes fadeIn {
            from {
                opacity: 0;
                transform: translateY(20px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }
        @keyframes glow {
            from {
                text-shadow: 0 0 20px rgba(0, 255, 0, 0.5),
                            0 0 40px rgba(0, 255, 0, 0.3),
                            0 0 60px rgba(0, 255, 0, 0.2);
            }
            to {
                text-shadow: 0 0 30px rgba(0, 255, 0, 0.7),
                            0 0 60px rgba(0, 255, 0, 0.5),
                            0 0 90px rgba(0, 255, 0, 0.3);
            }
        }
        @keyframes lineGlow {
            from {
                box-shadow: 0 0 10px rgba(0, 255, 0, 0.3),
                            0 0 20px rgba(0, 255, 0, 0.2);
            }
            to {
                box-shadow: 0 0 20px rgba(0, 255, 0, 0.5),
                            0 0 40px rgba(0, 255, 0, 0.3);
            }
        }
        .box {
            border: 1px solid #00ff00;
            padding: 20px;
            margin-bottom: 20px;
            border-radius: 10px;
            background: rgba(0, 255, 0, 0.1);
            box-shadow: 0 0 20px rgba(0, 255, 0, 0.2);
            animation: borderGlow 2s ease-in-out infinite alternate;
        }
        @keyframes borderGlow {
            from {
                box-shadow: 0 0 10px rgba(0, 255, 0, 0.2),
                            inset 0 0 10px rgba(0, 255, 0, 0.1);
            }
            to {
                box-shadow: 0 0 20px rgba(0, 255, 0, 0.4),
                            inset 0 0 20px rgba(0, 255, 0, 0.2);
            }
        }
        .status {
            color: #00ff00;
            font-size: 1.2em;
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.7; }
            100% { opacity: 1; }
        }
        .sleeping {
            color: #ff0000;
        }
        .message {
            color: #ffff00;
            font-size: 1.1em;
        }
        .response {
            color: #00ffff;
            font-size: 1.1em;
        }
        .time {
            color: #888888;
            font-size: 0.9em;
        }
        .title {
            font-size: 1.5em;
            margin-bottom: 20px;
            text-align: center;
            animation: slideIn 1s ease-out;
            color: #00ff00;
            text-shadow: 0 0 10px rgba(0, 255, 0, 0.5);
        }
        @keyframes slideIn {
            from {
                transform: translateY(-50px);
                opacity: 0;
            }
            to {
                transform: translateY(0);
                opacity: 1;
            }
        }
    </style>
    <script>
        function updateDisplay() {
            fetch('/get_display')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerHTML = data.status;
                    document.getElementById('message').innerHTML = data.message;
                    document.getElementById('response').innerHTML = data.response;
                    document.getElementById('time').innerHTML = data.time;
                });
        }
        setInterval(updateDisplay, 1000);
    </script>
</head>
<body>
    <div class="container">
        <div class="header">HMND-01</div>
        <div class="description">
            Our first humanoid robot. Designed to be customizable, modular, and reliable with a low Total Cost of Ownership.
        </div>
        <div class="title">Humanoid Robot Assistant</div>
        <div class="box">
            <h3>KEYWORDS</h3>
            <p>Wake Word: wake up</p>
            <p>Interrupt: rainbow, stop</p>
            <p>End Conversation: goodbye</p>
        </div>
        <div class="box">
            <p class="status" id="status">Status: {{ status }}</p>
            <p class="message" id="message">Last Message: {{ message }}</p>
            <p class="response" id="response">Last Response: {{ response }}</p>
            <p class="time" id="time">Time: {{ time }}</p>
        </div>
    </div>
</body>
</html>
"""

class DisplayManager:
    def __init__(self):
        self.terminal_width = shutil.get_terminal_size().columns
        self.status = "Sleeping"
        self.last_message = ""
        self.last_response = ""
        self.is_active = False
        self.keywords = {
            "Wake Word": "wake up",
            "Interrupt": "rainbow, stop",
            "End Conversation": "goodbye"
        }
        self.layout = Layout()
        self.layout.split_column(
            Layout(name="header"),
            Layout(name="body")
        )
        
    def clear_screen(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        
    def print_header(self):
        header_text = Text("HMND-01", style="bold green")
        header = Panel(header_text, box=box.DOUBLE, style="green")
        self.layout["header"].update(header)
        
    def print_keywords(self):
        table = Table(show_header=True, header_style="bold green", box=box.ROUNDED)
        table.add_column("Command", style="cyan")
        table.add_column("Keyword", style="yellow")
        
        for key, value in self.keywords.items():
            table.add_row(key, value)
            
        keywords_panel = Panel(table, title="Keywords", box=box.ROUNDED, style="green")
        self.layout["body"].update(keywords_panel)
        
    def print_status(self):
        status_color = "green" if self.is_active else "red"
        status_text = f"Status: {self.status}"
        message_text = f"Last Message: {self.last_message}"
        response_text = f"Last Response: {self.last_response}"
        time_text = f"Time: {datetime.now().strftime('%H:%M:%S')}"
        
        status_panel = Panel(
            f"[{status_color}]{status_text}[/{status_color}]\n"
            f"[yellow]{message_text}[/yellow]\n"
            f"[cyan]{response_text}[/cyan]\n"
            f"[dim]{time_text}[/dim]",
            title="Status",
            box=box.ROUNDED,
            style="green"
        )
        
        with Live(self.layout, refresh_per_second=1):
            self.layout["body"].update(status_panel)
        
    def update(self, status=None, message=None, response=None, is_active=None):
        if status:
            self.status = status
        if message:
            self.last_message = message
        if response:
            self.last_response = response
        if is_active is not None:
            self.is_active = is_active
            
        # Update terminal display
        self.clear_screen()
        self.print_header()
        self.print_keywords()
        self.print_status()
        
        # Update web display
        display_queue.put({
            'status': self.status,
            'message': self.last_message,
            'response': self.last_response,
            'time': datetime.now().strftime('%H:%M:%S')
        })

# Initialize display manager
display = DisplayManager()

@app.route('/')
def home():
    return render_template_string(HTML_TEMPLATE,
                                status=display.status,
                                message=display.last_message,
                                response=display.last_response,
                                time=datetime.now().strftime('%H:%M:%S'))

@app.route('/get_display')
def get_display():
    try:
        data = display_queue.get_nowait()
    except queue.Empty:
        data = {
            'status': display.status,
            'message': display.last_message,
            'response': display.last_response,
            'time': datetime.now().strftime('%H:%M:%S')
        }
    return data

def start_web_server():
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False)).start()
    webbrowser.open('http://localhost:5000')

# Configure logging
def setup_logger():
    logger = logging.getLogger('RainbowRobot')
    logger.setLevel(logging.INFO)
    
    # Create formatters
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # File handler with rotation
    file_handler = RotatingFileHandler(
        'rainbow_robot.log',
        maxBytes=1024*1024,  # 1MB
        backupCount=5
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    return logger

# Initialize logger
logger = setup_logger()

# Initialize OpenAI client with API key from environment variable
api_key = os.getenv('OPENAI_API_KEY')
if not api_key:
    logger.error("OPENAI_API_KEY not found in environment variables")
    raise ValueError("OPENAI_API_KEY environment variable is required")

client = OpenAI(api_key=api_key)

# Initialize pygame mixer with error handling
try:
    os.environ['SDL_AUDIODRIVER'] = 'pulseaudio'
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)
    logger.info("Audio system initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize audio system: {e}")
    logger.warning("Running without audio capabilities")

# Global flags
interrupted = False
is_active = False

def listen_for_wake_word():
    """Background thread to listen for wake up word"""
    global is_active
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 50
    recognizer.dynamic_energy_threshold = True
    recognizer.pause_threshold = 1.0
    recognizer.phrase_threshold = 0.3
    recognizer.non_speaking_duration = 0.8
    
    display.update(status="Sleeping", is_active=False)
    logger.info("Robot is sleeping. Say 'wake up' to activate!")
    
    with sr.Microphone() as source:
        logger.info("Adjusting for ambient noise...")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        logger.info("Ready to listen!")
        
        while not is_active:
            try:
                display.update(status="Listening for wake word...")
                audio = recognizer.listen(source, timeout=10, phrase_time_limit=5)
                text = recognizer.recognize_google(audio, language="en-US").lower()
                display.update(message=text)
                logger.info(f"Heard: {text}")
                
                if "wake up" in text:
                    is_active = True
                    display.update(status="Active", is_active=True)
                    logger.info("Wake word detected! Robot is now active.")
                    speak("Hello! I am Rainbow, your humanoid robot assistant. How can I help you today?")
                    break
            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                continue
            except Exception as e:
                logger.error(f"Error in wake word detection: {e}")
                continue

def check_for_interruption():
    """Background thread to check for interruption"""
    global interrupted
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 200
    
    with sr.Microphone() as source:
        while True:
            try:
                audio = recognizer.listen(source, timeout=1, phrase_time_limit=1)
                text = recognizer.recognize_google(audio, language="en-US").lower()
                if "rainbow" in text or "stop" in text:
                    interrupted = True
                    pygame.mixer.music.stop()
                    logger.info(f"Interrupted by user saying '{text}'")
                    break
            except:
                continue

def speak(text):
    """Convert text to speech using OpenAI's text-to-speech API"""
    global interrupted
    interrupted = False
    
    interruption_thread = threading.Thread(target=check_for_interruption)
    interruption_thread.daemon = True
    interruption_thread.start()
    
    display.update(status="Speaking", response=text)
    logger.info(f"Robot: {text}")
    
    try:
        response = client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text
        )
        
        output_file = "temp_speech.mp3"
        with open(output_file, 'wb') as f:
            f.write(response.content)
        
        try:
            pygame.mixer.music.load(output_file)
            pygame.mixer.music.play()
            
            while pygame.mixer.music.get_busy() and not interrupted:
                time.sleep(0.1)
                
            pygame.mixer.music.unload()
            os.remove(output_file)
            
            if interrupted:
                display.update(status="Interrupted")
                logger.info("Speech interrupted!")
                return True
                
        except Exception as e:
            logger.error(f"Error playing audio: {e}")
            return False
            
    except Exception as e:
        logger.error(f"Error in speech synthesis: {e}")
        return False
    
    return False

def get_speech_input(timeout=30, phrase_time_limit=20):
    """Get speech input from the user"""
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 50
    recognizer.dynamic_energy_threshold = True
    recognizer.pause_threshold = 1.0
    recognizer.phrase_threshold = 0.3
    recognizer.non_speaking_duration = 0.8
    
    with sr.Microphone() as source:
        display.update(status="Listening")
        logger.info(f"Please speak now... (I'll wait for {timeout} seconds)")
        try:
            logger.info("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            logger.info("Ready to listen!")
            
            audio = recognizer.listen(
                source,
                timeout=timeout,
                phrase_time_limit=phrase_time_limit
            )
            
            text = recognizer.recognize_google(
                audio,
                language="en-US",
                show_all=False
            )
            
            if text:
                display.update(message=text)
                logger.info(f"You said: {text}")
                return text
            else:
                display.update(status="No Input")
                logger.info("No speech detected")
                return None
                
        except sr.WaitTimeoutError:
            display.update(status="Timeout")
            logger.info(f"No speech detected for {timeout} seconds.")
            return None
        except sr.UnknownValueError:
            display.update(status="Error")
            logger.info("Sorry, I could not understand your speech")
            return None
        except sr.RequestError as e:
            display.update(status="Error")
            logger.error(f"Could not request results; {e}")
            return None
        except Exception as e:
            display.update(status="Error")
            logger.error(f"An error occurred: {e}")
            return None

def get_response(instruction: str) -> str:
    """Get a response from OpenAI's model"""
    try:
        company_info = """ Humanoid is a London-based humanoid robotics company founded in 2024, developing humanoid robots to address critical global labor shortages and enhance human well-being by taking on dangerous, repetitive, or complex tasks across the industrial and warehouse automation, service, manufacturing and household sectors.
Humanoid is building the world's most reliable and commercially viable humanoid robot.
HMND-01 is customisable, modular and reliable with a low Total Cost of Ownership."""

        display.update(status="Generating Response")
        logger.info("Generating response...")
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": f"You are a humanoid robot assistant for a company called HUMANOID. {company_info}. You are helping human with their questions. Please provide a brief and concise response (2-3 sentences maximum) that can be spoken naturally. Also, your answer must be in a friendly and engaging tone."},
                {"role": "user", "content": instruction}
            ],
            temperature=0.7,
            max_tokens=150
        )
        
        answer = response.choices[0].message.content
        display.update(response=answer)
        logger.info(f"Generated response: {answer}")
        return answer
        
    except Exception as e:
        logger.error(f"Error generating response: {e}")
        return "I apologize, but I'm having trouble generating a response right now. Could you please try asking your question again?"

def main():
    global is_active
    
    parser = argparse.ArgumentParser(description="Voice Assistant")
    parser.add_argument("--message", type=str, help="Initial message to speak")
    args = parser.parse_args()
    
    # Start web server
    start_web_server()
    
    display.update(status="Starting")
    logger.info("\n=== Rainbow Robot Assistant ===")
    logger.info("Say 'wake up' to activate")
    logger.info("Say 'Rainbow' or 'Stop' to interrupt")
    logger.info("Say 'goodbye' to end the conversation")
    logger.info("==============================\n")
    
    if args.message:
        speak(args.message)
        return
    
    silence_count = 0
    max_silence = 5

    while True:
        if not is_active:
            wake_thread = threading.Thread(target=listen_for_wake_word)
            wake_thread.daemon = True
            wake_thread.start()
            wake_thread.join()
            continue
            
        display.update(status="Ready for Question")
        logger.info("\nPlease ask your question:")
        speech_text = get_speech_input()
        
        if not speech_text:
            silence_count += 1
            if silence_count >= max_silence:
                speak("No speech detected for too long. Going back to sleep. Say 'wake up' to wake me up!")
                is_active = False
                continue
            speak(f"I didn't catch that. Please try again. {max_silence - silence_count} attempts remaining")
            continue
            
        silence_count = 0
            
        if "goodbye" in speech_text.lower():
            speak("Goodbye! Have a great day! Say 'wake up' when you need me again!")
            is_active = False
            continue
            
        response = get_response(speech_text)
        was_interrupted = speak(response)
        
        if was_interrupted:
            display.update(status="Interrupted")
            logger.info("Would you like to ask something else?")
        else:
            display.update(status="Ready for Next Question")
            logger.info("\nListening for your next question...")

if __name__ == "__main__":
    main() 