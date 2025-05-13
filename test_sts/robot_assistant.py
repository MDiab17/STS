import cv2
import numpy as np
import socket
import struct
import time
import threading
import speech_recognition as sr
import pygame
import openai
from datetime import datetime
import argparse
from openai import OpenAI

# Initialize OpenAI client
client = OpenAI(api_key="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")

# Initialize pygame mixer
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)

# Global flags
interrupted = False
is_active = False

def speak(text):
    """Convert text to speech using OpenAI's text-to-speech API"""
    global interrupted
    interrupted = False
    
    # Start interruption listener in background
    interruption_thread = threading.Thread(target=check_for_interruption)
    interruption_thread.daemon = True
    interruption_thread.start()
    
    print(f"Robot: {text}")
    
    try:
        # Generate speech
        response = client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text
        )
        
        # Save to a file
        output_file = "temp_speech.mp3"
        with open(output_file, 'wb') as f:
            f.write(response.content)
        
        # Play the audio
        pygame.mixer.music.load(output_file)
        pygame.mixer.music.play()
        
        # Wait for audio to finish or interruption
        while pygame.mixer.music.get_busy() and not interrupted:
            time.sleep(0.1)
            
        # Clean up
        pygame.mixer.music.unload()
        os.remove(output_file)
        
        if interrupted:
            print("Speech interrupted!")
            return True
            
    except Exception as e:
        print(f"Error in speech synthesis: {e}")
        return False
    
    return False

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
                    print(f"\nInterrupted by user saying '{text}'")
                    break
            except:
                continue

def get_speech_input(timeout=20, phrase_time_limit=15):
    """Get speech input from the user"""
    recognizer = sr.Recognizer()
    recognizer.dynamic_energy_threshold = True
    recognizer.energy_threshold = 200
    recognizer.pause_threshold = 1.0
    recognizer.non_speaking_duration = 0.5
    recognizer.phrase_threshold = 0.3
    
    with sr.Microphone() as source:
        print(f"Please speak now... (I'll wait for {timeout} seconds)")
        try:
            print("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            
            print("Listening...")
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
                print(f"You said: {text}")
                return text
            else:
                print("No speech detected")
                return None
                
        except sr.WaitTimeoutError:
            print(f"No speech detected for {timeout} seconds.")
            return None
        except sr.UnknownValueError:
            print("Sorry, I could not understand your speech")
            return None
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
            return None
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

def get_response(instruction: str) -> str:
    """Get a response from OpenAI's model"""
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "user", "content": f"""You are a humanoid robot. You are helping human with their questions. Please provide a brief and concise response (2-3 sentences maximum) that can be spoken naturally. Also, your answer must be in a friendly and engaging tone:

{instruction}"""}
        ],
        temperature=0.5,
        max_tokens=500
    )
    
    return response.choices[0].message.content

def detect_looking_status():
    """Detect if someone is looking at the camera"""
    # Initialize camera
    cap = cv2.VideoCapture(0)
    
    # Load face detection model
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
    
    print("[Looking Status] Starting face detection...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            
            looking_at_camera = False
            
            for (x, y, w, h) in faces:
                # Draw rectangle around face
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                
                # Region of interest for eyes
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                
                # Detect eyes
                eyes = eye_cascade.detectMultiScale(roi_gray)
                
                # If we detect eyes, consider the person is looking at the camera
                if len(eyes) > 0:
                    looking_at_camera = True
                    for (ex, ey, ew, eh) in eyes:
                        cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
            
            # Display the frame
            cv2.imshow('Face Detection', frame)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            return looking_at_camera
                
    except KeyboardInterrupt:
        print("\nFace detection stopped.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

def main():
    global is_active
    
    print("Starting Rainbow Robot Assistant...")
    
    # Play welcome message
    welcome_text = """Hello! I am Rainbow, your humanoid robot assistant. 
    To interact with me, please look at the camera and say 'Hey robot'.
    You can say 'Rainbow' or 'Stop' to interrupt me at any time.
    Say 'goodbye' to end our conversation."""
    speak(welcome_text)
    
    silence_count = 0
    max_silence = 3
    
    while True:
        # Check if someone is looking at the camera
        if detect_looking_status() and not is_active:
            is_active = True
            print("\nFace detected! Please say 'Hey robot' to start.")
            
            # Wait for wake word
            recognizer = sr.Recognizer()
            with sr.Microphone() as source:
                try:
                    audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
                    text = recognizer.recognize_google(audio, language="en-US").lower()
                    
                    if "hey robot" in text or "hey robo" in text:
                        speak("Hello! How can I help you today?")
                    else:
                        is_active = False
                        continue
                except:
                    is_active = False
                    continue
        
        if not is_active:
            time.sleep(0.1)
            continue
            
        print("\nPlease speak your question:")
        speech_text = get_speech_input()
        
        if not speech_text:
            silence_count += 1
            if silence_count >= max_silence:
                speak("No speech detected for too long. Going back to sleep. Look at the camera and say 'Hey robot' to wake me up!")
                is_active = False
                continue
            speak(f"I didn't catch that. Please try again. {max_silence - silence_count} attempts remaining")
            continue
            
        silence_count = 0
            
        if "goodbye" in speech_text.lower():
            speak("Goodbye! Have a great day! Look at the camera and say 'Hey robot' when you need me again!")
            is_active = False
            continue
            
        response = get_response(speech_text)
        was_interrupted = speak(response)
        
        if was_interrupted:
            print("Would you like to ask something else?")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rainbow Robot Assistant")
    parser.add_argument("--setup", action="store_true", help="Install required dependencies")
    args = parser.parse_args()
    
    if args.setup:
        import subprocess
        print("Installing required packages...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("All dependencies installed successfully!")
    else:
        main() 