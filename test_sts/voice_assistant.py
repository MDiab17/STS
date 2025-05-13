import openai
import os
import speech_recognition as sr
import tempfile
import pygame
import time
import threading
from openai import OpenAI
import argparse

# Initialize OpenAI client
client = OpenAI(api_key="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")

# Initialize pygame mixer
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)

# Global flags
interrupted = False
is_active = False

def listen_for_wake_word():
    """Background thread to listen for wake word"""
    global is_active
    recognizer = sr.Recognizer()
    # More sensitive settings for wake word detection
    recognizer.energy_threshold = 100  # Lower threshold to catch quieter speech
    recognizer.dynamic_energy_threshold = True
    recognizer.pause_threshold = 0.5   # Shorter pause threshold
    
    print("Robot is sleeping. Say 'Hey robot' to wake me up!")
    
    with sr.Microphone() as source:
        # Adjust for ambient noise
        recognizer.adjust_for_ambient_noise(source, duration=1)
        
        while True:
            try:
                print("Listening for wake word...")
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)
                text = recognizer.recognize_google(audio, language="en-US").lower()
                print(f"Heard: {text}")  # Debug print
                
                if "hey robot" in text or "hey robo" in text:  # More flexible wake word detection
                    is_active = True
                    print("\nWake word detected! Robot is now active.")
                    speak("Hello! I am Rainbow, your humanoid robot assistant. How can I help you today?")
                    break
            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                continue
            except Exception as e:
                print(f"Error in wake word detection: {e}")
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
                if "rainbow" in text or "stop" in text:  # Accept both "Rainbow" and "Stop"
                    interrupted = True
                    pygame.mixer.music.stop()
                    print(f"\nInterrupted by user saying '{text}'")
                    break
            except:
                continue

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
        
        # Save to a file in the current directory
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

def get_speech_input(timeout=20, phrase_time_limit=15):
    """Get speech input from the user"""
    recognizer = sr.Recognizer()
    # More robust recognition settings
    recognizer.dynamic_energy_threshold = True
    recognizer.energy_threshold = 200  # Even lower threshold for better sensitivity
    recognizer.pause_threshold = 1.0   # Increased pause threshold for better phrase detection
    recognizer.non_speaking_duration = 0.5  # Shorter non-speaking duration
    recognizer.phrase_threshold = 0.3  # Lower phrase threshold for better phrase detection
    
    with sr.Microphone() as source:
        print(f"Please speak now... (I'll wait for {timeout} seconds)")
        try:
            # Longer ambient noise adjustment
            print("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            
            # Listen with more generous timeouts
            print("Listening...")
            audio = recognizer.listen(
                source,
                timeout=timeout,
                phrase_time_limit=phrase_time_limit
            )
            
            # Use Google's speech recognition with more detailed settings
            text = recognizer.recognize_google(
                audio,
                language="en-US",
                show_all=False  # Get the most likely result
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
        temperature=0.5,  # Lower temperature for faster, more focused responses
        max_tokens=500
    )
    
    return response.choices[0].message.content

def main():
    global is_active
    
    parser = argparse.ArgumentParser(description="Voice Assistant")
    parser.add_argument("--message", type=str, help="Initial message to speak")
    args = parser.parse_args()
    
    print("Starting Rainbow Robot Assistant...")
    
    if args.message:
        # If a message is provided, speak it and exit
        speak(args.message)
        return
        
    print("Say 'Hey robot' to wake me up.")
    print("Say 'Rainbow' or 'Stop' to interrupt my speech.")
    print("Say 'goodbye' to end the conversation, or stay silent for 10 seconds.")
    
    # Start wake word listener
    wake_thread = threading.Thread(target=listen_for_wake_word)
    wake_thread.daemon = True
    wake_thread.start()
    
    silence_count = 0
    max_silence = 3

    while True:
        if not is_active:
            time.sleep(0.1)
            continue
            
        print("\nPlease speak your question:")
        speech_text = get_speech_input()
        
        if not speech_text:
            silence_count += 1
            if silence_count >= max_silence:
                speak("No speech detected for too long. Going back to sleep. Say 'Hey robot' to wake me up!")
                is_active = False
                # Restart wake word listener
                wake_thread = threading.Thread(target=listen_for_wake_word)
                wake_thread.daemon = True
                wake_thread.start()
                continue
            speak(f"I didn't catch that. Please try again. {max_silence - silence_count} attempts remaining")
            continue
            
        silence_count = 0
            
        if "goodbye" in speech_text.lower():
            speak("Goodbye! Have a great day! Say 'Hey robot' when you need me again!")
            is_active = False
            # Restart wake word listener
            wake_thread = threading.Thread(target=listen_for_wake_word)
            wake_thread.daemon = True
            wake_thread.start()
            continue
            
        response = get_response(speech_text)
        was_interrupted = speak(response)
        
        if was_interrupted:
            print("Would you like to ask something else?")

if __name__ == "__main__":
    main() 