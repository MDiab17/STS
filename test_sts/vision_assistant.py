import socket
import struct
import argparse
import time
from datetime import datetime
from voice_assistant import speak, listen_for_wake_word, get_speech_input, get_response
import threading

def receive_looking_status(port):
    # Set up UDP socket for receiving looking status
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[Looking Status Receiver] Listening on UDP port {port}")
    
    last_status = None
    is_active = False
    
    # Play welcome message
    welcome_text = """Hello! I am Rainbow, your humanoid robot assistant. 
    To interact with me, please look at the camera and say 'Hey robot'.
    You can say 'Rainbow' or 'Stop' to interrupt me at any time.
    Say 'goodbye' to end our conversation."""
    speak(welcome_text)
    
    try:
        while True:
            # Receive looking status data
            data, addr = sock.recvfrom(1024)
            
            # Unpack boolean status and timestamp
            looking_at_camera, timestamp = struct.unpack('?q', data)
            
            # Only print status when it changes
            if last_status != looking_at_camera:
                status_text = "LOOKING AT CAMERA" if looking_at_camera else "NOT LOOKING AT CAMERA"
                current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{current_time}] Status: {status_text}")
                last_status = looking_at_camera
                
                if looking_at_camera and not is_active:
                    is_active = True
                    # Start wake word detection in a separate thread
                    wake_thread = threading.Thread(target=listen_for_wake_word)
                    wake_thread.daemon = True
                    wake_thread.start()
                    
                    # Start conversation loop
                    silence_count = 0
                    max_silence = 3
                    
                    while is_active:
                        print("\nPlease speak your question:")
                        speech_text = get_speech_input()
                        
                        if not speech_text:
                            silence_count += 1
                            if silence_count >= max_silence:
                                speak("No speech detected for too long. Going back to sleep. Look at the camera and say 'Hey robot' to wake me up!")
                                is_active = False
                                break
                            speak(f"I didn't catch that. Please try again. {max_silence - silence_count} attempts remaining")
                            continue
                            
                        silence_count = 0
                            
                        if "goodbye" in speech_text.lower():
                            speak("Goodbye! Have a great day! Look at the camera and say 'Hey robot' when you need me again!")
                            is_active = False
                            break
                            
                        response = get_response(speech_text)
                        was_interrupted = speak(response)
                        
                        if was_interrupted:
                            print("Would you like to ask something else?")
                
    except KeyboardInterrupt:
        print("\nReceiver stopped.")
    finally:
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Looking Status Receiver")
    parser.add_argument("--port", type=int, default=65433, 
                        help="UDP port to listen for looking status (default: 65433)")
    args = parser.parse_args()
    
    receive_looking_status(args.port)