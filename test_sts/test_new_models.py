# --- 1. Imports -------------------------------------------------------------
import asyncio
import os
import time
import speech_recognition as sr
from typing import Optional

from openai import AsyncOpenAI
from openai.helpers import LocalAudioPlayer          # requires openai 1.7x
# ---------------------------------------------------------------------------

# --- 2. Configure credentials ----------------------------------------------
# KEEP SECRETS OUT OF SOURCE CODE — load from an env‑var or secret‑manager  
client = AsyncOpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
# ---------------------------------------------------------------------------

# --- 3. Friendly‑robot voice parameters ------------------------------------
VOICE = "nova"  # crisp, upbeat, “companion” vibe. Try "alloy" or "echo" too.
INSTRUCTIONS = (
    "Delivery: warm, slightly synthetic, with gentle rises at the ends of sentences. "
    "Voice: friendly service robot—cheerful, helpful and calm. "
    "Tone: upbeat and supportive; keep the pace medium‑fast with clear diction."
)
# ---------------------------------------------------------------------------

async def tts_stream(text: str) -> None:
    """
    Stream `text` through OpenAI TTS and play it through the system speakers.
    """
    async with client.audio.speech.with_streaming_response.create(
        model="gpt-4o-mini-tts",
        voice=VOICE,
        input=text,
        instructions=INSTRUCTIONS,
        response_format="pcm",     # default mp3 is fine too, but pcm is snappy
    ) as resp:
        await LocalAudioPlayer().play(resp)

def speak(text: str) -> None:
    """
    Synchronous wrapper so the *rest* of your code can stay blocking.
    """
    print(f"Robot 🤖: {text}")
    asyncio.run(tts_stream(text))
# ---------------------------------------------------------------------------

# ------------- 4. Everything else in your script stays the same ------------

def get_speech_input(timeout: int = 10) -> Optional[str]:
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print(f"Please speak now… (I'll wait {timeout}s)")
        try:
            audio = recognizer.listen(source, timeout=timeout)
            text  = recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text
        except sr.WaitTimeoutError:
            print("🔇  No speech detected.")
            return None
        except sr.UnknownValueError:
            print("🤔  Sorry, I didn’t catch that.")
            return None
        except sr.RequestError as e:
            print(f"Speech‑API error: {e}")
            return None

def get_response(prompt: str) -> str:
    """
    Tiny helper that hits GPT‑4o chat for a short, question‑ending reply.
    """
    chat = client.chat.completions.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3,
        max_tokens=100,
    )
    return chat.choices[0].message.content.strip()

# ------------------------- Main loop ---------------------------------------
if __name__ == "__main__":
    print("Say 'goodbye' to quit. Staying silent 3 times also ends the chat.")
    speak("Hello! I am your friendly robot assistant. How can I help?")
    silence_streak = 0
    MAX_SILENCE   = 3

    while True:
        utterance = get_speech_input()
        if not utterance:
            silence_streak += 1
            if silence_streak >= MAX_SILENCE:
                speak("It’s been quiet for a while. Ending our conversation. Goodbye!")
                break
            speak(f"I didn’t hear anything. {MAX_SILENCE - silence_streak} tries left.")
            continue

        silence_streak = 0
        if "goodbye" in utterance.lower():
            speak("Goodbye! Have a wonderful day!")
            break

        answer = get_response(utterance)
        speak(answer)
