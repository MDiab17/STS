# Rainbow Robot Assistant

This project contains two main scripts for running a humanoid robot assistant:

- **voice_assistant_new.py**: Voice assistant with a web-based (Flask) UI and enhanced terminal display.
- **voice_assistant.py**: Voice-only assistant for terminal-based conversation.

## Installation

1. **Set your OpenAI API key:**
   - Create a `.env` file in the `test_sts` directory with the following content:
     ```env
     OPENAI_API_KEY=sk-...
     ```
   - Or set the environment variable in your shell:
     ```bash
     export OPENAI_API_KEY=sk-...
     ```

## Usage

### 1. Voice Assistant with Web UI
This script provides both a voice assistant and a web-based UI (Flask) for visualization.

```bash
cd test_sts
python3 voice_assistant_new.py
```
- Open your browser to [http://localhost:5000](http://localhost:5000) to view the UI.

### 2. Voice-Only Assistant
This script runs the assistant in the terminal with voice interaction only (no web UI).

```bash
cd test_sts
python3 voice_assistant.py
```

## Features
- Wake word detection ("wake up" or "hey robot")
- Voice interruption ("rainbow" or "stop")
- End conversation ("goodbye")
- Uses OpenAI for speech and conversation
- Web UI (in `voice_assistant_new.py` only)

## Notes
- Make sure your microphone is connected and working.
- For best results, run in a quiet environment.
- The web UI is only available in `voice_assistant_new.py`.

