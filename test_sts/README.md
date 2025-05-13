# Rainbow Robot Assistant

An interactive AI-powered robot assistant that combines computer vision and voice interaction. The assistant can detect when someone is looking at the camera, respond to wake words, and engage in natural conversations.

## Features

- Face and eye detection using OpenCV
- Wake word detection ("Hey robot")
- Natural language processing using OpenAI's GPT
- Text-to-speech using OpenAI's TTS
- Speech recognition
- Interruption handling ("Rainbow" or "Stop")
- Automatic sleep mode when no interaction is detected

## Requirements

- Python 3.8 or higher
- Webcam
- Microphone
- Internet connection
- OpenAI API key

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/rainbow-robot-assistant.git
cd rainbow-robot-assistant
```

2. Install dependencies:
```bash
python robot_assistant.py --setup
```

3. Set up your OpenAI API key:
   - Get an API key from [OpenAI](https://platform.openai.com)
   - Replace the API key in `robot_assistant.py` with your own

## Usage

1. Run the assistant:
```bash
python robot_assistant.py
```

2. Interaction flow:
   - Look at the camera
   - Say "Hey robot" to start
   - Ask your questions
   - Say "Rainbow" or "Stop" to interrupt
   - Say "goodbye" to end the conversation

3. To stop the program:
   - Press 'q' in the face detection window
   - Or press Ctrl+C in the terminal

## Project Structure

- `robot_assistant.py`: Main application file
- `requirements.txt`: Python dependencies
- `setup.py`: Installation script

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

```bash
# 1. Source your workspace (after colcon build && colcon source)
source install/setup.bash

# 2. Start the action **server**
ros2 run my_robot_tasks friendly_robot_server
