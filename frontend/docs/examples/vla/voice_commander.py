import openai
import os

def transcribe_audio(file_path):
    """
    Transcribes audio using OpenAI Whisper API.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File {file_path} not found")

    with open(file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
    
    return transcript["text"]

def plan_action_from_text(text):
    """
    Simple planner mock using GPT.
    """
    prompt = f"Convert this command to ROS2 actions: '{text}'. Output JSON."
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )
    return response.choices[0].message.content

if __name__ == "__main__":
    # Mock usage
    print("VLA Pipeline Ready")
