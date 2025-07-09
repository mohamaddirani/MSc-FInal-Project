# sim_app/LLM.py

import speech_recognition as sr
from openai import OpenAI
import json
import sim_app.shared as shared

# Initialize OpenAI client with your API key
client = OpenAI(api_key="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")

# Predefined map of destination labels to coordinates
location_map = {
    "point a": (1.5, 3.0),
    "point b": (-2.0, 1.0),
    "kitchen": (8.125, 8.15)
}

# Step 1: Recognize speech from microphone
def recognize_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Say your command...")
        audio = recognizer.listen(source)
    try:
        text = recognizer.recognize_google(audio)
        print("📝 You said:", text)
        return text
    except sr.UnknownValueError:
        print("❌ Could not understand audio.")
        return None

# Step 2: Parse command using GPT
def parse_command_with_gpt(text):
    prompt = f"""
    You are an API that receives robot commands in natural language and returns JSON only.

    Example:
    Input: "Robot1 go to point A"
    Output: {{"robot_id": "Robot1", "destination": "point A"}}
    Input: "Robot one go to the kitchen"
    Output: {{"robot_id": "Robot1", "destination": "kitchen"}}
    Input: "Robot1 go to the kitchen"
    Output: {{"robot_id": "Robot1", "destination": "kitchen"}}

    Now parse:
    "{text}"
    Respond with JSON only. No explanation.
    """
    try:
        response = client.chat.completions.create(
            model="gpt-4-1106-preview",
            messages=[{"role": "user", "content": prompt}]
        )

        reply = response.choices[0].message.content.strip()

        # 🧹 Remove Markdown formatting and labels like "json"
        if reply.startswith("```"):
            reply = reply.split("```")[1].strip()
        if reply.lower().startswith("json"):
            reply = reply[4:].strip()

        print("🤖 GPT Output:", reply)
        return json.loads(reply)

    except Exception as e:
        print("❌ GPT error:", e)
        print("📝 Raw GPT output that caused error:", reply)
        return None



# Step 3: Convert destination string to coordinates
def get_coordinates(destination):
    return location_map.get(destination.lower())

# Step 4: Send movement command to the robot (stub)
def send_to_robot(robot_id, coordinates):
    print(f"🚀 Sending {robot_id} to coordinates {coordinates}")
    shared.robot_goal = list(coordinates)