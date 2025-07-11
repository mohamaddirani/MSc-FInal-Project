# sim_app/LLM.py

import speech_recognition as sr
from openai import OpenAI
import json
import sim_app.shared as shared

# Initialize OpenAI client with your API key
client = OpenAI(api_key="Hidden for security reasons")

# Predefined map of destination labels to coordinates
location_map = {
    "point a": (1.5, 3.0),
    "point b": (-2.0, 1.0),
    "point c": (8.125, 8.15)
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
    """
    Parses a natural language robot command using GPT and returns a structured JSON object.
    Given a text command, this function sends a prompt to a GPT model to extract:
    - The robot ID (if specified, e.g., "Robot1", "Robot0"; otherwise null)
    - The destination (e.g., "point A", "point B", "point C")
    The function expects the GPT model to respond with a JSON object in the format:
        {
            "robot_id": "Robot1" or null,
            "destination": "point A"
        }
    Args:
        text (str): The natural language command to parse.
    Returns:
        dict or None: A dictionary with keys "robot_id" and "destination" if parsing is successful,
                        or None if an error occurs.

    """
    prompt = f"""
        You are an API that receives robot commands in natural language and returns JSON only.

        Each command might include:
        - A destination (e.g., "point A", "point C")
        - An optional robot ID (like "Robot1", "Robot0")

        If no robot is mentioned, return null for "robot_id".

        Always return in this JSON format:
        {{"robot_id": "Robot1" or null, "destination": "point A"}}

        Examples:
        Input: "Robot1 go to point A"
        Output: {{"robot_id": "Robot1", "destination": "point A"}}

        Input: "Go to point B"
        Output: {{"robot_id": null, "destination": "point B"}}

        Input: "Send someone to point C"
        Output: {{"robot_id": null, "destination": "point C"}}

        Input: "Robot zero move to point C"
        Output: {{"robot_id": "Robot0", "destination": "point C"}}

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
    if destination is None:
        print("⚠️ No destination provided. Waiting for a valid command.")
        return None
    coords = location_map.get(destination.lower())
    if coords is None:
        print(f"⚠️ Unknown destination: '{destination}'. Waiting for a valid command.")
    return coords



# Step 4: Send movement command to the robot (stub)
def send_to_robot(robot_id, coordinates):
    if not coordinates:
        print("⚠️ No valid coordinates. Robot not dispatched.")
        shared.robot_goal = None
        shared.robot_name = None
        return
    print(f"🚀 Sending {robot_id or 'auto-selected robot'} to coordinates {coordinates}")
    shared.robot_goal = list(coordinates)
    shared.robot_name = robot_id.replace("Robot", "Rob") if robot_id else None


