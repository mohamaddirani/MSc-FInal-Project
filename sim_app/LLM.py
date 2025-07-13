# sim_app/LLM.py

import json
import speech_recognition as sr
from openai import OpenAI
import json
import sim_app.shared as shared
import time
import asyncio
from sim_app.check_nearest_robot import find_available_robot

# Initialize OpenAI client
client = OpenAI(api_key="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")

# Predefined map of destination labels to coordinates
location_map = {
    "point a": (1.5, 3.0),
    "point b": (-2.0, 1.0),
    "point c": (-6.24974, +6.36916)
}

# List of known robot names
robot_list = ["Rob0", "Rob1", "Rob2"]

def recognize_speech():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("\n🎙️ Say your command...")
        audio = recognizer.listen(source)
    try:
        text = recognizer.recognize_google(audio)
        print("📝 You said:", text)
        return text
    except sr.UnknownValueError:
        print("❌ Could not understand audio.")
        return None

def parse_command_with_gpt(text):
    prompt = f"""
    You are an API that receives robot commands in natural language and returns JSON only.

    Each command might include:
    - A destination (e.g., "point A", "point C")
    - An optional robot ID (like "Robot1", "Robot0")

    If no robot is mentioned, return null for "robot_id".

    Always return in this JSON format:
    {{"robot_id": "Robot1" or null, "destination": "point A"}}

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
        if reply.startswith("```"):
            reply = reply.split("```")[1].strip()
        if reply.lower().startswith("json"):
            reply = reply[4:].strip()

        print("🤖 GPT Output:", reply)
        return json.loads(reply)

    except Exception as e:
        print("❌ GPT error:", e)
        if 'reply' in locals():
            print("📝 Raw GPT output that caused error:", reply)
        return None


def get_coordinates(destination):
    if destination is None:
        print("⚠️ No destination provided.")
        return None
    coords = location_map.get(destination.lower())
    if coords is None:
        print(f"⚠️ Unknown destination: '{destination}'.")
    return coords

def find_available_robot():
    for robot in robot_list:
        if shared.robot_status.get(robot) == "idle":
            return robot
    return None



async def send_to_robot(sim, robot_id, goal_pos):
    if robot_id is None:
        robot_id, start_pos = await find_available_robot(sim, goal_pos)
        if robot_id is None:
            print("⚠️ Could not find available robot.")
            return
    else:
        robot_id = robot_id.replace("Robot", "Rob")
        start_pos = None  # Optional: can also look it up from controller if needed

    print(f"🚀 Sending {robot_id} to coordinates {goal_pos}")

    shared.robot_name = robot_id
    shared.robot_goal[robot_id] = goal_pos
    shared.robot_status[robot_id] = "busy"
    if start_pos:
        shared.robot_start = start_pos

    # Save to file so main.py picks it up
    with open("shared_goal.json", "w") as f:
        json.dump({robot_id: goal_pos}, f)


async def main_loop(sim):
    print("🤖 LLM Command Listener is active. Press Ctrl+C to stop.")
    while True:
        try:
            text = recognize_speech()
            if not text:
                continue
            
            parsed = parse_command_with_gpt(text)
            if parsed:
                dest = parsed.get("destination")
                robot = parsed.get("robot_id")
                coords = get_coordinates(dest)
                if coords is None:
                    print("🛑 Invalid location. Try again.")
                    continue
                await send_to_robot(sim, robot, coords)
            time.sleep(1)
        except KeyboardInterrupt:
            print("\n👋 Stopping LLM listener.")
            break

if __name__ == "__main__":
    import asyncio
    from sim_app.sim_client import get_sim

    async def start():
        client, sim = await get_sim()
        await main_loop(sim)

    asyncio.run(start())
