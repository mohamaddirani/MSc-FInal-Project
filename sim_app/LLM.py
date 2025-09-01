# sim_app/LLM.py
"""
Voice-driven command interface:
- Listens via microphone, parses user intent with GPT, and writes command/goal files.
- Supports move/status/stop/go_home/position/nearest_to_location and obstacle actions.
"""

import asyncio
import difflib
import json
import os
import time

import pyttsx3
import speech_recognition as sr
from dotenv import load_dotenv
from openai import OpenAI

import sim_app.shared as shared


# ============================================================================
# Setup
# ============================================================================

load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

recognizer = sr.Recognizer()
engine = pyttsx3.init()

# Files shared with main loop
CMD_FILE = "shared_cmd.json"
GOAL_FILE = "shared_goal.json"
REPLY_FILE = "shared_reply.json"

# ============================================================================
# Locations & robots
# ============================================================================

# Location map (as you had)
location_map = {
    "Meetings Table": (-1.500, -7.000),
    "Office Table": (-2.000, 2.000),
    "Cupboard A": (6.800, 1.000),
    "Cupboard B": (3.800, 1.000),
    "Cupboard C": (3.800, -3.000),
    "Cupboard D": (6.800, -3.000),
    "Meetings Rack": (-4.500, -4.000),
    "Office Rack": (-4.500, 0.000),
    "Rack A": (8.000, -5.000),
    "Rack B": (5.000, -5.000),
    "Rack C": (8.000, -8.500),
    "Rack D": (5.000, -8.500),
    "Rack 1": (3.000, -2.000),
    "Rack 2": (3.000, -6.000),
    "Rack 3": (3.000, -10.000),
    "test" : (7.000, -0.700),
}

robot_list = ["Rob0", "Rob1", "Rob2"]


# ============================================================================
# Speech helpers
# ============================================================================

def speak(text: str) -> None:
    """TTS helper: say and print the same text."""
    print(text)
    engine.say(text)
    engine.runAndWait()


def listen() -> str | None:
    """Capture audio from mic and return a lowercase transcript (Google SR)."""
    with sr.Microphone() as source:
        print("Listening...")
        audio = recognizer.listen(source)
    try:
        command = recognizer.recognize_google(audio)
        print(f"You said: {command}")
        return command.lower()
    except sr.UnknownValueError:
        speak("Sorry, I did not understand.")
        return None
    except sr.RequestError as e:
        speak(f"Google Speech Recognition service error: {e}")
        print("Troubleshooting tips:")
        print("- Check your internet connection.")
        print("- Make sure your firewall is not blocking Python.")
        print("- Try running on a different network.")
        return None


# ============================================================================
# Parsing & mapping
# ============================================================================

def parse_command_with_gpt(text: str):
    """
    The model returns ONE of these intent payloads:

    {
      "intent": "move",
      "robot_id": "Rob1" | null,
      "destination": "Meetings Table"
    }

    {
      "intent": "status",
      "robot_id": "Rob2" | null     # null => all
    }

    { "intent": "stop", "robot_id": "Rob1" }          # force stop
    { "intent": "go_home", "robot_id": "Rob0" }       # return to start

    {
      "intent": "position",
      "robot_id": "Rob2" | null     # null => all
    }

    {
      "intent": "nearest_to_location",
      "location_label": "Rack 1"
    }

    {
      "intent": "obstacle_action",
      "robot_id": "RobX" or null,
      "action": "wait" | "continue" | "stop" | "go_home" | "set_goal",
      "destination": "<valid location>"  # required only when action == "set_goal"
    }
    """
    prompt = f"""
You are a STRICT command parser. Output ONLY JSON (no prose, no code fences). Choose the single best intent.

Valid location labels: {list(location_map.keys())}
Valid robots: {robot_list}

Return exactly one of these schemas:

1) Move:
{{
  "intent": "move",
  "robot_id": "RobX" or null,
  "destination": "<one of the valid location labels>"
}}

2) Status query:
{{ "intent": "status", "robot_id": "RobX" or null }}

3) Force stop:
{{ "intent": "stop", "robot_id": "RobX" }}

4) Go back to start:
{{ "intent": "go_home", "robot_id": "RobX" }}

5) Position query:
{{ "intent": "position", "robot_id": "RobX" or null }}

6) Nearest robot to a named location:
{{ "intent": "nearest_to_location", "location_label": "<valid location>" }}

7) Obstacle control (when user says 'continue', 'wait', 'stop', 'go home', or sets a new goal during an obstacle pause):
{{
  "intent": "obstacle_action",
  "robot_id": "RobX" or null,
  "action": "wait" | "continue" | "stop" | "go_home" | "set_goal",
  "destination": "<valid location>"  # REQUIRED only when action == "set_goal"
}}

Rules:
- If the user says 'continue', 'keep going', 'proceed', 'handle it', or 'resume' → use intent "obstacle_action" with action "continue".
- If the user says 'wait' / 'hold on' → "obstacle_action" with action "wait".
- If the user says 'stop' / 'abort' / 'cancel' → prefer intent "stop" (robot-directed), else "obstacle_action" with action "stop".
- If the user says 'go home' / 'go back home' / 'return to base' → prefer intent "go_home", else "obstacle_action" with action "go_home".
- Only use "move" when a valid location label is present. Never invent locations from arbitrary words like 'continue'.
- Accept robot mentions like 'rob0', 'rob 0', 'robot0' as 'Rob0', etc.

Examples (these are EXAMPLES ONLY; your output must be JSON for the current user text):

User: "rob0 continue"
→ {{ "intent": "obstacle_action", "robot_id": "Rob0", "action": "continue" }}

User: "continue"
→ {{ "intent": "obstacle_action", "robot_id": null, "action": "continue" }}

User: "go back home rob2"
→ {{ "intent": "go_home", "robot_id": "Rob2" }}

User: "set a new goal to rack b for rob1"
→ {{ "intent": "obstacle_action", "robot_id": "Rob1", "action": "set_goal", "destination": "Rack B" }}

User: "nearest robot to meetings table"
→ {{ "intent": "nearest_to_location", "location_label": "Meetings Table" }}

Now parse only this user text:
"{text}"

Output JSON only.
"""

    response = client.chat.completions.create(
        model="gpt-4o-mini",  # small, fast
        messages=[{"role": "user", "content": prompt}],
    )
    reply = response.choices[0].message.content.strip()

    # Trim code fences or leading 'json'
    if reply.startswith("```"):
        reply = reply.split("```")[1].strip()
    if reply.lower().startswith("json"):
        reply = reply[4:].strip()

    print("🤖 GPT Output:", reply)
    return json.loads(reply)


def fuzzy_label(label: str) -> str | None:
    """Return the nearest valid location label, or None."""
    if not label:
        return None
    keys = list(location_map.keys())
    match = difflib.get_close_matches(label, keys, n=1, cutoff=0.5)
    return match[0] if match else None


def coords_for_label(label: str) -> tuple[str | None, tuple[float, float] | None]:
    """Fuzzy map a label to coordinates."""
    best = fuzzy_label(label)
    if not best:
        print(f"⚠️ Unknown location name: '{label}'")
        return None, None
    return best, location_map[best]


# ============================================================================
# File IO to talk with main loop
# ============================================================================

async def send_move_goal(sim, robot_id, dest_label):
    """Write a move goal to GOAL_FILE; 'auto' dispatch if robot_id is None."""
    label, coords = coords_for_label(dest_label)
    if not coords:
        print("🛑 Invalid location. Try again.")
        return

    if robot_id:
        robot_id = robot_id.replace("Robot", "Rob")  # sanitize

    payload = {(robot_id or "auto"): coords}  # main.py handles 'auto' by nearest idle
    print(f"🚀 Move: {robot_id or 'auto'} -> {label} @ {coords}")
    with open(GOAL_FILE, "w") as f:
        json.dump(payload, f)


def write_cmd(obj: dict) -> None:
    """Overwrite any previous pending command."""
    with open(CMD_FILE, "w") as f:
        json.dump(obj, f)


def try_read_reply():
    """Read and delete the shared reply file; return its JSON or None."""
    if not os.path.exists(REPLY_FILE):
        return None
    try:
        with open(REPLY_FILE, "r") as f:
            data = json.load(f)
        # best-effort cleanup
        try:
            os.remove(REPLY_FILE)
        except Exception:
            pass
        return data
    except Exception as e:
        print(f"⚠️ Failed to read reply: {e}")
        return None

def extract_robot_id_from_text(text: str) -> str | None:
    """
    Heuristics to pull a robot id from free text: 'rob0', 'rob 0', 'robot0', 'robot 2', case-insensitive.
    Returns canonical 'Rob0' / 'Rob1' / 'Rob2' or None.
    """
    t = text.lower().replace("-", " ")
    for i in range(3):
        patterns = [f"rob{i}", f"rob {i}", f"robot{i}", f"robot {i}"]
        if any(p in t for p in patterns):
            return f"Rob{i}"
    return None

# ============================================================================
# Main async loop
# ============================================================================

async def main_loop(sim):
    """Listen for voice commands, parse with GPT, and write goals/commands."""
    print("🤖 LLM Command Listener is active. Press Ctrl+C to stop.")
    #await send_move_goal(sim, "Rob0", "test")  # initial test goal
    while True:
        try:
            text = listen()
            if not text:
                continue
            rid_hint = extract_robot_id_from_text(text)  # may be None
            t = text.lower()            
            if any(kw in t for kw in ["continue", "keep going", "proceed", "handle it", "resume"]):
                write_cmd({"type": "obstacle_action", "robot_id": rid_hint, "action": "continue"})
                speak("Continuing with obstacle handling.")
                time.sleep(0.3)
                continue

            if any(kw in t for kw in ["wait", "hold on", "pause here"]):
                write_cmd({"type": "obstacle_action", "robot_id": rid_hint, "action": "wait"})
                speak("Waiting for the path to clear.")
                time.sleep(0.3)
                continue

            if any(kw in t for kw in ["stop", "abort", "cancel"]):
                target = rid_hint or None
                if target:
                    write_cmd({"type": "stop", "robot_id": target})
                    speak(f"Stopping {target}.")
                else:
                    # no robot specified -> stop is ambiguous; default to obstacle-action stop for the paused one
                    write_cmd({"type": "obstacle_action", "robot_id": None, "action": "stop"})
                    speak("Stopping.")
                time.sleep(0.3)
                continue

            if any(kw in t for kw in ["go home", "go back home", "return to base", "return home", "back to start"]):
                target = rid_hint or None
                if target:
                    write_cmd({"type": "go_home", "robot_id": target})
                    speak(f"Sending {target} home.")
                else:
                    # if no robot, default to 'auto' home for the currently paused robot via obstacle_action
                    write_cmd({"type": "obstacle_action", "robot_id": None, "action": "go_home"})
                    speak("Going home.")
                time.sleep(0.3)
                continue
            parsed = parse_command_with_gpt(text)
            intent = (parsed or {}).get("intent")

            if intent == "move":
                await send_move_goal(sim, parsed.get("robot_id"), parsed.get("destination"))

            elif intent == "status":
                write_cmd({"type": "status", "robot_id": parsed.get("robot_id")})
                await asyncio.sleep(0.2)
                reply = try_read_reply()
                print("📣 Status:", reply or "(no reply)")

            elif intent == "stop":
                write_cmd({"type": "stop", "robot_id": parsed.get("robot_id")})
                print(f"🛑 Stop sent to {parsed.get('robot_id')}")

            elif intent == "go_home":
                write_cmd({"type": "go_home", "robot_id": parsed.get("robot_id")})
                print(f"🏠 Go-home sent to {parsed.get('robot_id')}")

            elif intent == "position":
                write_cmd({"type": "position", "robot_id": parsed.get("robot_id")})
                await asyncio.sleep(0.2)
                reply = try_read_reply()
                print("📍 Position:", reply or "(no reply)")

            elif intent == "nearest_to_location":
                label = parsed.get("location_label")
                best = fuzzy_label(label)
                if not best:
                    print(f"⚠️ Unknown location: '{label}'")
                else:
                    write_cmd({"type": "nearest_to_location", "location_label": best})
                    await asyncio.sleep(0.2)
                    reply = try_read_reply()
                    print("🤖 Nearest robot:", reply or "(no reply)")

            elif intent == "obstacle_action":
                action = (parsed.get("action") or "").lower()
                rid = parsed.get("robot_id")
                if rid:
                    rid = rid.replace("Robot", "Rob")

                if action in ("wait", "continue"):
                    # obstacle_action is ONLY for the executor to read
                    write_cmd({
                        "type": "obstacle_action",
                        "robot_id": rid,   # None → target the paused robot
                        "action": action,
                    })
                    speak("Waiting for the path to clear." if action == "wait"
                        else "Continuing with obstacle handling.")
                    time.sleep(0.3)
                    continue

                if action in ("go_home", "stop"):
                    # send a *direct* command that main.py understands
                    if rid is None:
                        # If user didn't name a robot, try to infer from speech like "rob 0"
                        rid = extract_robot_id_from_text(text)
                    write_cmd({"type": "go_home" if action == "go_home" else "stop",
                            "robot_id": rid})
                    speak("Going home." if action == "go_home" else "Stopping.")
                    time.sleep(0.3)
                    continue

                if action == "set_goal":
                    dest = parsed.get("destination")
                    label, coords = coords_for_label(dest)
                    if coords:
                        payload = {(rid or "auto"): coords}
                        with open(GOAL_FILE, "w") as f:
                            json.dump(payload, f)
                        speak(f"Setting new goal {label}.")
                    else:
                        speak("Unknown destination.")
                    time.sleep(0.3)
                    continue


        except KeyboardInterrupt:
            print("\n👋 Stopping LLM listener.")
            break


# ============================================================================
# Entrypoint
# ============================================================================

if __name__ == "__main__":
    from sim_app.sim_client import get_sim

    async def start():
        client, sim = await get_sim()
        await main_loop(sim)

    asyncio.run(start())
