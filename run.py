# sim_app/run.py
import asyncio
from sim_app.main import run
from sim_app.LLM import recognize_speech, parse_command_with_gpt, get_coordinates, send_to_robot

def handle_voice_command():
    spoken_text = recognize_speech()
    if spoken_text:
        result = parse_command_with_gpt(spoken_text)
        if result:
            robot_id = result.get("robot_id") or "Robot1"
            destination = result.get("destination")
            coords = get_coordinates(destination)
            if coords:
                send_to_robot(robot_id, coords)
                return True
            else:
                print(f"❌ Unknown destination: {destination}")
        else:
            print("❌ Failed to parse command.")
    return False

if __name__ == "__main__":
    #print("🎤 Waiting for your voice command...")
    #if handle_voice_command():
    #    print("✅ Voice command parsed. Starting simulation...")
    asyncio.run(run())
    #else:
    #    print("🛑 Aborted due to command failure.")

