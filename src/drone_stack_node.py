#!/usr/bin/python3

from drone_stack import DroneStack
from drone_state_machine import DroneStateMachine
from llm_control.agentic_control import AgenticControl
from llm_control.audio_tools import AudioTools
import asyncio
import rospy


async def main():
    rospy.init_node('drone_stack_node')
    rate = 60

    # import api key from file
    with open("apikey.txt", "r") as f:
        api_key = f.read().strip()

    flight_stack = DroneStack()
    machine = DroneStateMachine(flight_stack)
    agent = AgenticControl(flight_stack, machine, api_key)
    audio = AudioTools()
    print("Drone stack node is running")
    try:
        await agent.connect()

        message_handler = asyncio.create_task(agent.handle_responses())
        print("Connected to OpenAI")
        print("Audio streaming is starting")
        streaming_handler = asyncio.create_task(audio.start_streaming(agent))

        #await agent.send_text("You are now flying a drone, what are your thoughts, write it in 5 words")

        while not rospy.is_shutdown():
            machine.run_states()
            await asyncio.sleep(1/rate)

    except Exception as e:
        print(f"Main exception: {e}")
    finally:
        await agent.close()
        print("Disconnected from OpenAI")
        print("Audio streaming has stopped")

if __name__ == '__main__':
    asyncio.run(main())
    