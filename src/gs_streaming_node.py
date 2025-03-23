#!/usr/bin/python3
import rospy


async def main():
    rospy.init_node('gs_node')
    rate = 60

    
    try:
        await agent.connect()

        #await agent.send_text("You are now flying a drone, what are your thoughts, write it in 5 words")

        while not rospy.is_shutdown():
            machine.run_states()
            await agent.check_status()
            await asyncio.sleep(1/rate)

    except Exception as e:
        print(f"Main exception: {e}")
    finally:
        await agent.close()
        print("Disconnected from OpenAI")
        print("Audio streaming has stopped")

if __name__ == '__main__':
    asyncio.run(main())
    