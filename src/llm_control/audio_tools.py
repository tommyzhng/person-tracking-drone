import rospy
import numpy as np
from llm_control.agentic_control import AgenticControl
import asyncio
import os

import asyncio
import pyaudio
import wave
import queue
import io
from typing import Optional

from std_msgs.msg import Int16MultiArray
import threading

class AudioTools():
    # def __init__(self):
    #     self.audio_sub = rospy.Subscriber('audio/audio', AudioData, self.audio_callback)
    #     self.decoded_audio = None
    #     self.audio_buffer = b""
    #     self.chunk = 1024

    # def audio_callback(self, msg):
    #     self.audio_buffer += np.frombuffer(msg.data, dtype=np.uint16).tobytes()

    # async def start_streaming(self, agent: AgenticControl):
    #     print(f"Current working directory: {os.getcwd()}")
    #     while not rospy.is_shutdown():
    #         if len(self.audio_buffer) >= self.chunk:
    #             with open("test_audio.raw", "ab") as f:
    #                 print("writing to file")
    #                 f.write(self.audio_buffer)
    #             await agent.stream_audio(self.audio_buffer)
    #             self.audio_buffer = b""  # reset buffer
    #         else:
    #             await asyncio.sleep(0.01)

    def __init__(self):
        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 24000
        self.chunk = 1024

        self.audio = pyaudio.PyAudio()
        # initialize audio as empty array
        self.data = np.array([], dtype=np.int16)
        self.audio_queue = queue.Queue(maxsize=100)


        # Recording params
        self.recording_stream: Optional[pyaudio.Stream] = None
        self.recording_thread = None
        self.recording = False

        # streaming params
        self.streaming = False
        self.stream = None

        # Playback params
        self.playback_stream = None
        self.playback_buffer = queue.Queue(maxsize=20)
        self.playback_event = threading.Event()
        self.playback_thread = None
        self.stop_playback = False


        # for i in range(self.audio.get_device_count()):
        #     info = self.audio.get_device_info_by_index(i)
        #     if info["maxInputChannels"] > 0:
        #         print(f"Index {i}: {info['name']} (Input channels: {info['maxInputChannels']})")

        self.audio_sub = rospy.Subscriber('audio', Int16MultiArray, self.audio_callback)

    def audio_callback(self, msg):
        """Callback function to process incoming audio messages."""
        # Decode the audio data
        audio_array = np.array(msg.data, dtype=np.int16)
        self.data = audio_array.astype(np.int16).tobytes()

        try:
            self.audio_queue.put_nowait(self.data)
        except queue.Full:
            print("Audio queue is full, dropping audio data.")


    async def start_streaming(self, client: AgenticControl):
        """Start continuous audio streaming."""
        if self.streaming:
            return
        
        self.streaming = True
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        print("\nStreaming audio...")
        
        while self.streaming:
            # Gather several chunks to form a continuous stream
            if not self.audio_queue.empty():
                stream_data = b""
                while not self.audio_queue.empty():
                    stream_data += self.audio_queue.get()
                try:
                    await client.stream_audio(stream_data)
                except Exception as e:
                    print(f"Error streaming: {e}")
            await asyncio.sleep(0.01)




