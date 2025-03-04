import rospy
from audio_common_msgs.msg import AudioData
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
        
        print("\nStreaming audio... Press 'q' to stop.")
        
        while self.streaming:
            try:
                # Read raw PCM data
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                # Stream directly without trying to decode
                await client.stream_audio(data)
            except Exception as e:
                print(f"Error streaming: {e}")
                break
            await asyncio.sleep(0.01)

    def stop_streaming(self):
        """Stop audio streaming."""
        self.streaming = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

    def stop_playback_immediately(self):
        """Stop audio playback immediately."""
        self.stop_playback = True
        self.playback_buffer.queue.clear()  # Clear any pending audio
        self.currently_playing = False
        self.playback_event.set()

    def cleanup(self):
        """Clean up audio resources"""
        self.stop_playback_immediately()

        self.stop_playback = True
        if self.playback_thread:
            self.playback_thread.join()

        self.recording = False
        if self.recording_stream:
            self.recording_stream.stop_stream()
            self.recording_stream.close()
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

        self.audio.terminate()
        




