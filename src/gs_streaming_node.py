#!/usr/bin/python3
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

from std_msgs.msg import Int16MultiArray
import threading

class GSStreamingNode:
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

            # for i in range(self.audio.get_device_count()):
            #     info = self.audio.get_device_info_by_index(i)
            #     if info["maxInputChannels"] > 0:
            #         print(f"Index {i}: {info['name']} (Input channels: {info['maxInputChannels']})")

            # ROS publishers
            self.audio_pub = rospy.Publisher('/audio', Int16MultiArray, queue_size=10)

    async def read_audio(self):
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
            try:
                # Read raw PCM data
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                # send over ros array
                audio_data = Int16MultiArray()
                audio_data.data = np.frombuffer(data, dtype=np.int16).tolist()
                # Publish audio data
                self.audio_pub.publish(audio_data)

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

    def stop_playback_immediately(self):
        """Stop audio playback immediately."""
        self.stop_playback = True
        self.playback_buffer.queue.clear()  # Clear any pending audio
        self.currently_playing = False
        self.playback_event.set()

async def main():
    rospy.init_node('gs_streaming_node')
    node = GSStreamingNode()
    rate = 60

    try:
        streaming = asyncio.create_task(node.read_audio())
        while not rospy.is_shutdown():
            await asyncio.sleep(1/rate)
             
    except Exception as e:
        print(f"Main exception: {e}")

    finally:
        node.cleanup()
        print("Audio streaming has stopped")
    

if __name__ == '__main__':
    asyncio.run(main())