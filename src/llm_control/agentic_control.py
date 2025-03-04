## file is based on https://github.com/run-llama/openai_realtime_client/blob/main/openai_realtime_client/client/realtime_client.py

from drone_stack import DroneStack
from drone_state_machine import DroneStateMachine
import rospy

# gpt-4o realtime
from openai import OpenAI
import asyncio
import json
import websockets
from llm_control.tools import OpenAITools
import base64


class AgenticControl():
    def __init__(self, 
                 drone_stack: DroneStack, 
                 drone_state: DroneStateMachine,
                 api: str,
                 model: str = "gpt-4o-realtime-preview-2024-12-17",
                 voice: str = "alloy",
                 instructions: str = "You are now flying a drone",  
                 temperature: float = 0.8,
                 ):
        self.drone = drone_stack
        self.state = drone_state

        # gpt-4o realtime
        self.api = api
        self.model = model
        self.voice = voice
        self.instructions = instructions
        self.temperature = temperature
        self.url = f"wss://api.openai.com/v1/realtime?model={self.model}"
        self.ws = None
        self.tools = OpenAITools().tools

        # responses
        self._response_id = None
        self._item_id = None
        self._is_responding = False

    async def connect(self) -> None:
        "websocket connection to realtime API"
        
        url = self.url
        headers = {
            "Authorization": "Bearer " + self.api,
            "OpenAI-Beta": "realtime=v1"
        }

        self.ws = await websockets.connect(url, extra_headers=headers)    

        tools = [{'type': 'function', **t['function']} for t in self.tools]

        config = {
            "modalities": ["text"],
            "instructions": self.instructions,
            "voice": self.voice,
            "temperature": self.temperature,
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {
                "model": "whisper-1",
            },
            "turn_detection": {
                "type": f"server_vad",
                "threshold": 0.5,
                "prefix_padding_ms": 500,
                "silence_duration_ms": 1000
            },
            "tools": tools,
            "tool_choice": "auto",
        }
        print("setup config")

        await self.session_update(config)

    async def session_update(self, config) -> None:
        "initialize session configuration"
        event = {
            "type": "session.update",
            "session": config
        }
        await self.ws.send(json.dumps(event))

    async def send_text(self, text: str) -> None:
        """Send text message to the API."""
        event = {
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [{
                    "type": "input_text",
                    "text": text
                }]
            }
        }
        await self.ws.send(json.dumps(event))
        await self.create_response()

    async def create_response(self) -> None:
        event = {
            "type": "response.create",
            "response": {
                "modalities": ["text", "audio"]
            }
        }
        await self.ws.send(json.dumps(event))

    async def stream_audio(self, data) -> None:
        "send audio to realtime API"
        decoded = base64.b64encode(data).decode()
        appended_event = {
            "type" : "input_audio_buffer.append",
            "audio" : decoded
        }
        await self.ws.send(json.dumps(appended_event))

    async def handle_interruption(self) -> None:
        "handle interruption during responses"
        if not self._is_responding:
            return

        if self._response_id:
            # cancel the response
            event = {
                "type": "response.cancel"
            }
            await self.ws.send(json.dumps(event))
        
        if self._item_id:
            # truncate the conversation
            event = {
                "type": "conversation.item.truncate",
                "item_id": self._item_id
            }
            await self.ws.send(json.dumps(event))

        self._is_responding = False
        self._response_id = None
        self._item_id = None

    


    async def handle_responses(self) -> None:
        try: 
            async for message in self.ws:
                event = json.loads(message)
                event_type = event.get("type")

                if event_type == "error":
                    print(f"Chat Error: {event}")
                    continue
                elif event_type == "response.created":
                    self._response_id = event.get("response", {}).get("id")
                    self._is_responding = True
                elif event_type == "response.output_item.added":
                    self._item_id = event.get("item", {}).get("id")
                elif event_type == "response.done":
                    self._is_responding = False
                    self._response_id = None
                    self._item_id = None

                elif event_type == "input_audio_buffer.speech_started":
                    print("\n[Speech detected]")
                    if self._is_responding:
                        await self.handle_interruption()
                    # add interrupt function to stop audio playback when using audio feedback from api
                    
                elif event_type == "input_audio_buffer.speech_stopped":
                    print("\n[Speech ended]")

                elif event_type == "response.content_part.done":
                    print(f"\n[Text response]: {event.get('part', {}).get('text')}")
                    
                elif event_type == "response.function_call_arguments.done":
                    print(f"\nFunction call: {event['call_id']}, event: {event['name']}, arguments: {json.loads(event['arguments'])}")

        except websockets.exceptions.ConnectionClosed:
            print("Connection closed")
        except Exception as e:
            print(f"Error: {e}")
    
    async def close(self) -> None:
        "close websocket"
        if self.ws:
            await self.ws.close()

    

    


    

    



