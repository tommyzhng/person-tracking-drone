class OpenAITools:
    def __init__(self):
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "enu_frame_control",
                    "description": "Controls the local drone position relative to home position in the ENU frame. This is relative meaning that your command will add onto your previous one. Strictly limit your commands to 5 meters in any direction",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "x": {
                                "type": "number",
                                "description": "X position in meters from home (East is positive)"
                            },
                            "y": {
                                "type": "number",
                                "description": "Y position in meters from home (North is positive)"
                            },
                            "z": {
                                "type": "number",
                                "description": "Z position in meters from home (Up is positive)"
                            }
                        },
                        "required": ["x", "y", "z"]
                    }
                }
            },
            # {
            #     "type": "function",
            #     "function": {
            #         "name": "body_frame_control",
            #         "description": "Controls the local drone position relative to the body frame. drone is horizontal, front is +x, right is +y, up is +z. This is relative meaning that your command will add onto your previous one. Must switch states first to body control to access these commands",
            #         "parameters": {
            #             "type": "object",
            #             "properties": {
            #                 "x": {
            #                     "type": "number",
            #                     "description": "Forward position in meters from current position (front is positive)"
            #                 },
            #                 "y": {
            #                     "type": "number",
            #                     "description": "Right position in meters from current position (right is positive)"
            #                 },
            #                 "z": {
            #                     "type": "number",
            #                     "description": "Up position in meters from current position (up is positive)"
            #                 }
            #             },
            #             "required": ["x", "y", "z"]
            #         }
            #     }
            # },
            {
                "type": "function",
                "function": {
                    "name": "set_state",
                    "description": "Sets the state of the drone",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "state": {
                                "type": "string",
                                "description": "State of the drone (IDLE, TAKEOFF, TRACKING, LANDING, ENU_FRAME_CONTROL, BODY_FRAME_CONTROL). ENU_FRAME_CONTROL and BODY_FRAME_CONTROL need to be selected to control the drone position. LANDING should be selected when user wants to land the drone. You already start in the ENU_CONTROL state"
                            }
                        },
                        "required": ["state"]
                    }
                }
            }
        ]