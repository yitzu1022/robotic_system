"""Prompt templates for the RoboAgent-style decision layer.

These strings intentionally mirror the capability-calling style used by the
RoboAgent reference implementation while constraining final actions to the
primitive interface supported by this ROS system.
"""

SCHEDULER_PROMPT = """Suppose you are a helpful robotic agent in an indoor environment. You have the following abilities and you can invoke them by function calling:
1. exploration_guidance(object_information): given the name or description of an object, output a direction for exploration
2. exploration_planner(exploration_direction): explore according to the direction
3. object_grounding(object_information): given the name or description of an object, find it in the egocentric view of the robot
4. scene_description(object_information): describe the egocentric observation of the robot
5. manipulation_planner(subtask): given a subtask instruction, output primitive robot actions
6. experience_summarization(subtask): summarize previous execution experience
7. question_answering(question): answer a question based on the egocentric view

You need to complete the task by sequentially generating ability calls and final primitive actions.
The only primitive actions allowed by the robot executor are:
- goto:<target>
- grasp:<object>
- place:<object>:<destination>
- handover:<object>

Return JSON with exactly these top-level fields:
{{
  "think": "...",
  "capability_calls": [{{"name": "capability_name", "args": "..."}}],
  "primitive_actions": ["goto:target"],
  "stop": false,
  "stop_reason": ""
}}

Task: {}
Visual observation: {}
Capability history:
{}
Action history:
{}
Execution feedback:
{}
"""

EXPLORATION_GUIDANCE_PROMPT = """Suppose you are a helpful robotic agent in an indoor environment. Your task is to find '{}', based on common house layouts and object placements. Currently, the robot may use semantic-map object names and the current egocentric observation. Output one exploration direction in the form of <relation> <object>, where <relation> is chosen from [target, in, on, near]."""

EXPLORATION_PLANNER_PROMPT = """Suppose you are a helpful robotic agent in an indoor environment. You are able to perform goto:<target>. Now, your task is to '{}'. Output a list of primitive actions."""

OBJECT_GROUNDING_PROMPT = """<image>
Locate {} in the current egocentric image. If you can find it, output a bounding box JSON list; if you cannot find it, output no."""

SCENE_DESCRIPTION_PROMPT = """<image>
This is an egocentric image observed by a robotic household agent. Please describe the scene around {}."""

MANIPULATION_PLANNER_PROMPT = """Suppose you are a helpful robotic agent in an indoor environment. You can use only these primitive actions:
1. goto:<target>
2. grasp:<object>
3. place:<object>:<destination>
4. handover:<object>
Your task is to '{}'. Output a list of primitive actions."""

EXPERIENCE_SUMMARIZATION_PROMPT = """<image>
Suppose you are a helpful robotic agent in an indoor environment. Your task is to '{}'. Here is a list of actions and execution feedback:
{}
Please summarize progress and analyze failures."""

QUESTION_ANSWERING_PROMPT = """<image>
Answer the following question based on the current egocentric observation: {}"""
