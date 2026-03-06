"""
scenario_library.py
-------------------
Library of high-level robot scenarios.
"""
import re
from typing import List
from .nl_planner import WorldModel, fmt_goto

# =========================================================
# SCENARIO DEFINITIONS
# =========================================================

def go_to_target(model: WorldModel, target: str) -> List[str]:
    """
    Scenario: go to <target>.
    Target can be a place (e.g., 'kitchen') or an object (e.g., 'apple').
    """
    # 1. Try to resolve as a Place (Saved Location) first
    pose = model.resolve_place(target)
    
    # 2. If not a place, try to resolve as an Object
    if not pose:
        pose = model.resolve_object(target)

    # 3. If neither, we can't move
    if not pose:
        raise ValueError(f"Unknown target location or object: '{target}'")

    return [fmt_goto(pose)]

def give_item(model: WorldModel, item: str, dest: str = "me") -> List[str]:
    """Generic scenario: give me <item>."""
    obj_pose = model.resolve_object(item)
    dest_pose = model.resolve_place(dest)
    if not obj_pose or not dest_pose:
        raise ValueError(f"Unknown object or destination ({item}->{dest})")
    return [
        fmt_goto(obj_pose),
        f"grasp:{item}",
        fmt_goto(dest_pose),
        f"place:{dest}",
    ]

def park_robot(model: WorldModel) -> List[str]:
    """Return robot to home position."""
    home_pose = model.resolve_place("home")
    return [fmt_goto(home_pose)]

def clean_table(model: WorldModel) -> List[str]:
    """Pick trash on table and move it to bin."""
    table_pose = model.resolve_place("table")
    bin_pose = model.resolve_place("trash_bin") or model.resolve_place("home")
    return [
        fmt_goto(table_pose),
        f"grasp:trash",
        fmt_goto(bin_pose),
        f"place:trash_bin",
    ]

def fetch_drink(model: WorldModel) -> List[str]:
    """Bring drink from fridge to person."""
    fridge_pose = model.resolve_place("fridge")
    me_pose = model.resolve_place("me")
    return [
        fmt_goto(fridge_pose),
        "grasp:drink",
        fmt_goto(me_pose),
        "place:me",
    ]
    
def go_from(model: WorldModel, arg: str) -> List[str]:
    s = arg.strip().lower()
    # remove prefix "from"
    s = re.sub(r'^\s*from\s+', '', s)
    # split by common delimiters
    if ' to ' in s:
        a, b = [p.strip() for p in s.split(' to ', 1)]
    elif '->' in s:
        a, b = [p.strip() for p in s.split('->', 1)]
    elif ' into ' in s:
        a, b = [p.strip() for p in s.split(' into ', 1)]
    else:
        parts = s.split()
        if len(parts) >= 2:
            a, b = parts[0], parts[1]
        else:
            raise ValueError(f"Cannot parse 'go from' arguments: '{arg}'")
    # remove (a/an/the)
    a = re.sub(r'^(the|a|an)\s+', '', a)
    b = re.sub(r'^(the|a|an)\s+', '', b)
    a_pose = model.resolve_object(a)
    b_pose = model.resolve_place(b)
    return [fmt_goto(a_pose), f"grasp:{a}", fmt_goto(b_pose), f"place:{b}"]

def test_arm(model: WorldModel) -> List[str]:
    """Scenario to test grasp and place actions."""
    return [
        "grasp:sofa",
        "place:table",
    ]

# =========================================================
# SCENARIO REGISTRY
# =========================================================
SCENARIO_REGISTRY = {
    "give me": give_item,
    "bring me": give_item,
    "go home": park_robot,
    "park": park_robot,
    "clean table": clean_table,
    "fetch drink": fetch_drink,
    # === NEW COMMAND ===
    "go to": go_to_target,
    "bring": go_from,  # Example of a new scenario that could be added
    "test arm": test_arm, # scenario to test grasp and place
}