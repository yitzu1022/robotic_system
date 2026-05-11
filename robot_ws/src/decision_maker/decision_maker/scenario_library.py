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
        f"handover:{item}",
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
        "place:trash:trash_bin",
    ]

def fetch_drink(model: WorldModel) -> List[str]:
    """Bring drink from fridge to person."""
    fridge_pose = model.resolve_place("fridge")
    me_pose = model.resolve_place("me")
    return [
        fmt_goto(fridge_pose),
        "grasp:drink",
        fmt_goto(me_pose),
        "handover:drink",
    ]
    
_PREPOSITIONS = r'(?:on|in|at|near|by|inside|from|above|below|under|beside|next\s+to)'

def _strip_article(text: str) -> str:
    return re.sub(r'^(the|a|an)\s+', '', text.strip())

def _parse_transfer_args(arg: str):
    """
    Parse "<object> [on/in/at/near/from... <location>] to <destination>".

    Examples:
      "the bottle on cabinet to table"
        => obj=bottle, nav_target=cabinet, dest=table
      "bottle to table"
        => obj=bottle, nav_target=bottle, dest=table
    """
    s = arg.strip().lower()
    # remove leading "from"
    s = re.sub(r'^\s*from\s+', '', s)

    # Split source phrase from destination on "to", "into"
    if ' to ' in s:
        src, dest = [p.strip() for p in s.split(' to ', 1)]
    elif ' into ' in s:
        src, dest = [p.strip() for p in s.split(' into ', 1)]
    else:
        parts = s.split()
        if len(parts) >= 2:
            src, dest = parts[0], parts[1]
        else:
            raise ValueError(f"Cannot parse transfer arguments: '{arg}'")

    # Strip articles from both sides
    src = _strip_article(src)
    dest = _strip_article(dest)
    # Strip any trailing preposition phrase from destination (e.g. "table on left")
    dest = re.split(r'\s+' + _PREPOSITIONS + r'\s+', dest)[0].strip()

    # Parse source: "<object> <preposition> <location>"  vs just "<object>"
    prep_match = re.search(r'\s+' + _PREPOSITIONS + r'\s+', src)
    if prep_match:
        obj = src[:prep_match.start()].strip()
        nav_target = src[prep_match.end():].strip()   # navigate to the location, not the object
    else:
        obj = src.strip()
        nav_target = obj                              # navigate to the object itself

    obj = _strip_article(obj)
    nav_target = _strip_article(nav_target)

    if not obj or not nav_target or not dest:
        raise ValueError(f"Cannot parse transfer arguments: '{arg}'")

    return obj, nav_target, dest

def _transfer_steps(obj: str, nav_target: str, dest: str, final_action: str) -> List[str]:
    if final_action == "place":
        final_step = f"place:{obj}:{dest}"
    elif final_action == "handover":
        final_step = f"handover:{obj}"
    else:
        raise ValueError(f"Unsupported final action: '{final_action}'")
    return [f"goto:{nav_target}", f"grasp:{obj}", f"goto:{dest}", final_step]

def go_from(model: WorldModel, arg: str) -> List[str]:
    """
    Parse "bring <object> [on/in/at/near... <location>] to <destination>".

    "bring" keeps the existing behavior and ends with a handover.
    """
    obj, nav_target, dest = _parse_transfer_args(arg)
    return _transfer_steps(obj, nav_target, dest, "handover")

def place_from(model: WorldModel, arg: str) -> List[str]:
    """Parse "place <object> [on/in/from... <location>] to <destination>"."""
    obj, nav_target, dest = _parse_transfer_args(arg)
    return _transfer_steps(obj, nav_target, dest, "place")

def handover_from(model: WorldModel, arg: str) -> List[str]:
    """Parse "handover <object> [on/in/from... <location>] to <destination>"."""
    obj, nav_target, dest = _parse_transfer_args(arg)
    return _transfer_steps(obj, nav_target, dest, "handover")

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
    "place": place_from,
    "handover": handover_from,
    "test arm": test_arm, # scenario to test grasp and place
}
