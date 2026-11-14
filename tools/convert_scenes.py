#!/usr/bin/env python3
"""Convert the Godot aircraft scenes of godot_aerodynamic_physics into JSON for the Bevy demo.

Usage:
    python3 tools/convert_scenes.py <godot_repo_demo_dir> <output_assets_dir>

Writes:
    <output_assets_dir>/aircraft/<scene>.json   one tree per scene (format: aero_physics::schema)
    <output_assets_dir>/models/<name>.glb       the glTF models the scenes instance

The Godot scenes are text (.tscn). Inherited glTF scenes are merged with their overrides, nested
scenes are inlined, and resources are inlined as JSON objects. Godot `.res` buoyancy meshes are
binary, so their geometry is read from the glTF node that carries the same mesh.
"""

import json
import math
import os
import re
import shutil
import struct
import sys

# ---------------------------------------------------------------------------------------------
# Scenes to convert, relative to the Godot demo directory, and the name they get in the output.
# ---------------------------------------------------------------------------------------------
SCENES = {
    "demo_plane": "aircraft_examples/demo_plane.tscn",
    "cessna172": "aircraft_examples/cessna172/cessna172.tscn",
    "f14": "aircraft_examples/f14/f14.tscn",
    "f14_mouse_control": "aircraft_examples/f14/f14_mouse_control.tscn",
    "aim9": "aircraft_examples/missile/aim-9.tscn",
    "quadcopter": "aircraft_examples/quadcopter.tscn",
    "su57": "aircraft_examples/su57/su57.tscn",
    "berkut": "aircraft_examples/berkut/berkut.tscn",
    "b2": "aircraft_examples/b2/b2.tscn",
    "v22": "aircraft_examples/tilt_rotor/v22.tscn",
    "blackhawk": "aircraft_examples/blackhawk/blackhawk.tscn",
    "catalina": "aircraft_examples/catalina/catalina.tscn",
    "boat": "aircraft_examples/boat/boat.tscn",
    "sailboat": "aircraft_examples/sailboat/sailboat.tscn",
    "floaty_ball": "aircraft_examples/floaty_ball/floaty_ball.tscn",
    "drag_ball": "aircraft_examples/drag_ball/ball.tscn",
}

# Godot `.res` buoyancy meshes -> (glTF file, node names whose mesh geometry is used).
# The .res files are binary ArrayMesh resources; the glTF nodes hold the same geometry.
BUOYANCY_SOURCES = {
    "boat/hull.res": [("boat/boat.glb", ["Hull"])],
    "sailboat/hull.res": [("sailboat/sailboat.glb", ["Hull"])],
    "catalina/fueslage.res": [("catalina/catalina.glb", ["Fueslage"])],
    "catalina/float.res": [("catalina/catalina.glb", ["FloatL"])],
    "cessna172/fueslage_aero_mesh.res": [("cessna172/cessna172.glb", ["Fueslage_aero"])],
    "f14/fueslage_aero_mesh.res": [("f14/f14.glb", ["Fueslage_aero"])],
    "blackhawk/fueslage_aero_mesh.res": [("blackhawk/blackhawk.glb", ["Fueslage_aero"])],
    "missile/fueslage_aero_mesh.res": [("missile/aim-9.glb", ["fueslage_aero"])],
    "tilt_rotor/fueslage_aero_mesh.res": [("tilt_rotor/v22.glb", ["fueslage_aero"])],
    "floaty_ball/Buoyancy mesh ball.res": [("floaty_ball/Buoyancy mesh ball.glb", ["Cube"])],
}

# Script file name -> class name used by the Rust loader.
SCRIPT_CLASSES = {
    "aero_body_3d.gd": "AeroBody3D",
    "aero_control_component.gd": "AeroControlComponent",
    "aero_influencer_3d.gd": "AeroInfluencer3D",
    "aero_surface_3d.gd": "AeroSurface3D",
    "manual_aero_surface_3d.gd": "ManualAeroSurface3D",
    "aero_drag_influencer.gd": "AeroDragInfluencer3D",
    "aero_thruster_3d.gd": "AeroThruster3D",
    "aero_jet_thruster_3d.gd": "AeroJetThruster3D",
    "aero_mover_3d.gd": "AeroMover3D",
    "aero_propeller_3d.gd": "AeroPropeller3D",
    "aero_cyclic_propeller_3d.gd": "AeroCyclicPropeller3D",
    "aero_buoyancy_sphere_3d.gd": "AeroBuoyancySphere3D",
    "aero_buoyancy_mesh_3d.gd": "AeroBuoyancyMesh3D",
    "aero_water_atmosphere_3d.gd": "AeroWaterAtmosphere3D",
    "aero_atmosphere_3d.gd": "AeroAtmosphere3D",
    "aero_rotor_hinge_3d.gd": "AeroRotorHinge3D",
    "camera_rotator.gd": "CameraRotator",
    "wind_indicator.gd": "WindIndicator",
    "flightpath_indicator.gd": "FlightPathIndicator",
    "ball_launcher.gd": "BallLauncher",
    "mouse_control.gd": "MouseControl",
    "pid.gd": "PID",
    "flight_assist.gd": "FlightAssist",
    "easy_lift_curve.gd": "LiftCurve",
    "aero_surface_config.gd": "SurfaceConfig",
    "manual_aero_surface_config.gd": "ManualSurfaceConfig",
    "aero_control_config.gd": "ControlConfig",
    "aero_control_config_axis.gd": "ControlAxis",
    "aero_influencer_control_config.gd": "ActuationConfig",
    "aero_influencer_control_axis_config.gd": "ActuationAxis",
}


# ---------------------------------------------------------------------------------------------
# Godot text-format parser
# ---------------------------------------------------------------------------------------------
class Ref:
    """ExtResource(...) or SubResource(...) reference."""

    def __init__(self, kind, ident):
        self.kind = kind
        self.ident = ident

    def __repr__(self):
        return f"{self.kind}({self.ident})"


class Parser:
    NUMBER = re.compile(r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|-?inf|-?nan")
    IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
    KEY = re.compile(r"[A-Za-z0-9_/]+")

    def __init__(self, text):
        self.s = text
        self.i = 0

    def error(self, msg):
        line = self.s.count("\n", 0, self.i) + 1
        raise ValueError(f"tscn parse error at line {line}: {msg}")

    def ws(self):
        while self.i < len(self.s) and self.s[self.i] in " \t\r\n":
            self.i += 1

    def eof(self):
        return self.i >= len(self.s)

    def peek(self):
        return self.s[self.i] if self.i < len(self.s) else ""

    def expect(self, ch):
        self.ws()
        if self.peek() != ch:
            self.error(f"expected {ch!r}, found {self.peek()!r}")
        self.i += 1

    def string(self):
        assert self.s[self.i] == '"'
        self.i += 1
        out = []
        while True:
            if self.eof():
                self.error("unterminated string")
            c = self.s[self.i]
            if c == '"':
                self.i += 1
                return "".join(out)
            if c == "\\":
                n = self.s[self.i + 1]
                if n == "u":
                    out.append(chr(int(self.s[self.i + 2 : self.i + 6], 16)))
                    self.i += 6
                    continue
                out.append({"n": "\n", "t": "\t", "r": "\r"}.get(n, n))
                self.i += 2
                continue
            out.append(c)
            self.i += 1

    def value(self):
        self.ws()
        c = self.peek()
        if c == '"':
            return self.string()
        if c == "&":
            self.i += 1
            return self.string()
        if c == "[":
            return self.array("]")
        if c == "{":
            return self.dictionary()
        m = self.NUMBER.match(self.s, self.i)
        if m and (c == "-" or c.isdigit() or c == "."):
            self.i = m.end()
            text = m.group(0)
            if text in ("inf", "-inf", "nan", "-nan"):
                return float(text)
            return float(text) if any(x in text for x in ".eE") else int(text)
        m = self.IDENT.match(self.s, self.i)
        if not m:
            self.error(f"unexpected character {c!r}")
        name = m.group(0)
        self.i = m.end()
        if name in ("true", "false"):
            return name == "true"
        if name == "null":
            return None
        if name in ("Array", "Dictionary") and self.peek() == "[":
            # Typed array: Array[Type]([...])
            depth = 0
            while True:
                ch = self.s[self.i]
                self.i += 1
                if ch == "[":
                    depth += 1
                elif ch == "]":
                    depth -= 1
                    if depth == 0:
                        break
            self.expect("(")
            inner = self.value()
            self.expect(")")
            return inner
        if self.peek() == "(":
            self.i += 1
            args = []
            self.ws()
            if self.peek() != ")":
                while True:
                    args.append(self.value())
                    self.ws()
                    if self.peek() == ",":
                        self.i += 1
                        continue
                    break
            self.expect(")")
            return self.construct(name, args)
        self.error(f"unknown identifier {name!r}")

    def construct(self, name, args):
        if name == "ExtResource":
            return Ref("ext", args[0])
        if name == "SubResource":
            return Ref("sub", args[0])
        if name == "NodePath":
            return args[0]
        if name in ("Dictionary",):
            return args[0] if args else {}
        flat = []
        for a in args:
            if isinstance(a, list):
                flat.extend(a)
            else:
                flat.append(a)
        if name.startswith("Packed") or name in ("Vector2", "Vector2i", "Vector3", "Vector3i", "Vector4", "Color", "Transform3D", "Basis", "Quaternion", "AABB", "Plane"):
            return flat
        return {"__ctor__": name, "args": args}

    def array(self, closer):
        self.expect("[")
        out = []
        self.ws()
        if self.peek() == closer:
            self.i += 1
            return out
        while True:
            out.append(self.value())
            self.ws()
            if self.peek() == ",":
                self.i += 1
                continue
            self.expect(closer)
            return out

    def dictionary(self):
        self.expect("{")
        out = {}
        self.ws()
        if self.peek() == "}":
            self.i += 1
            return out
        while True:
            key = self.value()
            self.expect(":")
            out[key] = self.value()
            self.ws()
            if self.peek() == ",":
                self.i += 1
                continue
            self.expect("}")
            return out

    def header(self):
        self.expect("[")
        m = self.IDENT.match(self.s, self.i)
        tag = m.group(0)
        self.i = m.end()
        attrs = {}
        while True:
            self.ws()
            if self.peek() == "]":
                self.i += 1
                return tag, attrs
            m = self.KEY.match(self.s, self.i)
            if not m:
                self.error("bad header attribute")
            key = m.group(0)
            self.i = m.end()
            self.expect("=")
            attrs[key] = self.value()

    def sections(self):
        out = []
        while True:
            self.ws()
            if self.eof():
                return out
            tag, attrs = self.header()
            props = {}
            while True:
                self.ws()
                if self.eof() or self.peek() == "[":
                    break
                m = self.KEY.match(self.s, self.i)
                if not m:
                    self.error(f"bad property after [{tag}]")
                key = m.group(0)
                self.i = m.end()
                self.expect("=")
                props[key] = self.value()
            out.append((tag, attrs, props))


class Scene:
    """A parsed .tscn: resources and nodes."""

    def __init__(self, path):
        self.path = path
        with open(path, encoding="utf-8") as f:
            text = f.read()
        self.ext = {}
        self.sub = {}
        self.nodes = []
        for tag, attrs, props in Parser(text).sections():
            if tag == "ext_resource":
                self.ext[attrs["id"]] = (attrs.get("type", ""), attrs["path"])
            elif tag == "sub_resource":
                self.sub[attrs["id"]] = (attrs.get("type", ""), props)
            elif tag == "node":
                self.nodes.append((attrs, props))


# ---------------------------------------------------------------------------------------------
# Value conversion
# ---------------------------------------------------------------------------------------------
class Converter:
    def __init__(self, demo_dir):
        self.demo_dir = demo_dir
        self.scene_cache = {}
        self.glb_cache = {}
        self.models_used = {}

    def res_path(self, res):
        """res://addons/godot_aerodynamic_physics/demo/x -> absolute path under the repo."""
        marker = "godot_aerodynamic_physics/demo/"
        rel = res.split(marker, 1)[1] if marker in res else res.replace("res://", "")
        return os.path.join(self.demo_dir, rel)

    def script_class(self, scene, ext_id):
        typ, path = scene.ext[ext_id]
        base = os.path.basename(path)
        return SCRIPT_CLASSES.get(base, base.split(".")[0])

    def convert_value(self, scene, v):
        if isinstance(v, Ref):
            if v.kind == "ext":
                typ, path = scene.ext[v.ident]
                return {"__ext__": path, "type": typ}
            return self.resource(scene, v.ident)
        if isinstance(v, list):
            return [self.convert_value(scene, x) for x in v]
        if isinstance(v, dict):
            if "__ctor__" in v:
                return v
            return {k: self.convert_value(scene, x) for k, x in v.items()}
        return v

    def resource(self, scene, sub_id):
        typ, props = scene.sub[sub_id]
        if typ == "GDScript":
            return {"__gdscript__": props.get("script/source", "")}
        script = props.get("script")
        cls = typ
        if isinstance(script, Ref):
            cls = self.script_class(scene, script.ident)
        if typ == "Curve" or cls == "LiftCurve":
            return self.curve(scene, props)
        out = {"class": cls}
        for key, value in props.items():
            if key == "script":
                continue
            out[key] = self.convert_value(scene, value)
        if typ == "ConvexPolygonShape3D" and "points" in out:
            flat = out["points"]
            out["points"] = [flat[i : i + 3] for i in range(0, len(flat), 3)]
        return out

    def curve(self, scene, props):
        limits = props.get("_limits", [0.0, 1.0, 0.0, 1.0])
        data = props.get("_data", [])
        points = []
        for i in range(0, len(data) - 4, 5):
            pos, lt, rt, lm, rm = data[i : i + 5]
            points.append({
                "x": pos["args"][0] if isinstance(pos, dict) else pos[0],
                "y": pos["args"][1] if isinstance(pos, dict) else pos[1],
                "left_tangent": lt,
                "right_tangent": rt,
                "left_mode": lm,
                "right_mode": rm,
            })
        return {
            "class": "Curve",
            "min_value": float(limits[0]),
            "max_value": float(limits[1]),
            "min_domain": float(limits[2]),
            "max_domain": float(limits[3]),
            "bake_resolution": int(props.get("bake_resolution", 100)),
            "points": points,
        }

    # -----------------------------------------------------------------------------------------
    # Scene -> node tree
    # -----------------------------------------------------------------------------------------
    def glb(self, path):
        if path not in self.glb_cache:
            self.glb_cache[path] = read_glb(path)
        return self.glb_cache[path]

    def glb_tree(self, path):
        """Node tree of a glTF scene, as (node dict, {path: node dict})."""
        gltf, binary = self.glb(path)
        base = os.path.basename(path)
        self.models_used[base] = path
        paths = {}

        def build(index, parent_path, is_root=False):
            n = gltf["nodes"][index]
            name = n.get("name", f"node{index}")
            if is_root:
                node_path = ""
            else:
                node_path = name if not parent_path else f"{parent_path}/{name}"
            node = {
                "name": name,
                "class": "Node3D",
                "transform": node_transform(n),
                "visible": True,
                "props": {},
                "children": [],
                "follow": [],
            }
            if "mesh" in n:
                node["class"] = "MeshInstance3D"
                node["mesh"] = {
                    "file": "models/" + base,
                    "primitives": [
                        {"mesh": n["mesh"], "primitive": p, "material": prim.get("material")}
                        for p, prim in enumerate(gltf["meshes"][n["mesh"]]["primitives"])
                    ],
                    "buoyancy_vertices": [],
                    "buoyancy_indices": [],
                }
            paths[node_path] = node
            for child in n.get("children", []):
                node["children"].append(build(child, node_path))
            return node

        scene = gltf["scenes"][gltf.get("scene", 0)]
        if len(scene["nodes"]) == 1:
            root = build(scene["nodes"][0], "", is_root=True)
            paths[""] = root
            return root, paths
        roots = [build(i, "") for i in scene["nodes"]]
        top = {"name": "root", "class": "Node3D", "transform": None, "visible": True, "props": {}, "children": roots, "follow": []}
        paths[""] = top
        return top, paths

    def build_scene(self, path):
        """Converts a .tscn into a node tree dict. Results are cached per path."""
        if path in self.scene_cache:
            return json.loads(json.dumps(self.scene_cache[path]))
        scene = Scene(path)
        root_attrs, root_props = scene.nodes[0]
        base_root, base_paths = None, {}
        if "instance" in root_attrs:
            typ, inst_path = scene.ext[root_attrs["instance"].ident]
            if inst_path.endswith(".glb"):
                base_root, base_paths = self.glb_tree(self.res_path(inst_path))
            else:
                base_root = self.build_scene(self.res_path(inst_path))
                base_paths = index_paths(base_root)
            # The root's own entry is replaced below by the scene's root.

        root = base_root or {"name": root_attrs["name"], "class": "Node3D", "transform": None, "visible": True, "props": {}, "children": [], "follow": []}
        root["name"] = root_attrs["name"]
        self.apply_overrides(scene, root, root_props, root_attrs["name"])
        # A scene root with body properties and no script (inherited from the glTF) is the body.
        if root["class"] in ("Node3D", "MeshInstance3D") and "mass" in root["props"]:
            root["class"] = "AeroBody3D"
        paths = dict(base_paths)
        paths[""] = root

        for attrs, props in scene.nodes[1:]:
            parent = attrs.get("parent", ".")
            parent_path = "" if parent == "." else parent
            name = attrs["name"]
            node_path = name if not parent_path else f"{parent_path}/{name}"
            if "instance" in attrs:
                inst_typ, inst_path = scene.ext[attrs["instance"].ident]
                sub = self.build_scene(self.res_path(inst_path))
                sub["name"] = name
                self.apply_overrides(scene, sub, props, name)
                self.attach(paths, parent_path, node_path, sub)
                continue
            if node_path in paths:
                self.apply_overrides(scene, paths[node_path], props, name, attrs)
                continue
            node = {
                "name": name,
                "class": attrs.get("type", "Node3D"),
                "transform": None,
                "visible": True,
                "props": {},
                "children": [],
                "follow": [],
            }
            self.apply_overrides(scene, node, props, name, attrs)
            self.attach(paths, parent_path, node_path, node)

        for node_path, node in list(paths.items()):
            self.finish_node(scene, node, node_path)
        self.scene_cache[path] = root
        return json.loads(json.dumps(root))

    def attach(self, paths, parent_path, node_path, node):
        parent = paths.get(parent_path, paths[""])
        parent["children"].append(node)
        paths.update(index_paths(node, node_path))

    def apply_overrides(self, scene, node, props, name, attrs=None):
        if attrs and "type" in attrs and node["class"] in ("Node3D", "Node") and attrs["type"] != node["class"]:
            node["class"] = attrs["type"]
        for key, value in props.items():
            if key == "transform":
                node["transform"] = [float(x) for x in value["args"]] if isinstance(value, dict) else [float(x) for x in value]
            elif key == "visible":
                node["visible"] = bool(value)
            elif key == "script":
                if isinstance(value, Ref) and value.kind == "ext":
                    node["class"] = self.script_class(scene, value.ident)
                elif isinstance(value, Ref) and value.kind == "sub":
                    typ, sprops = scene.sub[value.ident]
                    source = sprops.get("script/source", "")
                    node["class"] = detect_gdscript_class(source, node["class"])
                    node["gdscript"] = source
            elif key in ("metadata/_custom_type_script", "unique_id"):
                continue
            else:
                node["props"][key] = self.convert_value(scene, value)
        if "material" in node["props"] and isinstance(node["props"]["material"], dict) and "albedo_color" in node["props"]["material"]:
            node["props"]["albedo_color"] = node["props"]["material"]["albedo_color"]
        node["props"].pop("material", None)
        if "buoyancy_mesh" in node["props"]:
            self.attach_buoyancy(node)
        if "rotation_order" in node["props"]:
            pass

    def attach_buoyancy(self, node):
        ref = node["props"].pop("buoyancy_mesh")
        res = ref["__ext__"] if isinstance(ref, dict) and "__ext__" in ref else None
        if not res:
            return
        key = res.split("aircraft_examples/", 1)[1]
        sources = BUOYANCY_SOURCES.get(key)
        if not sources:
            raise SystemExit(f"no glTF source for buoyancy mesh {key}")
        vertices, indices = [], []
        for glb_rel, names in sources:
            gltf, binary = self.glb(os.path.join(self.demo_dir, "aircraft_examples", glb_rel))
            for mesh_name, mesh_index in node_mesh_index(gltf, names):
                prim = gltf["meshes"][mesh_index]["primitives"][0]
                pos = read_accessor(gltf, binary, prim["attributes"]["POSITION"])
                idx = read_accessor(gltf, binary, prim["indices"]) if "indices" in prim else list(range(len(pos)))
                offset = len(vertices)
                vertices.extend(pos)
                indices.extend(i + offset for i in idx)
        node["mesh"] = {"file": "", "primitives": [], "buoyancy_vertices": vertices, "buoyancy_indices": indices}

    def finish_node(self, scene, node, node_path):
        """Turns GDScript sub-resources into visual links and small behaviours."""
        source = node.pop("gdscript", None)
        if not source:
            return
        follows = parse_follows(source, node_path)
        node["follow"].extend(follows)
        if node["class"] == "AirspeedControl" or "wing_sweep" in source:
            node["class"] = "AirspeedControl"
            node["props"].setdefault("axis", "wing_sweep")
            for name, value in re.findall(r"@export var (\w+)\s*:\s*float\s*=\s*([-0-9.]+)", source):
                node["props"][name] = float(value)


def node_mesh_index(gltf, names):
    """(node name, mesh index) for each named glTF node. `None` selects every mesh node."""
    out = []
    for n in gltf["nodes"]:
        if "mesh" not in n:
            continue
        if names is None or n.get("name") in names:
            out.append((n.get("name"), n["mesh"]))
    if names is not None:
        missing = set(names) - {x for x, _ in out}
        if missing:
            raise SystemExit(f"glTF has no mesh node named {sorted(missing)}")
    return out


def detect_gdscript_class(source, fallback):
    if "set_control_input" in source and "wing_sweep" in source:
        return "AirspeedControl"
    if re.search(r"extends Label3D", source):
        return "FlightInfoLabel"
    return fallback


FOLLOW_LINE = re.compile(
    r"^\s*(\$(?:\"[^\"]*\"|[A-Za-z0-9_/.]+))\.rotation\.([xyz])\s*=\s*(-?)\s*(\$(?:\"[^\"]*\"|[A-Za-z0-9_/.]+))\.rotation\.([xyz])"
    r"(?:\s*\+\s*deg_to_rad\(\s*([-0-9.]+)\s*\))?\s*$",
    re.MULTILINE,
)


def resolve_godot_path(base, token):
    """Resolves `$Name/Child` or `$"../Sibling"` relative to the node at `base` (root-relative)."""
    token = token[1:].strip('"')
    parts = [p for p in base.split("/") if p] if base else []
    for seg in token.split("/"):
        if seg in ("", "."):
            continue
        if seg == "..":
            if parts:
                parts.pop()
            continue
        parts.append(seg)
    return "/".join(parts)


def parse_follows(source, node_path):
    follows = []
    for m in FOLLOW_LINE.finditer(source):
        target_tok, target_axis, minus, source_tok, source_axis, offset = m.groups()
        # The script runs on the node itself; `..` refers to its parent.
        target = resolve_godot_path(node_path, target_tok)
        src = resolve_godot_path(node_path, source_tok)
        follows.append({
            "target": target,
            "source": src,
            "axis": target_axis,
            "source_axis": source_axis,
            "sign": -1.0 if minus == "-" else 1.0,
            "offset": math.radians(float(offset)) if offset else 0.0,
        })
    return follows


def index_paths(root, prefix=""):
    """Root-relative paths of every node under `root`. `prefix` is the path of `root` itself."""
    out = {prefix: root}

    def walk(node, path):
        for child in node["children"]:
            child_path = child["name"] if not path else f"{path}/{child['name']}"
            out[child_path] = child
            walk(child, child_path)

    walk(root, prefix)
    return out


# ---------------------------------------------------------------------------------------------
# glTF reading (JSON chunk + binary buffer), enough for node transforms and mesh accessors
# ---------------------------------------------------------------------------------------------
def read_glb(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67:
        raise SystemExit(f"{path} is not a glb file")
    off = 12
    gltf, binary = None, None
    while off < len(data):
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8 : off + 8 + clen]
        if ctype == 0x4E4F534A:
            gltf = json.loads(chunk.decode("utf-8"))
        elif ctype == 0x004E4942:
            binary = chunk
        off += 8 + clen
    return gltf, binary


def node_transform(n):
    """Godot-style 12-float transform (basis columns, then origin) of a glTF node."""
    if "matrix" in n:
        m = n["matrix"]
    else:
        t = n.get("translation", [0.0, 0.0, 0.0])
        q = n.get("rotation", [0.0, 0.0, 0.0, 1.0])
        s = n.get("scale", [1.0, 1.0, 1.0])
        x, y, z, w = q
        rot = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        cols = [[rot[r][c] * s[c] for r in range(3)] for c in range(3)]
        return cols[0] + cols[1] + cols[2] + list(t)
    # glTF matrices are column-major: m[0..3] is the first column.
    return list(m[0:3]) + list(m[4:7]) + list(m[8:11]) + list(m[12:15])


def read_accessor(gltf, binary, index):
    acc = gltf["accessors"][index]
    view = gltf["bufferViews"][acc["bufferView"]]
    base = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    count = acc["count"]
    comp = acc["componentType"]
    kind = acc["type"]
    fmt = {5126: "f", 5125: "I", 5123: "H", 5121: "B"}[comp]
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[kind]
    size = struct.calcsize("<" + fmt)
    stride = view.get("byteStride", size * width)
    out = []
    for i in range(count):
        values = struct.unpack_from("<" + fmt * width, binary, base + i * stride)
        out.append(list(values) if width > 1 else values[0])
    if kind == "VEC3":
        return [tuple(v) for v in out]
    return out


# ---------------------------------------------------------------------------------------------
def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    demo_dir, out_dir = argv[1], argv[2]
    converter = Converter(demo_dir)
    aircraft_dir = os.path.join(out_dir, "aircraft")
    models_dir = os.path.join(out_dir, "models")
    os.makedirs(aircraft_dir, exist_ok=True)
    os.makedirs(models_dir, exist_ok=True)

    for name, rel in SCENES.items():
        path = os.path.join(demo_dir, rel)
        root = converter.build_scene(path)
        scene = {"name": name, "root": clean(root)}
        with open(os.path.join(aircraft_dir, name + ".json"), "w", encoding="utf-8") as f:
            json.dump(scene, f, separators=(",", ":"))
        print(f"{name}: {count_nodes(root)} nodes, root class {root['class']}")

    for base, src in converter.models_used.items():
        shutil.copyfile(src, os.path.join(models_dir, base))
    print(f"copied {len(converter.models_used)} models")
    return 0


# 2D HUD helpers inside the Godot scenes. The Bevy demo draws its own HUD, so they are dropped.
HUD_2D_CLASSES = {"SubViewport", "Camera2D", "Line2D", "Control", "Label", "Button", "OptionButton"}


def clean(node):
    """Drops empty fields and the 2D HUD helpers, so the JSON stays small."""
    node = dict(node)
    node["children"] = [c for c in node.get("children", []) if c["class"] not in HUD_2D_CLASSES]
    out = {"name": node["name"], "class": node["class"]}
    if node.get("transform") is not None:
        out["transform"] = node["transform"]
    if not node.get("visible", True):
        out["visible"] = False
    if node.get("props"):
        out["props"] = node["props"]
    if node.get("mesh"):
        out["mesh"] = node["mesh"]
    if node.get("follow"):
        out["follow"] = node["follow"]
    if node.get("children"):
        out["children"] = [clean(c) for c in node["children"]]
    return out


def count_nodes(node):
    return 1 + sum(count_nodes(c) for c in node["children"])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
