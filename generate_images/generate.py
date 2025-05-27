import blenderproc as bproc
import numpy as np
import os
import bpy
import random
import math
import time
import sys
import argparse
from datetime import datetime
from datetime import timedelta
import mathutils

bproc.init()
# Load your scene while preserving settings
scene = bproc.loader.load_blend("blender_files/main.blend")

bpy.context.scene.cycles.samples = 2048
bpy.context.scene.cycles.use_light_tree = True
bpy.context.scene.cycles.max_bounces = 12        # Maximum total bounces
bpy.context.scene.cycles.diffuse_bounces = 4      # Maximum diffuse bounces
bpy.context.scene.cycles.glossy_bounces = 4       # Maximum glossy bounces
bpy.context.scene.cycles.transmission_bounces = 12  # Maximum transmission bounces
bpy.context.scene.cycles.volume_bounces = 2       # Maximum volume bounces
bpy.context.scene.render.use_simplify = False
bpy.context.scene.cycles.use_spatial_splits = False
bpy.context.scene.cycles.use_persistent_data = False
bpy.context.scene.view_settings.exposure = -3

touch_z = 0.13
taskboard = None
light1 = None

for i, item in enumerate(scene):
    print(item.get_name().lower())
    if item.get_name().lower() in ["taskboard", "button_1", "button_2", "screen", "stylus"]:
        item.set_cp("category_id", i + 1)
        
        print(f"Setting category_id for {item.get_name()} to {i + 1}")
    else:
        item.set_cp("category_id", 0)
    
    

def init():
    global light1
    global taskboard

    bpy.ops.object.select_all(action='DESELECT')
    taskboard = bpy.data.objects.get("Top Assy")

    bproc.camera.set_resolution(1920, 1080)
    bproc.camera.set_intrinsics_from_blender_params(lens=1.2042, lens_unit="FOV")

    bpy.ops.object.light_add(type='AREA', radius=1)
    bpy_light1 = bpy.context.object
    light1 = bproc.types.Light(blender_obj=bpy_light1)

    set_camera()
    set_light()

    set_object(get_random_pose())

def set_camera(pose=[[0, 0, 0.8], [0, 0, 0]]):
    pose_matrix = bproc.math.build_transformation_mat(pose[0], pose[1])
    bproc.camera.add_camera_pose(pose_matrix)
    # bproc.camera.set_resolution(640, 480)
    bproc.camera.set_intrinsics_from_blender_params(lens=1.20428, lens_unit="FOV")

def set_light(pose=[[0, 0, 0.8], [3.14, 0, 0]]):
    global light1
    
    light1.set_location(pose[0])
    light1.set_rotation_euler(pose[1])
    light1.set_energy(50)

def set_object(pose=[[0, 0, 0], [0, 0, 0]]):
    # train_object.set_location(pose[0])
    # train_object.set_rotation_euler(pose[1])
    taskboard.location = pose[0]
    taskboard.rotation_euler = pose[1]
    return

def get_random_pose():
    taskboard_dimensions = taskboard.dimensions
    max_dim = max(taskboard_dimensions.x, taskboard_dimensions.y)
    half_width_x = (0.3 - max_dim) / 2  # Half of the width along the x-axis
    half_width_y = (0.3 - max_dim) / 2   # Half of the height along the y-axis
    # Generate random x, y coordinates   within the box's range
    x = random.uniform(-half_width_x, half_width_x)
    y = random.uniform(-half_width_y, half_width_y)
    

    roll = random.uniform(0, 2 * math.pi)

    return [[x, y, touch_z], [0, 0, roll]]

def get_recursive_bounding_box_center(obj):
    """
    Calculates the center of the combined bounding box of an object and its
    recursive children in world space.
    """
    from mathutils import Vector

    min_world = Vector((float('inf'), float('inf'), float('inf')))
    max_world = Vector((float('-inf'), float('-inf'), float('-inf')))
    
    has_geom = False

    objects_to_process = [obj] + list(obj.children_recursive)

    for current_obj in objects_to_process:
        if current_obj.type == 'MESH' and current_obj.bound_box:
            has_geom = True
            for i in range(8): # Iterate over 8 corners of the bound_box
                # Local corner
                local_corner = Vector(current_obj.bound_box[i])
                # World corner
                world_corner = current_obj.matrix_world @ local_corner
                
                min_world.x = min(min_world.x, world_corner.x)
                min_world.y = min(min_world.y, world_corner.y)
                min_world.z = min(min_world.z, world_corner.z)
                
                max_world.x = max(max_world.x, world_corner.x)
                max_world.y = max(max_world.y, world_corner.y)
                max_world.z = max(max_world.z, world_corner.z)
        elif not objects_to_process and current_obj.type == 'EMPTY': # Single empty with no children
             # If it's just an empty and nothing else, use its origin
             if not has_geom:
                return current_obj.matrix_world.translation


    if not has_geom: # No mesh geometry found in hierarchy
        return obj.matrix_world.translation # Fallback to the parent's origin

    return (min_world + max_world) / 2.0

def point_camera_at_object(target_object, frame=0):
    # Get the BProc camera's current world location (as set by previous add_camera_pose calls)
    # Ensure we get the pose for the correct frame if multiple camera keyframes exist.
    current_cam_pose_matrix = bproc.camera.get_camera_pose(frame=frame) 
    camera_location_world = current_cam_pose_matrix[:3, 3]

    # Calculate the center of the target object (and its children) in world space
    obj_center_world = get_recursive_bounding_box_center(target_object)

    # Calculate the rotation required for the camera to look at the object's center
    direction_vector = obj_center_world - mathutils.Vector(camera_location_world)
    
    direction_vector.normalize()
    # Blender cameras look down their local -Z axis. 'Y' is typically up.
    rotation_quaternion = direction_vector.to_track_quat('-Z', 'Y')
    rotation_euler = rotation_quaternion.to_euler()

    # Build the new transformation matrix for the camera pose (original location, new rotation)
    new_camera_pose_matrix = bproc.math.build_transformation_mat(camera_location_world, rotation_euler)

    # Update the BlenderProc camera system with this new pose for the given frame
    bproc.camera.add_camera_pose(new_camera_pose_matrix, frame=frame)
    print(f"Camera pointed at {obj_center_world} from {camera_location_world}")

def set_viewport_to_camera_view():
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            # Set view to camera
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.region_3d.view_perspective = 'CAMERA'
                    break
            break

def render_scene():
    # Render the scene
    bproc.renderer.enable_experimental_features()
    bproc.renderer.enable_normals_output()
    bproc.renderer.enable_segmentation_output(map_by=["category_id", "instance", "name"], default_values={"category_id": None})
    bproc.renderer.set_max_amount_of_samples(2048)
    bproc.renderer.set_denoiser("INTEL")
    bproc.renderer.set_output_format(file_format="JPEG", jpg_quality=200)
    data = bproc.renderer.render()

    # Write the rendering into an hdf5 file
    bproc.writer.write_coco_annotations(os.path.join(f"gen_data/taskboard"),
                                        instance_segmaps=data["instance_segmaps"],
                                        instance_attribute_maps=data["instance_attribute_maps"],
                                        colors=data["colors"],
                                        color_file_format="JPEG")

init()
point_camera_at_object(taskboard)
set_viewport_to_camera_view()

render_scene()