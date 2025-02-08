import bpy
import sys
import os
import bmesh
from math import radians

# Get command line arguments after "--"
argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

input_file = argv[0]
output_file = argv[1]
reduction_ratio = float(argv[2]) if len(argv) > 2 else 0.5

# Clear existing mesh objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Import STL
bpy.ops.wm.stl_import(filepath=input_file)

# Get the imported object
obj = bpy.context.selected_objects[0]
bpy.context.view_layer.objects.active = obj

# Enter edit mode
bpy.ops.object.mode_set(mode='EDIT')

# Get the bmesh
me = obj.data
bm = bmesh.from_edit_mesh(me)

# Mark sharp edges based on angle
for edge in bm.edges:
    if edge.calc_face_angle() > radians(30):
        edge.smooth = False

# Update the mesh
bmesh.update_edit_mesh(me)

# Return to object mode
bpy.ops.object.mode_set(mode='OBJECT')

# Add edge split modifier
edge_split = obj.modifiers.new(name="Edge Split", type='EDGE_SPLIT')
edge_split.use_edge_angle = True
edge_split.split_angle = radians(30)

# Apply edge split
bpy.ops.object.modifier_apply(modifier="Edge Split")

# Add limited dissolve to clean up the mesh
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.dissolve_limited(angle_limit=radians(5))
bpy.ops.object.mode_set(mode='OBJECT')

# Add decimate modifier
decimate = obj.modifiers.new(name="Decimate", type='DECIMATE')  # Changed from 'COLLAPSE'
# Set the decimate mode
decimate.decimate_type = 'COLLAPSE'  # Set as property after creation
decimate.ratio = reduction_ratio
decimate.use_collapse_triangulate = True

# Apply decimate
bpy.ops.object.modifier_apply(modifier="Decimate")

# Clean up
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=0.0001)
bpy.ops.object.mode_set(mode='OBJECT')

# Export STL
bpy.ops.wm.stl_export(filepath=output_file)