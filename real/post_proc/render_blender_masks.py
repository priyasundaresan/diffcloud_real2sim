import bpy
import numpy as np
import pickle
import os
from os.path import splitext, join, basename
import sys
from addon_utils import check, paths, enable
import sys
#import pyBulletSimImporter
from bpy.props import (
    StringProperty,
    CollectionProperty
)


def clear_scene():
    '''Clear existing objects in scene'''
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        if block.users == 0:
            bpy.data.materials.remove(block)
    for block in bpy.data.textures:
        if block.users == 0:
            bpy.data.textures.remove(block)
    for block in bpy.data.images:
        if block.users == 0:
            bpy.data.images.remove(block)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()


def add_camera(view_mode):
    if view_mode == 'side':
        bpy.ops.object.camera_add(location=(0.8, -0.8, 0.5), rotation=(1.1845214366912842, -0.0, 0.02)) # side, gates, fold + lift
    elif view_mode == 'overhead':
        #bpy.ops.object.camera_add(location=(1.125, 0.145, 1.05), rotation=(0.45, 0.1, 1.6)) # top, gates, fold
        #bpy.ops.object.camera_add(location=(1.21, 0.035, 1.05), rotation=(0.5, -0.05, 1.60)) # top, gates, fold
        bpy.ops.object.camera_add(location=(1.21, 0.035, 1.05), rotation=(0.5, -0.05, 1.60)) # top, gates, fold
    elif view_mode == 'angled_overhead':
        bpy.ops.object.camera_add(location=(1.17, -0.3, 0.9), rotation=(0.6, 0, 0.9)) # top
    bpy.context.scene.camera = bpy.context.object
    return bpy.context.object


def set_render_settings(render_size, engine='CYCLES', generate_masks=True):
    # Set rendering engine, dimensions, colorspace, images settings
    scene = bpy.context.scene
    scene.world.color = (1, 1, 1)
    scene.render.resolution_percentage = 100
    scene.render.engine = engine
    render_width, render_height = render_size
    scene.render.resolution_x = render_width
    scene.render.resolution_y = render_height
    scene.use_nodes = True
    scene.render.image_settings.file_format='JPEG'
    scene.view_settings.exposure = 1.3
    scene.cycles.samples = 1
    scene.view_settings.view_transform = 'Raw'
    scene.cycles.max_bounces = 1
    scene.cycles.min_bounces = 1
    scene.cycles.glossy_bounces = 1
    scene.cycles.transmission_bounces = 1
    scene.cycles.volume_bounces = 1
    scene.cycles.transparent_max_bounces = 1
    scene.cycles.transparent_min_bounces = 1
    scene.view_layers["View Layer"].use_pass_object_index = True
    scene.render.tile_x = 16
    scene.render.tile_y = 16

def render(episode, obj, output_dir):
    scene = bpy.context.scene
    tree = bpy.context.scene.node_tree
    links = tree.links
    render_node = tree.nodes["Render Layers"]
    if episode == 0:
        id_mask_node = tree.nodes.new(type="CompositorNodeIDMask")
        id_mask_node.use_antialiasing = True
        id_mask_node.index = obj.pass_index
        inv_node = tree.nodes.new(type="CompositorNodeInvert")
        composite = tree.nodes.new(type = "CompositorNodeComposite")
        links.new(render_node.outputs['IndexOB'], id_mask_node.inputs["ID value"])
        links.new(id_mask_node.outputs[0], inv_node.inputs["Color"])
        links.new(inv_node.outputs[0], composite.inputs["Image"])
        #links.new(id_mask_node.outputs[0], composite.inputs["Image"])
    scene.render.filepath = '%s/mask_%d.jpg'%(output_dir, episode)
    bpy.ops.render.render(write_still=True)


def load_pybullet_pkl(context, files, directory, compensation_factor):
    for file in files:
        filepath = join(directory, file)
        print(f'Processing {filepath}')
        with open(filepath, 'rb') as pickle_file:
            data = pickle.load(pickle_file)
            collection_name = splitext(basename(filepath))[0]
            collection = bpy.data.collections.new(collection_name)
            bpy.context.scene.collection.children.link(collection)
            context.view_layer.active_layer_collection = \
                context.view_layer.layer_collection.children[-1]

            for obj_key in data:
                pybullet_obj = data[obj_key]
                if 'finger_pad' in pybullet_obj['mesh_path']:
                    continue

                # Load mesh of each link
                assert pybullet_obj['type'] == 'mesh'
                extension = pybullet_obj['mesh_path'].split(
                    ".")[-1].lower()
                # Handle different mesh formats
                if 'obj' in extension:
                    bpy.ops.import_scene.obj(
                        filepath=pybullet_obj['mesh_path'],
                        axis_forward='Y', axis_up='Z')
                elif 'dae' in extension:
                    bpy.ops.wm.collada_import(
                        filepath=pybullet_obj['mesh_path'])
                elif 'stl' in extension:
                    bpy.ops.import_mesh.stl(
                        filepath=pybullet_obj['mesh_path'])
                else:
                    print("Unsupported File Format:{}".format(extension))
                    pass

                # Delete lights and camera
                parts = 0
                final_objs = []
                for import_obj in context.selected_objects:
                    bpy.ops.object.select_all(action='DESELECT')
                    import_obj.select_set(True)
                    if 'Camera' in import_obj.name \
                            or 'Light' in import_obj.name\
                            or 'Lamp' in import_obj.name:
                        bpy.ops.object.delete(use_global=True)
                    else:
                        scale = pybullet_obj['mesh_scale']
                        if scale is not None:
                            import_obj.scale.x = scale[0]
                            import_obj.scale.y = scale[1]
                            import_obj.scale.z = scale[2]
                        final_objs.append(import_obj)
                        parts += 1
                bpy.ops.object.select_all(action='DESELECT')
                for obj in final_objs:
                    if obj.type == 'MESH':
                        obj.select_set(True)
                if len(context.selected_objects):
                    context.view_layer.objects.active =\
                        context.selected_objects[0]
                    # join them
                    bpy.ops.object.join()
                blender_obj = context.view_layer.objects.active
                blender_obj.name = obj_key

                # Keyframe motion of imported object
                #compensation_factor = -1.24 #realsense
                for frame_count, frame_data in enumerate(
                        pybullet_obj['frames']):
                    percentage_done = frame_count / \
                        len(pybullet_obj['frames'])
                    print(f'\r[{percentage_done*100:.01f}% | {obj_key}]',
                          '#' * int(60*percentage_done), end='')
                    pos = frame_data['position']
                    orn = frame_data['orientation']
                    blender_obj.location.x = pos[0]
                    blender_obj.location.y = pos[1]
                    blender_obj.location.z = pos[2]
                    blender_obj.rotation_mode = 'QUATERNION'
                    blender_obj.rotation_quaternion.x = orn[0]
                    blender_obj.rotation_quaternion.y = orn[1]
                    blender_obj.rotation_quaternion.z = orn[2]
                    blender_obj.rotation_quaternion.w = orn[3]
                    blender_obj.keyframe_insert(data_path="location", frame=frame_count+compensation_factor)
                    blender_obj.keyframe_insert(data_path="rotation_quaternion", frame=frame_count+compensation_factor)
    return frame_count+1


def make_table(view_mode):
    if view_mode == 'side':
        bpy.ops.mesh.primitive_plane_add(size=6, location=(0.82,2.78,-1.25), rotation=(1.5708,0,0))
    else:
        bpy.ops.mesh.primitive_plane_add(size=3, location=(0,0,0))
    return bpy.context.object

def make_poles():
    bpy.ops.import_scene.obj(
        filepath='obstacles/poles.obj'
        )
    obj = bpy.data.objects['poles']
    obj.scale = (0.125,0.125,0.11)
    obj.rotation_euler = (0,0,-np.pi/2)
    obj.location = (0.8,0.065, 0)
    return obj

def delete_objs(ob_names):
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    for ob_name in ob_names:
        ob = bpy.data.objects[ob_name]
        ob.select_set(True)
    bpy.ops.object.delete()

def main():
    # Run using:
    # blender -b -P test_blender_pybullet.py
    argv = sys.argv
    argv = argv[argv.index("--") + 1:]

    traj_dir = argv[0]
    save_dir = argv[1]
    view_mode = argv[2]
    traj_recorder_file = argv[3]

    if view_mode == 'side' or view_mode == 'overhead':
        render_size = (480, 480)
        #compensation_factor = -1.5
        compensation_factor = -1.25
    else:
        render_size = (1032, 772)
        compensation_factor = -0.25

    set_render_settings(render_size)
    clear_scene()
    camera = add_camera(view_mode)

    #frame_count = load_pybullet_pkl(bpy.context, files=['traj_for_imgs_recorder.pkl'], directory=traj_dir, compensation_factor=compensation_factor)
    frame_count = load_pybullet_pkl(bpy.context, files=[traj_recorder_file], directory=traj_dir, compensation_factor=compensation_factor)

    output_dir = os.path.join(save_dir, 'mask')

    if not(os.path.exists(output_dir)):
        os.mkdir(output_dir)

    obj = make_table(view_mode)
    #make_poles()
    obj.pass_index = 1
    delete_objs(['gen3_robotiq_2f_85_0_left_inner_finger_pad_0', 'gen3_robotiq_2f_85_0_right_inner_finger_pad_0'])
    for i in range(frame_count):
        render(i, obj, output_dir)
        bpy.context.scene.frame_set(i)

if __name__ == '__main__':
    main()
