'''
Copyright 2019 Morgane Dufresne

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
he Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
bl_info = {
    "name": "Kinect Motion Capture plugin",
    "description": "Motion capture using MS Kinect v2",
    "author": "Morgane Dufresne",
    "version": (1, 0),
    "blender": (2, 80, 0),
    "warning": "You need a MS Kinect v2 sensor (XBox One)",
    "support": "COMMUNITY",
    "category": "Animation"
}

import bpy
import functools
import kinectMocap4Blender
import mathutils
from mathutils import Euler, Vector, Quaternion, Matrix
from time import sleep

###############################################
#                    Properties and misc
###############################################

def armature_callback(self, context):
    arms = [(str(arm.id_data.name), arm.name, arm.name) for arm in bpy.data.armatures]
    return arms

def validateTarget(self, context):
    if self.value != "" :
        found = False
        for bone in bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones:
            if self.value == bone.name :
                found = True
        if not found :
            self.value = ""
    return None

class KMC_PG_KmcTarget(bpy.types.PropertyGroup):
    name : bpy.props.StringProperty(name="KBone")
    value : bpy.props.StringProperty(name="TBone", update=validateTarget)

class KMC_PG_KmcProperties(bpy.types.PropertyGroup):
    fps : bpy.props.IntProperty(name="fps", description="Tracking frames per second", default=24, min = 1, max = 60)
    arma_list : bpy.props.EnumProperty(items = armature_callback, name="Armature", default=None)
    targetBones : bpy.props.CollectionProperty(type = KMC_PG_KmcTarget)
    currentFrame : bpy.props.IntProperty(name="currentFrame", description="current recording frame", default=0)
    record : bpy.props.BoolProperty(name="Record captured motion", description="activate recording while tracking")
    isTracking : bpy.props.BoolProperty(name="Tracking status", description="tracking status")
    stopTracking : bpy.props.BoolProperty(name="Stop trigger", description="tells to stop the tracking")

jointType = {
    "SpineBase":0,
    "SpineMid":1,
    "Neck":2,
    "Head":3,
    "ShoulderLeft":4,
    "ElbowLeft":5,
    "WristLeft":6,
    "HandLeft":7,
    "ShoulderRight":8,
    "ElbowRight":9,
    "WristRight":10,
    "HandRight":11,
    "HipLeft":12,
    "KneeLeft":13,
    "AnkleLeft":14,
    "FootLeft":15,
    "HipRight":16,
    "KneeRight":17,
    "AnkleRight":18,
    "FoorRight":19,
    "SpineShoulder":20,
    "HandTipLeft":21,
    "ThumbLeft":22,
    "HandTipRight":23,
    "ThumbRight":24
}

ordererBoneList = ["Head",
    "Neck",
    "Spine1",
    "Spine0",
    "LeftShoulder",
    "LeftUpperArm",
    "LeftLowerArm",
    "LeftHand",
    "RightShoulder",
    "RightUpperArm",
    "RightLowerArm",
    "RightHand",
    "LeftUpperLeg",
    "LeftLowerLeg",
    "LeftFoot",
    "RightUpperLeg",
    "RightLowerLeg",
    "RightFoot"]

defaultTargetBones = {
    "Head":"Head",
    "Neck":"Neck",
    "Spine1":"Spine2",
    "Spine0":"Hips",
    "LeftShoulder":"LeftShoulder",
    "LeftUpperArm":"LeftArm",
    "LeftLowerArm":"LeftForeArm",
    "LeftHand":"LeftHand",
    "RightShoulder":"RightShoulder",
    "RightUpperArm":"RightArm",
    "RightLowerArm":"RightForeArm",
    "RightHand":"RightHand",
    "LeftUpperLeg":"LeftUpLeg",
    "LeftLowerLeg":"LeftLeg",
    "LeftFoot":"LeftFoot",
    "RightUpperLeg":"RightUpLeg",
    "RightLowerLeg":"RightLeg",
    "RightFoot":"RightFoot"
}

bonesDefinition = {
    "Head":("Neck", "Head", Vector((0,0,1))),
    "Neck":("SpineShoulder", "Neck", Vector((0,0,1))),
    "Spine1":("SpineMid", "SpineShoulder", Vector((0,0,1))),
    "Spine0":("SpineBase", "SpineMid", Vector((0,0,1))),
    "LeftShoulder":("SpineShoulder", "ShoulderLeft", Vector((1,0,-0.5))),
    "LeftUpperArm":("ShoulderLeft", "ElbowLeft", None),
    "LeftLowerArm":("ElbowLeft", "WristLeft", None),
    "LeftHand":("WristLeft", "HandLeft", None),
    "RightShoulder":("SpineShoulder", "ShoulderRight", Vector((-1,0,-0.5))),
    "RightUpperArm":("ShoulderRight", "ElbowRight", None),
    "RightLowerArm":("ElbowRight", "WristRight", None),
    "RightHand":("WristRight", "HandRight", None),
    "LeftUpperLeg":("HipLeft", "KneeLeft", None),
    "LeftLowerLeg":("KneeLeft", "AnkleLeft", None),
    "LeftFoot":("AnkleLeft", "FootLeft", None),
    "RightUpperLeg":("HipRight", "KneeRight", None),
    "RightLowerLeg":("KneeRight", "AnkleRight", None),
    "RightFoot":("AnkleRight", "FoorRight", None)
}

restDirection = {}

def initialize(context):
    # reset pose
    bpy.ops.pose.select_all(action=('SELECT'))
    bpy.ops.pose.rot_clear()
    bpy.ops.pose.scale_clear()
    bpy.ops.pose.transforms_clear()
    bpy.ops.pose.select_all(action=('DESELECT'))
    context.scene.kmc_props.currentFrame = 0
    context.scene.kmc_props.stopTracking = False

    for target in context.scene.kmc_props.targetBones:
        if target.value is not None and target.value != "" :
            bone = bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones[target.value]
            bone.rotation_mode = 'QUATERNION'

            # Store rest pose angles for column, head and feet bones
            if bonesDefinition[target.name][2] is not None :
                baseDir =  bonesDefinition[target.name][2] * bone.matrix
                restDirection[target.name] = baseDir.rotation_difference(Vector((0,1,0)))


def updatePose(context, bone):
    sensor = context.scene.k_sensor
    
    for target in context.scene.kmc_props.targetBones:
        if target.value is not None and target.value == bone.name:
            # update bone pose
            head = sensor.getJoint(jointType[bonesDefinition[target.name][0]])
            tail = sensor.getJoint(jointType[bonesDefinition[target.name][1]])
            
            # axes matching
            X = 0 # inverted
            Y = 2
            Z = 1
            
            # update only tracked bones
            if(head[3] == 2) and (tail[3] == 2) :
                boneV = Vector((head[X] - tail[X], tail[Y] - head[Y], tail[Z] - head[Z]))
                
                # convert rotation in local coordinates
                boneV = boneV @ bone.matrix
                
                # compensate rest pose direction
                if target.name in restDirection :
                    boneV.rotate(restDirection[target.name])
                
                # calculate desired rotation
                rot = Vector((0,1,0)).rotation_difference(boneV)
                bone.rotation_quaternion = bone.rotation_quaternion @ rot
                if context.scene.kmc_props.currentFrame == 0:
                    # first captured frame, initiate recording by setting the current frame to 1
                    context.scene.kmc_props.currentFrame += 1
                if context.scene.kmc_props.record :
                    bone.keyframe_insert(data_path="rotation_quaternion", frame=context.scene.kmc_props.currentFrame)
                
    # update child bones
    for child in bone.children :
        updatePose(context, child)

###############################################
#                    UI
###############################################

# main panel
class KMC_PT_KinectMocapPanel(bpy.types.Panel):
    """ Creates a panel in Pose mode """
    bl_label = "Kinect Motion Capture Panel"
    bl_idname = "KMC_PT_KinectMocapPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Kinect MoCap"
    bl_context = "posemode"
    
    def draw(self, context):
        layout = self.layout
        obj = context.object
        
        # configure framerate
        layout.prop(context.scene.kmc_props, "fps")
        
        # choose armature
        layout.prop(context.scene.kmc_props, "arma_list")
        layout.separator()
        
        if len(context.scene.kmc_props.targetBones) == 0 :
            # initialization operator
            layout.operator("kmc.init")
        
        else:
            # bones retargeting
            box = layout.box()
            box.alignment = 'LEFT'
            box.label(text="             Bone Targeting")
            for strBone in ordererBoneList :
                for target in context.scene.kmc_props.targetBones :
                    if target.name == strBone :
                        box.prop(target, "value", text=target.name)
                        break
            
            # activate
            layout.separator()
            layout.operator("kmc.start")
            #layout.label(text="(right clic or 'Esc' to stop)")

            # stop
            layout.separator()
            layout.operator("kmc.stop")

            # activate record mode
            layout.prop(context.scene.kmc_props, "record")

            box = layout.box()
            box.alignment = 'CENTER'
            if context.scene.kmc_props.isTracking:
                box.label(text="Status : tracking")
            else:
                box.label(text="Status : stopped")
            
    def __del__(self):
        pass


###############################################
#                 Operators
###############################################

# initialize system
class KMC_OT_KmcInitOperator(bpy.types.Operator):
    bl_idname = "kmc.init"
    bl_label = "Initialize tracking system"
    
    def execute(self, context):
        for strBone in defaultTargetBones:
                newTarget = context.scene.kmc_props.targetBones.add()
                newTarget.name = strBone
                newTarget.value = ""
        return {'FINISHED'}

# timer function
def captureFrame(context):
    framerate = 1.0 / context.scene.kmc_props.fps
    
    if(context.scene.k_sensor.update() == 1):
        # update pose
        updatePose(context, bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones[0])

    if context.scene.kmc_props.currentFrame > 0 :
        context.scene.kmc_props.currentFrame += 1
        
    if context.scene.kmc_props.stopTracking:
        context.scene.kmc_props.stopTracking = False
        return None
    else:
        return framerate

# start tracking
class KMC_OT_KmcStartTrackingOperator(bpy.types.Operator):
    bl_idname = "kmc.start"
    bl_label = "Start / Stop"
    
    def execute(self, context):

        if context.scene.kmc_props.isTracking:
            context.scene.kmc_props.stopTracking = True
            context.scene.kmc_props.isTracking = False
            context.scene.k_sensor.close()

        else:
            # init system
            initialize(context)

            context.scene.k_sensor.init()
            bpy.app.timers.register(functools.partial(captureFrame, context))
            context.scene.kmc_props.isTracking = True

        return {'FINISHED'}


###############################################
#               Registration
###############################################

classes = (
    KMC_PG_KmcTarget,
    KMC_PG_KmcProperties,
    KMC_PT_KinectMocapPanel,
    KMC_OT_KmcInitOperator,
    KMC_OT_KmcStartTrackingOperator
)

def register():
    for c in classes :
        bpy.utils.register_class(c)
    bpy.types.Scene.k_sensor = kinectMocap4Blender.Sensor()
    bpy.types.Scene.kmc_props = bpy.props.PointerProperty(type=KMC_PG_KmcProperties)

def unregister():
    for c in reversed(classes) :
        bpy.utils.register_class(c)
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.k_sensor
    del bpy.types.Scene.kmc_props

if __name__ == "__main__":
    register()
