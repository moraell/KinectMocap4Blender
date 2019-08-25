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
    "version": (1, 4),
    "blender": (2, 79, 0),
    "support": "COMMUNITY",
    "category": "Animation"
}

import bpy
import kinectMocap4Blender
import mathutils
from mathutils import Euler, Vector, Quaternion, Matrix
from time import sleep

###############################################
#                    Properties and misc
###############################################

def armature_callback(self, context):
    arms = []
    for obj in bpy.data.objects :
        if 'ARMATURE' == obj.type :
            arms.append((str(obj.id_data.name), obj.name, obj.name))
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

KBonesEnum = [("Head", "Head", "Head"),
    ("Neck", "Neck", "Neck"),
    ("Spine1", "Spine1", "Spine1"),
    ("Spine0", "Spine0", "Spine0"),
    ("LeftShoulder", "LeftShoulder", "LeftShoulder"),
    ("LeftUpperArm", "LeftUpperArm", "LeftUpperArm"),
    ("LeftLowerArm", "LeftLowerArm", "LeftLowerArm"),
    ("LeftHand", "LeftHand", "LeftHand"),
    ("RightShoulder", "RightShoulder", "RightShoulder"),
    ("RightUpperArm", "RightUpperArm", "RightUpperArm"),
    ("RightLowerArm", "RightLowerArm", "RightLowerArm"),
    ("RightHand", "RightHand", "RightHand"),
    ("LeftUpperLeg", "LeftUpperLeg", "LeftUpperLeg"),
    ("LeftLowerLeg", "LeftLowerLeg", "LeftLowerLeg"),
    ("LeftFoot", "LeftFoot", "LeftFoot"),
    ("RightUpperLeg", "RightUpperLeg", "RightUpperLeg"),
    ("RightLowerLeg", "RightLowerLeg", "RightLowerLeg"),
    ("RightFoot", "RightFoot", "RightFoot")
]

KalmanStrengthEnum = [("Strong", "Strong", "Strong denoising (ideal for slow movement)"),
    ("Normal", "Normal", "Normal denoising (good balance between speed and precision)"),
    ("Low", "Low", "Low denoising (for fast movement, less precise)"),
    ("VeryLow", "Very low", "Very low denoising (only for very fast movement, almost no noise reduction")
]

class KmcTarget(bpy.types.PropertyGroup):
    name = bpy.props.StringProperty(name="KBone")
    value = bpy.props.StringProperty(name="TBone", update=validateTarget)

class KmcProperties(bpy.types.PropertyGroup):
    fps = bpy.props.IntProperty(name="fps", description="Tracking frames per second", default=24, min = 1, max = 60)
    arma_list = bpy.props.EnumProperty(items = armature_callback, name="Armature", default=None)
    targetBones = bpy.props.CollectionProperty(type = KmcTarget)
    currentFrame = bpy.props.IntProperty(name="currentFrame", description="current recording frame", default=0)
    record = bpy.props.BoolProperty(name="Record captured motion", description="activate recording while tracking")
    firstFramePosition = bpy.props.FloatVectorProperty(name="firstFramePosition", description="position of root bone in first frame", size=3)
    initialOffset = bpy.props.FloatVectorProperty(name="initialOffset", description="position of root bone in rest pose", size=3)
    lockHeight = bpy.props.BoolProperty(name="height", description="ignore vertical movement", default=True)
    lockwidth = bpy.props.BoolProperty(name="width", description="ignore lateral movement", default=True)
    lockDepth = bpy.props.BoolProperty(name="depth", description="ignore depth movement", default=True)
    rootBone = bpy.props.EnumProperty(name="root bone", items=KBonesEnum, default="Spine0", description="Kinect identifier of the bone that is used as root of the skeleton")
    kalmanStrength = bpy.props.EnumProperty(name="Denoising", items=KalmanStrengthEnum, default="Normal")

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
    context.scene.kmc_props.firstFramePosition = (-1,-1,-1)
    context.scene.kmc_props.initialOffset = (0,0,0)

    for target in context.scene.kmc_props.targetBones:
        if target.value is not None and target.value != "" :
            bone = bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones[target.value]
            bone.rotation_mode = 'QUATERNION'

            # Store initial position of spine0 bone
            if target.name == context.scene.kmc_props.rootBone:
                context.scene.kmc_props.initialOffset = bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones[target.value].matrix.translation

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
                
                # if first bone, update position (only for configured axes)
                if target.name == context.scene.kmc_props.rootBone:
                    # initialize firstFramePosition if fit isn't
                    if context.scene.kmc_props.firstFramePosition[1] == -1:
                        context.scene.kmc_props.firstFramePosition = (-1.0*head[X], head[Y], head[Z])
                        
                    ffp = context.scene.kmc_props.firstFramePosition
                    tx = context.scene.kmc_props.initialOffset[0]
                    ty = context.scene.kmc_props.initialOffset[2]
                    tz = context.scene.kmc_props.initialOffset[1]
                    if not context.scene.kmc_props.lockwidth:
                        tx += -head[X] - ffp[0]
                    if not context.scene.kmc_props.lockHeight:
                        ty += head[Z] - ffp[2]
                    if not context.scene.kmc_props.lockDepth:
                        tz += head[Y] - ffp[1]
                        
                    # translate bone
                    bone.matrix.translation = (tx, tz, ty)
                
                # convert rotation in local coordinates
                boneV = boneV * bone.matrix
                
                # compensate rest pose direction
                if target.name in restDirection :
                    boneV.rotate(restDirection[target.name])
                
                # calculate desired rotation
                rot = Vector((0,1,0)).rotation_difference(boneV)
                bone.rotation_quaternion = bone.rotation_quaternion * rot
                if context.scene.kmc_props.currentFrame == 0:
                    # first captured frame, initiate recording by setting the current frame to 1
                    context.scene.kmc_props.currentFrame += 1
                if context.scene.kmc_props.record :
                    bone.keyframe_insert(data_path="rotation_quaternion", frame=context.scene.kmc_props.currentFrame)
                    if target.name == context.scene.kmc_props.rootBone:
                        bone.keyframe_insert(data_path="location", frame=context.scene.kmc_props.currentFrame)
                
    # update child bones
    for child in bone.children :
        updatePose(context, child)

###############################################
#                    UI
###############################################

# main panel
class KinectMocapPanel(bpy.types.Panel):
    """ Creates a panel in Pose mode """
    bl_label = "Kinect Motion Capture Panel"
    bl_idname = "kmc_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
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
            box.label("        Bone Targeting")
            for strBone in ordererBoneList :
                for target in context.scene.kmc_props.targetBones :
                    if target.name == strBone :
                        box.prop(target, "value", text=target.name)
                        break
            layout.prop(context.scene.kmc_props, "rootBone")
            
            # configure movement tracking
            layout.separator()
            box = layout.box()
            box.label(text="Lock movement for :")
            col = box.column_flow(columns=3)
            col.prop(context.scene.kmc_props, "lockDepth")
            col.prop(context.scene.kmc_props, "lockHeight")
            col.prop(context.scene.kmc_props, "lockwidth")
            
            # denoising strength
            layout.separator()
            layout.prop(context.scene.kmc_props, "kalmanStrength")

            # activate
            layout.separator()
            layout.operator("kmc.start")
            layout.label("(right clic or 'Esc' to stop)")

            # activate record mode
            layout.prop(context.scene.kmc_props, "record")
            
    def __del__(self):
        pass


###############################################
#                 Operators
###############################################

# initialize system
class KmcInitOperator(bpy.types.Operator):
    bl_idname = "kmc.init"
    bl_label = "Initialize tracking system"
    
    def execute(self, context):
        for strBone in defaultTargetBones:
                newTarget = context.scene.kmc_props.targetBones.add()
                newTarget.name = strBone
                newTarget.value = ""
        return {'FINISHED'}

# start tracking
class KmcStartTrackingOperator(bpy.types.Operator):
    bl_idname = "kmc.start"
    bl_label = "Start tracking"
    
    def modal(self, context, event):
        wm = context.window_manager
        uNoise = 5.0
        if context.scene.kmc_props.kalmanStrength == "VeryLow" :
            uNoise=50.0
        elif context.scene.kmc_props.kalmanStrength == "Low" :
            uNoise=20.0
        elif context.scene.kmc_props.kalmanStrength == "Normal" :
            uNoise=5.0
        elif context.scene.kmc_props.kalmanStrength == "Strong" :
            uNoise=1.0
        context.scene.k_sensor.init(1.0 / context.scene.kmc_props.fps, 0.0005, uNoise)
        if event.type == 'TIMER':
            if(context.scene.k_sensor.update() == 1):
                # update pose
                updatePose(context, bpy.data.objects[context.scene.kmc_props.arma_list].pose.bones[0])
            if context.scene.kmc_props.currentFrame > 0 :
                context.scene.kmc_props.currentFrame += 1
        
        elif event.type in {'RIGHTMOUSE', 'ESC'}:
            # stop tracking
            wm.event_timer_remove(self._timer)
            context.scene.k_sensor.close()
            return {'CANCELLED'}
 
        return {'RUNNING_MODAL'}
 
    def invoke(self, context, event):
        # init system
        initialize(context)
        
        # start tracking
        wm = context.window_manager
        framerate = 1.0 / context.scene.kmc_props.fps
        self._timer = wm.event_timer_add(framerate, context.window)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

###############################################
#               Registration
###############################################

def register():
    bpy.utils.register_module(__name__)
    bpy.types.Scene.k_sensor = kinectMocap4Blender.Sensor()
    bpy.types.Scene.kmc_props = bpy.props.PointerProperty(type=KmcProperties)

def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.k_sensor
    del bpy.types.Scene.kmc_props

if __name__ == "__main__":
    register()
