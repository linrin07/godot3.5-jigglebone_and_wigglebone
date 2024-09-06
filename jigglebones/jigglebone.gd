tool
class_name Jigglebone extends Spatial

enum Axis {
	X_Plus, Y_Plus, Z_Plus, X_Minus, Y_Minus, Z_Minus
}

export var enable: bool = true
export (String) var bone_name
export(float, 0.01, 10, 0.01) var stiffness = 0.1
export(float, 0, 10, 0.01) var damping = 0.0
export(float, 0, 10, 0.01) var gravity_scale = 1.0
export var use_gravity = false
export var gravity = Vector3(0.0, -9.81, 0.0)
export(Axis) var forward_axis = Axis.Z_Minus
export(float, 0.0,1.0,0.001) var bone_y_plus = 0.1
export(float, 0.0, 100.0, 1.0) var apply_percent = 80.0

export var collision_shape: NodePath
var collision_sphere: CollisionShape

# Previous position
var prev_pos = Vector3()

# Rest length of the distance constraint
var rest_length = 1

export var skip_frame: int = 0
var frame: int = 0

func jiggleprint(text):
	print("[Jigglebones] Error: " + text)
	
func get_bone_forward_local() -> Vector3:
	match forward_axis:
		Axis.X_Plus: return Vector3(1,0,0) 
		Axis.Y_Plus: return Vector3(0,1,0) 
		Axis.Z_Plus: return Vector3(0,0,1)
		Axis.X_Minus: return Vector3(-1,0,0)
		Axis.Y_Minus: return Vector3(0,-1,0) 
		Axis.Z_Minus: return Vector3(0,0,-1)
		_: return Vector3(0,1,0) 

func _ready() -> void:
	if enable:
		set_as_toplevel(true)  # Ignore parent transformation
		prev_pos = global_transform.origin
		set_collision_shape(collision_shape)
		

func _process(delta: float) -> void:
	frame += 1
	if frame > skip_frame:
		frame = 0
		process(delta * float(1+skip_frame))

func process(delta: float) -> void:
	if not enable:
		return
	var skeleton: Skeleton
	var bone_id: int
	var bone_id_parent: int
	
	if Engine.editor_hint:
		skeleton = get_parent()
			
		if !(skeleton is Skeleton):
			jiggleprint("Jigglebone must be a direct child of a Skeleton node")
			return
		
		if !bone_name:
			jiggleprint("Please enter a bone name")
			return
		
		bone_id = skeleton.find_bone(bone_name)
		
		if bone_id == -1:
			jiggleprint("Unknown bone %s - please enter a valid bone name" % bone_name)
			return
			
		bone_id_parent = skeleton.get_bone_parent(bone_id)
	else:
		skeleton = get_parent()
		bone_id = skeleton.find_bone(bone_name)
		bone_id_parent = skeleton.get_bone_parent(bone_id)
		
	
		
		
	# Note:
	# Local space = local to the bone
	# Object space = local to the skeleton (confusingly called "global" in get_bone_global_pose)
	# World space = global
	
	# See https://godotengine.org/qa/7631/armature-differences-between-bones-custom_pose-transform
	
	var bone_transf_obj: Transform = skeleton.get_bone_global_pose(bone_id) # Object space bone pose
	var bone_transf_world: Transform = skeleton.global_transform * bone_transf_obj
	
	var bone_transf_rest_local: Transform = skeleton.get_bone_rest(bone_id)
	var bone_transf_rest_obj: Transform = skeleton.get_bone_global_pose(bone_id_parent) * bone_transf_rest_local 
	var bone_transf_rest_world: Transform = skeleton.global_transform * bone_transf_rest_obj
	
	############### Integrate velocity (Verlet integration) ##############	
	
	# If not using gravity, apply force in the direction of the bone (so it always wants to point "forward")
	var grav: Vector3 = bone_transf_rest_world.basis.xform(Vector3(0, 0, -1)).normalized() * 9.81
	var vel: Vector3 = (global_transform.origin - prev_pos) / delta
	
	if use_gravity:
		grav = gravity * gravity_scale
		
	grav *= stiffness
	vel += grav 
	vel -= vel * damping * delta  # Damping
	
	prev_pos = global_transform.origin
	global_transform.origin = global_transform.origin + vel * delta
	
	############### Solve distance constraint ##############
	
	var goal_pos: Vector3 = skeleton.to_global(skeleton.get_bone_global_pose(bone_id).origin)
	var new_pos_clamped: Vector3 = goal_pos + (global_transform.origin - goal_pos).normalized() * bone_y_plus
	global_transform.origin = new_pos_clamped
	
	if collision_sphere:
		# If bone is inside the collision sphere, push it out
		var test_vec: Vector3 = global_transform.origin - collision_sphere.global_transform.origin
#		var test_vec: Vector3 = goal_pos - collision_sphere.global_transform.origin
#		print(collision_sphere.global_transform.origin, collision_sphere.shape.radius)
#		print(collision_sphere.global_position,collision_sphere.global_transform.origin)
		var distance: float = test_vec.length() - collision_sphere.shape.radius
#		print(distance)
		if distance < 0:
			global_transform.origin -= test_vec.normalized() * distance
	
	
	############## Rotate the bone to point to this object #############

	var diff_vec_local: Vector3 = bone_transf_world.affine_inverse().xform(global_transform.origin).normalized() 
	
	var bone_forward_local: Vector3 = get_bone_forward_local()

	# The axis+angle to rotate on, in local-to-bone space
	var bone_rotate_axis: Vector3 = bone_forward_local.cross(diff_vec_local)
	var bone_rotate_angle: float = acos(bone_forward_local.dot(diff_vec_local))
	
	if bone_rotate_axis.length() < 1e-3:
		return  # Already aligned, no need to rotate
	
	bone_rotate_axis = bone_rotate_axis.normalized()

	# Bring the axis to object space, WITHOUT translation (so only the BASIS is used) since vectors shouldn't be translated
	var bone_rotate_axis_obj: Vector3 = bone_transf_obj.basis.xform(bone_rotate_axis).normalized()
	if bone_rotate_axis_obj.length() < 1e-3:
		return
	var bone_new_transf_obj = Transform(bone_transf_obj.basis.rotated(bone_rotate_axis_obj, bone_rotate_angle), bone_transf_obj.origin)  

	if is_nan(bone_new_transf_obj[0][0]):
		bone_new_transf_obj = Transform()  # Corrupted somehow

	skeleton.set_bone_global_pose_override(bone_id, bone_new_transf_obj, apply_percent / 100.0, true)
	
	# Orient this object to the jigglebone
	global_transform.basis = (skeleton.global_transform * skeleton.get_bone_global_pose(bone_id)).basis
	
	
	

func set_collision_shape(path:NodePath) -> void:
	collision_shape = path
	collision_sphere = get_node_or_null(path)
	if collision_sphere:
		assert(collision_sphere is CollisionShape and collision_sphere.shape is SphereShape, "%s: Only SphereShapes are supported for CollisionShapes" % [ name ])

