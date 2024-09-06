tool
class_name Wigglebone extends BoneAttachment

export var enable: bool = true
export(float, 0.01, 10, 0.01) var stiffness = 0.1
export(float, 0, 10, 0.01) var damping = 0.1
export(float, 0, 10, 0.01) var gravity_scale = 1.0
export var use_gravity = false
export var gravity = Vector3(0.0, -9.81, 0.0)
export(float, 0.0, 100.0, 1.0) var apply_percent = 80.0
export(float, 0.0, 1.0, 0.001) var length = 0.1
export(float, 0.01, 1, 0.01) var max_distance = 0.1

export var skip_frame: int = 0
var frame: int = 0
var _point_mass := PointMass.new()
var _acceleration :Vector3= Vector3.ZERO
var _prev_mass_center :Vector3= Vector3.ZERO
var _prev_velocity :Vector3= Vector3.ZERO
var _should_reset :bool= true
var _global_to_pose: Basis = Basis()

# Previous position
var prev_pos = Vector3()

func jiggleprint(text):
	print("[Jigglebones] Error: " + text)

func _ready() -> void:
	if enable:
		set_as_toplevel(true)  # Ignore parent transformation
		prev_pos = global_transform.origin
		

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
		
	
	# Wiggle bone
	var bone_pose: Transform = skeleton.get_bone_pose(bone_id)
	if bone_id_parent >= 0:
		bone_pose = skeleton.get_bone_global_pose(bone_id_parent) * skeleton.get_bone_rest(bone_id) * bone_pose

	var global_bone_pose: Transform = skeleton.global_transform * bone_pose
	_global_to_pose = global_bone_pose.basis.inverse()

	var new_acceleration :Vector3 = _update_acceleration(global_bone_pose, delta)
	_acceleration = _acceleration.linear_interpolate(new_acceleration, 30.0 * delta)

	# adjust for varying framerates
#	# this is only an approximation
	var delta_factor : float = log(delta * 60.0) / log(2.0) + 1.0
	_acceleration /= clamp(delta_factor, 1.0, 3.0) # TODO: adjust for rates higher than 60 fps

	var pose : Transform = bone_pose * _pose()
	
	skeleton.set_bone_global_pose_override(bone_id, pose, apply_percent / 100.0, true)
	
	_solve(_global_to_pose, _acceleration, delta)
	
#func _physics_process(delta: float) -> void:
	

func _clamp_length_soft(v: Vector3, min_length: float, max_length: float, k: float) -> Vector3:
	return v.normalized() * _smin(max(min_length, v.length()), max_length, k)


# https://iquilezles.org/articles/smin/
func _smin(a: float, b: float, k: float) -> float:
	var h := max(0.0, k - abs(a - b))
	return min(a, b) - h * h / (4.0 * k)

class PointMass:
	var p := Vector3.ZERO
	var v := Vector3.ZERO
	var a := Vector3.ZERO

	func solve(stiffness: float, damping: float, delta: float) -> void:
		# inertia
		v = v * (1.0 - damping) + a * delta
		p += v
		a = Vector3.ZERO

		# constraint
		v -= p * stiffness

	func accelerate(acc: Vector3, delta: float) -> void:
		v += acc * delta

	func apply_force(force: Vector3) -> void:
		a += force

	func reset() -> void:
		p = Vector3.ZERO
		v = Vector3.ZERO
		a = Vector3.ZERO

func _pose() -> Transform:
	var pose: Transform = Transform()
	var k: float = max_distance * 0.5
	var mass_constrained : Vector3 = _clamp_length_soft(_point_mass.p, 0.0, max_distance, k)

	pose.origin = mass_constrained

	return pose

func _update_acceleration(global_bone_pose: Transform, delta: float) -> Vector3:
	var mass_center := Vector3.ZERO

	mass_center = global_bone_pose * mass_center
	var delta_mass_center := _prev_mass_center - mass_center
	_prev_mass_center = mass_center

	if _should_reset:
		delta_mass_center = Vector3.ZERO

	var global_velocity := delta_mass_center / delta
	_acceleration = global_velocity - _prev_velocity
	_prev_velocity = global_velocity

	if _should_reset:
		_acceleration = Vector3.ZERO
		_should_reset = false

	return _acceleration

func _solve(global_to_local: Basis, acceleration: Vector3, delta: float) -> void:
	var global_force :Vector3 = gravity * gravity_scale
	if not use_gravity:
		global_force = Vector3.ZERO
	var local_force :Vector3 = global_to_local * global_force

	var mass_distance :float = length
	var local_acc := global_to_local * acceleration
	_point_mass.accelerate(local_acc, delta)
	_point_mass.apply_force(local_force)
	_point_mass.solve(stiffness, damping, delta)
