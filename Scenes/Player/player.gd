extends RigidBody3D

## 最大水平移动速度（单位：m/s）
@export var max_speed: float = 8.0
## 从 0 加速到 max_speed 需要的时间（单位：秒）
@export_range(0.001, 60.0, 0.001) var time_to_max_speed: float = 0.35
## 改变移动方向时，转向所需时间（单位：秒）。为 0 时瞬间转向。
@export_range(0.0, 10.0, 0.001) var turn_time: float = 0.18

## 允许的转向误差（弧度）。小于该值视为已对准。
@export_range(0.0, 0.5, 0.001) var turn_epsilon: float = 0.02

## 鼠标灵敏度（影响第三人称镜头绕玩家旋转）
@export var mouse_sensitivity: float = 0.05
## 第三人称镜头俯仰角限制（度）
@export var min_pitch: float = -80.0
@export var max_pitch: float = 50.0

var _current_move_dir: Vector3 = Vector3.FORWARD # 水平单位向量
var _target_move_dir: Vector3 = Vector3.FORWARD

var _turning: bool = false
var _turn_elapsed: float = 0.0
var _from_yaw: float = 0.0
var _to_yaw: float = 0.0

var _speed: float = 0.0

@onready var _pcam: PhantomCamera3D = _find_pcam()
@onready var _camera: Camera3D = _find_camera()


func _ready() -> void:
	# 锁住 X/Z 角度，只允许绕 Y 轴转（更像扫地机器人）
	axis_lock_angular_x = true
	axis_lock_angular_z = true

	_current_move_dir = _get_forward_dir()
	_target_move_dir = _current_move_dir


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	var delta := state.step
	_zero_physics_spin(state)

	var input_dir := _get_input_dir()
	if input_dir != Vector3.ZERO:
		# 输入方向变化：开始（或更新）转向，转完才移动
		if _turning:
			if _angle_between_dirs(_target_move_dir, input_dir) > turn_epsilon:
				_restart_turn(state, input_dir)
		else:
			if _angle_between_dirs(_current_move_dir, input_dir) > turn_epsilon:
				_start_turn(state, input_dir)

	# 执行转向（转向期间不移动）
	if _turning:
		_speed = 0.0
		_apply_planar_velocity(state, Vector3.ZERO)

		if turn_time <= 0.0:
			_set_yaw(state, _to_yaw)
			_finish_turn()
			return

		_turn_elapsed = min(_turn_elapsed + delta, turn_time)
		var t := _turn_elapsed / turn_time
		var yaw := lerp_angle(_from_yaw, _to_yaw, t)
		_set_yaw(state, yaw)

		if t >= 1.0 - 1e-6:
			_finish_turn()
		return

	# 不在转向：根据输入加/减速并移动
	if input_dir == Vector3.ZERO:
		_speed = _move_toward(_speed, 0.0, _accel() * delta)
		if _speed > 0.001:
			# 始终朝向并沿着"自身前方"移动，避免出现"朝向对但移动反"的情况
			_set_yaw(state, _yaw_from_dir(_current_move_dir))
			var fwd := _forward_dir_from_state(state)
			_apply_planar_velocity(state, fwd * _speed)
		else:
			_apply_planar_velocity(state, Vector3.ZERO)
		return

	_current_move_dir = input_dir
	_target_move_dir = input_dir
	_speed = _move_toward(_speed, max_speed, _accel() * delta)
	_set_yaw(state, _yaw_from_dir(_current_move_dir))
	var fwd := _forward_dir_from_state(state)
	_apply_planar_velocity(state, fwd * _speed)


func _unhandled_input(event: InputEvent) -> void:
	# ESC 切换鼠标捕获状态
	if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
		if Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		return

	# 鼠标控制 PhantomCamera 第三人称绕玩家旋转
	if Input.get_mouse_mode() != Input.MOUSE_MODE_CAPTURED:
		return
	if _pcam == null:
		return
	if event is InputEventMouseMotion:
		var rot_deg := _pcam.get_third_person_rotation_degrees()
		rot_deg.x -= event.relative.y * mouse_sensitivity
		rot_deg.x = clampf(rot_deg.x, min_pitch, max_pitch)
		rot_deg.y -= event.relative.x * mouse_sensitivity
		rot_deg.y = wrapf(rot_deg.y, 0.0, 360.0)
		_pcam.set_third_person_rotation_degrees(rot_deg)


func _start_turn(state: PhysicsDirectBodyState3D, new_dir: Vector3) -> void:
	_turning = true
	_turn_elapsed = 0.0
	_target_move_dir = new_dir
	# 开始转向的这一帧就立刻停住，避免"还没转完就滑出去"
	_speed = 0.0
	_apply_planar_velocity(state, Vector3.ZERO)
	_from_yaw = _get_yaw(state)
	_to_yaw = _yaw_from_dir(new_dir)


func _restart_turn(state: PhysicsDirectBodyState3D, new_dir: Vector3) -> void:
	# 转向中途改变方向：从当前朝向重新计时，保证"turn_time"始终是一次完整转向的耗时
	_target_move_dir = new_dir
	_speed = 0.0
	_apply_planar_velocity(state, Vector3.ZERO)
	_from_yaw = _get_yaw(state)
	_to_yaw = _yaw_from_dir(new_dir)
	_turn_elapsed = 0.0


func _finish_turn() -> void:
	_turning = false
	_current_move_dir = _target_move_dir


func _get_input_dir() -> Vector3:
	# 参考 PhantomCamera 示例代码的实现方式
	# Input.get_vector(left, right, up, down) 标准顺序
	var input_dir := Input.get_vector("left", "right", "up", "down")
	if input_dir == Vector2.ZERO:
		return Vector3.ZERO

	# 将 2D 输入映射到 3D：-x -> x（取反修正左右方向），y -> z
	var direction := Vector3(-input_dir.x, 0.0, input_dir.y).normalized()

	# 使用相机的 Y 轴旋转来旋转移动方向（取反修正旋转方向）
	if _camera != null:
		direction = direction.rotated(Vector3.UP, -_camera.rotation.y)

	return direction.normalized() if direction != Vector3.ZERO else Vector3.ZERO


func _accel() -> float:
	# 线性加速度：max_speed / time_to_max_speed
	return max_speed / max(time_to_max_speed, 0.001)


func _yaw_from_dir(dir: Vector3) -> float:
	# 让 -Z 作为"前方"时的 yaw=0
	return atan2(dir.x, -dir.z)


func _get_yaw(state: PhysicsDirectBodyState3D) -> float:
	return state.transform.basis.get_euler().y


func _set_yaw(state: PhysicsDirectBodyState3D, yaw: float) -> void:
	var xf := state.transform
	xf.basis = Basis(Vector3.UP, yaw)
	state.transform = xf


func _apply_planar_velocity(state: PhysicsDirectBodyState3D, planar_velocity: Vector3) -> void:
	var v := state.linear_velocity
	v.x = planar_velocity.x
	v.z = planar_velocity.z
	state.linear_velocity = v


func _zero_physics_spin(state: PhysicsDirectBodyState3D) -> void:
	# 物理碰撞/摩擦可能给刚体施加角速度，导致 y 轴"乱转"。
	# 这里我们把角速度强制清零，让朝向完全由脚本控制。
	state.angular_velocity = Vector3.ZERO


func _find_camera() -> Camera3D:
	# 优先找 Phantom Camera 示例里的唯一相机名
	if owner != null:
		var c := owner.get_node_or_null("%MainCamera3D")
		if c is Camera3D:
			return c
	return get_viewport().get_camera_3d()


func _find_pcam() -> PhantomCamera3D:
	if owner != null:
		var n := owner.get_node_or_null("%PlayerPhantomCamera3D")
		if n is PhantomCamera3D:
			return n
	return null


func _get_forward_dir() -> Vector3:
	# RigidBody3D 的"前方"约定为 -Z
	return (-global_transform.basis.z).normalized()


func _forward_dir_from_state(state: PhysicsDirectBodyState3D) -> Vector3:
	# 与 _set_yaw 使用的朝向保持一致：-Z 为前
	var z := state.transform.basis.z
	var fwd := Vector3(-z.x, 0.0, -z.z)
	return fwd.normalized() if fwd != Vector3.ZERO else Vector3.FORWARD


func _angle_between_dirs(a: Vector3, b: Vector3) -> float:
	var aa := Vector3(a.x, 0.0, a.z).normalized()
	var bb := Vector3(b.x, 0.0, b.z).normalized()
	if aa == Vector3.ZERO or bb == Vector3.ZERO:
		return 0.0
	return acos(clamp(aa.dot(bb), -1.0, 1.0))


func _move_toward(from: float, to: float, delta: float) -> float:
	if from < to:
		return min(from + delta, to)
	if from > to:
		return max(from - delta, to)
	return to
