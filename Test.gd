extends Spatial


# Declare member variables here. Examples:
# var a: int = 2
# var b: String = "text"


# Called when the node enters the scene tree for the first time.
#func _ready() -> void:
#	pass


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	$Spatial/test.translation.z -= Input.get_axis("ui_left", "ui_right") * delta * 8.0
	$Spatial/test.translation.y -= sin(float(get_tree().get_frame()) * 0.1) * 0.15
	

