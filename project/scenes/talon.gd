extends Node3D


@onready var plane := get_node("../Plane")


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	position = lerp(position, plane.position, 0.05)
	
	look_at_from_position(position, lerp(position, plane.position, 0.0002))
	pass
