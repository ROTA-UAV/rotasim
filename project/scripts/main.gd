extends Node3D

var plane = preload("res://scenes/Rascal110.tscn")
 
func add_plane():
	var p = plane.instantiate();
	p.name = "Plane";
	p.model_name = "Talon"
	p.init_name = "initGrnd.xml"
	add_child(p)
	p.get_child(2).fdm_node = get_child(get_child_count()-1).get_path()

func reset():
	if get_node_or_null("Plane") == null:
		add_plane()
	else:
		$Plane.free()
		add_plane()

func _input(event):
	if event.is_action_pressed("reset"):
		reset()


func _on_button_reset_pressed():
	reset()
