extends Node3D

var plane = preload("res://scenes/Rascal110.tscn");

func _input(event):
	if event.is_action_pressed("reset"):
		if get_node("Rascal110") == null:
			var p = plane.instantiate();
			p.name = "Rascal110";
			p.model_name = "Rascal110"
			add_child(p);
		else:
			$Rascal110.queue_free();
 
