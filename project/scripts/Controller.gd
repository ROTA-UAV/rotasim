extends FDM

func _process(_delta):
	#print("aileron input: ", Input.get_axis("aileron_neg", "aileron_pos"));
	#print("elevator input: ", Input.get_axis("elevator_neg", "elevator_pos"));
	#print("rudder input: ", Input.get_axis("rudder_neg", "rudder_pos"));
	#print("throttle input: ", Input.get_action_strength("throttle"));
	
	aileron = Input.get_axis("aileron_neg", "aileron_pos");
	elevator = -Input.get_axis("elevator_neg", "elevator_pos");
	rudder = -Input.get_axis("rudder_neg", "rudder_pos");
	throttle = Input.get_action_strength("throttle");
	pass
