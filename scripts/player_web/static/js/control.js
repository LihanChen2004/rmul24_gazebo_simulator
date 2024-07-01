
function key_down_controller(keyCode){
	active_map[key_map[keyCode]] = true
	$("#"+key_map[keyCode]).addClass("key_active");
}

function key_up_controller(keyCode){
	active_map[key_map[keyCode]] = false
	$("#"+key_map[keyCode]).removeClass("key_active");
}

function mouse_down_controller(){
	$("#shoot").addClass("key_active");
	active_map['shoot'] = true
}

function mouse_up_controller(){
	active_map['shoot'] = false
	$("#shoot").removeClass("key_active");
}
function select_robot(element, name, can_choose){
	console.log(can_choose)
	if(!can_choose) return
	$("#choose_robot_container").children(".selected").removeClass('selected')
	my_robot = name
	$(element).addClass('selected') 
	$('#start_btn').removeAttr("disabled");
	console.log(my_robot)
	$('#start_btn').text("开始游戏");
}