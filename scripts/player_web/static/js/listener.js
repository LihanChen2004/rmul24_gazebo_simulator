
//------------------------------------
// 监听全屏事件
//------------------------------------
function fullscreenChange(e, game_container) {

	if (document.webkitFullscreenElement === elem ||
		document.mozFullscreenElement === elem ||
		document.mozFullScreenElement === elem) { // 较旧的 API 大写 'S'.
		// 元素进入全屏模式了，
		
		elem.requestPointerLock = elem.requestPointerLock ||
			elem.mozRequestPointerLock ||
			elem.webkitRequestPointerLock;
			
		// 开启鼠标锁定
		elem.requestPointerLock();
		
		// 开启案件监听
		$(document).keydown(function(event) {
			key_down_controller(event.keyCode)
		});
		$(document).keyup(function(event) {
			key_up_controller(event.keyCode)
		});
		
		// 显示元素
		game_container.show()
		
		// 开启鼠标监听
		document.addEventListener("mousemove", mouseListener);

		// 开启获取图像
		start_socket_transfer()

		$(document).mousedown(function(e){
				if(1 == e.which){
					// console.log("你点击了鼠标左键");
					mouse_down_controller()
				}
			});
		$(document).mouseup(function(e){
				if(1 == e.which){
					// console.log("你松开了了鼠标左键");
					mouse_up_controller()
				}
			});

		
		// start_socket_transfer(socket)
		return
	}
	
	// 解除监听器
	document.removeEventListener("mousemove", mouseListener);
	window.socket.close()
	$(document).unbind("keydown");
	$(document).unbind("keyup");
	$(document).unbind("mousedown");
	$(document).unbind("mouseup");
	clearInterval(window.fp_timer);
	clearInterval(window.control_timer);
	$('#start_btn').attr('disabled',"true");
	$('#start_btn').text("连接中");
	init_socket()
	// $('#background').attr('src', $("#camera")[0].src)
	game_container.hide()

}

//------------------------------------
// 监听鼠标移动
//------------------------------------
function mouseListener(e) {
	var movementX = e.movementX ||
		e.mozMovementX ||
		e.webkitMovementX ||
		0,
		movementY = e.movementY ||
		e.mozMovementY ||
		e.webkitMovementY ||
		0;
		active_map["movementX"] = -movementX/yaw_factor
		active_map["movementY"] = movementY/pitch_factor
		
}


//------------------------------------
// 监听鼠标锁定事件
//------------------------------------
function pointerLockChange() {
	if (document.mozPointerLockElement === elem ||
		document.webkitPointerLockElement === elem) {
		console.log("指针锁定成功了。");
	} else {
		console.log("指针锁定已丢失。");
	}
}

//------------------------------------
// 监听锁定指针错误
//------------------------------------
function pointerLockError() {
	console.log("锁定指针时出错。");
}

function get_instruction() {
	
}