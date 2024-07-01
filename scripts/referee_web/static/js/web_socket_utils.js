//------------------------------------
// 初始化连接
//------------------------------------
function init_socket() {
    window.socket = io();
    socket.on('connect', function () {
        console.log('connect')

        socket.on('robot_status', function (msg) {
            // console.log(msg)
            let robot_name = msg.robot_name
            let hpbar = $(`#${robot_name}_hpbar`)
            let hit_projectiles = $(`#${robot_name}_hit_projectiles`)
            let hit_rate = $(`#${robot_name}_hit_rate`)
            let remain_projectiles = $(`#${robot_name}_remain_projectiles`)
            let used_projectiles = $(`#${robot_name}_used_projectiles`)
            let hp_text = $(`#${robot_name}_hp_text`)
            hpbar.css("width", `${msg.remain_hp / msg.max_hp * 100}%`)
            hp_text.text(msg.remain_hp + '')
            hit_projectiles.text(msg.hit_projectiles)
            hit_rate.text((msg.hit_projectiles/msg.used_projectiles).toFixed(2))
            remain_projectiles.text(msg.total_projectiles - msg.used_projectiles)
            used_projectiles.text(msg.used_projectiles)

        });

    });
    socket.on('robot_names', function (message) {
        let robot_names = message.list
        
        window.robot_names = [...robot_names]
        for (let i = 0; i < robot_names.length; i++) {
            console.log(robot_names[i])
            let color = robot_names[i].split("_")[0]
            let block = color+'_block';
            if(i == 0){
                console.log('#'+block)
                $('#'+block).empty()
            }
            $('#'+block).append(`
            <div class="robo_container colcenter shadow">
                <div class="uk-label colstart ${color=='red'?'uk-label-danger':''}" style="margin-bottom: 10px; padding: 5px;"><span
                        style="font-size: 13px;margin-bottom: 5px;">${robot_names[i]}</span>
                </div>
                <div class="rowstart" style="width: 100%;margin-bottom: 10px;margin-right: 10px;height:12%">
                    <div class="rowcenter badge bg-light text-dark" style=" height:100%; width: 17%;font-size: 14px">HP
                    </div>
                    <div class="progress" style="width: 100%;height:100%">
                        <div class="progress-bar bg-danger" role="progressbar" style="width: 0%;"
                            id="${robot_names[i]}_hpbar">
                            <div id="${robot_names[i]}_hp_text">未初始化</div>
                        </div>
                    </div>
                </div>
                <div class="rowcenter" style="width: 100%;height:10%">
                    <div class="rowstart" style="width: 50%;margin-bottom: 10px;margin-right: 10px;height:100%">
                        <div class="rowcenter badge bg-light text-dark" style=" height:100%;font-size: 14px">命中数
                        </div>
                        <div class="badge bg-dark" style=" height:100%;font-size: 14px" id="${robot_names[i]}_hit_projectiles">未初始化</div>
                    </div>
                    <div class="rowstart" style="width: 50%;margin-bottom: 10px;margin-right: 10px;height:100%">
                        <div class="rowcenter badge bg-light text-dark" style=" height:100%;font-size: 14px">命中率
                        </div>
                        <div class="badge bg-dark" style=" height:100%;font-size: 14px" id="${robot_names[i]}_hit_rate">未初始化</div>
                    </div>
                 </div>
                <div class="rowcenter" style="width: 100%;height:10%">
                
                    <div class="rowstart" style="width: 50%;margin-bottom: 10px;margin-right: 10px;height:100%">
                        <div class="rowcenter badge bg-light text-dark" style=" height:100%; font-size: 14px">弹药量
                        </div>
                        <div class="badge bg-dark" style=" height:100%;font-size: 14px" id="${robot_names[i]}_remain_projectiles">未初始化</div>
                    </div>
                    <div class="rowstart" style="width: 50%;margin-bottom: 10px;margin-right: 10px;height:100%">
                        <div class="rowcenter badge bg-light text-dark" style=" height:100%; font-size: 14px">弹药量
                        </div>
                        <div class="badge bg-dark" style=" height:100%;font-size: 14px" id="${robot_names[i]}_used_projectiles">未初始化</div>
                    </div>
                </div>
                <button type="button" class="btn btn-primary" style="margin-bottom: 10px;width: 100%;height:13%;"
                    onclick="revive('${robot_names[i]}')">复活</button>
                <button type="button" class="btn btn-danger" style="margin-bottom: 10px;width: 100%;height:13%;"
                    onclick="kill('${robot_names[i]}')">罚下</button>
            </div>
        
            
            `)
            // $('#choose_robot_container').append(`
            // <div class="ch_box"  onclick="select_robot(this,'${robot_names[i]}',${chosen_robot[robot_names[i]]===undefined || chosen_robot[robot_names[i]]===false})">
            // <img src="static/img/robot.jpg" class="ch_img">
            // <span class="badge bg-${chosen_robot[robot_names[i]]===undefined || chosen_robot[robot_names[i]]===false?"success":"secondary"}">${chosen_robot[robot_names[i]]===undefined || chosen_robot[robot_names[i]]===false?"可选":"不可选"}</span>
            // <h5 class="card-title">${robot_names[i]}</h5>
            // </div>
            // `)
        }

    });
    socket.on('disconnect', function () {
        console.log('disconnect')
    });

}

function revive(rob_name){
    let map = {
        'robot_name': rob_name,
        'instruction': 'REVIVE_ROBOT'
    }
    socket.emit('control', map);
    console.log('revive:'+rob_name)
    
}

function kill(rob_name){
    let map = {
        'robot_name': rob_name,
        'instruction': 'KILL_ROBOT'
    }
    socket.emit('control', map);
    console.log('kill:'+rob_name)
}

function start_game(){
    let map = {
        'instruction': 'START_GAME'
    }
    socket.emit('control', map);
    console.log('start game!')
}

function stop_game(){
    let map = {
        'instruction': 'STOP_GAME'
    }
    socket.emit('control', map);
    console.log('stop game!')
}

function reset_game(){
    let map = {
        'instruction': 'SELF_CHECKING'
    }
    socket.emit('control', map);
    console.log('reset game!')
}

//------------------------------------
// 处理msg
//------------------------------------



