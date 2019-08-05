"use strict";

$(function() {
    let CAMERA_NAMES = ["TOP", "FRONT", "LEFT"];
    let CAMERA_TOPICS = ['/head_camera/rgb/image_raw', '/rviz1/camera1/image', '/rviz1/camera2/image', '/rviz1/camera3/image'];
    let self = this;

    // Note: everything is string
    let parts = new Map();  // body part full name ---> body part id
    let actions = new Map();  // body part id ---> action full name
    let actionsAbbr = new Map();  // action full name ---> action ABBR

    let selectedId = "";  // id of the currently selected body part
    let selectedGrasp = "";  // currently selected grasp type
    let selectedAction = "";  // ABBR of the currently selected action
    let running = false;  // is the program running? (i.e. is "RUN" button clicked?)

    $(document).ready(function() {
        // ROS setup
        self.ros = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });
        self.ros.on('error', function(error) {
            document.getElementById("status_bar").innerHTML = 'Error connecting to websocket server.';
            console.log('Error connecting to websocket server.');
        });
        self.ros.on('connection', function () {
            document.getElementById("status_bar").innerHTML = "We are connected!";
            console.log("We are connected!");
        });
        self.ros.on('close', function(error) {
            document.getElementById("status_bar").innerHTML = 'We lost connection with ROS. All is lost';
            console.error('We lost connection with ROS. All is lost');
        });
        // ros topic
        self.serverRequest = new ROSLIB.Topic({
            ros: self.ros,
            name: 'web_app_request',
            messageType: 'limb_manipulation_msgs/WebAppRequest'
        });
        self.serverResponse = new ROSLIB.Topic({
            ros: self.ros,
            name: 'web_app_response',
            messageType: 'limb_manipulation_msgs/WebAppResponse'
        });
        self.serverResponse.subscribe(handleServerStatusResponse);

        // add camera views
        for (let i = 0; i < 3; i++) {
            let cameraDiv = document.createElement("div");
            cameraDiv.className = "camera_container";

            let title = document.createElement("p");
            title.innerHTML = CAMERA_NAMES[i];
            let camera = document.createElement("div");
            camera.id = "camera_view_div" + i;
            camera.className = "camera_view";

            cameraDiv.appendChild(title);
            cameraDiv.appendChild(camera);

            if (i == 0) {
                $("#camera_large").append(cameraDiv);
            } else {
                $("#camera_small").append(cameraDiv);
            }

            // add the video stream
            let cameraViewer = new MJPEGCANVAS.Viewer({
                divID : camera.id,
                host : 'localhost',
                width : 480,
                height : 480,
                topic : CAMERA_TOPICS[i]
            });
            let cameraSize = i == 0 ? "350px" : "150px";
            camera.firstChild.style.width = cameraSize;
            camera.firstChild.style.height = cameraSize;
        }
        
        // initial button states
        $("#action_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
        $("#record_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});

        // event handlers
        // buttons
        document.getElementById("gripper_btn").addEventListener("click", addOrRemoveSakeGripper);
        document.getElementById("record_btn").addEventListener("click", recordScene);
        document.getElementById("yes_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("no_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("sake_gripper_btn").addEventListener("click", releaseSakeGripper);

        document.getElementById("action_panel_btn").addEventListener("click", showPanel);
        document.getElementById("run_btn").addEventListener("click", run);
        document.getElementById("estop_btn").addEventListener("click", estop);

        document.getElementById("record_panel_btn").addEventListener("click", showPanel);
        document.getElementById("record_action_btn").addEventListener("click", recordAction);

        let graspDropdownLists = document.querySelectorAll(".grasp_dropdown_content a");
        for (let i = 0; i < graspDropdownLists.length; i++) {
            graspDropdownLists[i].addEventListener("click", makeGraspSelection);
        }

        let subGoBtns = document.querySelectorAll(".sub_go_btn");
        for (let i = 0; i < subGoBtns.length; i++) {
            subGoBtns[i].addEventListener("click", doSubAction);
        }

        // camera views
        let cameraViews = document.querySelectorAll(".camera_container");
        for (let i = 0; i < cameraViews.length; i++) {
            cameraViews[i].addEventListener("click", switchCamera);
        }
    });

    function publishRosMsg(type, args) {
        // disable all the buttons!!!
        $(':button:not(#estop_btn)').prop('disabled', true);
        // publish msg
        let msg = new ROSLIB.Message({
            type: type,
            args: args
        });
        self.serverRequest.publish(msg);
    };

    function addOrRemoveSakeGripper() {
        publishRosMsg(this.innerHTML.substring(0, 6).toLowerCase(), []);
        if (this.innerHTML == "Attach Gripper") {
            this.innerHTML = "Remove Gripper";
        } else {
            this.innerHTML = "Attach Gripper";
        }
    }

    function releaseSakeGripper() {
        publishRosMsg("release", []);
    }

    function recordScene() {
        $("#record_btn_confirm").css("display", "block");
    }

    function recordSceneConfirmed() {
        $("#record_btn_confirm").css("display", "none");
        let recordOcto = this.innerHTML == "Yes" ? "True" : "";
        publishRosMsg("record", [recordOcto]);
    }

    function recordAction() {
        if (this.innerHTML == "Record") {
            this.innerHTML = "Stop";
        } else {
            this.innerHTML = "Record";
            // TODO: ask to save action or not
        }
    }

    function showPanel() {
        if (this.innerHTML.substring(0, 6) == "Action") {
            $("#action_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
            $("#record_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});
            $("#action_step_container").css("display", "grid");
            $("#record_step_container").css("display", "none");
        } else {
            $("#action_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});
            $("#record_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
            $("#action_step_container").css("display", "none");
            $("#record_step_container").css("display", "grid");
        }
    }

    function switchCamera() {
        // remove the large view
        let selectedView = this.parentElement.querySelector(".camera_container");
        let largeView = document.getElementById("camera_large").querySelector(".camera_container");
        if (largeView != selectedView) {
            let largeSize = largeView.querySelector(".camera_view").firstChild.style.width;
            let smallSize = selectedView.querySelector(".camera_view").firstChild.style.width;
            $("#camera_large").remove(largeView);
            $("#camera_small").append(largeView);
            largeView.querySelector(".camera_view").firstChild.style.width = smallSize;
            largeView.querySelector(".camera_view").firstChild.style.height = smallSize;
            // add the selected view
            $("#camera_large").append(selectedView);
            $("#camera_small").remove(selectedView);
            selectedView.querySelector(".camera_view").firstChild.style.width = largeSize;
            selectedView.querySelector(".camera_view").firstChild.style.height = largeSize;
        }
    }

    function markDropdownSelection(selectedItem) {
        // mark the selection and return the selected item name
        selectedItem.parentElement.parentElement.querySelector(".dropbtn").innerHTML = selectedItem.innerHTML;
        return selectedItem.innerHTML;
    }

    function makeBodyPartSelection() {
        // mark the selection
        let itemName = markDropdownSelection(this);
        // get the item id
        let itemId = "";
        if (parts.has(itemName.toLowerCase())) {
            // a body part is selected
            itemId = parts.get(itemName.toLowerCase());
        }
        selectedId = itemId;
        // mark the selection in video stream
        publishRosMsg("prev_id", [selectedId]);
        // update available actions
        let actionDropdownLists = document.querySelectorAll(".action_dropdown_content");
        let availableActions = actions.get(itemId);
        for (let i = 0; i < actionDropdownLists.length; i++) {
            actionDropdownLists[i].innerHTML = "";
            if (selectedId != "") {
                for (let j = -1; j < availableActions.length; j++) {
                    // add DOM element
                    let entry = document.createElement("a");
                    entry.href = "#";
                    if (j == -1) {
                        entry.innerHTML = "Select an action";
                    } else {
                        let entryRaw = availableActions[j];
                        entry.innerHTML = entryRaw.length > 1 ? entryRaw.charAt(0).toUpperCase() + entryRaw.slice(1) : entryRaw;
                    }
                    actionDropdownLists[i].appendChild(entry);
                    // add event listener
                    entry.addEventListener("click", makeActionSelection);
                }
            }
        }
    }

    function makeActionSelection() {
        // mark the selection
        let actionName = markDropdownSelection(this).toLowerCase();
        // preview the action in video stream
        if (actionsAbbr.has(actionName)) {
            selectedAction = actionsAbbr.get(actionName);
            publishRosMsg("prev", [selectedAction, selectedId]);
        } else {
            selectedAction = "";
        }
    }

    function makeGraspSelection() {
        // mark the selection
        let graspType = markDropdownSelection(this).substring(0, 4);
        // record selected grasp type
        if (graspType == "Soft" || graspType == "Hard") {
            selectedGrasp = graspType[0].toLowerCase();
        } else {
            selectedGrasp = "";
        }
    }

    function doSubAction() {
        // get the action type
        let stepAction = this.parentElement.id.split("_")[0];
        if (stepAction == "go" && selectedId != "") {
            publishRosMsg("go", [selectedId]);
        } else if (stepAction == "grasp" && selectedId != "" && selectedGrasp != "") {
            publishRosMsg("grasp", [selectedGrasp]);
        } else if (stepAction == "action" && selectedId != "" && selectedGrasp != "" && selectedAction != "") {
            publishRosMsg("do", [selectedAction]);
        } else if (stepAction == "relax" || stepAction == "freeze") {
            publishRosMsg(stepAction, []);
        } else {
            console.log(stepAction);
        }
    }

    function run() {
        if (selectedId != "" && selectedGrasp != "" && selectedAction != "") {
            publishRosMsg("run", [selectedId, selectedGrasp, selectedAction]);
            running = true;
        } else {
            document.getElementById("status_bar").innerHTML = "Please make your selections for each step";
        }
    }

    function estop() {
        if (this.innerHTML == "STOP") {
            this.innerHTML = "RESET";
            publishRosMsg("stop", []);
        } else {
            this.innerHTML = "STOP";
            publishRosMsg("reset", []);
        }
    }

    function handleServerStatusResponse(msg) {
        if (msg.type == "run" && msg.msg == "DONE") {
            running = false;
        }
        if (msg.status && msg.type != "run" && !running) {
            // enable all the buttons
            $(':button').prop('disabled', false);
        }
        // show status
        if (msg.type == "attach" || msg.type == "remove" || msg.type == "record" || msg.type == "parts" || 
            msg.type == "actions" || msg.type == "prev" || msg.type == "prev_id" || msg.type == "reset" || 
            msg.type == "stop" || msg.type == "release" || msg.type == "run") {
            // main status bar
            document.getElementById("status_bar").innerHTML = msg.msg;
        } else {
            // status bar for each step
            let statusBars = document.querySelectorAll("." + msg.type + "_status");
            for (let i = 0; i < statusBars.length; i++) {
                statusBars[i].innerHTML = msg.msg;
            }
        }
        // process returned data
        if (msg.type == "parts" && msg.args.length > 0) {
            // add the list contents to dropdown menu
            let recordDropdownLists = document.querySelectorAll(".go_dropdown_content");
            parts.clear();
            for (let i = 0; i < recordDropdownLists.length; i++) {
                recordDropdownLists[i].innerHTML = "";
                for (let j = -1; j < msg.args.length; j++) {
                    // add DOM element
                    let entry = document.createElement("a");
                    entry.href = "#";
                    if (j == -1) {
                        entry.innerHTML = "Select a body part";
                    } else {
                        let msgArgs = msg.args[j].split(":");
                        // update body part information
                        if (i == 0) {
                            parts.set(msgArgs[1], msgArgs[0]);
                        }
                        let entryRaw = msgArgs[1];
                        entry.innerHTML = entryRaw.length > 1 ? entryRaw.charAt(0).toUpperCase() + entryRaw.slice(1) : entryRaw;
                    }
                    recordDropdownLists[i].appendChild(entry);
                    // add event listener
                    entry.addEventListener("click", makeBodyPartSelection);
                }
            }
        } else if (msg.type == "actions" && msg.args.length > 0) {
            // update action information
            for (let i = 0; i < msg.args.length; i++) {
                let msgArgs = msg.args[i].split(":");
                // update body part information
                if (!actions.has(msgArgs[0])) {
                    actions.set(msgArgs[0], []);
                }
                let value = actions.get(msgArgs[0]);
                value.push(msgArgs[1]);
                actions.set(msgArgs[0], value);
                actionsAbbr.set(msgArgs[1], msgArgs[2]);
            }
        }
    }

});