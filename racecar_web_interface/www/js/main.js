const debug = false; // User-defined: for debug information in console
const MAX_STEERING_INPUT = 40.0 * 3.141592 / 180.0;
const cameraTopic = "/racecar/raspicam_node/image";

var rosbridgeServer = null;
var remoteIPv4 = null;
var cmdVelocityTopic = null;

/* ================================================================================================================
 * rosbridge / roslibjs Section
 * ================================================================================================================ */

function connectROS() {
    rosbridgeServer = new ROSLIB.Ros({
        url: "ws://" + remoteIPv4 + ":9090"
    });

    rosbridgeServer.on("connection", () => {
        console.log("[ROSBridge] Connected to WebSocket server.");

        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#00ff00";
        connectionStatus.innerHTML = remoteIPv4;

        document.getElementById("dashboard").removeAttribute("style");
        document.getElementById("connect-form").style.display = "none";

        document.getElementById("connection-status-icon").src = "media/images/green_circle.svg";
        document.getElementById("disconnect-button").removeAttribute("disabled");
        
        const cameraFeed = document.getElementById("camera-feed");
        cameraFeed.setAttribute("width", "640");
        cameraFeed.src = `http://${remoteIPv4}:8080/stream?topic=${cameraTopic}`;

        cmdVelocityTopic = new ROSLIB.Topic({
            ros: rosbridgeServer,
            name: "/prop_cmd",
            messageType: "/geometry_msgs/Twist"
        });
        cmdVelocityTopic.advertise();
    });

    rosbridgeServer.on("error", (error) => {
        console.error("[ROSBridge] Error connecting to WebSocket server: ", error);
        remoteIPv4 = null;

        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#ff0000";
        connectionStatus.innerHTML = "Disconnected";
        document.getElementById("connection-status-icon").src = "media/images/red_circle.svg";
        document.getElementById("disconnect-button").setAttribute("disabled", "yes");

        document.getElementById("dashboard").style.display = "none";
        document.getElementById("connect-form").removeAttribute("style");
    });

    rosbridgeServer.on("close", () => {
        console.log("[ROSBridge] Closed connection to WebSocket server.");
        remoteIPv4 = null;

        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#ff0000";
        connectionStatus.innerHTML = "Disconnected";
        document.getElementById("connection-status-icon").src = "media/images/red_circle.svg";
        document.getElementById("disconnect-button").setAttribute("disabled", "yes");

        document.getElementById("dashboard").style.display = "none";
        document.getElementById("connect-form").removeAttribute("style");
    });
}

function disconnectROS() {
    if(rosbridgeServer != null) {
        rosbridgeServer.close();
        const cameraFeed = document.getElementById("camera-feed");
        cameraFeed.setAttribute("width", "480");
        cameraFeed.src = "media/images/camera-not-available.svg";
    } else {
        console.warn("[ROSBridge] Server is already closed.");
    }
    cmdVelocityTopic = null;
    if(debug) {console.log("cmdVelocityTopic set to null.");}
}

/* ================================================================================================================
 * Twist (message) Section
 * ================================================================================================================ */

var twist = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 1.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});

const twistLinearX = document.getElementById("twist-linear-x");
const twistLinearY = document.getElementById("twist-linear-y");
const twistLinearZ = document.getElementById("twist-linear-z");
const twistAngularX = document.getElementById("twist-angular-x");
const twistAngularY = document.getElementById("twist-angular-y");
const twistAngularZ = document.getElementById("twist-angular-z");

function roundTwist(x) {
    return Number.parseFloat(x).toFixed(3);
}

setInterval(() => {
    if(cmdVelocityTopic != null) {
        twistLinearX.innerHTML = `X: ${roundTwist(twist.linear.x)}`;
        twistLinearY.innerHTML = `Y: ${roundTwist(twist.linear.y)}`;
        twistLinearZ.innerHTML = `Z: ${roundTwist(twist.linear.z)}`;
        twistAngularX.innerHTML = `X: ${roundTwist(twist.angular.x)}`;
        twistAngularY.innerHTML = `Y: ${roundTwist(twist.angular.y)}`;
        twistAngularZ.innerHTML = `Z: ${roundTwist(twist.angular.z)}`;
        cmdVelocityTopic.publish(twist);
        if(debug) {console.log(twist);}
    }
}, 200);

/* ================================================================================================================
 * Form Handler Section
 * ================================================================================================================ */

function validateIPv4(ipv4) {
    const ipv4Pattern = new RegExp("^((25[0-5]|(2[0-4]|1[0-9]|[1-9]|)[0-9])(\.(?!$)|$)){4}$");
    return ipv4Pattern.test(ipv4);
}

function handleConnectForm(event) {
    event.preventDefault();

    if(rosbridgeServer != null && rosbridgeServer.isConnected) {
        alert("You MUST disconnect before connecting to another server.")
        return;
    }

    const ipv4 = document.getElementById("remote-ipv4-field").value.trim();

    if(!validateIPv4(ipv4)) {
        alert("Enter a valid IPv4 address.");
        return;
    } else if(rosbridgeServer != null && rosbridgeServer.isConnected) {
        alert("You MUST disconnect before connecting to another server.")
        return;
    } else {
        remoteIPv4 = ipv4;
        connectROS();
    }
}

document.getElementById("connect-form").addEventListener("submit", handleConnectForm);

/* ================================================================================================================
 * (Ergonomic) Controls Section
 * ================================================================================================================ */

const emergencyStop = document.getElementById("emergency-stop");
const dPadUp = document.getElementById("up");
const dPadLeft = document.getElementById("left");
const dPadDown = document.getElementById("down");
const dPadRight = document.getElementById("right");

var inputsDisabled = false;

function stopAll() {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    inputsDisabled = true;
    emergencyStop.style.backgroundColor = "#dd0000dd";
    for(let element of [dPadUp, dPadLeft, dPadDown, dPadRight]) {
        element.removeAttribute("style");
    }
    if(debug) {console.log("Emergency stop pressed.");}
}

function resetInputs() {
    inputsDisabled = false;
    emergencyStop.removeAttribute("style");
    if(debug) {console.log("Emergency stop released.");}
}

/* ========================================================================
 * Keyboard Events
 * ======================================================================== */

// Disable scrolling with arrow keys and space
window.addEventListener("keydown", (event) => {
    if(["Space","ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].indexOf(event.code) > -1) {
        event.preventDefault();
    }
}, false);

document.addEventListener("keydown", (event) => {
    if(event.key == " ") {
        stopAll();
        return;
    } else if(inputsDisabled) return;
    switch(event.key) {
        case " ":
        case "ArrowUp":
        case "w":
            twist.linear.x = 2.5;
            dPadUp.style.backgroundColor = "#686c74";
            if(debug) {console.log("You pressed up.");}
            break;
        case "ArrowLeft":
        case "a":
            twist.angular.z = MAX_STEERING_INPUT;
            dPadLeft.style.backgroundColor = "#686c74";
            if(debug) {console.log("You pressed left.");}
            break;
        case "ArrowDown":
        case "s":
            twist.linear.x = -2.5;
            dPadDown.style.backgroundColor = "#686c74";
            if(debug) {console.log("You pressed down.");}
            break;
        case "ArrowRight":
        case "d":
            twist.angular.z = -MAX_STEERING_INPUT;
            dPadRight.style.backgroundColor = "#686c74";
            if(debug) {console.log("You pressed right.");}
            break;
    }
});

document.addEventListener("keyup", (event) => {
    if(event.key == " ") {
        resetInputs();
        return;
    } else if(inputsDisabled) return;
    switch(event.key) {
        case "ArrowUp":
        case "w":
            twist.linear.x = 0.0;
            dPadUp.removeAttribute("style");
            if(debug) {console.log("You released up.");}
            break;
        case "ArrowLeft":
        case "a":
            twist.angular.z = 0.0;
            dPadLeft.removeAttribute("style");
            if(debug) {console.log("You released left.");}
            break;
        case "ArrowDown":
        case "s":
            twist.linear.x = 0.0;
            dPadDown.removeAttribute("style");
            if(debug) {console.log("You released down.");}
            break;
        case "ArrowRight":
        case "d":
            twist.angular.z = 0.0;
            dPadRight.removeAttribute("style");
            if(debug) {console.log("You released right.");}
            break;
    }
});

/* ========================================================================
 * Mouse Events
 * ======================================================================== */

emergencyStop.addEventListener("mousedown", () => {
    stopAll();
});
emergencyStop.addEventListener("mouseup", () => {
    resetInputs();
});
emergencyStop.addEventListener("mouseleave", () => {
    resetInputs();
});

dPadUp.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.linear.x = 2.5;
    dPadUp.style.backgroundColor = "#686c74";
});
dPadUp.addEventListener("mouseup", () => {
    twist.linear.x = 0.0;
    dPadUp.removeAttribute("style");
});
dPadUp.addEventListener("mouseleave", () => {
    twist.linear.x = 0.0;
    dPadUp.removeAttribute("style");
});

dPadLeft.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.angular.z = MAX_STEERING_INPUT;
    dPadLeft.style.backgroundColor = "#686c74";
});
dPadLeft.addEventListener("mouseup", () => {
    twist.angular.z = 0.0;
    dPadLeft.removeAttribute("style");
});
dPadLeft.addEventListener("mouseleave", () => {
    twist.angular.z = 0.0;
    dPadLeft.removeAttribute("style");
});

dPadDown.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.linear.x = -2.5;
    dPadDown.style.backgroundColor = "#686c74";
});
dPadDown.addEventListener("mouseup", () => {
    twist.linear.x = 0.0;
    dPadDown.removeAttribute("style");
});
dPadDown.addEventListener("mouseleave", () => {
    twist.linear.x = 0.0;
    dPadDown.removeAttribute("style");
});

dPadRight.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.angular.z = -MAX_STEERING_INPUT;
    dPadRight.style.backgroundColor = "#686c74";
});
dPadRight.addEventListener("mouseup", () => {
    twist.angular.z = 0.0;
    dPadRight.removeAttribute("style");
});
dPadRight.addEventListener("mouseleave", () => {
    twist.angular.z = 0.0;
    dPadRight.removeAttribute("style");
});

/* ========================================================================
 * Touch Events
 * ======================================================================== */

emergencyStop.addEventListener("touchstart", () => {
    stopAll();
});
emergencyStop.addEventListener("touchend", () => {
    resetInputs();
});

dPadUp.addEventListener("touchstart", () => {
    twist.linear.x = 2.5;
    dPadUp.style.backgroundColor = "#686c74";
});
dPadUp.addEventListener("touchend", () => {
    twist.linear.x = 0;
    dPadUp.removeAttribute("style");
});

dPadLeft.addEventListener("touchstart", () => {
    twist.angular.z = MAX_STEERING_INPUT;
    dPadLeft.style.backgroundColor = "#686c74";
});
dPadLeft.addEventListener("touchend", () => {
    twist.angular.z = 0;
    dPadLeft.removeAttribute("style");
});

dPadDown.addEventListener("touchstart", () => {
    twist.linear.x = -2.5;
    dPadDown.style.backgroundColor = "#686c74";
});
dPadDown.addEventListener("touchend", () => {
    twist.linear.x = 0;
    dPadDown.removeAttribute("style");
});

dPadRight.addEventListener("touchstart", () => {
    twist.angular.z = -MAX_STEERING_INPUT;
    dPadRight.style.backgroundColor = "#686c74";
});
dPadRight.addEventListener("touchend", () => {
    twist.angular.z = 0;
    dPadRight.removeAttribute("style");
});
