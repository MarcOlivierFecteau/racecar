const debug = false; // User-defined: for debug information in console
const MAX_STEERING_INPUT = 40.0 * 3.141592 / 180.0;
const cameraTopic = "/racecar/raspicam_node/image";

var rosbridgeServer = null;
var remoteUser = localStorage.getItem("remoteUser");
var remoteIPv4 = localStorage.getItem("remoteIPv4");
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
        connectionStatus.innerHTML = `${remoteUser} @ ${remoteIPv4}`;

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

        window.location.href = "connection.html";
    });

    rosbridgeServer.on("close", () => {
        console.log("[ROSBridge] Closed connection to WebSocket server.");
        remoteIPv4 = null;

        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#ff0000";
        connectionStatus.innerHTML = "Disconnected";
        document.getElementById("connection-status-icon").src = "media/images/red_circle.svg";
        document.getElementById("disconnect-button").setAttribute("disabled", "yes");

        window.location.href = "connection.html";
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
    statusBuffer.value = "";
    statusBuffer.value += "EMERGENCY STOP!\n";
}

function resetInputs() {
    inputsDisabled = false;
    emergencyStop.removeAttribute("style");
    if(debug) {console.log("Emergency stop released.");}
    statusBuffer.value = "";
    statusBuffer.value += "Reset.\n";
}

/* ========================================================================
 * Keyboard Events
 * ======================================================================== */

const statusBuffer = document.getElementById("status-buffer");

// Disable scrolling with arrow keys and space
window.addEventListener("keydown", (event) => {
    statusBuffer.scrollTop = statusBuffer.scrollHeight;
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
            if(debug) {console.log("Going forward.\n");}
            statusBuffer.value += "Going forward.\n";
            break;
        case "ArrowLeft":
        case "a":
            twist.angular.z = MAX_STEERING_INPUT;
            dPadLeft.style.backgroundColor = "#686c74";
            if(debug) {console.log("Going left.\n");}
            statusBuffer.value += "Going left.\n";
            break;
        case "ArrowDown":
        case "s":
            twist.linear.x = -2.5;
            dPadDown.style.backgroundColor = "#686c74";
            if(debug) {console.log("Going backward.\n");}
            statusBuffer.value += "Going backward.\n";
            break;
        case "ArrowRight":
        case "d":
            twist.angular.z = -MAX_STEERING_INPUT;
            dPadRight.style.backgroundColor = "#686c74";
            if(debug) {console.log("Going right.\n");}
            statusBuffer.value += "Going right.\n";
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
            if(debug) {console.log("Stopped forward.\n");}
            statusBuffer.value += "Stopped forward.\n";
            break;
        case "ArrowLeft":
        case "a":
            twist.angular.z = 0.0;
            dPadLeft.removeAttribute("style");
            if(debug) {console.log("Stopped left.\n");}
            statusBuffer.value += "Stopped left.\n";
            break;
        case "ArrowDown":
        case "s":
            twist.linear.x = 0.0;
            dPadDown.removeAttribute("style");
            if(debug) {console.log("Stopped backward.\n");}
            statusBuffer.value += "Stopped backward.\n";
            break;
        case "ArrowRight":
        case "d":
            twist.angular.z = 0.0;
            dPadRight.removeAttribute("style");
            if(debug) {console.log("Stopped right.\n");}
            statusBuffer.value += "Stopped right.\n";
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
    statusBuffer.value += "Going forward.\n";
});
dPadUp.addEventListener("mouseup", () => {
    twist.linear.x = 0.0;
    dPadUp.removeAttribute("style");
    statusBuffer.value += "Stopped forward.\n";
});
dPadUp.addEventListener("mouseleave", () => {
    twist.linear.x = 0.0;
    dPadUp.removeAttribute("style");
    statusBuffer.value += "Stopped forward.\n";
});

dPadLeft.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.angular.z = MAX_STEERING_INPUT;
    dPadLeft.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going left.\n";
});
dPadLeft.addEventListener("mouseup", () => {
    twist.angular.z = 0.0;
    dPadLeft.removeAttribute("style");
    statusBuffer.value += "Stopped left.\n";
});
dPadLeft.addEventListener("mouseleave", () => {
    twist.angular.z = 0.0;
    dPadLeft.removeAttribute("style");
    statusBuffer.value += "Stopped left.\n";
});

dPadDown.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.linear.x = -2.5;
    dPadDown.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going backward.\n";
});
dPadDown.addEventListener("mouseup", () => {
    twist.linear.x = 0.0;
    dPadDown.removeAttribute("style");
    statusBuffer.value += "Stopped backward.\n";
});
dPadDown.addEventListener("mouseleave", () => {
    twist.linear.x = 0.0;
    dPadDown.removeAttribute("style");
    statusBuffer.value += "Stopped backward.\n";
});

dPadRight.addEventListener("mousedown", () => {
    if(inputsDisabled) return;
    twist.angular.z = -MAX_STEERING_INPUT;
    dPadRight.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going right.\n";
});
dPadRight.addEventListener("mouseup", () => {
    twist.angular.z = 0.0;
    dPadRight.removeAttribute("style");
    statusBuffer.value += "Stopped right.\n";
});
dPadRight.addEventListener("mouseleave", () => {
    twist.angular.z = 0.0;
    dPadRight.removeAttribute("style");
    statusBuffer.value += "Stopped right.\n";
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
    statusBuffer.value += "Going forward.\n";
});
dPadUp.addEventListener("touchend", () => {
    twist.linear.x = 0;
    dPadUp.removeAttribute("style");
    statusBuffer.value += "Stopped forward.\n";
});

dPadLeft.addEventListener("touchstart", () => {
    twist.angular.z = MAX_STEERING_INPUT;
    dPadLeft.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going left.\n";
});
dPadLeft.addEventListener("touchend", () => {
    twist.angular.z = 0;
    dPadLeft.removeAttribute("style");
    statusBuffer.value += "Stopped left.\n";
});

dPadDown.addEventListener("touchstart", () => {
    twist.linear.x = -2.5;
    dPadDown.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going backward.\n";
});
dPadDown.addEventListener("touchend", () => {
    twist.linear.x = 0;
    dPadDown.removeAttribute("style");
    statusBuffer.value += "Stopped backward.\n";
});

dPadRight.addEventListener("touchstart", () => {
    twist.angular.z = -MAX_STEERING_INPUT;
    dPadRight.style.backgroundColor = "#686c74";
    statusBuffer.value += "Going right.\n";
});
dPadRight.addEventListener("touchend", () => {
    twist.angular.z = 0;
    dPadRight.removeAttribute("style");
    statusBuffer.value += "Stopped right.\n";
});
