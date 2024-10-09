var rosbridgeServer = null;
var remoteIPv4 = null;

const config = {
    serverBaseUrl: "http://127.0.0.1",
    cameraTopic: "/racecar/raspicam_node/image",
};

var twist = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});

function connectROS() {
    rosbridgeServer = new ROSLIB.Ros({
        url: "ws://" + remoteIPv4 + ":9090"
    });

    rosbridgeServer.on("connection", () => {
        console.log("[ROSBridge] Connected to WebSocket server.");
        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#00ff00";
        connectionStatus.innerHTML = remoteIPv4;
        document.getElementById("connection-status-icon").src = "media/images/green_circle.svg";
        document.getElementById("disconnect-button").removeAttribute("disabled");
        document.getElementById("camera-feed").src = `http://${remoteIPv4}:8080/stream?topic=${config.cameraTopic}`;

        cmdVelocityTopic = new ROSLIB.Topic({
            ros: rosbridgeServer,
            name: "/racecar/cmd_vel",
            messageType: "/geometry_msgs/Twist"
        });
    });

    rosbridgeServer.on("error", (error) => {
        console.error("[ROSBridge] Error connecting to WebSocket server: ", error);
        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#ff0000";
        connectionStatus.innerHTML = "Disconnected";
        document.getElementById("connection-status-icon").src = "media/images/red_circle.svg";
        document.getElementById("disconnect-button").setAttribute("disabled", "yes");
    });

    rosbridgeServer.on("close", () => {
        console.log("[ROSBridge] Closed connection to WebSocket server.");
        const connectionStatus = document.getElementById("connection-status");
        connectionStatus.style.color = "#ff0000";
        connectionStatus.innerHTML = "Disconnected";
        document.getElementById("connection-status-icon").src = "media/images/red_circle.svg";
        document.getElementById("disconnect-button").setAttribute("disabled", "yes");
    });
}

function disconnectROS() {
    if(rosbridgeServer != null) {
        document.getElementById("camera-feed").src = "media/images/camera-not-available.svg"
        rosbridgeServer.close();
    } else {
        console.warn("[ROSBridge] Server is already closed.");
    }
}

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
    } else {
        remoteIPv4 = ipv4;
    }

    connectROS();
}

document.getElementById("connect-form").addEventListener("submit", handleConnectForm);

/*
 * D-Pad section
 */

const dPadUp = document.getElementById("up");
const dPadLeft = document.getElementById("left");
const dPadDown = document.getElementById("down");
const dPadRight = document.getElementById("right");

document.addEventListener("keydown", (event) => {
    switch(event.key) {
        case "ArrowUp":
        case "w":
            dPadUp.style.backgroundColor = "#585858";
            console.log(`You pressed up.`);
            break;
        case "ArrowLeft":
        case "a":
            dPadLeft.style.backgroundColor = "#585858";
            console.log(`You pressed left.`);
            break;
        case "ArrowDown":
        case "s":
            dPadDown.style.backgroundColor = "#585858";
            console.log(`You pressed down.`);
            break;
        case "ArrowRight":
        case "d":
            dPadRight.style.backgroundColor = "#585858";
            console.log(`You pressed right.`);
            break;
    }
});

document.addEventListener("keyup", (event) => {
    switch(event.key) {
        case "ArrowUp":
        case "w":
            dPadUp.style.backgroundColor = "#484848";
            break;
        case "ArrowLeft":
        case "a":
            dPadLeft.style.backgroundColor = "#484848";
            break;
        case "ArrowDown":
        case "s":
            dPadDown.style.backgroundColor = "#484848";
            break;
        case "ArrowRight":
        case "d":
            dPadRight.style.backgroundColor = "#484848";
            break;
    }
});
