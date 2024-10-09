var rosbridgeServer = null;
var cmdVelocityTopic = null;
var remoteIPv4 = null;

const config = {
    serverBaseUrl: "http://127.0.0.1",
    connectEndpoint: "/api/connect",
    cameraTopic: "/racecar/raspicam_node/image"
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

// $(document).ready(() => {
//     console.log("[WEB INTERFACE] Ready.")
// })

function connectROS() {
    rosbridgeServer = new ROSLIB.Ros({
        url: "ws://" + remoteIPv4 + ":9090"
    });

    rosbridgeServer.on("connection", () => {
        console.log("[ROSBridge] Connected to WebSocket server.");

        cmdVelocityTopic = new ROSLIB.Topic({
            ros: rosbridgeServer,
            name: "/racecar/cmd_vel",
            messageType: "/geometrry_msgs/Twist"
        });
    });

    rosbridgeServer.on("error", (error) => {
        console.error("[ROSBridge] Error connecting to WebSocket server: ", error);
    });

    rosbridgeServer.on("close", () => {
        console.log("[ROSBridge] Closed connection to WebSocket server.");
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

    const ipv4 = document.getElementById("remote-ipv4-field").value.trim();

    if(!validateIPv4(ipv4)) {
        alert("Enter a valid IPv4 address.");
        return;
    }

    document.getElementById("camera-feed").src = "media/images/camera-not-available.svg";
}

document.getElementById("connect-form").addEventListener("submit", handleConnectForm);
