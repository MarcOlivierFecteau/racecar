var rosbridgeServer = null;
var cmdVelocityTopic = null;
var remoteIPv4 = null;

const config = {
    serverBaseUrl: "http://127.0.1.1",
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

$(document).ready(() => {
    console.log("[WEB INTERFACE] Ready.")
})

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

    const url = `${config.serverBaseUrl}${config.connectEndpoint}?ipv4=${encodeURIComponent(ipv4)}`;
    console.log(`[FORM HANDLER] Attempting to connect to: ${url}`);

    fetch(url, {
        method: "GET",
        headers: {
            "Access-Control-Allow-Origin": "http://127.0.1.1/",
            "Accept": "application/json",
        },
    })
    .then(response => {
        if(!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
        return response.text();
    })
    .then(text => {
        try {
            return JSON.parse(text);
        } catch(e) {
            console.error("[FORM HANDLER]", text);
            throw new Error("The server response was not valid JSON.");
        }
    })
    .then(data => {
        remoteIPv4 = data.ipv4;
        console.log("[FORM HANDLER] connected to" + remoteIPv4);
        connectROS();
        const cameraFeed = document.getElementById("camera-feed");
        cameraFeed.src = `http://${remoteIPv4}":8080/stream?topic=${config.cameraTopic}`;
    })
    .catch(error => {
        console.error("[FORM HANDLER]", error);
        alert("An error occured while connecting.")
    })
}

document.getElementById("connect-form").addEventListener("submit", handleConnectForm);
