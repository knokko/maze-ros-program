var WebSocketClient = require('websocket').client;
const fs = require('fs');

var client = new WebSocketClient();

client.on('connectFailed', function(error) {
    console.log('Connect Error: ' + error.toString());
});

client.on('connect', function(connection) {
    console.log('WebSocket Client Connected');
    connection.on('error', function(error) {
        console.log("Connection Error: " + error.toString());
    });
    connection.on('close', function() {
        console.log('echo-protocol Connection Closed');
    });

    let counter = 1;
    connection.on('message', function(message) {
        if (message.type === 'utf8') {
            const response = JSON.parse(message.utf8Data).msg.data;
            const binaryFrame = Buffer.from(response, 'base64');
            fs.writeFile('frames/frame' + counter++ + '.jpg', binaryFrame, error => error && console.log('error writing file:', error));
        }
    });
    connection.sendUTF('{"op":"subscribe","id":"subscribe:/db4/camera_node/image/compressed:4","type":"sensor_msgs/CompressedImage","topic":"/db4/camera_node/image/compressed","compression":"none","throttle_rate":100,"queue_length":0}');
    // connection.sendUTF('{"op":"subscribe","id":"subscribe:/db4/maze_detection/image/compressed:4","type":"sensor_msgs/CompressedImage","topic":"/db4/maze_detection/image/compressed","compression":"none","throttle_rate":100,"queue_length":0}');
});

client.connect('ws://db4.local:9001/');