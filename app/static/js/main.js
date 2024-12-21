// static/js/main.js

console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

let socket;

function initializeWebSocket() {
    socket = new WebSocket('ws://192.168.1.177:9001');

    // Handle connection events
    socket.onopen = () => {
        console.log('WebSocket connection established');
        updateConnectionStatus(true);
    };

    socket.onmessage = (event) => {
        console.log('Message from server:', event.data);
    };

    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        updateConnectionStatus(false);
    };

    socket.onclose = () => {
        console.log('WebSocket connection closed');
        updateConnectionStatus(false);
        scheduleReconnect(); // Schedule a reconnect attempt
    };
}

function scheduleReconnect() {
    console.log('Attempting to reconnect in 5 seconds...');
    setTimeout(() => {
        if (socket.readyState === WebSocket.CLOSED) {
            console.log('Reconnecting...');
            initializeWebSocket();
        }
    }, 5000);
}

initializeWebSocket();

// Initialize Left Joystick (Rotation)
const leftJoystick = new VirtualJoystick({
    container: document.body,
    strokeStyle: 'cyan',
    limitStickTravel: true,
    stickRadius: 120,
    mouseSupport: true

});

// Event Listener: Validate touch start (only left half)
leftJoystick.addEventListener('touchStartValidation', function(event){
    const touch = event.changedTouches[0];
    return touch.pageX < window.innerWidth / 2;
});

// Event Listener: Joystick started
leftJoystick.addEventListener('touchStart', function(){
    console.log('Left Joystick Started');
    updateStatus('Rotating');
});

// Event Listener: Joystick ended
leftJoystick.addEventListener('touchEnd', function(){
    console.log('Left Joystick Ended');
    updateStatus('Idle');
    sendI2CCommand({ "Y": { "s": 0, "o": null } }); // Stop rotation
});

// Event Listener: Joystick moved
leftJoystick.addEventListener('touchMove', function(){
    const speed = calculateYawSpeed(leftJoystick.deltaX(), leftJoystick.deltaY());
    const orientation = null; // Optional parameter; set if needed

    console.log('Left Joystick moved:', speed);

    // Send Rotation Command
    sendI2CCommand({ "Y": { "s": speed, "o": orientation } });
});

// Initialize Right Joystick (Translation/Strafe)
const rightJoystick = new VirtualJoystick({
    container: document.body,
    strokeStyle: 'orange',
    limitStickTravel: true,
    stickRadius: 60,
});

// Event Listener: Validate touch start (only right half)
rightJoystick.addEventListener('touchStartValidation', function(event){
    const touch = event.changedTouches[0];
    return touch.pageX >= window.innerWidth / 2;
});

// Event Listener: Joystick started
rightJoystick.addEventListener('touchStart', function(){
    console.log('Right Joystick Started');
    updateStatus('Translating');
});

// Event Listener: Joystick ended
rightJoystick.addEventListener('touchEnd', function(){
    console.log('Right Joystick Ended');
    updateStatus('Idle');
    sendI2CCommand({ "T": { "d": 0, "s": 0 } }); // Stop translation
});

// Event Listener: Joystick moved
rightJoystick.addEventListener('touchMove', function(){
    const { speed, direction } = calculateMove(rightJoystick.deltaX(), rightJoystick.deltaY(), 60);
    console.log('Right Joystick moved:', direction, speed);

    // Send Translation Command
    sendI2CCommand({ "T": { "d": direction, "s": speed } });
});

// Function to send I2CCommand
function sendI2CCommand(commandObj) {
    const jsonCommand = JSON.stringify(commandObj);
    console.log('Attempting to send:', jsonCommand); // Log every attempt

    if (socket.readyState === WebSocket.OPEN) {
        socket.send(jsonCommand);
        console.log('Sent:', jsonCommand);
    } else {
        console.warn('WebSocket is not open. Cannot send message:', jsonCommand);
    }
}

// Function to update the status display
function updateStatus(status) {
    const statusDisplay = document.getElementById('status');
    if (statusDisplay) {
        statusDisplay.textContent = `Status: ${status}`;
    }
}

function updateConnectionStatus(isConnected) {
    const statusCircle = document.getElementById('connection-status');
    if (isConnected) {
    statusCircle.style.backgroundColor = 'green';
    } else {
    statusCircle.style.backgroundColor = 'red';
    }
  }

function calculateYawSpeed(deltaX, deltaY) {
    // Calculate magnitude using Pythagorean theorem
    const magnitude = Math.sqrt(deltaX ** 2 + deltaY ** 2);

    // Normalize magnitude to range [0, 1]
    const normalizedMagnitude = Math.min(1, magnitude / 120); // Assuming 120 is the max joystick radius

    // Determine direction (sign) based on deltaX
    const sign = deltaX >= 0 ? 1 : -1;

    // Return signed normalized speed, capped between -1 and 1
    return sign * normalizedMagnitude;
}

function calculateMove(deltaX, deltaY) {
    // Calculate magnitude using Pythagorean theorem
    const magnitude = Math.sqrt(deltaX ** 2 + deltaY ** 2);

    // Normalize magnitude to range [-1, 1]
    const normalizedMagnitude = Math.min(1, magnitude / 60);

    // Calculate angle in degrees
    const angle = Math.atan2(deltaY, deltaX) * (180 / Math.PI);

    // Return capped magnitude and angle
    return {
        speed: normalizedMagnitude.toFixed(2),
        direction: (angle < 0 ? angle + 360 : angle).toFixed(0) // Ensure angle is in range [0, 360]
    };
}

setInterval(function() {
    var outputEl = document.getElementById('result');

    const yawSpeed = calculateYawSpeed(leftJoystick.deltaX(), leftJoystick.deltaY());
    const { speed, direction } = calculateMove(rightJoystick.deltaX(), rightJoystick.deltaY(), 60); // Adjust 60 if max radius differs


    outputEl.innerHTML =
        '<b>Yaw:</b> ' +
        + yawSpeed.toFixed(2) + // Display signed magnitude
        ' (Direction: ' + (yawSpeed > 0 ? 'Clockwise' : 'Counterclockwise') + ')' +
        '<br>' + // Line break between joysticks
        '<b>Move:</b> ' +
        ' Speed:' + speed +
        ' Direction:' + direction
}, 1 / 30 * 1000);

        document.addEventListener('touchmove', function(e) {
            e.preventDefault();
        }, { passive: false });
