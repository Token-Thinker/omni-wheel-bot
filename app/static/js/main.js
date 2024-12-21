// static/js/main.js

console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");

// Initialize WebSocket connection to Rust backend
const socket = new WebSocket('ws://192.168.1.177:9001');

// Handle connection events
socket.onopen = () => {
    console.log('WebSocket connection established');
    updateStatus('Connected');
};

socket.onmessage = (event) => {
    console.log('Message from server:', event.data);
};

socket.onerror = (error) => {
    console.error('WebSocket error:', error);
};

socket.onclose = () => {
    console.log('WebSocket connection closed');
    updateStatus('Disconnected');
};

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
    const speed = leftJoystick.deltaY(); // Assuming deltaY controls rotation speed
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
    const direction = rightJoystick.deltaX();
    const speed = rightJoystick.deltaY();

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

setInterval(function(){
    var outputEl = document.getElementById('result');
    outputEl.innerHTML	= '<b>Result:</b> '
        + ' dx:'+leftJoystick.deltaX()
		+ ' dy:'+leftJoystick.deltaY()
		+ (leftJoystick.right()	? ' right'	: '')
		+ (leftJoystick.up()	? ' up'		: '')
		+ (leftJoystick.left()	? ' left'	: '')
		+ (leftJoystick.down()	? ' down' 	: '')
}, 1/30 * 1000);





// Optional: Adjust joystick positions on window resize
window.addEventListener('resize', function(){
    // Implement if necessary, e.g., reposition joysticks or adjust settings
});
