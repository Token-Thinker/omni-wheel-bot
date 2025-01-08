// =======================================
// Disable text highlighting via CSS
// (You can place this in your main CSS file, too)
document.addEventListener('DOMContentLoaded', () => {
  document.body.style.userSelect = 'none';           // Standard
  document.body.style.webkitUserSelect = 'none';     // Chrome/Safari
  document.body.style.MozUserSelect = 'none';        // Firefox
  document.body.style.msUserSelect = 'none';         // IE/Edge
});

// =======================================
// Initialization and Configuration
// =======================================
console.log(
  "Touchscreen is",
  VirtualJoystick.touchScreenAvailable() ? "available" : "not available"
);

// WebSocket Configuration
const WS_URL = 'ws://192.168.1.194:88/ws';
const RECONNECT_INTERVAL = 15000; // 5 seconds
const SEND_INTERVAL = 1000 / 30; // ~30Hz

// =======================================
// WebSocket Management
// =======================================
class WebSocketManager {
  constructor(url) {
    this.url = url;
    this.socket = null;
    this.initializeWebSocket();
  }

  initializeWebSocket() {
    this.socket = new WebSocket(this.url, ["messages"]);

    this.socket.onopen = () => {
      console.log('WebSocket connection established');
      this.updateConnectionStatus(true);
    };

    this.socket.onmessage = (event) => {
      console.log('Message from server:', event.data);
    };

    this.socket.onerror = (error) => {
      console.error('WebSocket error:', error);
      this.updateConnectionStatus(false);
    };

    this.socket.onclose = () => {
      console.log('WebSocket connection closed');
      this.updateConnectionStatus(false);
      this.scheduleReconnect();
    };
  }

  scheduleReconnect() {
    console.log(`Attempting to reconnect in ${RECONNECT_INTERVAL / 1000} seconds...`);
    setTimeout(() => {
      if (this.socket.readyState === WebSocket.CLOSED) {
        console.log('Reconnecting...');
        this.initializeWebSocket();
      }
    }, RECONNECT_INTERVAL);
  }

  sendCommand(commandObj) {
    const jsonCommand = JSON.stringify(commandObj);
    console.log('Attempting to send:', jsonCommand);

    if (this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(jsonCommand);
      console.log('Sent:', jsonCommand);
    } else {
      console.warn('WebSocket is not open. Cannot send message:', jsonCommand);
    }
  }

  updateConnectionStatus(isConnected) {
    const statusCircle = document.getElementById('connection-status');
    if (statusCircle) {
      statusCircle.style.backgroundColor = isConnected ? 'green' : 'red';
    }
  }
}

const wsManager = new WebSocketManager(WS_URL);

// =======================================
// Joystick Management
// =======================================

// A global flag so we know if user is actively dragging a joystick
window.joystickActive = false;

/**
 * Modified JoystickController:
 * - We don't immediately set `joystickActive = true` on `touchStart`.
 * - Instead, we wait until the user moves beyond a small threshold in `onTouchMove`.
 *   That way, quick taps won't block the global tap/double-tap listeners.
 */
class JoystickController {
  constructor(options) {
    this.joystick = new VirtualJoystick(options);
    this.initializeEvents();
    this.dragging = false; // track actual movement beyond threshold
    this.startX = 0;
    this.startY = 0;
    this.ACTIVATE_THRESHOLD = 20; // px: how far user must move to activate joystick
  }

  initializeEvents() {
    this.joystick.addEventListener('touchStartValidation', this.validateTouchStart.bind(this));
    this.joystick.addEventListener('touchStart', this.onTouchStart.bind(this));
    this.joystick.addEventListener('touchEnd', this.onTouchEnd.bind(this));
    this.joystick.addEventListener('touchMove', this.onTouchMove.bind(this));
  }

  // To be overridden by subclasses
  validateTouchStart(event) {}

  onTouchStart(event) {
    const touch = event.changedTouches ? event.changedTouches[0] : event;
    this.startX = touch.pageX;
    this.startY = touch.pageY;
    this.dragging = false; // reset
  }

  onTouchEnd() {
    // Only if we were actually dragging do we turn off joystickActive
    if (this.dragging) {
      window.joystickActive = false;
    }
    this.dragging = false;
  }

  onTouchMove(event) {
    // If we haven't activated yet, check how far the user moved
    if (!this.dragging) {
      const touch = event.changedTouches ? event.changedTouches[0] : event;
      const dx = touch.pageX - this.startX;
      const dy = touch.pageY - this.startY;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist > this.ACTIVATE_THRESHOLD) {
        this.dragging = true;
        window.joystickActive = true;
      }
    }
    // Normal joystick logic continues
  }
}

// =======================================
// Left Joystick: Rotation
// =======================================
class LeftJoystick extends JoystickController {
  constructor() {
    super({
      container: document.body,
      strokeStyle: 'cyan',
      limitStickTravel: true,
      stickRadius: 120,
      mouseSupport: false,
    });
    this.previousCommand = { s: null, o: null };
  }

  validateTouchStart(event) {
    const touch = event.changedTouches[0];
    // Only activate left joystick if pressing on the left half
    return touch.pageX < window.innerWidth / 2;
  }

  onTouchEnd() {
    super.onTouchEnd();
    // If we were actually dragging, then send stop command
    if (this.dragging) {
      console.log('Left Joystick Ended');
      wsManager.sendCommand({
        ct: "i",
        ic: "y",
        s: 0,
        o: null
      });
    }
  }

  getCommand() {
    const speed = parseFloat(
      calculateYawSpeed(this.joystick.deltaX(), this.joystick.deltaY())
    );
    const orientation = null; // optional
    return {
      ct: "i",
      ic: "y",
      s: speed,
      o: orientation
    };
  }
}

// =======================================
// Right Joystick: Translation/Strafe
// =======================================
class RightJoystick extends JoystickController {
  constructor() {
    super({
      container: document.body,
      strokeStyle: 'orange',
      limitStickTravel: true,
      stickRadius: 120,
      mouseSupport: true,
    });
    this.previousCommand = { d: null, s: null };
  }

  validateTouchStart(event) {
    const touch = event.changedTouches[0];
    // Only activate right joystick if pressing on the right half
    return touch.pageX >= window.innerWidth / 2;
  }

  onTouchEnd() {
    super.onTouchEnd();
    // If we were actually dragging, then send stop command
    if (this.dragging) {
      console.log('Right Joystick Ended');
      wsManager.sendCommand({
        ct: "i",
        ic: "t",
        d: 0,
        s: 0
      });
    }
  }

  getCommand() {
    const { speed, direction } = calculateMove(
      this.joystick.deltaX(),
      this.joystick.deltaY()
    );
    return {
      ct: "i",
      ic: "t",
      d: direction,
      s: speed
    };
  }
}

const leftJoystick = new LeftJoystick();
const rightJoystick = new RightJoystick();

// =======================================
// Utility Functions
// =======================================
function calculateYawSpeed(deltaX, deltaY) {
  const magnitude = Math.sqrt(deltaX ** 2 + deltaY ** 2);
  const normalizedMagnitude = Math.min(1, magnitude / 120);
  const sign = deltaX >= 0 ? 1 : -1;
  const yawSpeed = sign * normalizedMagnitude;
  return yawSpeed.toFixed(1);
}

function calculateMove(deltaX, deltaY) {
  const magnitude = Math.sqrt(deltaX ** 2 + deltaY ** 2);
  const normalizedMagnitude = Math.min(1, magnitude / 120);
  let angle = (Math.atan2(deltaY, deltaX) * (180 / Math.PI)) + 90;
  angle = (angle + 360) % 360;
  const speed = magnitude === 0 ? 0 : parseFloat(normalizedMagnitude.toFixed(2));
  const direction = magnitude === 0 ? 0 : Math.round(angle);
  return { speed, direction };
}

// =======================================
// Command Sending Logic
// =======================================

// We'll keep track of previous commands so we don't spam the server
let previousYCommand = { s: null, o: null };
let previousTCommand = { d: null, s: null };

// Select the yaw and move elements for display
const yawEl = document.querySelector('.yaw');
const moveEl = document.querySelector('.move');

// Send commands ~30x/sec
setInterval(() => {
  // 1) Get current commands from each joystick
  //    Left => rotation
  //    Right => translation
  const currentY = leftJoystick.getCommand();  // { ct, ic, s, o }
  const currentT = rightJoystick.getCommand(); // { ct, ic, d, s }


  // Update UI elements, if they exist
  if (yawEl) yawEl.textContent = currentY.s;
  if (moveEl) moveEl.textContent = `(${currentT.s}, ${currentT.d})`;

  // 2) Check if the Y command changed before sending
  //    (compare currentY.s/o to previousYCommand.s/o)
  if (
    currentY.s !== previousYCommand.s ||
    currentY.o !== previousYCommand.o
  ) {
    wsManager.sendCommand(currentY);
    previousYCommand = { ...currentY };
    console.log('Yaw Command:', currentY);
  }

  // 3) Check if the T command changed before sending
  //    (compare currentT.s/d to previousTCommand.s/d)
  if (
    currentT.d !== previousTCommand.d ||
    currentT.s !== previousTCommand.s
  ) {
    wsManager.sendCommand(currentT);
    previousTCommand = { ...currentT };
    console.log('Move Command:', currentT);
  }
}, SEND_INTERVAL);

/******************************************************
 *          DOUBLE-TAP & LONG-PRESS LOGIC
 ******************************************************/

// Thresholds
const DOUBLE_TAP_THRESHOLD = 300;   // ms between taps on same side
const LONG_PRESS_THRESHOLD = 1000;  // ms to hold for "POWER TOGGLE"
const MOVE_THRESHOLD = 20;          // px movement allowed before ignoring tap

let lastTapTimeLeft = 0;
let lastTapTimeRight = 0;
let isLightsOn = false;

// For the current touch
let startX = 0;
let startY = 0;
let touchStartTime = 0;
let touchSide = null;  // 'left' or 'right'
let touchMoved = false;
let longPressTimer = null;

/** onGlobalTouchStart */
function onGlobalTouchStart(event) {
  // If joystick is active, skip global gestures
  // (User is presumably dragging a joystick)
  if (window.joystickActive) return;

  const touch = event.changedTouches ? event.changedTouches[0] : event;
  startX = touch.pageX;
  startY = touch.pageY;
  touchStartTime = Date.now();
  touchMoved = false;

  // Determine side
  touchSide = (touch.pageX < window.innerWidth / 2) ? 'left' : 'right';

  // Start a timer for long press => power toggle
  longPressTimer = setTimeout(() => {
    if (touchSide === 'left') {
      console.log('Long press on LEFT => toggling power (left)!');
      wsManager.sendCommand({ POWER_LEFT: 'TOGGLE' });
    } else {
      console.log('Long press on RIGHT => toggling power (right)!');
      wsManager.sendCommand({ POWER_RIGHT: 'TOGGLE' });
    }
    longPressTimer = null; // reset
  }, LONG_PRESS_THRESHOLD);
}

/** onGlobalTouchMove */
function onGlobalTouchMove(event) {
  if (window.joystickActive) return;

  const touch = event.changedTouches ? event.changedTouches[0] : event;
  const deltaX = touch.pageX - startX;
  const deltaY = touch.pageY - startY;
  const dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

  if (dist > MOVE_THRESHOLD) {
    touchMoved = true;
    // Cancel the long-press timer if still active
    if (longPressTimer) {
      clearTimeout(longPressTimer);
      longPressTimer = null;
    }
  }
}

/** onGlobalTouchEnd */
function onGlobalTouchEnd(event) {
  if (window.joystickActive) return;

  // Cancel any remaining long-press timer
  if (longPressTimer) {
    clearTimeout(longPressTimer);
    longPressTimer = null;
  }

  // If user moved too far, ignore tap
  if (touchMoved) return;

  const now = Date.now();

  if (touchSide === 'left') {
    // Check if within double-tap threshold
    if (now - lastTapTimeLeft < DOUBLE_TAP_THRESHOLD) {
      console.log('Double tap on LEFT side! Toggling LED...');

      // Toggle logic
      if (isLightsOn) {
        // Turn LEDs off
        wsManager.sendCommand({
          ct: "l",
          lc: "off"
        });
        isLightsOn = false;
      } else {
        // Turn LEDs on
        wsManager.sendCommand({
          ct: "l",
          lc: "on"
        });
        isLightsOn = true;
      }
      lastTapTimeLeft = 0; // reset
    } else {
      lastTapTimeLeft = now;
    }
  } else {
    // Right side
    if (now - lastTapTimeRight < DOUBLE_TAP_THRESHOLD) {
      console.log('Double tap on RIGHT side!');
      wsManager.sendCommand({ LIGHTS_RIGHT: 'TOGGLE' });
      lastTapTimeRight = 0;
    } else {
      lastTapTimeRight = now;
    }
  }
}

// Attach Global Listeners
document.addEventListener('touchstart', onGlobalTouchStart, { passive: false });
document.addEventListener('touchmove', onGlobalTouchMove, { passive: false });
document.addEventListener('touchend', onGlobalTouchEnd, { passive: false });

// (Optional) For desktop testing with mouse:
document.addEventListener('mousedown', onGlobalTouchStart);
document.addEventListener('mousemove', onGlobalTouchMove);
document.addEventListener('mouseup', onGlobalTouchEnd);