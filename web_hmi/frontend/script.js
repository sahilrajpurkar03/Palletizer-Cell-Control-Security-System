// Connect to WebSocket
const socket = new WebSocket(`ws://${window.location.host}/ws`);

// DOM Elements
const doorStatus = document.getElementById('door-status');
const eButtonStatus = document.getElementById('e-button-status');
const stackLight = document.getElementById('stack-light');
const barcodeDisplay = document.getElementById('barcode');
const lastRequest = document.getElementById('last-request');
const lastResponse = document.getElementById('last-response');

// WebSocket Event Handlers
socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    // Update UI
    doorStatus.textContent = data.door_closed ? "CLOSED" : "OPEN";
    doorStatus.className = data.door_closed ? "status-indicator on" : "status-indicator off";
    
    eButtonStatus.textContent = data.e_button_pressed ? "PRESSED" : "INACTIVE";
    eButtonStatus.className = data.e_button_pressed ? "status-indicator off" : "status-indicator on";

    document.getElementById("last-request").textContent = JSON.stringify(data.last_request, null, 2);
    document.getElementById("last-response").textContent = JSON.stringify(data.last_response, null, 2);
    
    // Stack light colors
    stackLight.className = "light-circle ";
    if (data.stack_light === 0) stackLight.classList.add("green");
    else if (data.stack_light === 1) stackLight.classList.add("yellow");
    else stackLight.classList.add("red");
    
    barcodeDisplay.textContent = data.barcode;
    
    if (data.last_request) {
        lastRequest.textContent = JSON.stringify(data.last_request, null, 2);
    }
    if (data.last_response) {
        lastResponse.textContent = JSON.stringify(data.last_response, null, 2);
    }
};

// Manual control buttons
document.getElementById('toggle-door').addEventListener('click', () => {
    fetch('/toggle_door', { method: 'POST' });
});

document.getElementById('trigger-emergency').addEventListener('click', () => {
    fetch('/trigger_emergency', { method: 'POST' });
});

document.getElementById('reset-emergency').addEventListener('click', () => {
    fetch('/reset_emergency', { method: 'POST' });
});