document.addEventListener('DOMContentLoaded', function() {
    const servoContainer = document.getElementById('servo-container');
    const socket = io();
    const servoCount = 6;
    let debounceTimers = {};

    // Initialize servo cards
    function initializeServoCards() {
        for (let i = 1; i <= servoCount; i++) {
            const servoCard = createServoCard(i);
            servoContainer.appendChild(servoCard);
        }
    }

    // Create a servo card element
    function createServoCard(servoId) {
        const card = document.createElement('div');
        card.className = 'servo-card';
        card.id = `servo-${servoId}`;

        const header = document.createElement('div');
        header.className = 'servo-header';

        const title = document.createElement('div');
        title.className = 'servo-title';
        title.textContent = getServoName(servoId);

        const idBadge = document.createElement('div');
        idBadge.className = 'servo-id';
        idBadge.textContent = `ID: ${servoId}`;

        header.appendChild(title);
        header.appendChild(idBadge);

        const info = document.createElement('div');
        info.className = 'servo-info';
        info.innerHTML = `
            <div class="info-item">
                <span class="info-label">Position</span>
                <span class="info-value" id="position-${servoId}">Loading...</span>
            </div>
            <div class="info-item">
                <span class="info-label">Temperature</span>
                <span class="info-value" id="temperature-${servoId}">Loading...</span>
            </div>
            <div class="info-item">
                <span class="info-label">Voltage</span>
                <span class="info-value" id="voltage-${servoId}">Loading...</span>
            </div>
            <div class="info-item">
                <span class="info-label">Speed</span>
                <span class="info-value" id="speed-${servoId}">Loading...</span>
            </div>
        `;

        const sliderContainer = document.createElement('div');
        sliderContainer.className = 'slider-container';

        const sliderLabel = document.createElement('div');
        sliderLabel.className = 'slider-label';
        sliderLabel.innerHTML = `
            <span id="min-${servoId}">Min</span>
            <span id="max-${servoId}">Max</span>
        `;

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.id = `slider-${servoId}`;
        slider.min = 0;  // Will be updated with actual values
        slider.max = 1000;  // Will be updated with actual values
        slider.value = 0;  // Will be updated with actual values

        slider.addEventListener('input', function() {
            const positionValue = document.getElementById(`position-${servoId}`);
            positionValue.textContent = slider.value;
        });

        slider.addEventListener('change', function() {
            updateServoPosition(servoId, parseInt(slider.value));
        });

        // Add debounced input event for smoother updates during sliding
        slider.addEventListener('input', function() {
            const value = parseInt(slider.value);
            
            // Clear previous timer
            if (debounceTimers[servoId]) {
                clearTimeout(debounceTimers[servoId]);
            }
            
            // Set new timer
            debounceTimers[servoId] = setTimeout(() => {
                updateServoPosition(servoId, value);
            }, 100); // 100ms debounce time
        });

        sliderContainer.appendChild(sliderLabel);
        sliderContainer.appendChild(slider);

        const errorContainer = document.createElement('div');
        errorContainer.className = 'error';
        errorContainer.id = `error-${servoId}`;
        errorContainer.style.display = 'none';

        card.appendChild(header);
        card.appendChild(info);
        card.appendChild(sliderContainer);
        card.appendChild(errorContainer);

        return card;
    }

    // Get descriptive name for each servo
    function getServoName(servoId) {
        const names = [
            'Base Rotation',
            'Shoulder',
            'Elbow',
            'Wrist Tilt',
            'Wrist Rotation',
            'Gripper'
        ];
        return names[servoId - 1] || `Servo ${servoId}`;
    }

    // Update servo position via API
    function updateServoPosition(servoId, position) {
        fetch(`/api/servo/${servoId}`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ position: position }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.error) {
                showError(servoId, data.error);
            } else {
                hideError(servoId);
            }
        })
        .catch(error => {
            showError(servoId, 'Failed to update servo position');
            console.error('Error:', error);
        });
    }

    // Show error message
    function showError(servoId, message) {
        const errorElement = document.getElementById(`error-${servoId}`);
        errorElement.textContent = message;
        errorElement.style.display = 'block';
    }

    // Hide error message
    function hideError(servoId) {
        const errorElement = document.getElementById(`error-${servoId}`);
        errorElement.style.display = 'none';
    }

    // Update servo information in the UI
    function updateServoInfo(servoData) {
        const servoId = servoData.id;
        
        // Update info values
        document.getElementById(`position-${servoId}`).textContent = servoData.position;
        
        const temperatureElement = document.getElementById(`temperature-${servoId}`);
        temperatureElement.textContent = `${servoData.temperature}°C`;
        if (servoData.temperature > 55) {
            temperatureElement.classList.add('temperature-warning');
        } else {
            temperatureElement.classList.remove('temperature-warning');
        }
        
        const voltageElement = document.getElementById(`voltage-${servoId}`);
        voltageElement.textContent = `${(servoData.voltage / 1000).toFixed(2)}V`;
        if (servoData.voltage < 6000 || servoData.voltage > 8400) {
            voltageElement.classList.add('voltage-warning');
        } else {
            voltageElement.classList.remove('voltage-warning');
        }
        
        document.getElementById(`speed-${servoId}`).textContent = servoData.speed;
        
        // Update slider min/max values and current position
        const slider = document.getElementById(`slider-${servoId}`);
        slider.min = servoData.min;
        slider.max = servoData.max;
        slider.value = servoData.position;
        
        // Update min/max labels
        document.getElementById(`min-${servoId}`).textContent = servoData.min;
        document.getElementById(`max-${servoId}`).textContent = servoData.max;
        
        // Show error if any
        if (servoData.error && typeof servoData.error === 'string') {
            showError(servoId, servoData.error);
        } else {
            hideError(servoId);
        }
    }

    // Socket.io event handlers
    socket.on('connect', function() {
        console.log('Connected to server');
    });

    socket.on('disconnect', function() {
        console.log('Disconnected from server');
    });

    socket.on('servo_update', function(msg) {
        const servoDataArray = msg.data;
        servoDataArray.forEach(updateServoInfo);
    });

    // Initialize the UI
    initializeServoCards();

    // Initial fetch of all servo data
    fetch('/api/servos')
        .then(response => response.json())
        .then(data => {
            data.forEach(updateServoInfo);
        })
        .catch(error => {
            console.error('Error fetching servo data:', error);
        });
    // URDF Viewer Setup
    let urdfRobot, urdfJoints = {};
    const urdfDiv = document.getElementById('urdf-viewer');
    const renderer = new THREE.WebGLRenderer({antialias:true});
    renderer.setSize(urdfDiv.clientWidth, urdfDiv.clientHeight);
    urdfDiv.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, urdfDiv.clientWidth/urdfDiv.clientHeight, 0.1, 1000);
    camera.position.set(0.3, 0.3, 0.6);
    camera.lookAt(0,0,0);
    scene.add(new THREE.AmbientLight(0xffffff, 1));
    // Load URDF
    const loader = new URDFLoader();
    loader.load('/api/urdf', function(robot) {
        urdfRobot = robot;
        scene.add(robot);
        // Collect joints for easy access
        robot.traverse(function(obj) {
            if(obj.isURDFJoint) urdfJoints[obj.name] = obj;
        });
        animate();
    });
    function animate() {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    }
    // Map servo index to URDF joint names (adjust as needed)
    const jointNames = [
        'Shoulder_Rotation',
        'Shoulder_Pitch',
        'Elbow',
        'Wrist_Tilt',
        'Wrist_Rotation',
        'Gripper' // Adjust if gripper is a joint
    ];
    // Mirror servo positions to URDF
    function mirrorToURDF(servoDataArray) {
        if(!urdfRobot) return;
        for(let i=0; i<jointNames.length; ++i) {
            const joint = urdfJoints[jointNames[i]];
            if(joint) {
                // Map servo value to joint angle (adjust scaling as needed)
                const pos = servoDataArray[i]?.position || 0;
                // Example: map servo range to joint range
                const min = servoDataArray[i]?.min || 0;
                const max = servoDataArray[i]?.max || 1000;
                const angle = (pos-min)/(max-min)*Math.PI - Math.PI/2; // Example mapping
                joint.setJointValue(angle);
            }
        }
    }
    // Listen for servo updates
    socket.on('servo_update', function(msg) {
        mirrorToURDF(msg.data);
    });
});