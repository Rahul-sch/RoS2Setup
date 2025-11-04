// Camera JavaScript

const cameraFeed = document.getElementById('camera-feed');
const trackingStatus = document.getElementById('tracking-status');
const statusText = document.getElementById('status-text');

// Update camera feed
function updateCameraFeed() {
    fetch('/api/camera_frame')
        .then(response => response.json())
        .then(data => {
            if (data.frame) {
                cameraFeed.src = data.frame;
            }
        })
        .catch(err => {
            console.error('Camera feed error:', err);
        });
}

// Update status
function updateStatus() {
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {
            if (data.tracking_active) {
                trackingStatus.classList.add('active');
                statusText.textContent = 'Tracking: Active';
            } else {
                trackingStatus.classList.remove('active');
                statusText.textContent = 'Tracking: Inactive';
            }
            
            if (data.safety_stop) {
                statusText.textContent += ' (STOPPED)';
            }
        })
        .catch(err => {
            console.error('Status error:', err);
        });
}

// Click on image to select tracking point
cameraFeed.addEventListener('click', (e) => {
    const rect = cameraFeed.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    // TODO: Send point selection to ROS2
    console.log('Selected point:', x, y);
    // You can add a fetch call here to send the point to ROS2
});

// Update camera feed every 100ms (10 FPS)
setInterval(updateCameraFeed, 100);

// Update status every second
setInterval(updateStatus, 1000);

// Initial updates
updateCameraFeed();
updateStatus();

