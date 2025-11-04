// Manual Control JavaScript

let moveInterval = null;

function moveRobot(direction) {
    // Stop any existing movement
    if (moveInterval) {
        clearInterval(moveInterval);
    }
    
    // Send move command
    fetch('/api/move', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            direction: direction,
            speed: 50
        })
    }).catch(err => console.error('Move error:', err));
    
    // Keep sending command while button is held
    moveInterval = setInterval(() => {
        fetch('/api/move', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                direction: direction,
                speed: 50
            })
        }).catch(err => console.error('Move error:', err));
    }, 100); // Send every 100ms
}

function stopRobot() {
    if (moveInterval) {
        clearInterval(moveInterval);
        moveInterval = null;
    }
    
    fetch('/api/stop', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    }).catch(err => console.error('Stop error:', err));
}

function controlTopHalf(action) {
    fetch('/api/top_half', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            action: action
        })
    }).catch(err => console.error('Top half control error:', err));
}

function stopTopHalf() {
    // Stop top half movement
    fetch('/api/top_half', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            action: 'stop'
        })
    }).catch(err => console.error('Stop top half error:', err));
}

// Keyboard support
document.addEventListener('keydown', (e) => {
    switch(e.key) {
        case 'ArrowUp':
            e.preventDefault();
            moveRobot('up');
            break;
        case 'ArrowDown':
            e.preventDefault();
            moveRobot('down');
            break;
        case 'ArrowLeft':
            e.preventDefault();
            moveRobot('left');
            break;
        case 'ArrowRight':
            e.preventDefault();
            moveRobot('right');
            break;
    }
});

document.addEventListener('keyup', (e) => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        e.preventDefault();
        stopRobot();
    }
});

