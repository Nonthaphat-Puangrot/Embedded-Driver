from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt
import json
import threading
import time
from datetime import datetime
import logging

# Configure logging - only show warnings and errors for most components
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

# Create a specific logger for alerts with INFO level
alert_logger = logging.getLogger('alerts')
alert_logger.setLevel(logging.INFO)
alert_handler = logging.StreamHandler()
alert_handler.setFormatter(logging.Formatter('%(asctime)s - ALERT - %(message)s'))
alert_logger.addHandler(alert_handler)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'racing_simulator_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*", logger=False, engineio_logger=False)

# MQTT Configuration
MQTT_BROKER = "10.23.166.129"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60

# Global variables to store car data
car_data = {
    'steering': 0,      # Should be between -32767 and +32767
    'throttle': 0,      # Should be between 0 and 100
    'brake': 0,         # Should be between 0 and 100
    'gear': 0,          # Should be between 0 and 5
    'rpm': 0.0,         # Should be between 0 and ~140 (PWM value)
    'timestamp': datetime.now().strftime('%H:%M:%S'),
    'connected': False
}

# Store latest alert
latest_alert = None
alert_id_counter = 0

# Parental control settings
parental_control = {
    'enabled': False,
    'max_speed_gear': 3,  # Limit to gear 3
    'max_throttle': 50    # Limit throttle to 50%
}

# MQTT Client setup
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    # logger.info(f"Connected to MQTT broker with result code {rc}")  # Debug disabled
    car_data['connected'] = True
    # Subscribe to all car telemetry topics
    client.subscribe("car/driver1/steering")
    client.subscribe("car/driver1/throttle")
    client.subscribe("car/driver1/brake")
    client.subscribe("car/driver1/gear")
    client.subscribe("car/driver1/rpm")
    # Subscribe to parental control topic for OTA updates
    client.subscribe("car/driver1/parental_control")
    # Subscribe to alerts topic
    client.subscribe("car/driver1/alerts")
    
    # Emit connection status to all clients
    socketio.emit('connection_status', {'connected': True})

def on_message(client, userdata, msg):
    topic = msg.topic
    try:
        # Decode the payload
        payload_str = msg.payload.decode()
        
        # Handle alerts
        if "alerts" in topic:
            try:
                import json
                global latest_alert, alert_id_counter
                alert_data = json.loads(payload_str)
                
                # Add timestamp and unique ID
                alert_id_counter += 1
                alert_data['id'] = alert_id_counter
                alert_data['received_at'] = datetime.now().isoformat()
                
                # Store as latest alert
                latest_alert = alert_data
                
                alert_logger.info(f"Alert received: {alert_data}")  # Keep alert debug
                
                # Still emit to WebSocket clients (as backup)
                socketio.emit('alert_notification', alert_data)
                return
            except json.JSONDecodeError as e:
                alert_logger.error(f"Invalid JSON in alert message: {payload_str} - {e}")
                return
        
        value = float(payload_str)
        
        # Update car data based on topic
        if "steering" in topic:
            # Keep steering as integer but ensure it's properly handled
            car_data['steering'] = int(value)
            # logger.info(f"Steering updated: {car_data['steering']} (raw: {payload_str})")  # Debug disabled
        elif "throttle" in topic:
            car_data['throttle'] = int(value)
        elif "brake" in topic:
            car_data['brake'] = int(value)
        elif "gear" in topic:
            car_data['gear'] = int(value)
        elif "rpm" in topic:
            car_data['rpm'] = round(value, 1)
            
        car_data['timestamp'] = datetime.now().strftime('%H:%M:%S')
        
        # logger.info(f"Updated {topic}: {value} -> car_data: {car_data}")  # Debug disabled
        
        # Data is updated but not automatically sent to web clients
        # Clients need to manually request updates
        
    except ValueError as e:
        logger.error(f"Error parsing message from {topic}: {msg.payload} - {e}")
    except Exception as e:
        logger.error(f"Unexpected error processing {topic}: {e}")

def on_disconnect(client, userdata, rc):
    # logger.warning("Disconnected from MQTT broker")  # Debug disabled
    car_data['connected'] = False
    socketio.emit('connection_status', {'connected': False})

# Configure MQTT client
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.on_disconnect = on_disconnect

def connect_mqtt():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
        mqtt_client.loop_start()
        # logger.info("MQTT client started")  # Debug disabled
    except Exception as e:
        logger.error(f"Failed to connect to MQTT broker: {e}")

# Add a periodic task to send heartbeat (disabled for manual updates)
# def send_heartbeat():
#     while True:
#         time.sleep(5)  # Send heartbeat every 5 seconds
#         socketio.emit('heartbeat', {'timestamp': datetime.now().strftime('%H:%M:%S')})

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/car_data')
def get_car_data():
    # logger.info(f"API car_data requested: {car_data}")  # Debug disabled
    return jsonify(car_data)

@app.route('/api/debug')
def debug_data():
    """Debug endpoint to check current data values"""
    debug_info = {
        'car_data': car_data,
        'car_data_types': {k: type(v).__name__ for k, v in car_data.items()},
        'mqtt_connected': car_data.get('connected', False)
    }
    return jsonify(debug_info)

@app.route('/api/latest_alert')
def get_latest_alert():
    """Get the latest alert if any"""
    if latest_alert:
        return jsonify({
            'has_alert': True,
            'alert': latest_alert
        })
    else:
        return jsonify({
            'has_alert': False,
            'alert': None
        })

@app.route('/api/clear_alert', methods=['GET', 'POST'])
def clear_alert():
    """Clear the current alert"""
    global latest_alert
    latest_alert = None
    return jsonify({'status': 'success', 'message': 'Alert cleared'})

@app.route('/api/parental_control', methods=['GET', 'POST'])
def parental_control_api():
    if request.method == 'GET':
        return jsonify(parental_control)
    
    elif request.method == 'POST':
        data = request.get_json()
        
        if 'enabled' in data:
            parental_control['enabled'] = data['enabled']
        if 'max_speed_gear' in data:
            parental_control['max_speed_gear'] = int(data['max_speed_gear'])
        if 'max_throttle' in data:
            parental_control['max_throttle'] = int(data['max_throttle'])
        
        # Send OTA update to Arduino via MQTT
        control_message = {
            'enabled': parental_control['enabled'],
            'max_gear': parental_control['max_speed_gear'],
            'max_throttle': parental_control['max_throttle']
        }
        
        mqtt_client.publish("car/driver1/parental_control_settings", 
                          json.dumps(control_message))
        
        # logger.info(f"Parental control updated: {parental_control}")  # Debug disabled
        
        # Settings updated but not automatically broadcast to clients
        # Clients must manually request updates
        
        return jsonify({
            'status': 'success', 
            'message': 'Parental control settings updated',
            'settings': parental_control
        })

@socketio.on('connect')
def handle_connect():
    # logger.info('Client connected')  # Debug disabled
    # Only send connection status, not automatic data updates
    emit('connection_status', {'connected': car_data['connected']})
    # Clients must manually request data using 'request_car_data' event

@socketio.on('disconnect')
def handle_disconnect():
    # logger.info('Client disconnected')  # Debug disabled
    pass

@socketio.on('request_car_data')
def handle_request_car_data():
    # logger.info(f'Client requested car data: {car_data}')  # Debug disabled
    emit('car_data_update', car_data)

@socketio.on('request_parental_control')
def handle_request_parental_control():
    # logger.info('Client requested parental control settings')  # Debug disabled
    emit('parental_control_update', parental_control)

@socketio.on('ping')
def handle_ping():
    emit('pong', {'timestamp': datetime.now().strftime('%H:%M:%S')})

@socketio.on('dismiss_alert')
def handle_dismiss_alert(data):
    global latest_alert
    alert_logger.info(f'Alert dismissed: {data}')  # Keep alert debug
    # Clear the latest alert when dismissed
    latest_alert = None
    # You can log dismissed alerts or take other actions here
    emit('alert_dismissed', {'status': 'success'})

@socketio.on('engage_parental_control')
def handle_engage_parental_control(data):
    global latest_alert
    alert_logger.info('Engaging parental control from alert')  # Keep alert debug
    
    # Clear the alert since user took action
    latest_alert = None
    
    # Enable parental control with default safe settings
    parental_control['enabled'] = True
    parental_control['max_speed_gear'] = 3
    parental_control['max_throttle'] = 50
    
    # Send OTA update to Arduino
    control_message = {
        'enabled': True,
        'max_gear': 3,
        'max_throttle': 50
    }
    
    mqtt_client.publish("car/driver1/parental_control_settings", 
                      json.dumps(control_message))
    
    alert_logger.info("Parental control engaged from alert system")  # Keep alert debug
    emit('parental_control_engaged', {'status': 'success', 'settings': parental_control})

if __name__ == '__main__':
    # Start MQTT client in a separate thread
    mqtt_thread = threading.Thread(target=connect_mqtt)
    mqtt_thread.daemon = True
    mqtt_thread.start()
    
    # Start Flask-SocketIO server
    # logger.info("Starting Flask-SocketIO server on http://0.0.0.0:5000")  # Debug disabled
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)