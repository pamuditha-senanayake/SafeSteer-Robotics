import cv2
import dlib
import time
from twilio.rest import Client
import numpy as np
from scipy.spatial import distance as dist
import sys
import threading
import requests
import json
import subprocess
import os
import queue
import select
import traceback
import serial # Import the pyserial library
import glob  # Used to find serial devices like /dev/ttyACM*
import serial.tools.list_ports  # For listing connected serial ports


# --- Configuration ---
# Serial Port - IMPORTANT: Change this to your Arduino's serial port path!
# On Linux (Raspberry Pi OS), this is typically /dev/ttyACM0 or /dev/ttyUSB0
# You can find it by plugging in the Arduino and checking `ls /dev/tty*` before and after.
ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
ARDUINO_BAUD_RATE = 115200
SERIAL_READ_TIMEOUT = 1.0 # Timeout for reading serial line (seconds)
SERIAL_WRITE_INTERVAL_NO = 1.0 # Send "No" command every X seconds to keep Arduino fail-safe off

THINGSPEAK_API_KEY = 'XXX'
THINGSPEAK_URL = f'https://api.thingspeak.com/update?api_key={THINGSPEAK_API_KEY}'
# ThingSpeak free tier limit is typically 1 update per 15 seconds per channel.
THINGSPEAK_UPDATE_INTERVAL_SECONDS = 15 # MUST be >= 15 for free tier free tier

# ThingSpeak Fields Mapping (Sending 8 fields)
# field1: Heart Rate (from Arduino)
# field2: Active Touch Sensors Count (from Arduino's "TOUCH" string)
# field3: Accelerometer X (from Arduino)
# field4: Current System Warning Level (from Arduino)
# field5: Camera Drowsy Event Triggered (Pi-side flag, 1 or 0)
# field6: Motor Speed (from Arduino)
# field7: Accelerometer Y (from Arduino)
# field8: Fail Safe Mode (from Arduino's "FAIL_SAFE" string, 1 or 0)


TWILIO_SID = 'XXX'  # Replace with your Twilio SID
TWILIO_AUTH_TOKEN = 'XXXX'  # Replace with your Twilio Auth Token

WHATSAPP_FROM = 'whatsapp:+14155238886'  # Your Twilio WhatsApp number (from Twilio, e.g., sandbox number) - MUST be whatsapp:+E.164 format
EMERGENCY_WHATSAPP = 'whatsapp:+94701155114'  # The emergency WhatsApp number (destination) - MUST be whatsapp:+E.164 format

# Camera Stream URL - IMPORTANT: Change this to your IP Webcam app's URL
CAMERA_STREAM_URL = 'http://192.168.1.5:8080/video'

MAX_RECONNECT_ATTEMPTS = 10
RECONNECT_DELAY_SECONDS = 5

# Camera Drowsiness Thresholds
EAR_THRESH = 0.28  # Eye Aspect Ratio threshold for detecting eye closure per frame
MAR_THRESH = 0.7  # Mouth Aspect Ratio threshold for detecting yawn per frame
YAWN_CONSEC_FRAMES = 15  # Consecutive frames MAR must be above threshold to count as a yawn event
# Duration eyes must be continuously closed (EAR < THRESH) to trigger a camera drowsiness event
EYES_CLOSED_DURATION_THRESH = 2.0  # Seconds

# Pi-Side Trigger Logic Thresholds (used by Pi based on *received* sensor data)
# These should match the thresholds used in the Arduino's fail-safe / jerk detection logic
ARDUINO_HR_LOWER_THRESH = 60 # Arduino triggers fail-safe L1 if HR is consistently below this while offline
ARDUINO_TOUCH_LOST_THRESHOLD = 1 # Arduino triggers fail-safe L1 if <= this many touch sensors active consistently while offline
ARDUINO_ACCEL_SUDDEN_MOVEMENT_THRESHOLD = 40 # Arduino's JERK_THRESHOLD (40)

# Warning Cooldowns (Prevent spamming)
LEVEL1_TRIGGER_COOLDOWN_SECONDS = 20  # Cooldown for Pi sending L1 trigger command to Arduino
LEVEL3_WHATSAPP_COOLDOWN_SECONDS = 60  # Cooldown for Pi sending L3 WhatsApp message

# --- Audio File Paths ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
AUDIO_FILE_LEVEL1 = os.path.join(SCRIPT_DIR, "alert1.mp3")
AUDIO_FILE_LEVEL2 = os.path.join(SCRIPT_DIR, "alert2.mp3")
AUDIO_FILE_LEVEL3 = os.path.join(SCRIPT_DIR, "alert3.mp3")

# --- Audio Alert Message Texts (for printing) ---
LEVEL1_VOICE_ALERT_TEXT = "Level 1 Alert: Wake Up! Please acknowledge."
LEVEL2_VOICE_ALERT_TEXT = "Level 2 Warning: Stay Alert! Increased warning."
LEVEL3_VOICE_ALERT_TEXT = "Level 3 Emergency: System Stopping Car! Immediate attention needed!"

# --- Location Services Config ---
# Using ip-api.com (free tier limits requests, consider alternatives for production)
IP_LOCATION_API_URL = "http://ip-api.com/json/"

# --- Dlib Face and Landmark Predictor ---
# The model file 'shape_predictor_68_face_landmarks.dat' is required.
detector = dlib.get_frontal_face_detector()
predictor = None # Initialize predictor as None
try:
    # Ensure the shape predictor model file 'shape_predictor_68_face_landmarks.dat' is in the same directory
    predictor_path = os.path.join(SCRIPT_DIR, "shape_predictor_68_face_landmarks.dat")
    if os.path.exists(predictor_path):
        predictor = dlib.shape_predictor(predictor_path)
        print("[*] Dlib shape predictor model loaded.")
    else:
        print(f"[!!!] Error: shape_predictor_68_face_landmarks.dat not found at {predictor_path}")
        print("Dlib face detection and landmark prediction will NOT work!")

except RuntimeError as e:
    print(f"[!!!] Error loading shape predictor model: {e}")
    print("Dlib face detection and landmark prediction will NOT work!")
    predictor = None
except Exception as e: # Catch other potential errors during loading
     print(f"[!!!] Unexpected error loading shape predictor model: {e}")
     predictor = None

predictor_warned = False # Flag to track if the predictor warning has been printed (to only print once)

# Landmark Indices (used by EAR/MAR calculation)
LEFT_EYE_IDX = list(range(42, 48))
RIGHT_EYE_IDX = list(range(36, 42))
MOUTH_IDX = list(range(60, 68))


# === Global State Variables (Shared between Main Loop and Serial Thread) ===
# These are updated by the serial thread and read by the main loop
latest_sensor_data = {}  # Stores the last received parsed sensor data from Arduino
current_warning_level = 0  # System level reported by Arduino (0-3), updated by serial thread
motor_speed_reported = 0 # Motor speed reported by Arduino (0-180), updated by serial thread
fail_safe_mode_reported = False # Fail-safe mode reported by Arduino (True/False), updated by serial thread
previous_warning_level_for_print = -1  # Track previous level *for printing the transition message*

# Variables managed by the main loop for Pi-side analysis (camera)
camera_drowsy_event_triggered = False  # Flag set by camera analysis (eyes closed duration)
camera_yawn_detected = False  # Flag set by camera analysis (yawn consecutive frames)

# State for camera detection counters
blink_frame_count = 0  # Number of consecutive frames eyes have been closed (EAR < THRESH)
yawn_frame_count = 0  # Number of consecutive frames mouth has been open (MAR > THRESH)
eye_closed_start_time = None  # Timestamp when eyes *started* being closed

# State for periodic print
last_face_print_time = 0
FACE_PRINT_INTERVAL_SECONDS = 5

# State for Pi-side L1 trigger logic based on sensor data (need to track last reading for diff)
last_accel_x_for_diff = None  # Store last accel_x from Arduino data to calculate difference

# State for cooldowns
last_level1_trigger_time = 0  # Timestamp of last Pi-side L1 trigger sent to Arduino
last_level3_whatsapp_time = 0  # Timestamp of last Level 3 WhatsApp message sent
last_thingspeak_send_time = 0 # Timestamp of last ThingSpeak send
last_pi_online_signal_time = 0 # Timestamp of last "No" command sent to Arduino


# --- Serial Port Object ---
ser = None # Global serial object, initialized later

# --- Audio Playback State ---
audio_process = None  # Stores the subprocess.Popen object for the current audio playback
last_played_alert_level = -1  # Track the last level for which audio was *successfully started*

# --- Control Flags ---
level3_whatsapp_sent = False # Flag to signal the main loop to exit after L3 WhatsApp is sent
running = True # Flag to signal all threads to stop (used by Ctrl+C and L3 WhatsApp)


# --- Helper Functions (for EAR/MAR calculation) ---
def eye_aspect_ratio(eye):
    # calculate the euclidean distances between the two sets of vertical eye landmarks
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    # compute the euclidean distance between the horizontal eye landmark
    C = dist.euclidean(eye[0], eye[3])
    # compute the eye aspect ratio
    ear = (A + B) / (2.0 * C + 1e-6) # Add epsilon to avoid division by zero
    return ear

def mouth_aspect_ratio(mouth):
    # calculate the euclidean distances between the two sets of vertical mouth landmarks
    A = dist.euclidean(mouth[1], mouth[7])
    B = dist.euclidean(mouth[3], mouth[5])
    # compute the euclidean distance between the horizontal mouth landmark
    C = dist.euclidean(mouth[0], mouth[4])
    # compute the mouth aspect ratio
    mar = (A + B) / (2.0 * C + 1e-6) # Add epsilon to avoid division by zero
    return mar

# --- Audio Playback Functions ---
def stop_audio():
    """Stops any currently playing audio managed by this script."""
    global audio_process
    if audio_process is not None:
        if audio_process.poll() is None: # Check if process is still running
            # print("[AUDIO] Stopping current audio playback...")
            try:
                audio_process.terminate() # Request graceful termination
                audio_process.wait(timeout=1) # Wait for process to terminate
                # print("[AUDIO] Audio playback stopped gracefully.")
            except subprocess.TimeoutExpired:
                print("[AUDIO] Audio process did not terminate gracefully, killing...")
                audio_process.kill() # Force kill if terminate didn't work
                audio_process.wait()
                # print("[AUDIO] Audio playback killed.")
            except Exception as e:
                print(f"[AUDIO ERROR] Error stopping audio process: {e}")
        audio_process = None # Clear the process reference

def play_audio(file_path):
    """
    Plays an audio file using mpg123 via subprocess in a non-blocking way.
    """
    if not os.path.exists(file_path):
        print(f"[AUDIO ERROR] Audio file not found: {file_path}")
        return False # Indicate failure

    # Ensure mpg123 is callable
    try:
        subprocess.run(['mpg123', '-V'], check=True, timeout=5, capture_output=True)
    except FileNotFoundError:
        print("[AUDIO ERROR] mpg123 not found. Please install it: sudo apt-get install mpg123")
        return False
    except subprocess.CalledProcessError:
         # mpg123 -V might return non-zero on some systems even if installed
         print("[AUDIO WARN] mpg123 version check returned non-zero status.")
    except Exception as e:
         print(f"[AUDIO ERROR] mpg123 command check failed: {e}")
         return False

    try:
        # print(f"[AUDIO] Starting playback: {os.path.basename(file_path)}")
        global audio_process
        # Use subprocess.Popen to start mpg123 non-blocking
        # '-q' is for quiet mode, redirect stdout/stderr
        audio_process = subprocess.Popen(
            ['mpg123', '-q', file_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        return True # Indicate success

    except Exception as e:
        print(f"[AUDIO ERROR] Failed to play audio file {file_path}: {e}")
        traceback.print_exc()
        return False # Indicate failure

def trigger_audio_alert(level):
    """
    Plays the appropriate audio file for the given level if the level has changed.
    Stops any currently playing audio first.
    Updates the last_played_alert_level state.
    """
    global last_played_alert_level

    # If the level is the same as the last level we successfully *started* playing audio for, do nothing.
    # This prevents re-playing the same audio repeatedly for a sustained level.
    if level == last_played_alert_level:
        return

    # Always stop any currently playing audio whenever the level changes (even to level 0)
    stop_audio()

    # Determine which audio to play based on the *new* level
    played = False
    if level == 1:
        print(f"[VOICE ALERT] {LEVEL1_VOICE_ALERT_TEXT}")
        played = play_audio(AUDIO_FILE_LEVEL1)
    elif level == 2:
        print(f"[VOICE ALERT] {LEVEL2_VOICE_ALERT_TEXT}")
        played = play_audio(AUDIO_FILE_LEVEL2)
    elif level == 3:
        print(f"[VOICE ALERT] {LEVEL3_VOICE_ALERT_TEXT}")
        played = play_audio(AUDIO_FILE_LEVEL3)
    elif level == 0:
        # Level dropped back to 0. We already stopped audio above.
        print("[VOICE ALERT] System level reset to 0. Audio stopped.")
        last_played_alert_level = 0 # Ensure we register level 0 state
        return # Exit early for level 0

    # Update last_played_alert_level only if we attempted to play and it succeeded
    if played:
        last_played_alert_level = level
    else:
        last_played_alert_level = -1 # Indicate playback failed


# --- Communication Functions ---

def get_location_from_ip():
    """Gets approximate location based on public IP address and returns formatted string and coords."""
    try:
        headers = {'User-Agent': 'DriverMonitoringSystem/1.0 (RaspberryPi)'}
        response = requests.get(IP_LOCATION_API_URL, timeout=10, headers=headers)
        response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)
        data = response.json()

        if data and data.get('status') == 'success':
            city = data.get('city', 'Unknown City')
            region = data.get('regionName', 'Unknown Region')
            country = data.get('country', 'Country')
            lat = data.get('lat', 0.0)
            lon = data.get('lon', 0.0)
            # Ensure coordinates are floats before formatting and returning
            lat_f = float(lat) if lat is not None else 0.0
            lon_f = float(lon) if lon is not None else 0.0
            # Return the formatted string AND the coordinates
            formatted_string = f"Location: {city}, {region}, {country} (Lat: {lat_f:.4f}, Lon: {lon_f:.4f})"
            return formatted_string, lat_f, lon_f
        else:
            print(f"[LOCATION] IP-API Status: {data.get('status', 'Unknown')}, Message: {data.get('message', 'N/A')}")
            # Return error string and None coordinates on failure
            return "Location: Could not determine (IP-API error)", None, None

    except requests.exceptions.RequestException as e:
        print(f"[LOCATION] Error getting location from IP-API: {e}")
        traceback.print_exc() # Optional: print full traceback
        # Return error string and None coordinates on network error
        return "Location: Could not determine (Network error)", None, None
    except Exception as e:
        print(f"[LOCATION] An unexpected error occurred getting location: {e}")
        traceback.print_exc() # Optional: print full traceback
        # Return error string and None coordinates on unexpected error
        return "Location: Could not determine (Unexpected error)", None, None


def send_to_thingspeak(data):
    """Sends data dictionary to ThingSpeak."""
    global last_thingspeak_send_time, camera_drowsy_event_triggered

    current_time = time.time()
    # Check the cooldown BEFORE preparing and sending data
    if current_time - last_thingspeak_send_time < THINGSPEAK_UPDATE_INTERVAL_SECONDS:
        # print(f"[ThingSpeak] On cooldown. Next send possible in {THINGSPEAK_UPDATE_INTERVAL_SECONDS - (current_time - last_thingspeak_send_time):.1f}s") # Too verbose
        return # Do nothing if on cooldown

    try:
        # Prepare data based on available latest_sensor_data and Pi's state
        ts_payload = {}

        # Map received Arduino sensor data keys to ThingSpeak fields (using 8 fields)
        # Keys from Arduino: HR, SPO2, ACCEL_X, ACCEL_Y, ACCEL_Z, TOUCH, ALERT_LEVEL, MOTOR_SPEED, FAIL_SAFE
        # Field 1: Heart Rate
        ts_payload['field1'] = data.get('HR') # Allow None if key is missing

        # Field 2: Active Touch Sensors Count (from the TOUCH string "1,0,1,1")
        touch_string = data.get('TOUCH', "")
        try:
            # Count '1's in the comma-separated string
            active_touches = touch_string.count('1')
        except:
            active_touches = 0  # Default if touch string is malformed
        ts_payload['field2'] = active_touches

        # Field 3: Accelerometer X
        ts_payload['field3'] = data.get('ACCEL_X')

        # Field 4: Current System Warning Level (from Arduino's data)
        ts_payload['field4'] = data.get('ALERT_LEVEL')

        # Field 5: Camera Drowsy Event Triggered (Pi-side flag)
        # Convert boolean flag to 1 or 0
        ts_payload['field5'] = 1 if camera_drowsy_event_triggered else 0

        # Field 6: Motor Speed (from Arduino's data)
        ts_payload['field6'] = data.get('MOTOR_SPEED')

        # Field 7: Accelerometer Y (from Arduino's data)
        ts_payload['field7'] = data.get('ACCEL_Y')

        # Field 8: Fail Safe Mode (from Arduino's data, '1' or '0' string)
        ts_payload['field8'] = 1 if data.get('FAIL_SAFE') == '1' else 0 # Convert string '1'/'0' to 1/0 integer

        # Remove None values from the payload before sending, as ThingSpeak doesn't accept them
        ts_payload_clean = {k: v for k, v in ts_payload.items() if v is not None}

        # Include the API key as a parameter
        params = {k: v for k, v in ts_payload_clean.items()}
        params['api_key'] = THINGSPEAK_API_KEY

        # Only send if there's some data to send (at least the API key and maybe one field)
        if not params or 'api_key' not in params:
             # print("[ThingSpeak] No valid data or API key to send.") # Too verbose
             return

        # --- Added: Print each field value being sent ---
        # Updated the print statement to include the configured interval
        print(f"[{time.strftime('%H:%M:%S')}] ThingSpeak Send Data (Interval: {THINGSPEAK_UPDATE_INTERVAL_SECONDS}s):")
        # Sort keys for consistent print order (optional but helpful)
        for key in sorted(ts_payload_clean.keys()):
             print(f"  {key}: {ts_payload_clean[key]}")
        # --------------------------------------------------


        r = requests.get(THINGSPEAK_URL, params=params, timeout=10) # Added timeout
        if r.status_code == 200:
            entry_id_str = r.text.strip()
            # ThingSpeak returns '0' if no update (rate limit) or error
            if entry_id_str and entry_id_str != '0':
                 # print(f"[✓] ThingSpeak sent data (Entry ID: {entry_id_str}).") # Too verbose
                 # Update the timer ONLY on a successful send (status 200 and non-zero entry ID)
                 last_thingspeak_send_time = current_time
            elif entry_id_str == '0':
                 # This indicates rate limit or potentially other issues
                 # print("[ThingSpeak] Received '0' from API (likely rate limit or no data changed).") # Too verbose
                 pass # Suppress this print for cleaner logs
            else:
                 print(f"[✗] ThingSpeak error: Empty or unexpected response '{r.text.strip()}' for status 200.")

        else:
            # This prints the error if status code is not 200 (e.g., 400 from rate limit)
            print(f"[✗] ThingSpeak API Error: Status {r.status_code} - Response: '{r.text.strip()}' | Data Attempted: {ts_payload_clean}")

    except requests.exceptions.RequestException as e:
        print(f"[✗] ThingSpeak Request Failed: {e}")
        traceback.print_exc() # Optional: print full traceback
    except Exception as e:
        print(f"[✗] ThingSpeak failed: {e}")
        traceback.print_exc() # Optional: print full traceback


def send_whatsapp_message(to, body):
    """Sends a WhatsApp message using Twilio."""
    global client, level3_whatsapp_sent, last_level3_whatsapp_time
    try:
        # Ensure the global client object is initialized before calling this
        if client is None:
            print("[✗] Twilio client not initialized. Cannot send WhatsApp.")
            return # Exit if client is not ready

        print(f"[💬] Attempting to send WhatsApp message to {to}...")
        # Note: twilio-python library handles underlying HTTP requests and timeouts.
        # Explicitly adding timeout here might depend on the library version/method signature.
        message = client.messages.create(
            from_=WHATSAPP_FROM,
            body=body,
            to=to
        )

        print(f"[✓] WhatsApp message sent successfully to {to}. SID: {message.sid}")
        # Set the flag here to indicate the message was sent successfully
        level3_whatsapp_sent = True
        # Update cooldown *only on successful send*
        last_level3_whatsapp_time = time.time()
        print("[*] Level 3 WhatsApp sent successfully. Signaling main loop to terminate.")

    except Exception as e:
        print(f"[✗] WhatsApp sending failed: {e}")
        traceback.print_exc() # Print traceback for detailed error info (IMPORTANT for debugging!)
        # Do NOT set level3_whatsapp_sent = True here if sending fails.


def send_serial_command(command):
    """Sends a command string followed by newline to the Arduino over serial."""
    global ser, running # Use global ser and running flag
    if ser and ser.isOpen():
        try:
            command_bytes = (command + '\n').encode('utf-8')
            # print(f"[SERIAL OUT] Sending: {command_bytes.strip()}") # Uncomment for debug
            ser.write(command_bytes)
            # Add a small delay after writing to allow Arduino to process if needed
            time.sleep(0.01) # 10ms delay
        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Failed to send command '{command}': {e}")
            # If serial fails, set running = False to trigger cleanup and exit?
            # Or just print and let the serial read thread handle critical errors?
            # Let the read thread handle critical errors for consistency.
        except Exception as e:
            print(f"[SERIAL ERROR] Unexpected error sending command '{command}': {e}")
            traceback.print_exc()
    # else: # print(f"[SERIAL WARN] Cannot send command '{command}', serial port not open.") # Too verbose


# --- Real Serial Communication Thread ---

def parse_sensor_data_string(data_string):
    """Parses the Arduino's SENSOR_DATA string into a dictionary."""
    parsed_data = {}
    if not isinstance(data_string, str) or not data_string.startswith("SENSOR_DATA:"):
        return parsed_data # Not a sensor data line

    content = data_string[len("SENSOR_DATA:"):].strip()
    pairs = content.split(',')

    for pair in pairs:
        if '=' in pair:
            key, value = pair.split('=', 1)
            key = key.strip()
            value = value.strip()
            try:
                # Attempt to convert known numeric values
                if key in ['HR', 'SPO2', 'ACCEL_X', 'ACCEL_Y', 'ACCEL_Z', 'ALERT_LEVEL', 'MOTOR_SPEED']:
                    # Use a regex or better check if value is numeric
                    # This check handles negative numbers and decimals for floats
                    if value.replace('.', '', 1).isdigit() or (value.startswith('-') and value[1:].replace('.', '', 1).isdigit()):
                        if '.' in value:
                            parsed_data[key] = float(value)
                        else:
                            parsed_data[key] = int(value)
                    else:
                         parsed_data[key] = value # Keep as string if not numeric

                # Handle specific string values like FAIL_SAFE
                elif key == 'FAIL_SAFE':
                    # Arduino sends '1' or '0'
                    parsed_data[key] = value # Store as '1' or '0' string

                # Handle the TOUCH string format "1,0,1,1"
                elif key == 'TOUCH':
                    parsed_data[key] = value  # Store as the raw string "1,0,1,1"
                else:
                    parsed_data[key] = value  # Store as string by default
            except ValueError:
                # This catches errors during int/float conversion if the check above wasn't sufficient
                parsed_data[key] = value # Keep as string if conversion fails
            except Exception as e:
                 print(f"[SERIAL PARSE ERROR] Failed to parse key '{key}' with value '{value}': {e}")
                 traceback.print_exc() # Optional
                 parsed_data[key] = value # Keep original value if parsing fails

    return parsed_data


def real_serial_read_thread(serial_port_obj):
    """
    Reads data from the real serial port connected to Arduino.
    Parses SENSOR_DATA strings and updates global state variables.
    Exits when the global 'running' flag is False.
    """
    global latest_sensor_data, current_warning_level, motor_speed_reported, fail_safe_mode_reported, running

    print("[*] Real serial read thread started.")

    if not serial_port_obj or not serial_port_obj.isOpen():
         print("[!!!] Serial port object is not valid or not open. Real serial thread exiting.")
         running = False # Signal exit if serial wasn't setup correctly
         return

    try:
        # Clear any potential partial line in the buffer
        # serial_port_obj.reset_input_buffer() # Might be useful depending on behavior

        while running:
            try:
                # Read a line from the serial port with a timeout
                # read_until(b'\n') is usually better than readline() if Arduino always ends lines with \n
                # Using read_until is blocking but respects the timeout.
                line_bytes = serial_port_obj.read_until(b'\n', size=200) # Read up to 200 bytes or until newline
                # Check if any data was read
                if line_bytes:
                    line = line_bytes.decode('utf-8', errors='ignore').strip() # Decode and strip whitespace

                    if line.startswith("SENSOR_DATA:"):
                        # print(f"[SERIAL IN] Received: {line}") # Uncomment for detailed serial logs
                        parsed_data = parse_sensor_data_string(line)

                        # Update global state variables from parsed data
                        # Only update if the key exists in the parsed data to avoid errors
                        if parsed_data:
                             # Update the whole dictionary. This needs to be atomic if accessed concurrently often.
                             # For simple types updated infrequently, direct assignment is usually fine.
                             latest_sensor_data = parsed_data

                             # Update specific variables based on the keys
                             # Use get with default values if keys might be missing
                             level = parsed_data.get('ALERT_LEVEL')
                             if isinstance(level, (int, float)):
                                  current_warning_level = int(level)
                             # else: pass # Keep previous current_warning_level if invalid data

                             speed = parsed_data.get('MOTOR_SPEED')
                             if isinstance(speed, (int, float)):
                                 motor_speed_reported = int(speed)
                             # else: pass # Keep previous motor_speed_reported if invalid data

                             fail_safe = parsed_data.get('FAIL_SAFE')
                             if fail_safe in ['1', '0']:
                                fail_safe_mode_reported = (fail_safe == '1')
                             # else: pass # Keep previous fail_safe_mode_reported if invalid data

                        # else: # print(f"[SERIAL PARSE WARN] Parsed data is empty for line: {line}") # Too verbose

                    # Handle other potential messages from Arduino (e.g., debug prints)
                    elif line: # If line is not empty and didn't start with SENSOR_DATA:
                         # print(f"[ARDUINO MSG] {line}") # Uncomment to see other Arduino prints
                         pass # Ignore other messages for now

            except serial.SerialTimeoutException:
                # Timeout occurred, no data received within the timeout period
                # This is expected if Arduino sends data periodically (e.g. every 200ms)
                # print("[SERIAL] Read timeout.") # Too verbose
                pass
            except serial.SerialException as e:
                print(f"[SERIAL ERROR] Serial communication error: {e}")
                running = False # Critical error, signal exit
            except Exception as e:
                print(f"[SERIAL ERROR] Unexpected error in serial read thread: {e}")
                traceback.print_exc()
                # Decide if this error should stop the thread/system
                # For now, print error and continue loop. Could add error count for critical exit.

    except Exception as e: # Catch errors outside the inner while loop (less common)
        print(f"[SERIAL CRITICAL ERROR] An unexpected error occurred stopping the serial thread: {e}")
        traceback.print_exc()

    finally:
        # Ensure serial port is closed when the thread exits
        if serial_port_obj and serial_port_obj.isOpen():
            try:
                serial_port_obj.close()
                print("[*] Serial port closed by thread.")
            except Exception as e:
                print(f"[SERIAL ERROR] Error closing serial port: {e}")


# --- Headless Input Listener Thread ---
# Modified to send "TEST" or "No" commands
def headless_input_listener_thread():
    """
    Listens for keyboard input ('t' + Enter) for TEST or ('n' + Enter) for No.
    Exits when the global 'running' flag is False or stdin is closed.
    """
    global running
    print("[*] Headless input listener started (type 't' + Enter for TEST, 'n' + Enter for No).")
    try:
        while running:
            try:
                # Use select with a small timeout to avoid blocking indefinitely
                # select.select([sys.stdin.fileno()], ...) might be needed on some systems
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist: # If sys.stdin is in rlist, there's data to read
                    # Read a line from stdin. Using sys.stdin.readline() is typical.
                    line = sys.stdin.readline()
                    if not line: # stdin was closed (e.g., piped input finished)
                        print("[*] stdin closed. Signaling exit.")
                        running = False
                        break
                    command = line.strip()

                    if command.lower() == 't':
                        print("[*] Input received: 't'. Sending TEST command to Arduino...")
                        send_serial_command("TEST") # Send TEST command
                    elif command.lower() == 'n':
                         print("[*] Input received: 'n'. Sending No command to Arduino...")
                         send_serial_command("No") # Send No command
                    # Add 's' or 'stop' for simulating the stop button action *on the Arduino* if needed
                    # Note: The real Stop Button logic is on the Arduino, this is just simulating sending
                    # a command *as if* the Arduino sent it, which isn't how the system works.
                    # It's better to use the actual Arduino code and button.
                    # elif command.lower() == 's':
                    #     print("[*] Input received: 's'. NOTE: This does NOT simulate the Arduino button.")
                    #     print("    The Stop Button logic runs on Arduino and sends a specific signal.")
                    #     print("    Use 't' for TEST or 'n' for No to interact via serial commands.")


            except select.error as e:
                 # Handle potential select errors (e.g., file descriptor issues)
                 # print(f"[INPUT LISTENER ERROR] Select error: {e}") # Too verbose
                 time.sleep(0.1) # Fallback sleep
            except Exception as e:
                print(f"[INPUT LISTENER CRITICAL ERROR] Error in input thread: {e}")
                traceback.print_exc()
                running = False # Critical error, signal exit

    except EOFError:
        print("[*] EOFError on stdin. Signaling exit.")
        running = False
    except Exception as e: # Catch errors outside the inner while loop
        print(f"[INPUT LISTENER CRITICAL ERROR] An unexpected error occurred stopping input thread: {e}")
        traceback.print_exc()
        running = False

    print("[*] Headless input listener stopping.")


# --- Pi-Side Level 1 Trigger Logic (based on Camera OR Sensor Data) ---

def check_sensor_drowsiness_indicators(sensor_data):
    """
    Analyzes parsed sensor data dictionary to detect indicators for the Pi-side L1 trigger.
    Uses thresholds matching Arduino's internal logic.
    Returns True and a list of trigger details if any sensor indicates drowsiness,
    False and an empty list otherwise.
    """
    global last_accel_x_for_diff # Use global for state

    if not sensor_data: # No parsed data received yet
        # print("[SENSOR L1 CHECK] No sensor data available yet.") # Too verbose
        return False, []

    is_sensor_indicator_active = False
    trigger_details = []

    # Check Heart Rate (using Arduino's threshold 60)
    heart_rate = sensor_data.get('HR')
    # Ensure heart_rate is a number before comparison
    if isinstance(heart_rate, (int, float)) and heart_rate > 0 and heart_rate < ARDUINO_HR_LOWER_THRESH:
        # Note: Arduino's fail-safe checks for *consistent* low HR while offline.
        # Pi's check here is per data packet. This might be more sensitive or need temporal logic.
        # For now, simple check based on received value.
        is_sensor_indicator_active = True
        trigger_details.append(f"Sensor Low HR ({heart_rate} bpm < {ARDUINO_HR_LOWER_THRESH})")

    # Check Touch Sensors Count (from the TOUCH string "1,0,1,1")
    touch_string = sensor_data.get('TOUCH', "")
    if isinstance(touch_string, str): # Ensure it's a string before counting
        try:
            # Count '1's in the comma-separated string to get active count
            active_touches = touch_string.count('1')
            if active_touches <= ARDUINO_TOUCH_LOST_THRESHOLD:
                # Note: Arduino's fail-safe checks for *duration* of poor grip while offline.
                # Pi's check here is per data packet. This might be more sensitive or need temporal logic.
                 is_sensor_indicator_active = True
                 trigger_details.append(f"Sensor Touch lost ({active_touches}/4 active <= {ARDUINO_TOUCH_LOST_THRESHOLD})")
        except:
            pass # Ignore if touch string is malformed

    # Check Accelerometer Sudden Movement (using Arduino's threshold 40 on X diff)
    current_accel_x = sensor_data.get('ACCEL_X')

    # Ensure current_accel_x is a number before comparison
    if isinstance(current_accel_x, (int, float)):
        # Only calculate difference if we have a previous valid reading
        if last_accel_x_for_diff is not None and isinstance(last_accel_x_for_diff, (int, float)):
            accel_diff = abs(current_accel_x - last_accel_x_for_diff)
            if accel_diff > ARDUINO_ACCEL_SUDDEN_MOVEMENT_THRESHOLD:
                # Note: Arduino's jerk detection counts jerks in a window.
                # Pi's check here is per data packet. This might be more sensitive or need window logic.
                # For simplicity, just check if the instantaneous difference is above threshold.
                is_sensor_indicator_active = True
                trigger_details.append(
                    f"Sensor Sudden Accel X Movement (Diff: {accel_diff} > {ARDUINO_ACCEL_SUDDEN_MOVEMENT_THRESHOLD})")
        last_accel_x_for_diff = current_accel_x # Update last reading regardless of diff check result
    else:
        last_accel_x_for_diff = None # Reset last accel if current is invalid/missing

    # print(f"[SENSOR L1 CHECK] Indicators active: {is_sensor_indicator_active}, Reasons: {trigger_details}") # Too verbose
    return is_sensor_indicator_active, trigger_details


def should_trigger_level1():
    """
    Determines if the Pi should send the 'Level 01' command trigger to the Arduino
    based on real camera drowsiness OR real sensor data received, respecting cooldown.
    This only happens if the Arduino currently reports Level 0 or 1.
    Returns True and a list of trigger details if triggered, False and empty list otherwise.
    """
    global last_level1_trigger_time, camera_drowsy_event_triggered, camera_yawn_detected, running # Check 'running' flag

    current_time = time.time()

    # Only trigger if:
    # 1. The main loop is running (`running` flag).
    # 2. Arduino reports being in a low state (0 or 1) (`current_warning_level`).
    # 3. Pi-side L1 trigger cooldown is over.
    # 4. latest_sensor_data has been populated at least once to check sensor indicators.
    cooldown_active = (current_time - last_level1_trigger_time) < LEVEL1_TRIGGER_COOLDOWN_SECONDS

    # Use current_warning_level which is updated by the serial thread based on Arduino data
    if not running or current_warning_level > 1 or cooldown_active or not latest_sensor_data:
        # print(f"[L1 TRIGGER] Not triggering. Running={running}, Arduino Level={current_warning_level}, Cooldown Active={cooldown_active}, Data Available={bool(latest_sensor_data)}") # Too verbose
        return False, [] # Return False and empty list if not triggered

    # Check real camera indicators (flags set in the main loop based on EAR/MAR/Timers)
    camera_indicates_drowsy = camera_drowsy_event_triggered or camera_yawn_detected
    camera_trigger_details = []
    if camera_drowsy_event_triggered:
        camera_trigger_details.append(f"Camera Eyes Closed Duration ({EYES_CLOSED_DURATION_THRESH}s)")
    if camera_yawn_detected:
        camera_trigger_details.append(f"Camera Yawn ({YAWN_CONSEC_FRAMES} frames)")


    # Check real sensor indicators (from latest_sensor_data parsed by serial thread)
    # Only check if sensor data has been received
    sensor_indicates_drowsy = False
    sensor_trigger_details = []
    if latest_sensor_data:
       sensor_indicates_drowsy, sensor_trigger_details = check_sensor_drowsiness_indicators(latest_sensor_data)


    # Trigger if *either* real camera indicates drowsiness *OR* real sensor indicates drowsiness
    if camera_indicates_drowsy or sensor_indicates_drowsy:
        all_trigger_details = camera_trigger_details + sensor_trigger_details
        last_level1_trigger_time = current_time # Reset cooldown timer
        print(f"[!] Pi-side LEVEL 1 trigger condition met: {' OR '.join(all_trigger_details)}. Sending 'Level 01' command to Arduino.") # Explicit print
        return True, all_trigger_details # Return True and the list of reasons
    else:
        # print("[L1 TRIGGER] No trigger conditions met (camera or sensor).") # Too verbose
        return False, []


# === Main Execution Block ===
# This ensures the code runs only when the script is executed directly
if __name__ == "__main__":
    # --- Initialize Twilio Client ---
    client = None # Initialize to None
    try:
        print("[*] Attempting to initialize Twilio client...")
        client = Client(TWILIO_SID, TWILIO_AUTH_TOKEN)
        print("[*] Twilio client initialized successfully.")

    except Exception as e:
        print(f"[!!!] CRITICAL WARNING: Could not initialize Twilio client. Check TWILIO_SID, TWILIO_AUTH_TOKEN, and network connection to api.twilio.com.")
        print(f"Error details: {e}")
        traceback.print_exc() # Print traceback for detailed error information
        client = None # Ensure client is None if initialization failed


    # === Initialize Serial Port ===
    ser = None # Initialize to None
    try:
        print(f"[*] Attempting to open serial port: {ARDUINO_SERIAL_PORT} at {ARDUINO_BAUD_RATE}...")
        # Configure serial port for reading and writing
        # timeout=0 means non-blocking, readline will be fast
        # timeout=None means blocking indefinitely
        # timeout=X means block for X seconds (good for readline)
        # Use timeout=SERIAL_READ_TIMEOUT for read_until
        ser = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_BAUD_RATE, timeout=SERIAL_READ_TIMEOUT)
        # Allow some time for the Arduino to reset after opening the serial port
        # This depends on the Arduino bootloader; a short delay is often needed.
        time.sleep(2) # Wait 2 seconds
        print("[*] Serial port opened successfully.")

    except serial.SerialException as e:
        print(f"[!!!] CRITICAL ERROR: Could not open serial port {ARDUINO_SERIAL_PORT}: {e}")
        print("[!!!] Please ensure:")
        print("      1. The Arduino is plugged in.")
        print("      2. The correct port is specified in ARDUINO_SERIAL_PORT.")
        print("      3. You have permissions to access the port (e.g., add user to 'dialout' group: 'sudo usermod -a -G dialout $USER').")
        print("      4. No other program is using the port.")
        sys.exit(1) # Exit with error code
    except Exception as e:
         print(f"[!!!] CRITICAL ERROR: An unexpected error occurred opening serial port: {e}")
         traceback.print_exc()
         sys.exit(1)


    # === Camera Initialization (Using IP Webcam Stream) ===
    print(f"[*] Attempting to open video stream: {CAMERA_STREAM_URL}")
    # Add cv2.CAP_FFMPEG explicitly as network streams often rely on it
    cap = cv2.VideoCapture(CAMERA_STREAM_URL, cv2.CAP_FFMPEG)

    # Check if the stream opened successfully initially. If not, print and exit.
    if not cap.isOpened():
        print(f"[!!!] CRITICAL ERROR: Could not open video stream {CAMERA_STREAM_URL}")
        print("[!!!] Please ensure:")
        print("      1. Your IP webcam app is running and streaming.")
        print("      2. Your phone and Raspberry Pi are on the SAME local network.")
        print("      3. Your phone's firewall/settings allow connections on the app's port.")
        print(f"      4. The IP address and port '{CAMERA_STREAM_URL}' are CORRECT.")
        print("      5. OpenCV on the Pi has FFmpeg support for network streams (`cv2.CAP_FFMPEG` used).")
        sys.exit(1) # Exit with error code
    else:
        print("[*] Video stream opened successfully.")
        # Optional: Set buffer size to 1 for potentially lower latency, if supported
        # try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) except cv2.error as e: print(f"[WARN] Failed to set CAP_PROP_BUFFERSIZE: {e}")


    # === Start Background Threads ===

    # 1. Real Serial Read Thread: Reads data from Arduino
    # Pass the serial object to the thread
    real_serial_thread = threading.Thread(target=real_serial_read_thread, args=(ser,), daemon=True)
    real_serial_thread.start()

    # 2. Headless Input Listener Thread: Listens for 't' + Enter (sends TEST) or 'n' (sends No)
    # Check if stdin is interactive before starting the input thread
    if sys.stdin.isatty():
        input_listener_thread = threading.Thread(target=headless_input_listener_thread, daemon=True)
        input_listener_thread.start()
    else:
        print("[*] stdin is not a TTY. Headless input listener disabled. Cannot use 't' or 'n'.")


    print("\n[*] System running with IP webcam stream and real serial communication.")
    print("[*] Monitoring sensor data (from Arduino) and camera feed (Pi side) for drowsiness.")
    print("[*] ThingSpeak updates sent periodically (every >={THINGSPEAK_UPDATE_INTERVAL_SECONDS} seconds) with 8 fields.")
    print("[*] WhatsApp alert on Level 3 + Motor Stop (sent once successfully).")
    print("[*] Audio alerts will play via default system audio output.")

    print("\nPress Ctrl+C to stop.")
    if sys.stdin.isatty():
        print("Type 't' followed by Enter to send TEST command to Arduino.")
        print("Type 'n' followed by Enter to send No command to Arduino (simulates Pi reporting online).")


    # Check if audio files exist and warn if not
    audio_files_check_passed = True
    for f in [AUDIO_FILE_LEVEL1, AUDIO_FILE_LEVEL2, AUDIO_FILE_LEVEL3]:
        if not os.path.exists(f):
            print(f"[!!!] WARNING: Audio file missing: {f}")
            audio_files_check_passed = False

    if not audio_files_check_passed:
        print("[!!!] WARNING: One or more audio files are missing. Audio alerts for missing files will NOT play.")
    else:
        print(f"[*] Audio files expected in {SCRIPT_DIR}.")

    # Also perform the mpg123 check here
    try:
        subprocess.run(['mpg123', '-V'], check=True, timeout=5, capture_output=True)
        print("[*] mpg123 check passed. Audio playback should work if files exist and output is configured.")
    except FileNotFoundError:
        print("[!!!] WARNING: mpg123 not found. Please install it (sudo apt-get install mpg123). Audio will NOT play.")
    except subprocess.CalledProcessError:
         print("[!!!] WARNING: mpg123 version check returned non-zero status. Audio playback might have issues.")
    except Exception as e:
         print(f"[!!!] WARNING: mpg123 check failed: {e}. Audio playback might have issues.")

    print("Ensure your audio output (e.g., Bluetooth speaker) is configured.")


    # --- Main Loop ---
    # Loop continues as long as the 'running' flag is True AND the L3 WhatsApp hasn't been sent
    try:
        reconnect_attempts = 0 # Counter for stream reconnection attempts
        # last_face_print_time = time.time() # Initialized globally

        while running and not level3_whatsapp_sent:
            current_time = time.time()

            # Check if any audio process started by play_audio has finished
            if audio_process is not None and audio_process.poll() is not None:
                # print("[AUDIO] Audio process finished.") # Too verbose
                audio_process = None  # Clear the reference if it finished

            # --- Periodically send "No" to Arduino ---
            # This keeps Arduino's Pi status timer reset, preventing it from entering Fail-safe *unless*
            # serial communication is broken for ARDUINO_PI_TIMEOUT duration.
            # Ensure serial is open before trying to write
            if ser and ser.isOpen() and (current_time - last_pi_online_signal_time) >= SERIAL_WRITE_INTERVAL_NO:
                send_serial_command("No")
                last_pi_online_signal_time = current_time # Update timer after sending

            # --- Get Frame from Video Stream ---
            # This blocks until a frame is available or fails.
            ret, frame = cap.read()

            # --- Handle Stream Read Errors and Attempt Reconnect ---
            if not ret:
                # Only print the error once per attempt cycle or periodically during attempts
                if reconnect_attempts == 0 or (current_time - last_face_print_time) >= RECONNECT_DELAY_SECONDS: # Use a delay for print
                     print(
                         f"[!!!] Error reading frame from stream or stream closed ({cap.isOpened()}). Attempt {reconnect_attempts + 1}/{MAX_RECONNECT_ATTEMPTS}).")
                     # Update last_face_print_time to avoid printing this repeatedly during the delay
                     last_face_print_time = current_time # Use this timer for throttling error prints

                reconnect_attempts += 1
                if reconnect_attempts <= MAX_RECONNECT_ATTEMPTS:
                    print(f"[*] Attempting to re-open video stream in {RECONNECT_DELAY_SECONDS} seconds...")
                    if cap is not None: cap.release() # Release the current object first
                    time.sleep(RECONNECT_DELAY_SECONDS) # Wait before trying to re-open
                    # Try re-opening. Keep using cv2.CAP_FFMPEG
                    cap = cv2.VideoCapture(CAMERA_STREAM_URL, cv2.CAP_FFMPEG)

                    if cap is not None and cap.isOpened(): # Check if the new capture object is valid and opened
                        print("[*] Successfully re-opened video stream.")
                        reconnect_attempts = 0 # Reset counter on success
                        # Try setting buffer again (optional)
                        # try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) except cv2.error as e: print(f"[WARN] Failed to set CAP_PROP_BUFFERSIZE on reconnect: {e}")
                        continue  # Skip processing this potentially bad frame and continue loop
                    else:
                       # print("[!!!] Failed to re-open video stream after attempt. Will retry.") # Too verbose during reconnects
                       pass # The loop will continue and hit this error block again

                else:
                    print(f"[!!!] Maximum reconnection attempts ({MAX_RECONNECT_ATTEMPTS}) reached. Signaling exit.")
                    running = False # Signal threads to stop
                    break  # Break out of the while loop if max attempts failed

            # If we successfully read a frame (ret is True)
            # Reset reconnect attempts ONLY if a frame was successfully read
            reconnect_attempts = 0

            # --- Dlib Face and Landmark Detection (Real) ---
            # Convert the frame to grayscale for Dlib
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            face_detected = False  # Assume no face until detected
            frame_ear = None  # EAR for this frame (None if no face or error)
            frame_mar = None  # MAR for this frame (None if no face or error)

            faces = []  # Initialize faces list

            # --- Check if Predictor is Loaded and Attempt Detection ---
            if predictor is None:
                # Only print warning once if predictor is not loaded
                if not predictor_warned:
                    print("[CAMERA] Dlib predictor not loaded, skipping face/landmark detection.")
                    predictor_warned = True
                # Detection cannot happen, face_detected remains False.
                # Reset camera-based event flags if predictor is not available
                camera_drowsy_event_triggered = False
                camera_yawn_detected = False
                blink_frame_count = 0
                yawn_frame_count = 0
                eye_closed_start_time = None

            else:  # Predictor is loaded, attempt detection
                # Detect faces in the grayscale frame
                # Use upsampling=1 for potentially better detection of smaller faces
                faces = detector(gray, 1)
                face_detected = len(faces) > 0  # Flag to track if a face is detected

                # --- Periodic Print of Face Detection Status ---
                # This replaces the visual "No Face Detected" text overlay
                if current_time - last_face_print_time >= FACE_PRINT_INTERVAL_SECONDS:
                    if face_detected:
                         print(f"[{time.strftime('%H:%M:%S')}] Face Detection Status: Face Detected.")
                    else:
                        print(f"[{time.strftime('%H:%M:%S')}] Face Detection Status: No Face Detected.")
                    last_face_print_time = current_time

                # Process the first face found (assuming one driver)
                if face_detected:
                    face_rect = faces[0] # Get the first detected face rectangle

                    # Determine the facial landmarks for the face region
                    try:
                        landmarks = predictor(gray, face_rect)
                        landmarks_points = []
                        for i in range(0, landmarks.num_parts):
                            landmarks_points.append((landmarks.part(i).x, landmarks.part(i).y))

                        # Convert to NumPy array for easier slicing
                        landmarks_points = np.array(landmarks_points)

                        # Extract eye and mouth landmarks using the defined indices
                        left_eye = landmarks_points[LEFT_EYE_IDX]
                        right_eye = landmarks_points[RIGHT_EYE_IDX]
                        mouth = landmarks_points[MOUTH_IDX]

                        # Calculate EAR and MAR
                        frame_ear = (eye_aspect_ratio(left_eye) + eye_aspect_ratio(right_eye)) / 2.0
                        frame_mar = mouth_aspect_ratio(mouth)

                        # --- Drowsiness and Yawn Detection Logic (Real) ---

                        # Check for eye closure (drowsiness indicator per frame)
                        if frame_ear is not None and frame_ear < EAR_THRESH:
                            blink_frame_count += 1  # Increment consecutive drowsy frames
                            # Start timer when eyes first close (or continue existing timer)
                            if eye_closed_start_time is None:
                                eye_closed_start_time = current_time
                                # print(f"[CAMERA] Eyes started closing at {eye_closed_start_time:.2f}") # Too verbose

                        else:  # Eyes are open (frame_ear >= EAR_THRESH OR frame_ear is None)
                            # Check if eyes were closed long enough to trigger a camera drowsiness *event*
                            if eye_closed_start_time is not None and (
                                    current_time - eye_closed_start_time) >= EYES_CLOSED_DURATION_THRESH:
                                # This flag indicates a full drowsiness event occurred.
                                camera_drowsy_event_triggered = True  # Set the event flag
                                # print(f"[CAMERA] Drowsiness event detected! Eyes closed for {current_time - eye_closed_start_time:.2f}s (>= {EYES_CLOSED_DURATION_THRESH}s)") # Print moved to L1 trigger


                            # Reset blink counter and timer if eyes open
                            blink_frame_count = 0
                            eye_closed_start_time = None

                        # Check for yawning (yawn indicator per frame)
                        if frame_mar is not None and frame_mar > MAR_THRESH:
                            yawn_frame_count += 1  # Increment consecutive yawn frames
                        else:  # Mouth is closed (frame_mar <= MAR_THRESH OR frame_mar is None)
                            # Check if yawning occurred long enough to trigger a camera yawn *event*
                            if yawn_frame_count >= YAWN_CONSEC_FRAMES:
                                # This flag indicates a full yawn event occurred.
                                camera_yawn_detected = True  # Set the event flag
                                # print(f"[CAMERA] Yawn event detected! Mouth open for {yawn_frame_count} frames (>= {YAWN_CONSEC_FRAMES} frames)") # Print moved to L1 trigger

                            # Reset yawn frame count if mouth closes
                            yawn_frame_count = 0

                        # Optional: Print EAR/MAR for debugging if needed
                        # if frame_ear is not None and frame_mar is not None:
                        #    print(f"EAR: {frame_ear:.2f}, MAR: {frame_mar:.2f}, Blink Frames: {blink_frame_count}, Yawn Frames: {yawn_frame_count}")


                    except Exception as e:
                        # Handle potential errors during landmark prediction or calculation
                        # (e.g., landmarks not found even if detector finds a box)
                        print(f"[CAMERA ERROR] Error during landmark processing for detected face: {e}")
                        traceback.print_exc() # Optional: print full traceback
                        # Reset detection state if processing fails for the current face
                        blink_frame_count = 0
                        yawn_frame_count = 0
                        eye_closed_start_time = None
                        camera_drowsy_event_triggered = False # Reset event flags if processing fails
                        camera_yawn_detected = False
                        frame_ear = None  # Clear frame specific values
                        frame_mar = None


                # This 'else' block handles "No face detected" (when predictor was loaded)
                else: # No face detected (and predictor was loaded)
                    # Reset counters/timers and event flags because face is lost
                    blink_frame_count = 0
                    yawn_frame_count = 0
                    eye_closed_start_time = None
                    camera_drowsy_event_triggered = False # Reset event flags
                    camera_yawn_detected = False
                    # frame_ear and frame_mar remain None

            # --- Pi-Side Level 1 Trigger Logic (sends command to Arduino) ---
            # This checks Pi's camera analysis AND the latest sensor data from Arduino.
            # It respects cooldown and Arduino's reported level.
            is_triggered, trigger_reasons = should_trigger_level1()
            if is_triggered:
                send_serial_command("Level 01") # Send the command to Arduino
                # Reset Pi-side Camera Event Flags *AFTER* Triggering L1 command
                # This prevents immediately re-triggering L1 command if condition persists
                # The cooldown in should_trigger_level1 also helps prevent spamming L1 commands.
                camera_drowsy_event_triggered = False
                camera_yawn_detected = False


            # --- React to Arduino's Current Warning Level ---
            # This level is updated by the serial thread based on Arduino's SENSOR_DATA
            if current_warning_level != previous_warning_level_for_print:
                print(f"[PI STATE] Pi registered system level change from Arduino: {previous_warning_level_for_print} -> {current_warning_level}")
                previous_warning_level_for_print = current_warning_level

            # Trigger audio alerts based on the current warning level
            # This function handles stopping previous audio and only plays if the level changes.
            trigger_audio_alert(current_warning_level)

            # Handle Level 3 actions (WhatsApp message)
            # Check if Arduino reports Level 3 AND motor is reported stopped.
            # Use motor_speed_reported updated by the serial thread.
            # Ensure we have received motor speed data at least once and it's <= 0 for 'stopped'.
            # latest_sensor_data must be populated for motor_speed_reported to be meaningful.
            if (current_warning_level == 3 and latest_sensor_data and
                isinstance(motor_speed_reported, (int, float)) and motor_speed_reported <= 0 and
                client is not None): # Check if Twilio client is initialized

                current_time = time.time()
                whatsapp_cooldown_over = (current_time - last_level3_whatsapp_time) > LEVEL3_WHATSAPP_COOLDOWN_SECONDS

                # --- DEBUG PRINT FOR WHATSAPP TRIGGER CONDITIONS ---
                # print(f"[{time.strftime('%H:%M:%S')}] [DEBUG WhatsApp] L3 Active={current_warning_level==3}, Motor Stopped Reported={motor_speed_reported <= 0}, Cooldown Over={whatsapp_cooldown_over}, Already Sent Flag={level3_whatsapp_sent}, Twilio Client Initialized={client is not None}") # Too verbose

                if whatsapp_cooldown_over and not level3_whatsapp_sent:
                    print("[!!!] LEVEL 3 Active AND Motor Stopped Condition Met! Attempting to Send Emergency WhatsApp.")
                    location_info_str, lat, lon = get_location_from_ip()
                    whatsapp_body = f"🚨 EMERGENCY ALERT! Driver unresponsive (System Level 3). Vehicle has stopped.\n{location_info_str}\nNeeds immediate attention!"
                    if lat is not None and lon is not None:
                        google_maps_url = f"https://www.google.com/maps?q={lat},{lon}"
                        whatsapp_body += f"\n\nGoogle Maps: {google_maps_url}"
                    # send_whatsapp_message updates last_level3_whatsapp_time and sets level3_whatsapp_sent flag on success
                    send_whatsapp_message(EMERGENCY_WHATSAPP, whatsapp_body)
                    # The loop condition checks level3_whatsapp_sent to terminate after sending.


            # --- ThingSpeak Update ---
            # Pass the raw latest_sensor_data dictionary, send_to_thingspeak parses for fields
            # send_to_thingspeak also reads Pi-side flags like camera_drowsy_event_triggered
            # Only attempt to send if we have received data from Arduino at least once
            if latest_sensor_data:
                send_to_thingspeak(latest_sensor_data)


            # Minimal delay to prevent aggressive polling and allow threads to run
            time.sleep(0.001) # Sleep for 1ms

    # Handle exit conditions
    except KeyboardInterrupt:
        # Handle Ctrl+C cleanly
        print("\n[*] Keyboard interrupt received. Signaling exit.")
        running = False  # Signal the main loop and threads to stop
    except Exception as e:
        print(f"[CRITICAL ERROR] An unexpected error occurred in the main loop: {e}")
        traceback.print_exc() # Print full traceback for unexpected errors
        running = False  # Signal exit


    finally:
        # Ensure cleanup happens
        print("\n[*] Starting cleanup process.")
        # The main 'running' flag controls the serial read thread and input listener now

        # Signal threads to stop (already done by setting 'running' flag in except/KeyboardInterrupt)
        # Give daemon threads a moment to check their flag before the main thread exits.
        print("[*] Waiting briefly for background threads to finish...")
        time.sleep(0.5) # Give threads half a second

        # Stop any playing audio
        stop_audio()

        # Release the video capture object
        if 'cap' in locals() and cap is not None and cap.isOpened():
            print("[*] Releasing video capture.")
            cap.release()

        # Serial port is closed by the serial thread upon exiting its loop
        # if 'ser' in globals() and ser is not None and ser.isOpen():
        #    print("[*] Serial port will be closed by the serial thread.")
            # ser.close() # Explicit close here is not needed if the thread handles it

        # cv2.destroyAllWindows() is not needed in headless mode.

        print("[*] Script finished.")