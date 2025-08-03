'''
This script configures the RPi5 to:
1) act as an MQTT client ready to subscribe and publish to topics on the Mosquitto broker
2) visualize the received data via a GUI
'''


# --- Importing Libraries ---
# For configuring the RPi5 as MQTT Client
import paho.mqtt.client as mqtt

# For data visualization
import tkinter as tk
from tkinter import font
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# For multi-threading
import queue
import struct

# For logging
import csv
import datetime


# --- Globals ---
# For MQTT
BROKER_ADDRESS = "localhost"                    # This indicates that the broker on the same device as this client
FFT_DATA_TOPIC = "esp32/fft_data"               # This is the topic for receiving the processed FFT data
MOTOR_STATE_TOPIC = "esp32/motor_state"         # This is the topic for receiving the motor state
MOTOR_OVERRIDE_TOPIC = "rpi5/motor_override"    # This is the topic for sending the motor override commands

# For FFT
FFT_POINTS = 1024
FFT_PAYLOAD_SIZE = 6144                         # 1024 points * 6 bytes/point = 6144 bytes

# For Multi-Threading
data_queue = queue.Queue()                      # A thread-safe queue to pass data from the MQTT thread to the GUI thread


# --- MQTT Client Thread ---

class MqttClient:
    '''This class handles all MQTT communication in the background.'''
    def __init__(self):
        '''This configures the RPi5 as an MQTT client.'''
        self.client = mqtt.Client(client_id="rpi_dashboard_client") # We create a unique client ID for the RPi5
        # NOTE: By default, paho-mqtt will attempt to connect using MQTT v3.1.1
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def connect(self):
        '''This method is for connecting the RPi5 client to the Mosquitto broker.
        Mosquitto is always active and running on RPi5 by default.'''
        # Send a CONNECT packet from the RPi5 to the Mosquitto broker:
        self.client.connect(BROKER_ADDRESS, 1883, 60)
        # loop_start() runs the MQTT client in a background thread.
        # This is crucial for not freezing the GUI.
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        '''
        This is the callback for when the client receives a CONNACK response from the server.
        A CONNACK or "Connection Acknowledgment" packet is the direct response
        sent from the Mosquitto broker back to the client that has just sent a CONNECT packet.
        
        Parameters:
        - client: This is the paho-mqtt client instance that initiated the connection. We use this to
        subscribe to certain topics AFTER the MQTT connection with the broker has been established.
        - userdata: This is a placeholder that can be used to pass custom user data, created alongside the client.
        In this code, this is unused and defaults to None.
        - flags: This is a dictionary containing response flags sent by the broker. The flag flags['session present']
        indicates whether the broker is resuming a previous session for this client.
        If 0, it's a new, clean session. If 1, the broker remembered this client's previous subscriptions.
        - rc: Every CONNACK packet contains a "Return Code" / rc. rc = 0 indicates connection was successfully established.
        rc > 0 indicates connection failed to establish.
        '''
        if rc == 0:     # Indicates connection was successfully established
            print("Connected to MQTT Broker!")
            # Subscribing to the topics the ESP32 will publish to:
            client.subscribe(FFT_DATA_TOPIC, qos=2)     # QoS 2 ensures that the FFT data is recevied exactly once
            print(f"Subscribed to: {FFT_DATA_TOPIC}")
            client.subscribe(MOTOR_STATE_TOPIC, qos=1)  # QoS 1 ensures that the motor state is received at least once
            print(f"Subscribed to: {MOTOR_STATE_TOPIC}")
        else:           # Indicates connection failed to establish
            print(f"Failed to connect, return code {rc}\n")

    def on_message(self, client, userdata, msg):
        '''
        This is the callback for when the RPi5 client receives a message from the broker.
        
        Parameters:
        - client: This is the paho-mqtt client instance that received the message.
        - userdata: This is a placeholder that can be used to pass custom user data, created alongside the client.
        In this code, this is unused and defaults to None.
        - msg: This object contains all the information about the received message.
            - msg.topic: The topic on which the message was published.
            - msg.payload: The actual content of the message in raw bytes.
            - msg.qos: The Quality of Service level of the message (0, 1, 2).
            - msg.retain: True if this is a "retained" message (the last known good value for a topic, saved by the broker)
        '''
        # Check which topic the message arrived on:
        if msg.topic == FFT_DATA_TOPIC:
            if len(msg.payload) == FFT_PAYLOAD_SIZE:
                # Put the raw binary payload into the queue for the GUI to process:
                data_queue.put(('fft', msg.payload))
            else:
                print(f"Warning: Received unexpected FFT payload size: {len(msg.payload)}")
        
        elif msg.topic == MOTOR_STATE_TOPIC:
            # Decode the motor state ("ON" or "OFF") and put it in the queue:
            state = msg.payload.decode('utf-8')
            data_queue.put(('motor_state', state))

    def publish_command(self, command):
        """This method is used for publishing an override command to control the motor."""
        self.client.publish(MOTOR_OVERRIDE_TOPIC, command, qos=1)
        # This publishes our command to the MOTOR_OVERRIDE_TOPIC, using QoS 1,
        # which gaurantees that the command will be received at least once (duplicates are possible).
        print(f"Sent command to {MOTOR_OVERRIDE_TOPIC}: {command}")


# --- GUI Thread ---

class RpiDashboardApp:
    def __init__(self, root, mqtt_client):
        self.root = root
        self.mqtt_client = mqtt_client
        self.root.title("ESP32 Control Dashboard")
        self.root.configure(bg='#f0f0f0')

        # --- Matplotlib FFT Plot ---
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.fig.tight_layout(pad=3)
        self.ax.set_title("Live FFT Data from ESP32")
        self.ax.set_xlabel("Frequency Bin")
        self.ax.set_ylabel("Normalized Magnitude Squared")
        self.ax.set_xlim(0, FFT_POINTS)
        self.ax.grid(True)
        self.ax.set_yscale('log') # Using a log scale to display the FFT data on y-axis. This compresses the large values and expands the small values.

        # Initialize the plot with empty data
        self.line, = self.ax.plot([], [], lw=1.5)

        # Embed the plot into the Tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

        # --- Control and Status Frame ---
        control_frame = tk.Frame(self.root, bg='#f0f0f0')
        control_frame.pack(pady=10, fill=tk.X)

        # Motor Status Indicator
        status_font = font.Font(family='Helvetica', size=12, weight='bold')
        self.status_label = tk.Label(control_frame, text="Motor Status: UNKNOWN", font=status_font, bg='#f0f0f0', width=25)
        self.status_label.pack(side=tk.LEFT, padx=20)

        # Motor Control Buttons
        button_font = font.Font(family='Helvetica', size=12)
        on_button = tk.Button(control_frame, text="Turn Motor ON", command=self.turn_motor_on, font=button_font, bg='#4CAF50', fg='white', width=15)
        on_button.pack(side=tk.LEFT, padx=10)

        off_button = tk.Button(control_frame, text="Turn Motor OFF", command=self.turn_motor_off, font=button_font, bg='#f44336', fg='white', width=15)
        off_button.pack(side=tk.LEFT, padx=10)

        # Start the animation to update the plot
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False)

        # --- Initialization for logging ---
        try:
            # 1. Define the log file name
            log_filename = f"datalog_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            print(f"Logging data to {log_filename}")

            # 2. Open the file in 'append' mode. 'newline=""' is needed for csv.
            self.log_file = open(log_filename, 'a', newline='')
            
            # 3. Create a CSV writer object
            self.csv_writer = csv.writer(self.log_file)

            # 4. Write the header row to the CSV file
            header = [
                'timestamp',
                'event_type',
                'fft_avg_magnitude',
                'fft_max_magnitude',
                'fft_peak_bin',
                'motor_state'
            ]
            self.csv_writer.writerow(header)
            
            # 5. Set up a graceful shutdown to close the file
            self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        except IOError as e:
            print(f"Error: Could not open log file for writing: {e}")
            self.log_file = None
            self.csv_writer = None

    def on_close(self):
        '''
        This method handles the event when the GUI window closes.
        - It ensures the log file is properly saved.
        - It stops the MQTT background thread upon closing the GUI window.
        '''
        print("Closing application and saving log file...")
        if self.log_file:
            self.log_file.close()
        
        # When the GUI is closed, stop the MQTT client's background thread:
        mqtt_handler.client.loop_stop()

        self.root.destroy()

    def turn_motor_on(self):
        self.mqtt_client.publish_command("ON")

    def turn_motor_off(self):
        self.mqtt_client.publish_command("OFF")

    def update_plot(self, frame):
        '''This method is called periodically by FuncAnimation.'''
        try:
            # Process all pending items in the queue
            while not data_queue.empty():
                data_type, payload = data_queue.get_nowait()

                # Timestamp for the log entry
                timestamp = datetime.datetime.now().isoformat()
                
                if data_type == 'fft':
                    if len(payload) == FFT_PAYLOAD_SIZE: # Should be 6144 bytes
                        
                        # 1. Reshape the incoming 6-byte data.
                        reshaped_data = np.frombuffer(payload, dtype=np.uint8).reshape(FFT_POINTS, 6)
                        
                        # 2. Create an 8-byte padded buffer and pad with zeros at the BEGINNING, due to Big-Endian format.
                        #    This creates a target array that can hold 1024 of our 8-byte (64-bit) integers.
                        padded_data = np.zeros((FFT_POINTS, 8), dtype=np.uint8)
                        padded_data[:, 2:] = reshaped_data # Copy 6 bytes into the last 6 slots of the 8-byte buffer
                        
                        # 3. View the data as a Big-Endian 64-bit integer.
                        #    The '>' symbol tells numpy to use Big-Endian byte order.
                        magnitudes = padded_data.view(dtype='>u8') # '>u8' is Big-Endian Unsigned 8-byte
                        
                        # 4. Update the plot data
                        x_data = np.arange(FFT_POINTS)
                        self.line.set_data(x_data, np.clip(magnitudes, 1, None))
                        # np.clip(array, min_value, max_value)
                        # We set min_value=1 and max_value=None (meaning no upper limit).
                        # This effectively replaces all values < 1 (including zeros) with 1.
                        self.ax.relim()     # Recalculate plot limits
                        self.ax.autoscale_view(True,True,True) # Autoscale y-axis

                        # 5. Log the FFT stats
                        if self.csv_writer:
                            # Calculate summary statistics
                            max_mag = np.max(magnitudes)
                            avg_mag = np.mean(magnitudes)
                            peak_bin = np.argmax(magnitudes)
                            
                            # Write the data row
                            log_row = [timestamp, 'FFT_DATA', avg_mag, max_mag, peak_bin, '']
                            self.csv_writer.writerow(log_row)

                    else:
                        print(f"Warning: Received unexpected FFT payload size: {len(payload)}")

                elif data_type == 'motor_state':
                    self.status_label.config(text=f"Motor Status: {payload}")
                    if payload == "ON":
                        self.status_label.config(fg='#4CAF50') # Green
                    else:
                        self.status_label.config(fg='#f44336') # Red
                    
                    # Log the motor state
                    if self.csv_writer:
                        # Write the data row (leaving FFT columns blank)
                        log_row = [timestamp, 'MOTOR_STATE', '', '', '', payload]
                        self.csv_writer.writerow(log_row)

        except queue.Empty:
            pass  # No new data, just continue
        
        # Return the plot line object to be redrawn
        return self.line,

    def run(self):
        self.root.mainloop()


# --- Main Execution ---
if __name__ == "__main__":
    # 1. Start the MQTT Client thread in the background
    mqtt_handler = MqttClient()     # Configure the RPi5 as an MQTT client
    mqtt_handler.connect()          # Connect to the Mosquitto broker

    # 2. Set up and run the GUI thread
    root_window = tk.Tk()
    app = RpiDashboardApp(root_window, mqtt_handler)
    app.run()

    # Print when the GUI window is closed:
    print("Application closed.")
