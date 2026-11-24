import serial
import pandas as pd
import os
from datetime import datetime, timezone
from pathlib import Path
import time

script_dir = Path(__file__).resolve().parent

# Configure the serial port
# For Windows, it might be 'COM3' or similar.
# For Linux/Raspberry Pi, it might be '/dev/ttyUSB0' or '/dev/ttyACM0'.
# For macOS, it might be '/dev/tty.usbmodem...' or '/dev/tty.SLAB_USBtoUART'.
# Ensure the baudrate matches the transmitting device (e.g., Arduino or microcontroller).


ser = serial.Serial(
    port='/dev/ttyUSB3',  # Replace with your port name
    baudrate=115200,
    timeout=1  # Set a timeout (in seconds) to prevent the program from blocking indefinitely
)

# Clear the input buffer
ser.reset_input_buffer()

# Create Empty Dataframe
df = pd.DataFrame(columns=[
                            'Time_s',
                            'PacketID',
                            'Voltage1_mV',
                            'Voltage2_mV',
                            'Voltage3_mV',
                            'temp_C',
                            # 'rtd_C',
                            'pressure_hPa',
                            'aanderaa_cond',
                            'aanderaa_sal',
                            'aanderaa_temp',
                            'rssi',
                            'snr'])

# start timer
start_time = time.monotonic()


try:
    print("Logger started.")
    print("Press Ctrl+C to stop and save.")
    print()

    while True:
        # Read a line of bytes terminated by a newline character
        if ser.in_waiting > 0:
            encoded_data = ser.readline()


            # Decode the bytes to a string and remove leading/trailing whitespace (like \r\n)
            decoded_string = encoded_data.decode('utf-8').strip()
            
            if decoded_string: # Process the string if it's not empty
                data = decoded_string.split(',')

                if data[0].lower() == 'packetid':
                    continue

                if len(data) != 11:
                    print(f"Skipping bad line: {decoded_string}")
                    continue

                utc_now = datetime.now(timezone.utc)
                elapsed_time_s = time.monotonic() - start_time
            

                data_df = pd.DataFrame([{
                    'Time_s': elapsed_time_s,
                    'PacketID': data[0],
                    'Voltage1_mV': float(data[1]),
                    'Voltage2_mV': float(data[2]),
                    'Voltage3_mV': float(data[3]),
                    'temp_C': float(data[4]),
                    # 'rtd_C': float(data[3]),
                    'pressure_hPa': float(data[5]),
                    'aanderaa_cond': float(data[6]),
                    'aanderaa_sal': float(data[7]),
                    'aanderaa_temp': float(data[8]),
                    'rssi': float(data[9]),
                    'snr': float(data[10])
                }])
                
                df = pd.concat([df, data_df], ignore_index=True) 
                print(data)

except KeyboardInterrupt:
    print("Exiting Program")

finally:
    
    utc_now = datetime.now(timezone.utc)
    filename = f"recording_{utc_now.strftime('%y-%m-%d_%H-%M-%S')}.csv"
    foldername = "CTD_Recordings"
    filepath = script_dir / foldername
    os.makedirs(filepath, exist_ok=True)

    df.to_csv(filepath / filename, index=False)
    ser.close() # Always close the port when done

    print(f"Data saved to {filepath / filename}")
