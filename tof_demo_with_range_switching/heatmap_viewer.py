import serial
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Successfully connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# --- Setup the Plot ---
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_title("VL53L8CX Live Auto-Scaling Heatmap")

# Start with a default 8x8 grid visualization
grid = np.zeros((8, 8))
cax = ax.imshow(grid, cmap='inferno', interpolation='nearest', vmin=0, vmax=2000)
fig.colorbar(cax, label='Distance (mm)')

# State tracking variables
current_size = 8 
annotations = [] # Holds the text objects for the overlay

# Define valid status codes
VALID_STATUS_CODES = [5, 6, 9]

def update(frame):
    global current_size, annotations
    
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            data = json.loads(line)
            
            if 'distances' in data and 'status' in data:
                num_zones = len(data['distances'])
                
                # Determine dimensions and max distance
                if num_zones == 64:
                    new_size = 8
                    new_vmax = 2000 
                    font_size = 8 # Smaller font for 8x8
                elif num_zones == 16:
                    new_size = 4
                    new_vmax = 4000 
                    font_size = 12 # Larger font for 4x4
                else:
                    return cax, 
                
                # --- Handle Resolution Changes ---
                if new_size != current_size:
                    cax.set_extent([-0.5, new_size-0.5, new_size-0.5, -0.5])
                    cax.set_clim(vmin=0, vmax=new_vmax)
                    current_size = new_size
                    
                    # Clear out the old text overlays when switching modes
                    for txt in annotations:
                        txt.remove()
                    annotations.clear()

                # --- Update Heatmap Colors ---
                dist_grid = np.array(data['distances']).reshape((new_size, new_size))
                status_grid = np.array(data['status']).reshape((new_size, new_size))
                cax.set_data(dist_grid)
                
                # --- Handle Text Overlays ---
                # 1. If the annotations list is empty, create them
                if len(annotations) == 0:
                    for i in range(new_size): # row (y)
                        for j in range(new_size): # col (x)
                            # Create an empty text object
                            txt = ax.text(j, i, "", ha="center", va="center", 
                                          fontsize=font_size, fontweight='bold')
                            annotations.append(txt)
                
                # 2. Update the string values and colors for the current frame
                idx = 0
                for i in range(new_size):
                    for j in range(new_size):
                        d = dist_grid[i, j]
                        s = status_grid[i, j]
                        
                        # Format: Distance on top, Status in parentheses below
                        text_str = f"{d}\n({s})"
                        annotations[idx].set_text(text_str)
                        
                        # Apply conditional coloring based on status
                        if s in VALID_STATUS_CODES:
                            annotations[idx].set_color("cyan")  # Good reading
                        else:
                            annotations[idx].set_color("red")   # Error/weak reading
                            
                        idx += 1
                
    except json.JSONDecodeError:
        pass 
    except Exception as e:
        pass 
        
    return cax,

# Run the animation
ani = animation.FuncAnimation(fig, update, interval=30, blit=False)
plt.tight_layout()
plt.show()

ser.close()
