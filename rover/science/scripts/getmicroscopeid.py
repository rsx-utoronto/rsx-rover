import subprocess

def get_usb_camera_device(camera_name):
    # Run the v4l2-ctl command and capture the output
    result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)

    # Split the output into lines
    lines = result.stdout.splitlines()
    
    usb_camera_device = None
    found_usb_camera = False

    for line in lines:
        # Check for the USB camera device header
        if camera_name in line:
            found_usb_camera = True
            continue
        
        # If we are in the USB camera section, look for the device path
        if found_usb_camera:
            if line.strip():  # If the line is not empty
                usb_camera_device = line.strip()
                break  # Stop after getting the first device path

    return usb_camera_device
