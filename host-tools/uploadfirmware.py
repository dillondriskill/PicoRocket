from pathlib import Path
import serial
import os
import math

OK = b'\0'

def main():
    os.system("clear")

    print("Welcome to the PicoRocket host tools firmware uploader V0.1\n")

    # Get open the Pico as a file we can write to in byte mode
    device_path = input("Enter the device path: ")
    device = serial.Serial(device_path, 115200)
    print("Opened device: " + device.name)

    # Open the file to upload using a Path type, so we can read it as an array of characters
    file_path_str = input("Enter the filepath to the program to upload: ")
    file_path = Path(file_path_str)
    file = file_path.read_bytes()
    print("Opened file: " + file_path_str + "\n")
    
    print("Commanding device to enter programming mode...\n")
    device.write(b'p\r')

    # Wait for response from device. We wait until the OK signal because it will also type the user prompt
    while device.read() != OK:
        pass
    print("Device is ready")
    print()

    # Send length 
    filelen = len(file)
    numpages = math.ceil(filelen/256) # Number of pages, rounded up to next whole page
    print("File length is: " + str(filelen))
    print("Number of pages: " + str(numpages))
    print("Uploading to device: " + str(numpages.to_bytes(1, 'little')))
    device.write(numpages.to_bytes(1, 'little'))

    # Recieve acknowledgement
    returned = device.read()
    print("Device reported readiness for: " + str(returned) +  " pages")
    print()

    # Send data
    print("Uploading data...")
    written = 0
    while written < (256 * numpages):
        # Write page
        print("Writing page " + str(int(written/256)))
        for i in range(0, 256):
            if written < len(file):
                device.write(file[i].to_bytes(1, 'big'))
            else:
                device.write(b'\x00')
            written += 1
        # Recieve acknowledgement
        print("Device returned: " + str(device.read()))
    
    print("Data uploaded")
    print("Finished")
    device.close()
    return

if __name__ == "__main__":
    main()
