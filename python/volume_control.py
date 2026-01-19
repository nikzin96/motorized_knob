# Author: Nikolas Zingraf
# Description: Python script to interface with the motorized volume knob to control windows volume over BLE.
#              You can use nuitka or similar to create an exectuable and add it to your task scheduler to always run in the background.

from ctypes import POINTER, cast
import asyncio
import sys
from bleak import BleakClient, BleakScanner
import multiprocessing
from multiprocessing import Process, Queue
import traceback
import time

# BLE UUIDs (must match ESP32)
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
RX_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # Write to ESP32
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Read from ESP32

BT_DEVICE_NAME = "VolumeKnob"

is_muted = False
last_volume = None
last_mute_sent = None

is_pc_volume_change = False
ignore_knob_until_time = 0
last_unmute_time = 0
ignore_next_pc_volume = False
knob_just_unmuted = False  # Track if knob just unmuted (more specific than ignore_next_pc_volume)

# Replace the com_worker_process function to refresh the device periodically

def com_worker_process(cmd_queue, response_queue):
    """Separate process for COM operations with audio endpoint notification callback"""
    import pythoncom
    import comtypes
    from comtypes import CLSCTX_ALL
    from pycaw.pycaw import IAudioEndpointVolume, IMMDeviceEnumerator, IAudioEndpointVolumeCallback
    from pycaw.constants import CLSID_MMDeviceEnumerator
    from ctypes import POINTER, cast
    import ctypes
    import time
    
    pythoncom.CoInitialize()
    
    # Define the notification data structure properly
    class AUDIO_VOLUME_NOTIFICATION_DATA(ctypes.Structure):
        _fields_ = [
            ('guidEventContext', comtypes.GUID),
            ('bMuted', ctypes.c_bool),
            ('fMasterVolume', ctypes.c_float),
            ('nChannels', ctypes.c_uint),
            ('afChannelVolumes', ctypes.c_float * 1)
        ]
    
    # Audio endpoint callback class
    class AudioEndpointVolumeCallback(comtypes.COMObject):
        _com_interfaces_ = [IAudioEndpointVolumeCallback]
        
        def __init__(self, response_queue, initial_volume=None, initial_mute=None, skip_first_notification=False):
            self.response_queue = response_queue
            self.last_mute = initial_mute
            self.last_volume = initial_volume
            self.last_send_time = 0
            self.min_send_interval = 0.05
            self.is_muted = initial_mute if initial_mute is not None else False
            self.knob_changing = False
            self.last_knob_time = 0
            self.knob_timeout = 0.3
            self.skip_first_notification = skip_first_notification
            self.first_notification_received = False
            super().__init__()
        
        def OnNotify(self, pNotify):
            try:
                notify_data = ctypes.cast(pNotify, POINTER(AUDIO_VOLUME_NOTIFICATION_DATA)).contents
                
                current_time = time.time()
                muted = bool(notify_data.bMuted)
                volume_percent = int(notify_data.fMasterVolume * 100)
                
                # Skip first notification if this callback was created for a device change
                if self.skip_first_notification and not self.first_notification_received:
                    self.first_notification_received = True
                    self.last_mute = muted
                    self.last_volume = volume_percent
                    return 0
                
                self.first_notification_received = True
                
                # Update knob_changing state based on timeout
                if self.knob_changing and (current_time - self.last_knob_time) > self.knob_timeout:
                    self.knob_changing = False
                
                # Only send mute changes if they're not caused by volume changes
                if muted != self.last_mute and not self.knob_changing:
                    if current_time - self.last_send_time > 0.1:
                        self.response_queue.put(("mute_changed", muted))
                        self.last_mute = muted
                        self.is_muted = muted
                        self.last_send_time = current_time
                
                # Only send volume changes if they're not from the knob
                if (current_time - self.last_send_time) >= self.min_send_interval:
                    if self.last_volume is None or abs(volume_percent - self.last_volume) >= 1:
                        if not self.knob_changing:
                            self.response_queue.put(("volume_changed", volume_percent))
                            self.last_volume = volume_percent
                            self.last_send_time = current_time
                
            except Exception as e:
                pass
            
            return 0
    
    def get_audio_interface_id():
        """Get unique ID of currently selected default audio endpoint"""
        try:
            enum = comtypes.CoCreateInstance(
                CLSID_MMDeviceEnumerator,
                IMMDeviceEnumerator,
                comtypes.CLSCTX_INPROC_SERVER
            )
            device = enum.GetDefaultAudioEndpoint(0, 1)
            device_id = device.GetId()
            return device_id
        except Exception as e:
            print(f"Error getting device ID: {e}")
            return None
    
    def get_audio_interface():
        """Get the currently selected default audio endpoint"""
        try:
            enum = comtypes.CoCreateInstance(
                CLSID_MMDeviceEnumerator,
                IMMDeviceEnumerator,
                comtypes.CLSCTX_INPROC_SERVER
            )
            device = enum.GetDefaultAudioEndpoint(0, 1)
            vol_interface = device.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
            return cast(vol_interface, POINTER(IAudioEndpointVolume))
        except Exception as e:
            print(f"Error getting audio interface: {e}")
            return None
    
    vol_interface = None
    callback = None
    current_device_id = None
    last_device_check = 0
    device_check_interval = 2.0  # Check for device changes every 2 seconds
    
    try:
        vol_interface = get_audio_interface()
        current_device_id = get_audio_interface_id()
        if vol_interface:
            # Get initial volume and mute state
            try:
                initial_volume = int(vol_interface.GetMasterVolumeLevelScalar() * 100)
                initial_mute = bool(vol_interface.GetMute())
            except:
                initial_volume = None
                initial_mute = None
            
            callback = AudioEndpointVolumeCallback(response_queue, initial_volume, initial_mute, skip_first_notification=False)
            vol_interface.RegisterControlChangeNotify(callback)
            response_queue.put(("ready", None))
            print("COM worker process initialized with audio callbacks")
        else:
            response_queue.put(("error", "Failed to get audio interface"))
            return
        
        while True:
            current_time = time.time()
            
            # Periodically check if audio device has changed
            if (current_time - last_device_check) > device_check_interval:
                try:
                    new_device_id = get_audio_interface_id()
                    # Compare device IDs instead of COM pointers
                    if new_device_id and new_device_id != current_device_id:
                        print(f"Audio device changed from {current_device_id} to {new_device_id}")
                        # Unregister old callback
                        if callback and vol_interface:
                            try:
                                vol_interface.UnregisterControlChangeNotify(callback)
                            except Exception as e:
                                print(f"Error unregistering old callback: {e}")
                        
                        # Get new interface
                        new_interface = get_audio_interface()
                        if new_interface:
                            vol_interface = new_interface
                            current_device_id = new_device_id
                            
                            # Get current volume and mute state from new device
                            try:
                                current_volume = int(vol_interface.GetMasterVolumeLevelScalar() * 100)
                                current_mute = bool(vol_interface.GetMute())
                            except Exception as e:
                                print(f"Error getting state after device change: {e}")
                                current_volume = None
                                current_mute = None
                            
                            # Create new callback with current state (don't skip first notification - just initialize state)
                            callback = AudioEndpointVolumeCallback(response_queue, current_volume, current_mute, skip_first_notification=False)
                            vol_interface.RegisterControlChangeNotify(callback)
                            
                            # Send the state to knob
                            if current_volume is not None and current_mute is not None:
                                response_queue.put(("device_changed", (current_volume, current_mute)))
                                print(f"Device switched - sending volume {current_volume}% and mute {current_mute} to knob")
                            else:
                                response_queue.put(("device_changed", None))
                except Exception as e:
                    print(f"Error checking device: {e}")
                
                last_device_check = current_time
            
            try:
                if not cmd_queue.empty():
                    cmd, data = cmd_queue.get(timeout=0.1)
                    
                    if cmd == "exit":
                        break
                    elif cmd == "set_volume":
                        if vol_interface:
                            try:
                                vol_interface.SetMasterVolumeLevelScalar(data, None)
                                # Update callback to ignore the volume change we just made
                                if callback:
                                    callback.knob_changing = True
                                    callback.last_knob_time = time.time()
                            except Exception as e:
                                print(f"Error setting volume: {e}")
                    elif cmd == "set_mute":
                        if vol_interface:
                            try:
                                vol_interface.SetMute(data, None)
                                if callback:
                                    callback.is_muted = bool(data)
                            except Exception as e:
                                print(f"Error setting mute: {e}")
                    elif cmd == "get_mute":
                        if vol_interface:
                            try:
                                result = bool(vol_interface.GetMute())
                                response_queue.put(("mute_state", result))
                            except Exception as e:
                                print(f"Error getting mute: {e}")
                    elif cmd == "get_volume":
                        if vol_interface:
                            try:
                                volume = vol_interface.GetMasterVolumeLevelScalar()
                                volume_percent = int(volume * 100)
                                response_queue.put(("volume_state", volume_percent))
                            except Exception as e:
                                print(f"Error getting volume: {e}")
                    elif cmd == "knob_active":
                        if callback:
                            callback.knob_changing = True
                            callback.last_knob_time = time.time()
                    elif cmd == "reset_knob_flag":
                        if callback:
                            callback.knob_changing = False
                            callback.last_knob_time = 0  # Reset the timeout timer
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"COM error: {e}")
                import traceback
                traceback.print_exc()
        
        # Unregister callback before exit
        if vol_interface and callback:
            try:
                vol_interface.UnregisterControlChangeNotify(callback)
            except:
                pass
        
    finally:
        pythoncom.CoUninitialize()
        print("COM worker process terminated")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


async def find_device():
    """Scan for the VolumeKnob BLE device"""
    print(f"Scanning for BLE device '{BT_DEVICE_NAME}'...")
    devices = await BleakScanner.discover(timeout=10.0)
    
    for device in devices:
        if device.name == BT_DEVICE_NAME:
            print(f"Found {BT_DEVICE_NAME} at {device.address}")
            return device.address
    
    print(f"Device '{BT_DEVICE_NAME}' not found")
    return None


async def notification_handler(sender, data, com_queue):
    """Handle notifications from ESP32"""
    global is_muted, last_volume, ignore_knob_until_time, ignore_next_pc_volume, last_mute_sent, knob_just_unmuted
    
    try:
        message = data.decode('utf-8').strip()
        
        if message.lower() == "mute":
            com_queue.put(("set_mute", 1))
            is_muted = True
            knob_just_unmuted = False
        elif message.lower() == "unmute":
            com_queue.put(("set_mute", 0))
            is_muted = False
            knob_just_unmuted = True  # Set flag to skip only the IMMEDIATE echo
            print(f"Knob unmuted -> will skip immediate PC volume echo to preserve knob's volume")
        else:
            # Try to parse as volume value
            try:
                val = int(message)
                val = clamp(val, 0, 100)
                scalar = val / 100.0
                
                # Ignore knob volume changes within the ignore window
                current_time = time.time()
                if current_time < ignore_knob_until_time:
                    print(f"Ignoring knob volume {val}% (within ignore window, {ignore_knob_until_time - current_time:.2f}s remaining)")
                    return
                
                # Always accept volume changes from knob (reset_last_volume for device changes)
                if last_volume is None:
                    print(f"Knob volume {val}% (first value after device change)")
                    com_queue.put(("knob_active", True))
                    com_queue.put(("set_volume", scalar))
                    last_volume = scalar
                elif abs(scalar - last_volume) >= 0.005:
                    print(f"Knob volume {val}% (changed from {int(last_volume * 100)}%)")
                    # Mark that knob is changing volume
                    com_queue.put(("knob_active", True))
                    com_queue.put(("set_volume", scalar))
                    last_volume = scalar
                # else: volume didn't change enough, ignore it
            except ValueError:
                pass
    except Exception as e:
        print(f"Error handling notification: {e}")


async def connect_and_run(address, com_queue, com_response_queue):
    """Connect to the BLE device and handle communication"""
    global is_muted, is_pc_volume_change, ignore_knob_until_time, last_unmute_time, ignore_next_pc_volume, last_mute_sent, knob_just_unmuted
    
    async with BleakClient(address) as client:
        print(f"Connected to {BT_DEVICE_NAME}")
        
        # Debug: List all available services and characteristics
        print("Available services:")
        for service in client.services:
            print(f"  Service: {service.uuid}")
            for char in service.characteristics:
                print(f"    Characteristic: {char.uuid}")
        
        # Try to get characteristics with error handling
        try:
            rx_char = client.services.get_characteristic(RX_CHAR_UUID)
            tx_char = client.services.get_characteristic(TX_CHAR_UUID)
            
            if rx_char is None or tx_char is None:
                print(f"ERROR: Could not find characteristics!")
                print(f"  RX_CHAR ({RX_CHAR_UUID}): {'Found' if rx_char else 'NOT FOUND'}")
                print(f"  TX_CHAR ({TX_CHAR_UUID}): {'Found' if tx_char else 'NOT FOUND'}")
                return
        except Exception as e:
            print(f"Error getting characteristics: {e}")
            return
        
        # Set flag to ignore knob values for the first second after connection
        ignore_knob_until_time = time.time() + 1.0
        
        # Send handshake
        await client.write_gatt_char(RX_CHAR_UUID, b"Volume\n")
        
        # Start notifications with com_queue passed as user data
        await client.start_notify(TX_CHAR_UUID, lambda s, d: asyncio.create_task(notification_handler(s, d, com_queue)))
        
        # Get initial mute and volume state
        com_queue.put(("get_mute", None))
        com_queue.put(("get_volume", None))
        await asyncio.sleep(0.2)
        
        try:
            while not com_response_queue.empty():
                response_type, value = com_response_queue.get_nowait()
                if response_type == "mute_state":
                    is_muted = value
                    mute_msg = b"mute\n" if is_muted else b"unmute\n"
                    await client.write_gatt_char(RX_CHAR_UUID, mute_msg)
                elif response_type == "volume_state":
                    # Optionally send initial volume to device
                    volume_msg = f"vol:{value}\n".encode()
                    await client.write_gatt_char(RX_CHAR_UUID, volume_msg)
        except:
            pass
        
        print("Listening for volume changes...")
        
        try:
            while client.is_connected:
                try:
                    while not com_response_queue.empty():  # Process all pending messages
                        response_type, value = com_response_queue.get_nowait()
                        
                        if response_type == "mute_changed":
                            is_muted = value
                            mute_msg = b"mute\n" if is_muted else b"unmute\n"
                            await client.write_gatt_char(RX_CHAR_UUID, mute_msg)
                            # Update last_mute_sent when PC mutes/unmutes (this is the authoritative state)
                            last_mute_sent = is_muted
                            # Clear the unmute flag when PC mutes/unmutes directly (only knob unmute should set it)
                            knob_just_unmuted = False
                            print(f"PC muted -> sent to knob" if is_muted else "PC unmuted -> sent to knob")
                        
                        elif response_type == "volume_changed":
                            # If knob just unmuted, only skip if this is the FIRST volume change (the echo)
                            # This preserves the knob's volume position from when it was muted
                            if knob_just_unmuted:
                                print(f"Skipping volume {value}% to knob (knob just unmuted, preserving knob's volume)")
                                knob_just_unmuted = False  # Clear flag for next volume change
                            else:
                                # Send volume updates immediately
                                volume_msg = f"vol:{value}\n".encode()
                                try:
                                    await client.write_gatt_char(RX_CHAR_UUID, volume_msg)
                                    # Update last_volume so we recognize this value when knob echoes it back
                                    last_volume = value / 100.0
                                    print(f"PC volume changed to {value}% -> sent to knob")
                                except Exception as e:
                                    print(f"ERROR sending volume to knob: {e}")
                                
                                # Query actual mute state after volume change (Windows may auto-unmute when volume changes while muted)
                                await asyncio.sleep(0.05)  # Small delay to ensure volume is processed first
                                com_queue.put(("get_mute", None))
                                await asyncio.sleep(0.05)
                                
                                # Get the mute state from COM worker response
                                current_mute = is_muted
                                if not com_response_queue.empty():
                                    try:
                                        resp_type, resp_value = com_response_queue.get_nowait()
                                        if resp_type == "mute_state":
                                            current_mute = resp_value
                                    except:
                                        pass
                                
                                # Send mute state only if it changed from what we last sent
                                if current_mute != last_mute_sent:
                                    mute_msg = b"mute\n" if current_mute else b"unmute\n"
                                    await client.write_gatt_char(RX_CHAR_UUID, mute_msg)
                                    last_mute_sent = current_mute
                                    # If we just sent unmute, reset last_volume so next volume change is always sent
                                    # and set flag to skip the next knob volume echo (if it changes position due to unmute)
                                    if not current_mute:
                                        last_volume = None  # Reset so next PC volume change is detected
                                        knob_just_unmuted = True
                                    print(f"Mute state synced after volume change: {'Muted' if current_mute else 'Unmuted'}")
                        
                        elif response_type == "device_changed":
                            if value is not None:
                                current_volume, current_mute = value
                                # Set flag to ignore knob-originated values for 1 second after device change
                                ignore_knob_until_time = time.time() + 1.0
                                # Reset knob_changing flag in COM worker IMMEDIATELY so PC volume changes work
                                com_queue.put(("reset_knob_flag", None))
                                # Reset last_volume so next PC volume change is always detected
                                last_volume = None
                                # Reset last_mute_sent to the new device's mute state so we don't resend it unnecessarily
                                last_mute_sent = current_mute
                                # Clear flags so PC volume changes are sent normally after device change
                                knob_just_unmuted = False
                                # Send both mute and volume to knob immediately
                                mute_msg = b"mute\n" if current_mute else b"unmute\n"
                                await client.write_gatt_char(RX_CHAR_UUID, mute_msg)
                                volume_msg = f"vol:{current_volume}\n".encode()
                                await client.write_gatt_char(RX_CHAR_UUID, volume_msg)
                                print(f"Device switched - sent volume {current_volume}% and mute {current_mute} to knob, ignoring knob-originated values for 1 second")
                            else:
                                print("Audio device changed - callbacks updated on COM side")
                
                except Exception as e:
                    print(f"Error processing response: {e}")
                
                await asyncio.sleep(0.01)  # Reduced sleep time from 0.05 to 0.01
        
        except Exception as e:
            print(f"Error in main loop: {e}")
            traceback.print_exc()
        finally:
            await client.stop_notify(TX_CHAR_UUID)


async def main():
    # Create queues for inter-process communication
    com_queue = Queue()
    com_response_queue = Queue()
    
    # Start COM worker in separate process
    com_process = Process(target=com_worker_process, args=(com_queue, com_response_queue), daemon=True)
    com_process.start()
    
    # Wait for COM initialization
    try:
        status, _ = com_response_queue.get(timeout=5)
        if status != "ready":
            print("Failed to initialize COM interface")
            return
        print("Volume interface initialized")
    except:
        print("Timeout waiting for COM initialization")
        return
    
    try:
        while True:
            try:
                address = await find_device()
                
                if address is None:
                    print("Retrying in 5 seconds...")
                    await asyncio.sleep(5)
                    continue
                
                await connect_and_run(address, com_queue, com_response_queue)
                
            except Exception as e:
                print(f"Connection error: {e}")
                traceback.print_exc()
                print("Retrying in 5 seconds...")
                await asyncio.sleep(5)
    finally:
        com_queue.put(("exit", None))
        com_process.join(timeout=2)
        com_process.terminate()


if __name__ == "__main__":
    # Required for Windows multiprocessing
    multiprocessing.freeze_support()
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
