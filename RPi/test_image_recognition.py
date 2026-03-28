#!/usr/bin/env python3
"""
Test Image Recognition - Raspberry Pi
Captures image, sends to API server, displays results with bounding boxes and labels

Usage:
    python3 test_image_recognition.py
    
Press 'c' to capture and recognize
Press 'q' to quit
"""

import cv2
import requests
import json
import time
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import os

# Import settings
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from settings import API_IP, API_PORT
from consts import SYMBOL_MAP

# Camera settings
CAMERA_RESOLUTION = (640, 480)
CAMERA_FRAMERATE = 30

# Colors for bounding boxes (BGR format)
BOX_COLOR = (0, 255, 0)  # Green
TEXT_COLOR = (255, 255, 255)  # White
TEXT_BG_COLOR = (0, 255, 0)  # Green background


def capture_image_from_camera(camera, raw_capture):
    """Capture a single frame from the camera"""
    raw_capture.truncate(0)
    camera.capture(raw_capture, format="bgr")
    return raw_capture.array.copy()


def send_image_to_api(image, filename="test_image.jpg"):
    """
    Send image to API server and get detection results with bounding boxes
    """
    # Save image temporarily
    cv2.imwrite(filename, image)
    
    # Send to API
    url = f"http://{API_IP}:{API_PORT}/image"
    
    try:
        with open(filename, 'rb') as f:
            response = requests.post(url, files={"file": (filename, f)}, timeout=30)
        
        if response.status_code == 200:
            return response.json(), None
        else:
            return None, f"API Error: {response.status_code}"
    
    except requests.exceptions.Timeout:
        return None, "API Timeout - server took too long to respond"
    except requests.exceptions.ConnectionError:
        return None, f"Connection Error - Cannot reach {API_IP}:{API_PORT}"
    except Exception as e:
        return None, f"Error: {str(e)}"


def draw_results_on_image(image, results):
    """
    Draw bounding boxes and labels on the image using actual pixel coordinates
    """
    annotated = image.copy()
    height, width = image.shape[:2]
    
    if results is None:
        return annotated
    
    segments = results.get('segments', [])
    
    if len(segments) == 0:
        # No detection
        cv2.putText(annotated, "No Detection", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return annotated
    
    # Draw bounding boxes and labels for each detection
    for i, segment in enumerate(segments):
        class_id = segment.get('class_id', 'NA')
        class_name = segment.get('class_name', 'Unknown')
        confidence = segment.get('confidence', 0)
        is_ambiguous = segment.get('is_ambiguous', False)
        bbox = segment.get('bbox', None)
        
        # Get symbol name from map
        symbol_name = SYMBOL_MAP.get(class_id, class_name)
        
        # Determine color based on confidence
        if is_ambiguous:
            box_color = (0, 165, 255)  # Orange for ambiguous
            label_color = (0, 165, 255)
        else:
            box_color = (0, 255, 0)  # Green for confident
            label_color = (0, 255, 0)
        
        # Draw bounding box if coordinates are available
        if bbox and all(k in bbox for k in ['x1', 'y1', 'x2', 'y2']):
            x1, y1 = bbox['x1'], bbox['y1']
            x2, y2 = bbox['x2'], bbox['y2']
            
            # Draw rectangle around detected object
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 3)
            
            # Create label text
            label = f"{symbol_name} {confidence*100:.0f}%"
            if is_ambiguous:
                label += " [!]"
            
            # Calculate label size and position
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
            )
            
            # Draw label background (above the box)
            label_y = max(y1 - 10, label_h + 10)
            cv2.rectangle(
                annotated,
                (x1, label_y - label_h - 10),
                (x1 + label_w + 10, label_y),
                label_color,
                -1
            )
            
            # Draw label text
            cv2.putText(
                annotated,
                label,
                (x1 + 5, label_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),  # Black text
                2
            )
            
            # Draw detection number in bottom-left corner of box
            number_label = f"#{i+1}"
            cv2.putText(
                annotated,
                number_label,
                (x1 + 5, y2 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),  # White text
                2
            )
        else:
            # Fallback: display as text overlay if no bbox coordinates
            y_offset = 30 + (i * 35)
            label = f"{i+1}. {symbol_name} ({confidence*100:.1f}%)"
            if is_ambiguous:
                label += " [AMBIGUOUS]"
            
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (5, y_offset - text_h - 5), 
                         (15 + text_w, y_offset + 5), label_color, -1)
            cv2.putText(annotated, label, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    # Add summary at top
    summary = f"Detected: {len(segments)} object(s)"
    cv2.putText(annotated, summary, (10, height - 10), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return annotated


def main():
    """Main function for testing image recognition"""
    print("="*60)
    print("Image Recognition Test - Raspberry Pi")
    print("="*60)
    print(f"API Server: http://{API_IP}:{API_PORT}")
    print("\nControls:")
    print("  'c' - Capture and recognize image")
    print("  's' - Save annotated image")
    print("  'q' - Quit")
    print("="*60)
    
    # Test API connection
    print("\nTesting API connection...")
    try:
        response = requests.get(f"http://{API_IP}:{API_PORT}/status", timeout=3)
        if response.status_code == 200:
            status = response.json()
            print(f"✅ API Server is running")
            print(f"   Model: {status.get('model', 'unknown')}")
            print(f"   YOLO: {status.get('yolo_available', False)}")
            print(f"   Algorithm: {status.get('algorithm_available', False)}")
        else:
            print(f"⚠️  API returned status {response.status_code}")
    except Exception as e:
        print(f"❌ Cannot connect to API server: {e}")
        print(f"   Make sure the server is running on {API_IP}:{API_PORT}")
        return
    
    # Initialize camera
    print("\nInitializing camera...")
    camera = PiCamera()
    camera.resolution = CAMERA_RESOLUTION
    camera.framerate = CAMERA_FRAMERATE
    raw_capture = PiRGBArray(camera, size=CAMERA_RESOLUTION)
    
    # Warm up camera
    time.sleep(2)
    print("✅ Camera ready!")
    print("\nPress 'c' to capture and recognize an image...")
    
    # Create window for display
    cv2.namedWindow('Image Recognition Test', cv2.WINDOW_NORMAL)
    
    current_frame = None
    annotated_frame = None
    last_result = None
    processing = False
    
    try:
        # Main loop
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            current_frame = frame.array
            
            # Display current or annotated frame
            if annotated_frame is not None:
                display_frame = annotated_frame
            else:
                display_frame = current_frame.copy()
                # Add instructions
                cv2.putText(display_frame, "Press 'c' to capture", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if processing:
                cv2.putText(display_frame, "Processing...", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow('Image Recognition Test', display_frame)
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nExiting...")
                break
            
            elif key == ord('c') and not processing:
                print("\n" + "="*60)
                print("Capturing image...")
                processing = True
                
                # Capture current frame
                test_image = current_frame.copy()
                timestamp = int(time.time())
                filename = f"test_capture_{timestamp}.jpg"
                
                print(f"Sending to API server...")
                start_time = time.time()
                
                # Send to API
                result, error = send_image_to_api(test_image, filename)
                
                elapsed = time.time() - start_time
                
                if error:
                    print(f"❌ {error}")
                    annotated_frame = test_image.copy()
                    cv2.putText(annotated_frame, f"ERROR: {error}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    print(f"✅ Response received in {elapsed:.2f}s")
                    print(f"\nResults:")
                    print(f"  Obstacle ID: {result.get('obstacle_id', 'N/A')}")
                    print(f"  Image ID: {result.get('image_id', 'NA')}")
                    
                    segments = result.get('segments', [])
                    if segments:
                        print(f"  Detected {len(segments)} object(s):")
                        for i, seg in enumerate(segments):
                            class_id = seg.get('class_id', 'NA')
                            class_name = seg.get('class_name', 'Unknown')
                            confidence = seg.get('confidence', 0)
                            is_ambiguous = seg.get('is_ambiguous', False)
                            
                            symbol_name = SYMBOL_MAP.get(class_id, class_name)
                            ambiguous_marker = " [AMBIGUOUS]" if is_ambiguous else ""
                            
                            print(f"    {i+1}. {symbol_name} - {confidence*100:.1f}%{ambiguous_marker}")
                    else:
                        print("  No objects detected")
                    
                    # Draw results
                    annotated_frame = draw_results_on_image(test_image, result)
                    last_result = result
                    
                    # Save annotated image
                    annotated_filename = f"annotated_{timestamp}.jpg"
                    cv2.imwrite(annotated_filename, annotated_frame)
                    print(f"\n💾 Saved: {annotated_filename}")
                
                print("="*60)
                print("Press 'c' for another capture, 'q' to quit")
                processing = False
            
            elif key == ord('s') and annotated_frame is not None:
                # Save current annotated frame
                save_filename = f"saved_{int(time.time())}.jpg"
                cv2.imwrite(save_filename, annotated_frame)
                print(f"💾 Saved: {save_filename}")
            
            # Clear stream for next frame
            raw_capture.truncate(0)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Cleanup
        camera.close()
        cv2.destroyAllWindows()
        print("\nCamera closed. Program terminated.")


if __name__ == "__main__":
    main()
