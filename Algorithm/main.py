import time
import os
import cv2
from datetime import datetime
from algo.algo import MazeSolver 
from flask import Flask, request, jsonify
from flask_cors import CORS
# from model import *
from helper import command_generator
from ultralytics import YOLO
from PIL import Image
import io
import os

app = Flask(__name__)
CORS(app)
# model = load_model()
# Class names
class_names = [
    'A','B','Bullseye','C','D','E','F','G','H','S','T','U','V','W','X','Y','Z',
    'circle','down','eight','five','four','left','nine','one','right',
    'seven','six','three','two','up'
]

# Load model
print("Loading model...")
model = YOLO('bestL160epoch.pt')
print("Model loaded successfully!")

# model = None
@app.route('/status', methods=['GET'])
def status():
    """
    This is a health check endpoint to check if the server is running
    :return: a json object with a key "result" and value "ok"
    """
    return jsonify({"result": "ok"})


@app.route('/path', methods=['POST'])
def path_finding():
    """
    This is the main endpoint for the path finding algorithm
    :return: a json object with a key "data" and value a dictionary with keys "distance", "path", and "commands"
    """
    # Get the json data from the request
    content = request.json

    # Get the obstacles, big_turn, retrying, robot_x, robot_y, and robot_direction from the json data
    obstacles = content['obstacles']
    # big_turn = int(content['big_turn'])
    retrying = content['retrying']
    robot_x, robot_y = content['robot_x'], content['robot_y']
    robot_direction = int(content['robot_dir'])

    # Input is in 20x20 coordinates; scale ×2 to match the internal 40x40 grid (5cm/cell)
    robot_x_scaled = robot_x * 2
    robot_y_scaled = robot_y * 2
    obstacles_scaled = [
        {**ob, 'x': ob['x'] * 2, 'y': ob['y'] * 2}
        for ob in obstacles
    ]

    # Initialize MazeSolver object with 40x40 grid (5cm/cell), robot position, and whether to use a big turn.
    maze_solver = MazeSolver(40, 40, robot_x_scaled, robot_y_scaled, robot_direction, big_turn=None)

    # Add each obstacle into the MazeSolver. Each obstacle is defined by its x,y positions, its direction, and its id
    for ob in obstacles_scaled:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

    start = time.time()
    # Get shortest path
    optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retrying)
    print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
    print(f"Distance to travel: {distance} units")
    
    # Based on the shortest path, generate commands for the robot
    # Use scaled obstacles so SNAP direction comparisons match the 40x40 path coordinates
    commands = command_generator(optimal_path, obstacles_scaled)
    print(f"Command: {commands}")

    # Get the starting location and add it to path_results
    path_results = [optimal_path[0].get_dict()]
    # Process each command individually and append the location the robot should be after executing that command to path_results
    i = 0
    for command in commands:
        if command.startswith("SNAP"):
            continue
        if command.startswith("FIN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) // 5
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) // 5
        else:
            i += 1
        path_results.append(optimal_path[i].get_dict())
    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands
        },
        "error": None
    })

@app.route('/image-new', methods=['POST'])
def predict_image():
    """
    Prediction function for image detection
    Expects:
        - file: Image file
        - confidence: Confidence threshold (0-1), optional, default 0.25
    Returns:
        JSON with detection results
    """
    # Check if file is in request
    if 'file' not in request.files:
        return jsonify({"error": "No file uploaded"}), 400
    
    file = request.files['file']
    
    if file.filename == '':
        return jsonify({"error": "No file selected"}), 400
    
    # Read file content once
    file_content = file.read()
    
    # Save original image to uploads folder
    os.makedirs('uploads', exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # Get filename without extension and extension
    filename_without_ext = os.path.splitext(file.filename)[0]
    file_ext = os.path.splitext(file.filename)[1]
    
    upload_filename = f'uploads/{timestamp}_{filename_without_ext}{file_ext}'
    with open(upload_filename, 'wb') as f:
        f.write(file_content)
    
    # Get confidence threshold from request, default to 0.25
    confidence = float(request.form.get('confidence', 0.25))
    
    # Convert file to PIL Image
    image = Image.open(io.BytesIO(file_content))
    
    # Run prediction
    results = model.predict(
        source=image,
        conf=confidence,
        save=False
    )
    
    # Get annotated image with bounding boxes
    annotated_img = results[0].plot()
    
    # Save annotated image
    os.makedirs('own_results', exist_ok=True)
    output_filename = f'own_results/result_{timestamp}_{filename_without_ext}.jpg'
    cv2.imwrite(output_filename, annotated_img)
    
    # Get detection details
    boxes = results[0].boxes
    
    detections = []
    if len(boxes) > 0:
        for i, box in enumerate(boxes):
            cls_idx = int(box.cls.cpu().numpy()[0])
            conf = float(box.conf.cpu().numpy()[0])
            class_name = class_names[cls_idx]
            
            detections.append({
                "class": class_name,
                "confidence": conf
            })
        
        result = {
            "success": True,
            "count": len(boxes),
            "detections": detections,
            "result_image": output_filename
        }
    else:
        result = {
            "success": False,
            "count": 0,
            "detections": [],
            "message": "No objects detected. Try lowering the confidence threshold.",
            "result_image": output_filename
        }
    
    return jsonify(result)

@app.route('/image', methods=['POST'])
def image_predict():
    """
    This is the main endpoint for the image prediction algorithm
    :return: a json object with a key "result" and value a dictionary with keys "obstacle_id" and "image_id"
    """
    file = request.files['file']
    filename = file.filename
    file.save(os.path.join('uploads', filename))
    # filename format: "<timestamp>_<obstacle_id>_<signal>.jpeg"
    constituents = file.filename.split("_")
    obstacle_id = constituents[1]

    ## Week 8 ## 
    signal = constituents[2].strip(".jpg")
    image_id = predict_image(filename, model, signal)

    ## Week 9 ## 
    # We don't need to pass in the signal anymore
    # image_id = predict_image_week_9(filename,model)

    # Return the obstacle_id and image_id
    result = {
        "obstacle_id": obstacle_id,
        "image_id": image_id
    }
    return jsonify(result)

@app.route('/stitch', methods=['GET'])
def stitch():
    """
    This is the main endpoint for the stitching command. Stitches the images using two different functions, in effect creating two stitches, just for redundancy purposes
    """
    img = stitch_image()
    img.show()
    img2 = stitch_image_own()
    img2.show()
    return jsonify({"result": "ok"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
