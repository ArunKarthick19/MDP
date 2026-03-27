import BaseAPI, { methodType } from "./BaseAPI";

export default class QueryAPI extends BaseAPI {
  // Check if the API server is healthy
  static healthCheck(callback) {
    this.JSONRequest("/status", methodType.get, {}, {}, {})
      .then((res) => {
        if (callback) {
          callback({
            data: res,
            error: null,
          });
        }
      })
      .catch((err) => {
        console.log(err);
        if (callback) {
          callback({
            data: null,
            error: err,
          });
        }
      });
  }

  // Query the path from backend server
  static query(obstacles, robotX, robotY, robotDir, callback) {
    /* Construct the content of the request
		obstacles: the array of obstacles (in 40x40 coords, divided by 2 to send as 20x20)
		robotX: the x coordinate of the robot
		robotY: the y coordinate of the robot
		robotDir: the direction of the robot
		retrying: whether the robot is retrying
	*/
    // API expects 20x20 coordinates; divide internal 40x40 values by 2
    const content = {
      obstacles: obstacles.map(ob => ({ ...ob, x: ob.x / 2, y: ob.y / 2 })),
      robot_x: robotX / 2,
      robot_y: robotY / 2,
      robot_dir: robotDir,
      retrying: false,
    };

    // Send the request to the backend server
    console.log("Algo input:", content)
    this.JSONRequest("/path", methodType.post, {}, {}, content)
      .then((res) => {
        if (callback) {
          callback({
            data: res,
            error: null,
          });
        }
      })
      .catch((err) => {
        console.log(err);
        if (callback) {
          callback({
            data: null,
            error: err,
          });
        }
      });
  }
}
