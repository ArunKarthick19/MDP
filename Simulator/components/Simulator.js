import React from "react";
import { useState, useEffect } from "react";
import QueryAPI from "./QueryAPI";

const Direction = {
  NORTH: 0,
  EAST: 2,
  SOUTH: 4,
  WEST: 6,
  SKIP: 8,
};

const ObDirection = {
  NORTH: 0,
  EAST: 2,
  SOUTH: 4,
  WEST: 6,
  SKIP: 8,
};

const DirectionToString = {
  0: "Up",
  2: "Right",
  4: "Down",
  6: "Left",
  8: "None",
};

const transformCoord = (x, y) => {
  // Change the coordinate system from (0, 0) at top left to (0, 0) at bottom left
  // Updated for 40x40 grid (was 19 for 20x20)
  return { x: 39 - y, y: x };
};

function classNames(...classes) {
  return classes.filter(Boolean).join(" ");
}

export default function Simulator() {
  const [robotState, setRobotState] = useState({
    x: 2,
    y: 2,
    d: Direction.NORTH,
    s: -1,
  });
  const [robotX, setRobotX] = useState(2);
  const [robotY, setRobotY] = useState(2);
  const [robotDir, setRobotDir] = useState(0);
  const [obstacles, setObstacles] = useState([]);
  const [obXInput, setObXInput] = useState(0);
  const [obYInput, setObYInput] = useState(0);
  const [directionInput, setDirectionInput] = useState(ObDirection.NORTH);
  const [isComputing, setIsComputing] = useState(false);
  const [path, setPath] = useState([]);
  const [commands, setCommands] = useState([]);
  const [page, setPage] = useState(0);
  const [healthStatus, setHealthStatus] = useState(null); // null, 'checking', 'success', 'error'

  const generateNewID = () => {
    // Find the highest existing ID and return the next sequential number
    if (obstacles.length === 0) {
      return 1;
    }
    const maxId = Math.max(...obstacles.map(ob => ob.id));
    return maxId + 1;
  };

  const generateRobotCells = () => {
    const robotCells = [];
    let markerX = 0;
    let markerY = 0;

    // Robot is 25cm x 25cm = 5x5 cells at 5cm/cell (40x40 grid)
    // Direction marker sits at the outermost edge facing the robot's direction
    if (Number(robotState.d) === Direction.NORTH) {
      markerY += 2;
    } else if (Number(robotState.d) === Direction.EAST) {
      markerX += 2;
    } else if (Number(robotState.d) === Direction.SOUTH) {
      markerY -= 2;
    } else if (Number(robotState.d) === Direction.WEST) {
      markerX -= 2;
    }

    // 5x5 footprint: i and j from -2 to 2
    for (let i = -2; i < 3; i++) {
      for (let j = -2; j < 3; j++) {
        const coord = transformCoord(robotState.x + i, robotState.y + j);
        // Skip cells that fall outside the 40x40 grid
        if (coord.x < 0 || coord.x >= 40 || coord.y < 0 || coord.y >= 40) continue;
        if (markerX === i && markerY === j) {
          robotCells.push({ x: coord.x, y: coord.y, d: robotState.d, s: robotState.s });
        } else {
          robotCells.push({ x: coord.x, y: coord.y, d: null, s: -1 });
        }
      }
    }

    return robotCells;
  };

  const onChangeX = (event) => {
    // If the input is an integer and is in the range [0, 39], set ObXInput to the input
    if (Number.isInteger(Number(event.target.value))) {
      const nb = Number(event.target.value);
      if (0 <= nb && nb < 40) {
        setObXInput(nb);
        return;
      }
    }
    // If the input is not an integer or is not in the range [0, 39], set the input to 0
    setObXInput(0);
  };

  const onChangeY = (event) => {
    // If the input is an integer and is in the range [0, 39], set ObYInput to the input
    if (Number.isInteger(Number(event.target.value))) {
      const nb = Number(event.target.value);
      if (0 <= nb && nb <= 39) {
        setObYInput(nb);
        return;
      }
    }
    // If the input is not an integer or is not in the range [0, 39], set the input to 0
    setObYInput(0);
  };

  const onChangeRobotX = (event) => {
    // If the input is an integer and is in the range [2, 37], set RobotX to the input
    if (Number.isInteger(Number(event.target.value))) {
      const nb = Number(event.target.value);
      if (2 <= nb && nb <= 37) {
        setRobotX(nb);
        return;
      }
    }
    // If the input is not valid, set the input to 2
    setRobotX(2);
  };

  const onChangeRobotY = (event) => {
    // If the input is an integer and is in the range [2, 37], set RobotY to the input
    if (Number.isInteger(Number(event.target.value))) {
      const nb = Number(event.target.value);
      if (2 <= nb && nb <= 37) {
        setRobotY(nb);
        return;
      }
    }
    // If the input is not valid, set the input to 2
    setRobotY(2);
  };

  const onClickObstacle = () => {
    // If the input is not valid, return
    if (!obXInput && !obYInput) return;
    // Create a new array of obstacles
    const newObstacles = [...obstacles];
    // Add the new obstacle to the array
    newObstacles.push({
      x: obXInput,
      y: obYInput,
      d: directionInput,
      id: generateNewID(),
    });
    // Set the obstacles to the new array
    setObstacles(newObstacles);
  };

  const onClickRobot = () => {
    // Set the robot state to the input

    setRobotState({ x: robotX, y: robotY, d: robotDir, s: -1 });
  };

  const onDirectionInputChange = (event) => {
    // Set the direction input to the input
    setDirectionInput(Number(event.target.value));
  };

  const onRobotDirectionInputChange = (event) => {
    // Set the robot direction to the input
    setRobotDir(event.target.value);
  };

  const onRemoveObstacle = (ob) => {
    // If the path is not empty or the algorithm is computing, return
    if (path.length > 0 || isComputing) return;
    // Create a new array of obstacles
    const newObstacles = [];
    // Add all the obstacles except the one to remove to the new array
    for (const o of obstacles) {
      if (o.x === ob.x && o.y === ob.y) continue;
      newObstacles.push(o);
    }
    // Set the obstacles to the new array
    setObstacles(newObstacles);
  };

  const onGridClick = (x, y) => {
    if (isComputing || path.length > 0) return;
    
    // Check if clicking on robot position
    const robotCells = generateRobotCells();
    for (const cell of robotCells) {
      if (cell.x === x && cell.y === y) {
        return; // Don't allow placing obstacle on robot
      }
    }
    
    // Check if obstacle already exists at this position (any of the 4 cells of a 2x2 obstacle)
    for (const ob of obstacles) {
      const rowBottom = 39 - ob.y;
      const rowTop    = 38 - ob.y;
      const colLeft   = ob.x;
      const colRight  = ob.x + 1;

      if ((x === rowBottom || x === rowTop) && (y === colLeft || y === colRight)) {
        // Cycle through directions instead of removing
        const directions = [ObDirection.NORTH, ObDirection.EAST, ObDirection.SOUTH, ObDirection.WEST, ObDirection.SKIP];
        const currentIndex = directions.indexOf(ob.d);
        const nextIndex = (currentIndex + 1) % directions.length;
        
        const newObstacles = obstacles.map(o => 
          o.id === ob.id ? { ...o, d: directions[nextIndex] } : o
        );
        setObstacles(newObstacles);
        return;
      }
    }
    
    // Add new obstacle - convert grid coordinates back to original
    // The grid renders with transformed coordinates, so we need to reverse it
    // transformCoord does: { x: 39 - y, y: x }
    // So to reverse: if grid shows (x, y), original is (y, 39 - x)
    const newObstacles = [...obstacles];
    newObstacles.push({
      x: y,           // original x = grid y
      y: 39 - x,      // original y = 39 - grid x
      d: ObDirection.NORTH,  // Default direction, can be changed by clicking again
      id: generateNewID(),
    });
    setObstacles(newObstacles);
  };
  const getCommandExplanation = (command) => {
    if (!command) return "";
    
    // Extract command type and distance
    const cmdType = command.substring(0, 2);
    const distance = command.substring(2);
    
    const explanations = {
      'FW': `Move forward ${parseInt(distance, 10)}cm`,
      'BW': `Move backward ${parseInt(distance, 10)}cm`,
      'FR': `Turn forward-right`,
      'FL': `Turn forward-left`,
      'BR': `Turn backward-right`,
      'BL': `Turn backward-left`,
    };
    
    return explanations[cmdType] || command;
  };
  const checkHealth = () => {
    setHealthStatus('checking');
    QueryAPI.healthCheck((response) => {
      if (response.error) {
        setHealthStatus('error');
        // Clear status after 10 seconds
        setTimeout(() => setHealthStatus(null), 10000);
      } else {
        setHealthStatus('success');
        // Clear status after 10 seconds
        setTimeout(() => setHealthStatus(null), 10000);
      }
    });
  };

  const compute = () => {
    // Set computing to true, act like a lock
    setIsComputing(true);
    // Call the query function from the API
    QueryAPI.query(obstacles, robotX, robotY, robotDir, (data, err) => {
      if (data) {
        // If the data is valid, set the path
        setPath(data.data.path);
        // Set the commands
        const commands = [];
        for (let x of data.data.commands) {
          // If the command is a snapshot, skip it
          if (x.startsWith("SNAP")) {
            continue;
          }
          commands.push(x);
        }
        setCommands(commands);
      }
      // Set computing to false, release the lock
      setIsComputing(false);
    });
  };

  const onResetAll = () => {
    setRobotX(2);
    setRobotDir(0);
    setRobotY(2);
    setRobotState({ x: 2, y: 2, d: Direction.NORTH, s: -1 });
    setPath([]);
    setCommands([]);
    setPage(0);
    setObstacles([]);
  };

  const onReset = () => {
    setRobotX(2);
    setRobotDir(0);
    setRobotY(2);
    setRobotState({ x: 2, y: 2, d: Direction.NORTH, s: -1 });
    setPath([]);
    setCommands([]);
    setPage(0);
  };

  const renderGrid = () => {
    const CELL = 18; // px — fixed size guarantees a square grid
    const N = 40;
    const robotCells = generateRobotCells();
    const items = [];

    for (let i = 0; i < N; i++) {
      // Y-axis label (leftmost cell of each row)
      items.push(
        <div
          key={`yl-${i}`}
          style={{ width: CELL, height: CELL }}
          className="bg-gray-100 border border-gray-300 flex items-center justify-center overflow-hidden"
        >
          <span className="text-gray-600 font-bold leading-none select-none" style={{ fontSize: '6px' }}>
            {N - 1 - i}
          </span>
        </div>
      );

      for (let j = 0; j < N; j++) {
        let foundOb = null;
        let foundObIsFaceCell = false;
        let foundRobotCell = null;

        // 2x2 obstacle: SW corner at (ob.x, ob.y)
        // grid row for algo y=ob.y   → N-1-ob.y  (rowBottom)
        // grid row for algo y=ob.y+1 → N-2-ob.y  (rowTop)
        for (const ob of obstacles) {
          const rowBottom = N - 1 - ob.y;
          const rowTop    = N - 2 - ob.y;
          const colLeft   = ob.x;
          const colRight  = ob.x + 1;
          if ((i === rowBottom || i === rowTop) && (j === colLeft || j === colRight)) {
            foundOb = ob;
            if (ob.d === Direction.NORTH && i === rowTop)    foundObIsFaceCell = true;
            if (ob.d === Direction.SOUTH && i === rowBottom) foundObIsFaceCell = true;
            if (ob.d === Direction.EAST  && j === colRight)  foundObIsFaceCell = true;
            if (ob.d === Direction.WEST  && j === colLeft)   foundObIsFaceCell = true;
            break;
          }
        }

        if (!foundOb) {
          for (const rc of robotCells) {
            if (rc.x === i && rc.y === j) { foundRobotCell = rc; break; }
          }
        }

        if (foundOb) {
          let cls = 'bg-slate-700 border border-gray-600 cursor-pointer';
          if (foundObIsFaceCell) {
            if (foundOb.d === Direction.NORTH) cls = 'bg-slate-700 border border-gray-600 border-t-2 border-t-rose-500 cursor-pointer';
            else if (foundOb.d === Direction.SOUTH) cls = 'bg-slate-700 border border-gray-600 border-b-2 border-b-rose-500 cursor-pointer';
            else if (foundOb.d === Direction.EAST)  cls = 'bg-slate-700 border border-gray-600 border-r-2 border-r-rose-500 cursor-pointer';
            else if (foundOb.d === Direction.WEST)  cls = 'bg-slate-700 border border-gray-600 border-l-2 border-l-rose-500 cursor-pointer';
          }
          items.push(<div key={`${i}-${j}`} style={{ width: CELL, height: CELL }} className={cls} onClick={() => onGridClick(i, j)} />);
        } else if (foundRobotCell) {
          const cls = foundRobotCell.d !== null
            ? `border border-white ${foundRobotCell.s !== -1 ? 'bg-rose-500' : 'bg-amber-400'}`
            : 'bg-emerald-500 border border-white';
          items.push(<div key={`${i}-${j}`} style={{ width: CELL, height: CELL }} className={cls} />);
        } else {
          items.push(
            <div
              key={`${i}-${j}`}
              style={{ width: CELL, height: CELL }}
              className="border border-gray-200 bg-white cursor-pointer hover:bg-slate-100 transition-colors"
              onClick={() => onGridClick(i, j)}
            />
          );
        }
      }
    }

    // X-axis labels (bottom row)
    items.push(
      <div key="corner" style={{ width: CELL, height: CELL }} className="bg-gray-100 border border-gray-300" />
    );
    for (let j = 0; j < N; j++) {
      items.push(
        <div
          key={`xl-${j}`}
          style={{ width: CELL, height: CELL }}
          className="bg-gray-100 border border-gray-300 flex items-center justify-center overflow-hidden"
        >
          <span className="text-gray-600 font-bold leading-none select-none" style={{ fontSize: '6px' }}>
            {j}
          </span>
        </div>
      );
    }

    return (
      <div style={{ display: 'grid', gridTemplateColumns: `${CELL}px repeat(${N}, ${CELL}px)` }}>
        {items}
      </div>
    );
  };

  useEffect(() => {
    if (page >= path.length) return;
    setRobotState(path[page]);
  }, [page, path]);

  return (
    <div className="flex flex-col items-center justify-center px-4 py-6 min-h-screen bg-gray-50">
      <div className="w-full max-w-7xl">
        <div className="flex flex-col items-center text-center bg-white border-b-4 border-slate-900 rounded-lg shadow-sm mb-8 p-8">
          <h1 className="text-3xl md:text-4xl font-bold text-slate-900 mb-2">Algorithm Simulator</h1>
          <p className="text-slate-600 text-sm">Click on grid to add obstacles • Click existing obstacles to change direction</p>
        </div>

        {/* Two Column Layout */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">

          <div className="flex flex-col bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <h2 className="text-lg font-semibold text-slate-900 mb-4 pb-2 border-b border-gray-200">
              Robot Position
            </h2>
          <div className="form-control">
            <label className="input-group input-group-horizontal">
              <span className="bg-slate-800 text-white font-medium p-2 text-sm">X</span>
              <input
                onChange={onChangeRobotX}
                type="number"
                placeholder="2"
                min="2"
                max="37"
                value={robotX}
                className="input input-bordered bg-white text-slate-900 font-medium w-20 border-gray-300"
              />
              <span className="bg-slate-800 text-white font-medium p-2 text-sm">Y</span>
              <input
                onChange={onChangeRobotY}
                type="number"
                placeholder="2"
                min="2"
                max="37"
                value={robotY}
                className="input input-bordered bg-white text-slate-900 font-medium w-20 border-gray-300"
              />
              <span className="bg-slate-800 text-white font-medium p-2 text-sm">D</span>
              <select
                onChange={onRobotDirectionInputChange}
                value={robotDir}
                className="select select-bordered bg-white text-slate-900 font-medium py-2 pl-2 pr-6 border-gray-300"
              >
                <option value={ObDirection.NORTH}>Up</option>
                <option value={ObDirection.SOUTH}>Down</option>
                <option value={ObDirection.WEST}>Left</option>
                <option value={ObDirection.EAST}>Right</option>
              </select>
              <button className="btn bg-slate-900 hover:bg-slate-700 text-white font-medium p-2 border-0" onClick={onClickRobot}>
                Set
              </button>
            </label>
          </div>
        </div>

          <div className="flex flex-col bg-white rounded-lg shadow-sm border border-gray-200 p-6">
            <h2 className="text-lg font-semibold text-slate-900 mb-4 pb-2 border-b border-gray-200">
              Add Obstacles
            </h2>
        <div className="form-control">
          <label className="input-group input-group-horizontal">
            <span className="bg-slate-800 text-white font-medium p-2 text-sm">X</span>
            <input
              onChange={onChangeX}
              type="number"
              placeholder="0"
              min="0"
              max="39"
              value={obXInput}
              className="input input-bordered bg-white text-slate-900 font-medium w-20 border-gray-300"
            />
            <span className="bg-slate-800 text-white font-medium p-2 text-sm">Y</span>
            <input
              onChange={onChangeY}
              type="number"
              placeholder="0"
              min="0"
              max="39"
              value={obYInput}
              className="input input-bordered bg-white text-slate-900 font-medium w-20 border-gray-300"
            />
            <span className="bg-slate-800 text-white font-medium p-2 text-sm">D</span>
            <select
              onChange={onDirectionInputChange}
              value={directionInput}
              className="select select-bordered bg-white text-slate-900 font-medium py-2 pl-2 pr-6 border-gray-300"
            >
              <option value={ObDirection.NORTH}>Up</option>
              <option value={ObDirection.SOUTH}>Down</option>
              <option value={ObDirection.WEST}>Left</option>
              <option value={ObDirection.EAST}>Right</option>
              <option value={ObDirection.SKIP}>None</option>
            </select>
            <button className="btn bg-slate-900 hover:bg-slate-700 text-white font-medium p-2 border-0" onClick={onClickObstacle}>
              Add
            </button>
          </label>
        </div>
      </div>
      </div>

      {/* Obstacles List */}
      {obstacles.length > 0 && (
        <div className="bg-white p-6 rounded-lg shadow-sm border border-gray-200 mb-6">
          <h3 className="text-lg font-semibold text-slate-900 mb-4 pb-2 border-b border-gray-200 text-center">
            Obstacles ({obstacles.length})
          </h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3 items-center">
            {obstacles.map((ob) => {
              return (
                <div
                  key={ob.id}
                  className="relative flex flex-col bg-gray-50 text-slate-900 rounded-md p-3 border border-gray-300 hover:border-slate-400 transition-all"
                >
                  <button
                    onClick={() => onRemoveObstacle(ob)}
                    className="absolute -top-2 -right-2 btn btn-circle btn-xs bg-slate-900 hover:bg-slate-700 text-white border-0"
                    title="Remove obstacle"
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      className="h-4 w-4"
                      fill="none"
                      viewBox="0 0 24 24"
                      stroke="currentColor"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth="2"
                        d="M6 18L18 6M6 6l12 12"
                      />
                    </svg>
                  </button>
                  <div className="text-sm font-medium">
                    <div className="flex justify-between">
                      <span className="text-slate-600">ID:</span>
                      <span className="font-bold text-slate-900">{ob.id}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-slate-600">X:</span>
                      <span className="font-semibold">{ob.x}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-slate-600">Y:</span>
                      <span className="font-semibold">{ob.y}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-slate-600">Dir:</span>
                      <span className="font-semibold">{DirectionToString[ob.d]}</span>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      )}
      {/* Action Buttons */}
      <div className="flex flex-wrap justify-center gap-3 mb-6">
        <button 
          className={classNames(
            "btn font-medium px-6 border-0",
            healthStatus === 'success' ? "bg-emerald-500 hover:bg-emerald-600 text-white" : 
            healthStatus === 'error' ? "bg-red-500 hover:bg-red-600 text-white" :
            "bg-slate-200 hover:bg-slate-300 text-slate-900"
          )}
          onClick={checkHealth}
          disabled={healthStatus === 'checking'}
        >
          {healthStatus === 'checking' ? (
            <>
              <span className="loading loading-spinner loading-sm"></span>
              Checking...
            </>
          ) : healthStatus === 'success' ? (
            <>
              ✓ API Connected
            </>
          ) : healthStatus === 'error' ? (
            <>
              ✗ Connection Failed
            </>
          ) : (
            'Check API'
          )}
        </button>
        <button className="btn bg-slate-200 hover:bg-slate-300 text-slate-900 font-medium px-6 border-0" onClick={onResetAll}>
          Reset All
        </button>
        <button className="btn bg-slate-200 hover:bg-slate-300 text-slate-900 font-medium px-6 border-0" onClick={onReset}>
          Reset Robot
        </button>
        <button 
          className="btn bg-slate-900 hover:bg-slate-700 text-white font-semibold px-6 border-0" 
          onClick={compute}
          disabled={isComputing || obstacles.length === 0}
        >
          {isComputing ? (
            <>
              <span className="loading loading-spinner loading-sm"></span>
              Computing...
            </>
          ) : (
            'Start Pathfinding'
          )}
        </button>
      </div>

      {/* Path Navigation */}
      {path.length > 0 && (
        <div className="flex flex-row items-center justify-center text-center bg-white p-6 rounded-lg shadow-sm border border-gray-200 mb-6 gap-4">
          <button
            className="btn bg-slate-900 hover:bg-slate-700 btn-circle border-0"
            disabled={page === 0}
            onClick={() => {
              setPage(page - 1);
            }}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="h-6 w-6"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth="2"
                d="M15 19l-7-7 7-7"
              />
            </svg>
          </button>

          <div className="flex flex-col items-center mx-4">
            <span className="text-sm text-slate-600 font-medium">
              Step {page + 1} of {path.length}
            </span>
            <span className="text-lg font-bold text-slate-900">{commands[page]}</span>
            <span className="text-sm text-slate-500 mt-1">{getCommandExplanation(commands[page])}</span>
          </div>
          
          <button
            className="btn bg-slate-900 hover:bg-slate-700 btn-circle border-0"
            disabled={page === path.length - 1}
            onClick={() => {
              setPage(page + 1);
            }}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="h-6 w-6"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth="2"
                d="M9 5l7 7-7 7"
              />
            </svg>
          </button>
        </div>
      )}
      {/* Grid */}
      <div className="flex flex-col items-center">
        <div className="bg-white p-4 rounded-lg shadow-sm border border-gray-200 mb-4">
          <div className="flex flex-wrap justify-center gap-4 text-sm">
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 bg-emerald-500 border border-gray-300"></div>
              <span className="font-medium text-slate-700">Robot</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 bg-amber-400 border border-gray-300"></div>
              <span className="font-medium text-slate-700">Camera</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 bg-slate-700 border border-red-500 border-l-4"></div>
              <span className="font-medium text-slate-700">Obstacle</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 bg-rose-500 border border-gray-300"></div>
              <span className="font-medium text-slate-700">Capturing</span>
            </div>
          </div>
        </div>
        <div className="bg-white p-4 rounded-lg shadow-sm border border-gray-200 overflow-auto">
          {renderGrid()}
        </div>
      </div>
      </div>
    </div>
  );
}
