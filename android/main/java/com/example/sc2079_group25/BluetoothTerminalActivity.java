package com.example.sc2079_group25;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.core.app.ActivityCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.google.android.material.tabs.TabLayout;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.Set;

public class BluetoothTerminalActivity extends AppCompatActivity implements BluetoothEventListener {

    private BluetoothAdapter btAdapter;
    private BluetoothSerialService serial;

    private DeviceListAdapter deviceAdapter;

    private TextView txtConnState, txtTerminal, txtRobotStatus;
    private EditText edtSend;
    private ScrollView scrollTerminal;
    private ArenaView arenaView;

    private TabLayout tabLayout;
    private View layoutBluetooth, layoutGrid;

    private boolean pendingStartDiscovery = false;
    private AlertDialog scanDialog;

    private final ActivityResultLauncher<Intent> enableBtLauncher =
            registerForActivityResult(new ActivityResultContracts.StartActivityForResult(), result -> {
                if (btAdapter != null && btAdapter.isEnabled()) {
                    if (pendingStartDiscovery) {
                        pendingStartDiscovery = false;
                        startScanFlow();
                    }
                    preloadPairedDevices();
                } else {
                    Toast.makeText(this, "Bluetooth not enabled", Toast.LENGTH_LONG).show();
                }
            });

    private final BroadcastReceiver discoveryReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null) deviceAdapter.upsert(device);

            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
                appendTerminal("[Scan] Discovery finished");
                if (scanDialog != null && scanDialog.isShowing()) {
                    scanDialog.setTitle("Select Device (Scan finished)");
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.system_ui);

        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        tabLayout = findViewById(R.id.tabLayout);
        layoutBluetooth = findViewById(R.id.layoutBluetooth);
        layoutGrid = findViewById(R.id.layoutGrid);

        txtConnState = findViewById(R.id.txtConnState);
        txtTerminal = findViewById(R.id.txtTerminal);
        txtRobotStatus = findViewById(R.id.txtRobotStatus);
        edtSend = findViewById(R.id.edtSend);
        scrollTerminal = findViewById(R.id.scrollTerminal);
        arenaView = findViewById(R.id.arenaView);

        Button btnScan = findViewById(R.id.btnScan);
        Button btnSend = findViewById(R.id.btnSend);
        Button btnDisconnect = findViewById(R.id.btnDisconnect);
        Button btnReconnect = findViewById(R.id.btnReconnect);
        Button btnUndo = findViewById(R.id.btnUndo);
        Button btnRedo = findViewById(R.id.btnRedo);
        Button btnReset = findViewById(R.id.btnReset);
        Button btnSendObs = findViewById(R.id.btnSendObs);
        Button btnTask1 = findViewById(R.id.btnTask1);
        Button btnTask2 = findViewById(R.id.btnTask2);

        // Robot control buttons
        ImageButton btnForward = findViewById(R.id.btnForward);
        ImageButton btnReverse = findViewById(R.id.btnReverse);
        ImageButton btnLeft = findViewById(R.id.btnLeft);
        ImageButton btnRight = findViewById(R.id.btnRight);

        RecyclerView recycler = findViewById(R.id.recyclerDevices);
        recycler.setLayoutManager(new LinearLayoutManager(this));
        deviceAdapter = new DeviceListAdapter(this, this::onDeviceSelected);
        recycler.setAdapter(deviceAdapter);

        btAdapter = BluetoothAdapter.getDefaultAdapter();
        if (btAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        serial = new BluetoothSerialService(this, this);

        btnScan.setOnClickListener(v -> {
            pendingStartDiscovery = true;
            startScanFlow();
        });

        btnDisconnect.setOnClickListener(v -> serial.disconnect());

        btnReconnect.setOnClickListener(v -> {
            if (!hasConnectPermission()) {
                requestBtPermissions();
                return;
            }
            serial.reconnect();
        });

        btnSend.setOnClickListener(v -> {
            String text = edtSend.getText().toString();
            if (text.trim().isEmpty()) return;

            sendBluetoothCommand(text);
            edtSend.setText("");
        });

        btnUndo.setOnClickListener(v -> arenaView.revert());
        btnRedo.setOnClickListener(v -> arenaView.deRevert());
        btnReset.setOnClickListener(v -> {
            arenaView.clearMap();
            txtRobotStatus.setText("Waiting");
            Toast.makeText(this, "Arena reset", Toast.LENGTH_SHORT).show();
        });

        btnSendObs.setOnClickListener(v -> {
            try {
                JSONObject root = new JSONObject();
                root.put("cat", "obstacles");
                
                JSONArray obsArray = new JSONArray();
                for (ArenaView.Obstacle obs : arenaView.getObstacles()) {
                    JSONObject obsJson = new JSONObject();
                    obsJson.put("x", (int) obs.x);
                    obsJson.put("y", (int) obs.y);
                    obsJson.put("id", obs.id);
                    
                    int d = 0; // N=0, E=2, S=4, W=6 based on common 8-dir models, or just 0,1,2,3
                    // The current code uses 0=N, 1=E, 2=S, 3=W. Let's keep that or map to degrees.
                    // Assuming target wants a specific mapping for direction:
                    int dirValue = 0;
                    if (obs.direction == 1) dirValue = 1; // E
                    else if (obs.direction == 2) dirValue = 2; // S
                    else if (obs.direction == 3) dirValue = 3; // W
                    
                    obsJson.put("d", dirValue);
                    obsArray.put(obsJson);
                }
                root.put("value", obsArray);
                
                String jsonStr = root.toString();
                sendBluetoothCommand(jsonStr);
                Toast.makeText(this, "Obstacles sent (JSON)", Toast.LENGTH_SHORT).show();
                
            } catch (JSONException e) {
                Toast.makeText(this, "Error creating JSON", Toast.LENGTH_SHORT).show();
            }
        });

        btnTask1.setOnClickListener(v -> {
            sendBluetoothCommand("Task1:Start");
            Toast.makeText(this, "Task 1 started", Toast.LENGTH_SHORT).show();
        });

        btnTask2.setOnClickListener(v -> {
            sendBluetoothCommand("Task2:Start");
            Toast.makeText(this, "Task 2 started", Toast.LENGTH_SHORT).show();
        });

        // Control button listeners with LOCAL UI update first
        btnForward.setOnClickListener(v -> {
            arenaView.moveRobotForward();
            sendBluetoothCommand(RobotCommands.FORWARD);
        });
        btnReverse.setOnClickListener(v -> {
            arenaView.moveRobotBackward();
            sendBluetoothCommand(RobotCommands.REVERSE);
        });
        btnLeft.setOnClickListener(v -> {
            arenaView.turnRobotLeft();
            sendBluetoothCommand(RobotCommands.TURN_LEFT);
        });
        btnRight.setOnClickListener(v -> {
            arenaView.turnRobotRight();
            sendBluetoothCommand(RobotCommands.TURN_RIGHT);
        });

        tabLayout.addOnTabSelectedListener(new TabLayout.OnTabSelectedListener() {
            @Override
            public void onTabSelected(TabLayout.Tab tab) {
                if (tab.getPosition() == 0) {
                    layoutBluetooth.setVisibility(View.VISIBLE);
                    layoutGrid.setVisibility(View.GONE);
                } else {
                    layoutBluetooth.setVisibility(View.GONE);
                    layoutGrid.setVisibility(View.VISIBLE);
                }
            }

            @Override
            public void onTabUnselected(TabLayout.Tab tab) {
            }

            @Override
            public void onTabReselected(TabLayout.Tab tab) {
            }
        });

        ensureBluetoothEnabled();
        preloadPairedDevices();
    }

    private void sendBluetoothCommand(String cmd) {
        if (serial.getState() != BtConstants.STATE_CONNECTED) {
            Toast.makeText(this, "Not connected to a device", Toast.LENGTH_SHORT).show();
            return;
        }
        serial.writeLine(cmd);
        appendTerminal("[TX] " + cmd);
    }

    @Override
    protected void onStart() {
        super.onStart();
        IntentFilter f = new IntentFilter();
        f.addAction(BluetoothDevice.ACTION_FOUND);
        f.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(discoveryReceiver, f);
    }

    @Override
    protected void onStop() {
        super.onStop();
        try {
            unregisterReceiver(discoveryReceiver);
        } catch (Exception ignored) {}

        if (btAdapter == null) return;
        try {
            if (hasScanPermission()) {
                if (btAdapter.isDiscovering()) {
                    btAdapter.cancelDiscovery();
                }
            }
        } catch (SecurityException ignored) {}
    }

    private void ensureBluetoothEnabled() {
        if (!btAdapter.isEnabled()) {
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            enableBtLauncher.launch(intent);
        }
    }

    private void preloadPairedDevices() {
        if (btAdapter == null || !btAdapter.isEnabled() || !hasConnectPermission()) return;

        try {
            Set<BluetoothDevice> bonded = btAdapter.getBondedDevices();
            if (bonded != null) {
                for (BluetoothDevice d : bonded) deviceAdapter.upsert(d);
            }
        } catch (SecurityException ignored) {}
    }

    private void startScanFlow() {
        if (btAdapter == null) return;
        if (!btAdapter.isEnabled()) {
            ensureBluetoothEnabled();
            return;
        }
        if (!hasScanPermission() || !hasConnectPermission()) {
            requestBtPermissions();
            return;
        }
        startDiscovery();
    }

    private void startDiscovery() {
        if (btAdapter == null) return;
        deviceAdapter.clear();
        preloadPairedDevices();

        // Create a scrollable RecyclerView with a fixed height inside the dialog
        RecyclerView dialogRv = new RecyclerView(this);
        dialogRv.setLayoutManager(new LinearLayoutManager(this));
        dialogRv.setAdapter(deviceAdapter);
        
        // Wrap RecyclerView in a container to enforce fixed size
        FrameLayout container = new FrameLayout(this);
        container.addView(dialogRv);
        container.setLayoutParams(new ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 
                250 // Height in pixels
        ));
        
        // Set RecyclerView to fill the container
        dialogRv.setLayoutParams(new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 
                ViewGroup.LayoutParams.MATCH_PARENT
        ));

        scanDialog = new AlertDialog.Builder(this)
                .setTitle("Select Device")
                .setView(container) // Add the container, not just the RV
                .setNegativeButton("Cancel", (dialog, which) -> {
                    try {
                        if (btAdapter.isDiscovering()) btAdapter.cancelDiscovery();
                    } catch (SecurityException ignored) {}
                })
                .create();
        
        scanDialog.show();

        try {
            if (btAdapter.isDiscovering()) btAdapter.cancelDiscovery();
            btAdapter.startDiscovery();
            appendTerminal("[Scan] Discovery started");
        } catch (SecurityException ignored) {}
    }

    private void onDeviceSelected(BluetoothDevice device) {
        if (scanDialog != null && scanDialog.isShowing()) {
            scanDialog.dismiss();
        }
        if (!hasConnectPermission()) {
            requestBtPermissions();
            return;
        }
        try {
            if (btAdapter != null && btAdapter.isDiscovering()) btAdapter.cancelDiscovery();
        } catch (SecurityException ignored) {}
        serial.connect(device);
    }

    private void appendTerminal(String line) {
        txtTerminal.append(line + "\n");
        scrollTerminal.post(() -> scrollTerminal.fullScroll(ScrollView.FOCUS_DOWN));
    }

    private boolean hasScanPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
                    == PackageManager.PERMISSION_GRANTED;
        } else {
            return ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    == PackageManager.PERMISSION_GRANTED;
        }
    }

    private boolean hasConnectPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    == PackageManager.PERMISSION_GRANTED;
        }
        return true;
    }

    private void requestBtPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestPermissions(new String[] {
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT
            }, 2001);
        } else {
            requestPermissions(new String[] {
                    Manifest.permission.ACCESS_FINE_LOCATION
            }, 2002);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        boolean granted = (grantResults.length > 0);
        for (int r : grantResults) granted &= (r == PackageManager.PERMISSION_GRANTED);

        if (!granted) {
            Toast.makeText(this, "Permissions denied", Toast.LENGTH_LONG).show();
            return;
        }
        if (pendingStartDiscovery) startDiscovery();
        else preloadPairedDevices();
    }

    @Override
    public void onConnectionStateChanged(int state, String detail) {
        String s;
        if (state == BtConstants.STATE_CONNECTED) s = "Connected";
        else if (state == BtConstants.STATE_CONNECTING) s = "Connecting";
        else if (state == BtConstants.STATE_LISTENING) s = "Listening (Waiting for device)";
        else s = "Not connected";

        txtConnState.setText(s + " - " + detail);
        appendTerminal("[State] " + s);
    }

    @Override
    public void onLineReceived(String line) {
        appendTerminal("[RX] " + line);
        
        // Handle "TARGET, <id>, <Value>" protocol
        if (line.startsWith("TARGET,")) {
            try {
                String[] parts = line.split(",");
                if (parts.length >= 3) {
                    int id = Integer.parseInt(parts[1].trim());
                    String value = parts[2].trim();
                    
                    // New restriction: Only accept first two characters and must be Digit
                    if (value.length() >= 2) {
                        String firstTwo = value.substring(0, 2);
                        if (Character.isDigit(firstTwo.charAt(0)) && Character.isDigit(firstTwo.charAt(1))) {
                            arenaView.updateObstacleValue(id, firstTwo);
                        } else {
                            appendTerminal("[Error] First two characters must be digits: " + firstTwo);
                        }
                    } else {
                        appendTerminal("[Error] Target value too short (need at least 2 digits): " + value);
                    }
                }
            } catch (Exception e) {
                appendTerminal("[Error] Failed to parse target update: " + line);
            }
            return;
        }

        // Handle "ROBOT, <x>, <y>, <direction>" protocol
        if (line.startsWith("ROBOT,")) {
            try {
                String[] parts = line.split(",");
                if (parts.length >= 4) {
                    float rx = Float.parseFloat(parts[1].trim());
                    float ry = Float.parseFloat(parts[2].trim());
                    String dirStr = parts[3].trim().toUpperCase();
                    float rotation = 0;
                    if (dirStr.equals("E")) rotation = 90;
                    else if (dirStr.equals("S")) rotation = 180;
                    else if (dirStr.equals("W")) rotation = 270;
                    
                    arenaView.updateRobot(rx, ry, rotation);
                }
            } catch (Exception e) {
                appendTerminal("[Error] Failed to parse robot update: " + line);
            }
            return;
        }

        tryParseRobotData(line);
    }

    private void tryParseRobotData(String line) {
        if (line == null) return;

        try {
            // Find start of any JSON object in the line
            int start = line.indexOf("{");
            if (start == -1) return;
            int end = line.lastIndexOf("}");
            if (end == -1 || end < start) return;

            String jsonStr = line.substring(start, end + 1);
            JSONObject json = new JSONObject(jsonStr);

            // Handle Status
            if (json.has("status")) {
                txtRobotStatus.setText(json.getString("status"));
            }

            // Handle Robot Position: {"robot": {"x": 10, "y": 5, "r": 90}}
            if (json.has("robot")) {
                JSONObject robot = json.getJSONObject("robot");
                float x = (float) robot.getDouble("x");
                float y = (float) robot.getDouble("y");
                float r = (float) robot.getDouble("r");
                arenaView.updateRobot(x, y, r);
            }

            // Handle Obstacle: {"obstacle": {"id": 1, "x": 8, "y": 8}}
            if (json.has("obstacle")) {
                JSONObject obs = json.getJSONObject("obstacle");
                int id = obs.getInt("id");
                float ox = (float) obs.getDouble("x");
                float oy = (float) obs.getDouble("y");
                arenaView.addObstacle(id, ox, oy);
            }

            // Handle Map Clear: {"clear": true}
            if (json.has("clear") && json.getBoolean("clear")) {
                arenaView.clearMap();
            }

        } catch (JSONException ignored) {
            // Not a valid JSON or doesn't match expected pattern
        }
    }

    @Override
    public void onError(String message, Throwable t) {
        appendTerminal("[Error] " + message);
    }
}
