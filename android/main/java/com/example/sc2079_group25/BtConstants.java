package com.example.sc2079_group25;

import java.util.UUID;

public final class BtConstants {
    private BtConstants() {}

    // Standard SPP UUID (Serial Port Profile)
    public static final UUID SPP_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    public static final int STATE_NONE = 0;
    public static final int STATE_CONNECTING = 1;
    public static final int STATE_CONNECTED = 2;
    public static final int STATE_LISTENING = 3;

    public static final String PREFS_NAME = "BT_TERMINAL_PREFS";
    public static final String KEY_LAST_DEVICE = "last_device_addr";
}
