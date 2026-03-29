package com.example.sc2079_group25;

public interface BluetoothEventListener {
    void onConnectionStateChanged(int state, String detail);
    void onLineReceived(String line);
    void onError(String message, Throwable t);
}
