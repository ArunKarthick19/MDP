package com.example.sc2079_group25;

import android.Manifest;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class DeviceListAdapter extends RecyclerView.Adapter<DeviceListAdapter.VH> {

    public interface OnDeviceClick {
        void onClick(BluetoothDevice device);
    }

    private final Map<String, BluetoothDevice> unique = new LinkedHashMap<>();
    private final OnDeviceClick onDeviceClick;
    private final Context context;

    public DeviceListAdapter(Context context, OnDeviceClick onDeviceClick) {
        this.context = context.getApplicationContext();
        this.onDeviceClick = onDeviceClick;
    }

    public void upsert(BluetoothDevice d) {
        if (d == null) return;

        String address;
        try {
            address = d.getAddress();
        } catch (SecurityException e) {
            return;
        }
        
        if (address == null) return;

        unique.put(address, d);
        notifyDataSetChanged();
    }

    public void clear() {
        unique.clear();
        notifyDataSetChanged();
    }

    public List<BluetoothDevice> getItems() {
        return new ArrayList<>(unique.values());
    }

    @NonNull
    @Override
    public VH onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        TextView tv = (TextView) LayoutInflater.from(parent.getContext())
                .inflate(android.R.layout.simple_list_item_1, parent, false);
        return new VH(tv);
    }

    @Override
    public void onBindViewHolder(@NonNull VH holder, int position) {
        BluetoothDevice d = getItems().get(position);
        holder.bind(d);
        holder.itemView.setOnClickListener(v -> onDeviceClick.onClick(d));
    }

    @Override
    public int getItemCount() {
        return unique.size();
    }

    // ===== Permission-safe helpers =====

    private boolean hasConnectPermission() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) return true;
        return ActivityCompat.checkSelfPermission(
                context,
                Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED;
    }

    private String getSafeName(BluetoothDevice d) {
        if (d == null) return "Unknown Device";

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S && !hasConnectPermission()) {
            return "Unknown Device";
        }

        try {
            String name = d.getName();
            return (name != null && !name.isEmpty()) ? name : "Unknown Device";
        } catch (SecurityException e) {
            return "Unknown Device";
        }
    }

    // ===== ViewHolder =====

    static class VH extends RecyclerView.ViewHolder {
        VH(@NonNull View itemView) { super(itemView); }

        void bind(BluetoothDevice d) {
            TextView tv = (TextView) itemView;
            DeviceListAdapter adapter = (DeviceListAdapter) getBindingAdapter();

            String address;
            try {
                address = d.getAddress();
            } catch (SecurityException e) {
                address = "Unknown Address";
            }

            String name = (adapter != null) ? adapter.getSafeName(d) : "Unknown Device";

            // Display Name and Address on separate lines
            tv.setText(name + "\n" + address);
        }
    }
}
