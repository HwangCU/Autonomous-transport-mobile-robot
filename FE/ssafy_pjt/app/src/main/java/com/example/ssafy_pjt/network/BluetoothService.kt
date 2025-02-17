package com.example.ssafy_pjt.network

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager

class BluetoothService private constructor(private val context: Context) {
    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null

    companion object {
        @Volatile
        private var instance: BluetoothService? = null

        fun getInstance(context: Context): BluetoothService =
            instance ?: synchronized(this) {
                instance ?: BluetoothService(context.applicationContext).also { instance = it }
            }
    }

    @SuppressLint("MissingPermission")
    fun getPairedDevices(): Set<BluetoothDevice> {
        return if (hasBluetoothPermissions()) {
            bluetoothAdapter?.bondedDevices ?: emptySet()
        } else {
            emptySet()
        }
    }
    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        if (hasBluetoothPermissions()) {
            try {
                // HC-06 블루투스 모듈의 기본 UUID
                val uuid = UUID.fromString("12345678-1234-5678-1234-56789abcdef1")
                bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
                bluetoothSocket?.connect()
                outputStream = bluetoothSocket?.outputStream
                Log.d("BluetoothService", "Connected to ${device.name}")
            } catch (e: IOException) {
                Log.e("BluetoothService", "Connection failed", e)
            }
        } else {
            Log.e("BluetoothService", "Missing Bluetooth permissions")
        }
    }

    fun sendMessage(x: Float, y: Float) {
        try {
            val jsonString = """{"x":$x,"y":$y}"""
            outputStream?.write(jsonString.toByteArray())
            Log.d("BluetoothService", "Sent: $jsonString")
        } catch (e: IOException) {
            Log.e("BluetoothService", "Send failed", e)
        }
    }

    fun disconnect() {
        try {
            bluetoothSocket?.close()
        } catch (e: IOException) {
            Log.e("BluetoothService", "Disconnect failed", e)
        }
    }

    private fun hasBluetoothPermissions(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            context.checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED &&
                    context.checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED
        } else {
            context.checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        }
    }
}