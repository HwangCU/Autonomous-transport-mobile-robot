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
import android.content.BroadcastReceiver
import android.content.Intent
import android.content.IntentFilter
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

class BluetoothService private constructor(private val context: Context) {
    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager =
            context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }
    private val UUID_SPP = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var _isConnected = MutableStateFlow(false)
    val isConnected = _isConnected.asStateFlow()
    private val _discoveredDevices = MutableStateFlow<Set<BluetoothDevice>>(emptySet())
    val discoveredDevices = _discoveredDevices.asStateFlow()

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
        if (!hasBluetoothPermissions()) {
            throw SecurityException("Bluetooth permissions not granted")
        }
        return bluetoothAdapter?.bondedDevices ?: emptySet()
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        // 블루투스 어댑터 상태 확인
        if (bluetoothAdapter == null || !bluetoothAdapter!!.isEnabled) {
            Log.e("TAG", "블루투스가 비활성화되었습니다")
            _isConnected.value = false
            return
        }

        // 권한 확인
        if (!hasBluetoothPermissions()) {
            Log.e("TAG", "블루투스 권한이 없습니다")
            _isConnected.value = false
            return
        }

        disconnect() // 기존 연결 정리

        try {
            bluetoothAdapter?.cancelDiscovery()

            bluetoothSocket = device.createRfcommSocketToServiceRecord(UUID_SPP)
            bluetoothSocket?.let { socket ->
                try {
                    // 타임아웃 구현을 위한 스레드 사용
                    Thread {
                        try {
                            socket.connect()

                            // UI 스레드에서 상태 업데이트
                            outputStream = socket.outputStream
                            _isConnected.value = true
                            Log.d("TAG", "연결 성공: ${device.name}")
                        } catch (e: IOException) {
                            Log.e("TAG", "연결 실패: ${e.message}", e)
                            _isConnected.value = false
                            socket.close()
                        }
                    }.apply {
                        // 데몬 스레드로 설정
                        isDaemon = true
                        // 10초 후 인터럽트
                        start()
                        join(10000)

                        // 여전히 연결되지 않았다면 인터럽트
                        if (!_isConnected.value) {
                            interrupt()
                            socket.close()
                            Log.e("TAG", "연결 시간 초과")
                        }
                    }
                } catch (e: Exception) {
                    Log.e("TAG", "연결 시도 중 오류", e)
                    _isConnected.value = false
                    socket.close()
                }
            }
        } catch (e: Exception) {
            Log.e("TAG", "연결 프로세스 오류", e)
            _isConnected.value = false
            bluetoothSocket?.close()
            bluetoothSocket = null
            outputStream = null
        }
    }

    fun disconnect() {
        try {
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            Log.e("TAG", "연결 해제 중 오류", e)
        } finally {
            outputStream = null
            bluetoothSocket = null
            _isConnected.value = false
        }
    }

    fun sendMessage(x: Float, y: Float) {
        try {
            val jsonString = """{"x":$x,"y":$y}"""
            outputStream?.write(jsonString.toByteArray())
            Log.d("TAG", "Sent: $jsonString")
        } catch (e: IOException) {
            Log.e("TAG", "Send failed", e)
        }
    }

    private fun hasBluetoothPermissions(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            context.checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED &&
                    context.checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                    context.checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        } else {
            context.checkSelfPermission(Manifest.permission.BLUETOOTH) == PackageManager.PERMISSION_GRANTED &&
                    context.checkSelfPermission(Manifest.permission.BLUETOOTH_ADMIN) == PackageManager.PERMISSION_GRANTED &&
                    context.checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        }
    }

    @SuppressLint("MissingPermission")
    fun startDiscovery() {
        // 블루투스 어댑터 및 권한 확인 강화
        if (bluetoothAdapter == null) {
            Log.e("TAG", "블루투스 어댑터를 찾을 수 없습니다")
            return
        }

        if (!bluetoothAdapter!!.isEnabled) {
            Log.e("TAG", "블루투스가 비활성화되었습니다")
            return
        }

        // 권한 확인 (위치 권한 추가)
        if (!hasBluetoothPermissions()) {
            Log.e("TAG", "블루투스 검색 권한이 없습니다")
            return
        }

        // 기존 검색 중지
        bluetoothAdapter?.cancelDiscovery()

        // 검색 리시버 등록
        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
        }
        context.registerReceiver(discoveryReceiver, filter)

        // 검색 시작 및 로깅 강화
        try {
            val isStarted = bluetoothAdapter?.startDiscovery() ?: false
            Log.d("TAG", "블루투스 검색 시작: $isStarted")

            // 검색된 기기 초기화
            _discoveredDevices.value = emptySet()
        } catch (e: SecurityException) {
            Log.e("TAG", "블루투스 검색 권한 오류", e)
        }
    }


    @SuppressLint("MissingPermission")
    fun cancelDiscovery() {
        bluetoothAdapter?.cancelDiscovery()
    }

    // 리시버 등록 해제
    fun unregisterDiscoveryReceiver() {
        try {
            context.unregisterReceiver(discoveryReceiver)
        } catch (e: IllegalArgumentException) {
            // 리시버가 이미 등록 해제된 경우 예외 처리
        }
    }

    private val discoveryReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                BluetoothDevice.ACTION_FOUND -> {
                    // 새로운 기기 발견
                    val device: BluetoothDevice? =
                        intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                    device?.let {
                        // 이름이 있는 모든 기기 추가 (페어링 상태 무관)
                        if (it.name != null) {
                            val currentDevices = _discoveredDevices.value.toMutableSet()
                            currentDevices.add(it)
                            _discoveredDevices.value = currentDevices

                            Log.d(
                                "TAG",
                                "발견된 기기: 이름=${it.name}, 주소=${it.address}, 페어링 상태=${it.bondState}"
                            )
                        }
                    }
                }

                BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                    Log.d("TAG", "블루투스 검색 완료")
                    context.unregisterReceiver(this)
                }
            }
        }
    }
}