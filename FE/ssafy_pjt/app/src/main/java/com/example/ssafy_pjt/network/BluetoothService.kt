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
        Log.d("TAG", "연결 시작: 이름=${device.name}, 주소=${device.address}")
        Log.d("TAG", "블루투스 장치 본딩 상태: ${
            when(device.bondState) {
                BluetoothDevice.BOND_NONE -> "연결 안됨"
                BluetoothDevice.BOND_BONDING -> "연결 중"
                BluetoothDevice.BOND_BONDED -> "연결됨"
                else -> "알 수 없음"
            }
        }")

        // 블루투스 어댑터 상태 확인
        if (bluetoothAdapter == null) {
            Log.e("TAG", "블루투스 어댑터가 null입니다")
            _isConnected.value = false
            return
        }

        if (!bluetoothAdapter!!.isEnabled) {
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
        Log.d("TAG", "기존 연결 정리 완료")

        try {
            Log.d("TAG", "검색 취소 중...")
            bluetoothAdapter?.cancelDiscovery()
            Log.d("TAG", "소켓 생성 중...")

            bluetoothSocket = device.createRfcommSocketToServiceRecord(UUID_SPP)
            Log.d("TAG", "소켓 생성됨: $bluetoothSocket")

            bluetoothSocket?.let { socket ->
                try {
                    // 개선된 연결 로직 - 별도의 타임아웃 처리
                    val connectionThread = Thread {
                        try {
                            Log.d("TAG", "소켓 연결 시작 - 스레드 ID: ${Thread.currentThread().id}")
                            socket.connect()
                            Log.d("TAG", "소켓 연결 성공 - 스레드 ID: ${Thread.currentThread().id}")

                            if (Thread.currentThread().isInterrupted) {
                                Log.e("TAG", "연결은 성공했으나 스레드가 중단됨")
                                socket.close()
                                return@Thread
                            }

                            outputStream = socket.outputStream
                            _isConnected.value = true
                            Log.d("TAG", "연결 성공: ${device.name}")
                        } catch (e: IOException) {
                            Log.e("TAG", "연결 실패 상세: ${e.message}, 스레드 ID: ${Thread.currentThread().id}", e)
                            if (e.message?.contains("read failed") == true) {
                                Log.e("TAG", "읽기 실패 - 기기 응답 없음")
                            } else if (e.message?.contains("timeout") == true) {
                                Log.e("TAG", "연결 시간 초과 - 기기 응답 지연")
                            }
                            _isConnected.value = false
                            try {
                                socket.close()
                            } catch (closeEx: IOException) {
                                Log.e("TAG", "소켓 닫기 실패", closeEx)
                            }
                        } catch (e: InterruptedException) {
                            Log.e("TAG", "연결 스레드 중단됨", e)
                            _isConnected.value = false
                            socket.close()
                        }
                    }

                    connectionThread.isDaemon = true
                    connectionThread.start()

                    // 타임아웃 별도 처리
                    Thread {
                        try {
                            Thread.sleep(10000)
                            if (!_isConnected.value && connectionThread.isAlive) {
                                Log.e("TAG", "연결 시간 초과 - 스레드 중단")
                                connectionThread.interrupt()
                                bluetoothSocket?.close()
                            }
                        } catch (e: Exception) {
                            Log.e("TAG", "타임아웃 처리 중 오류", e)
                        }
                    }.start()

                } catch (e: Exception) {
                    Log.e("TAG", "연결 시도 중 오류: ${e.message}", e)
                    _isConnected.value = false
                    socket.close()
                }
            }
        } catch (e: Exception) {
            Log.e("TAG", "연결 프로세스 오류: ${e.message}", e)
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