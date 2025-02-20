package com.example.ssafy_pjt.network

import android.content.Context
import android.graphics.BitmapFactory
import android.net.ConnectivityManager
import android.util.Log
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.ssafy_pjt.ViewModel.RobotViewModel
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import org.json.JSONObject
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.InetSocketAddress
import java.net.Socket

class SocketService private constructor() {
    private var robotViewModel: RobotViewModel? =null
    private var socket: Socket? = null
    private var reader: BufferedReader? = null
    private var writer: PrintWriter? = null
    private val _serverResponses = MutableStateFlow<List<String>>(emptyList())
    val serverResponses = _serverResponses.asStateFlow()
    private val _isConnected = MutableStateFlow(false)
    val isConnected = _isConnected.asStateFlow()
    private var reconnectJob: Job? = null
    private var messageListenerJob: Job? = null
    private var pingJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.IO + Job()) // 코루틴 스코프 정의
    private var lastMessageTime = 0L // 마지막 메시지 수신 시간

    fun setRobotViewModel(viewModel: RobotViewModel) {
        this.robotViewModel = viewModel
        Log.d("socket", "SocketService에 RobotViewModel 설정됨")
    }

    companion object {
        private const val READ_TIMEOUT = 200L // 200ms
        private const val MAX_MESSAGES = 100 // 최대 메시지 수

        @Volatile
        private var instance: SocketService? = null

        fun getInstance(): SocketService =
            instance ?: synchronized(this) {
                instance ?: SocketService().also { instance = it }
            }
    }

    suspend fun connect() = withContext(Dispatchers.IO) {
        try {
            if (_isConnected.value) return@withContext

            socket = Socket().apply {
                connect(InetSocketAddress("c103.duckdns.org", 9999), 600000)
                keepAlive = true // TCP 수준의 연결 유지
                tcpNoDelay = true // Nagle 알고리즘 비활성화
            }
            reader = BufferedReader(InputStreamReader(socket?.inputStream))
            writer = PrintWriter(socket?.outputStream, true)

            _isConnected.value = true
            lastMessageTime = System.currentTimeMillis()
            Log.d("socket", "Connected Successfully")

            startMessageListener()
        } catch (e: Exception) {
            Log.e("socket", "Connection failed: ${e.message}", e)
            _isConnected.value = false
            disconnect()
        }
    }

    private fun startMessageListener() {
        messageListenerJob?.cancel()

        messageListenerJob = scope.launch {
            try {
                while (_isConnected.value) {
                    try {
                        val message = withTimeoutOrNull(READ_TIMEOUT) {
                            reader?.readLine()
                        }
                        /*Log.d("TAG","메시지 ${message}")*/
                        if (message == null) {
                            // 읽기 타임아웃이 발생했지만 소켓 상태 확인
                            if (!isSocketAlive()) {
                                Log.d("socket", "소켓 연결 끊김 감지, 재연결 시도")
                                reconnect()
                                continue
                            }
                        } else {
                            // 메시지 수신 성공
                            lastMessageTime = System.currentTimeMillis()
                            processMessage(message)
                        }
                    } catch (e: Exception) {
                        if (e is CancellationException) throw e

                        if (e.message?.contains("EOF") == true ||
                            e.message?.contains("closed") == true ||
                            e.message?.contains("reset") == true ||
                            e.message?.contains("종료") == true) {
                            Log.w("socket", "연결 오류 감지: ${e.message}, 재연결 시도")
                            reconnect()
                            continue
                        }

                        // 일반적인 읽기 타임아웃은 무시 (과도한 로깅 방지)
                        if (e.message?.contains("timed out") != true) {
                            Log.w("socket", "읽기 오류: ${e.message}")
                        }
                    }

                    delay(10) // 짧은 딜레이
                }
            } catch (e: Exception) {
                Log.e("socket", "리스너 오류: ${e.message}")
                disconnect()
            }
        }
    }

    private fun processMessage(message: String) {
        try {
            // 메시지 형식 확인 및 처리
            if (message.startsWith("{") && message.endsWith("}")) {
                try {
                    val json = JSONObject(message)
                    val action = json.optString("action", "")
                    val status = json.optString("status", "")
                    Log.d("socket",action)
                    // action 기반 처리
                    when (action) {
                        "pong" -> {
                            return
                        }
                        "get_gps" -> {
                            val latitude = json.optDouble("latitude")
                            val longitude = json.optDouble("longitude")
                            val robotId = json.optString("robot_id")
                            robotViewModel?.let { viewModel ->
                                viewModel.saveLocation(latitude, longitude)
                                viewModel.saveBettery(75)
                                viewModel.saveRobotId(robotId)
                                Log.d("socket", "위치: ${viewModel.location.value}, 로봇ID: ${viewModel.robot_id.value}")
                            }
                        }
                        "robot_weight" -> {
                            val weight = json.optDouble("weight")
                            robotViewModel?.let { viewModel->
                                viewModel.saveWeight(weight.toInt())
                                Log.d("socket","무게요 ${viewModel.weight.value}")
                            }
                        }
                        "camera_image" -> {
                            val image = json.optString("image")
                            val imageBytes = android.util.Base64.decode(image, android.util.Base64.DEFAULT)
                            val bitMap = BitmapFactory.decodeByteArray(imageBytes,0,imageBytes.size)
                            robotViewModel?.saveImage(bitMap)
                        }
                    }

                    // status 기반 처리
                    if (status == "success") {
                        val message = json.optString("message", "")
                        val mode = json.optInt("mode", -1)
                        Log.d("socket", "상태 메시지 수신: status=$status, message=$message, mode=$mode")
                    }
                } catch (e: Exception) {
                    Log.w("socket", "JSON 파싱 오류: ${e.message}")
                }
            }

            // 메시지 저장
            _serverResponses.update { currentList ->
                (currentList + message).takeLast(MAX_MESSAGES)
            }
        } catch (e: Exception) {
            Log.e("socket", "메시지 처리 오류: ${e.message}")
        }
    }


    private suspend fun reconnect() {
        try {
            disconnect()
            delay(1000) // 잠시 대기
            connect()
        } catch (e: Exception) {
            Log.e("socket", "재연결 실패: ${e.message}")
        }
    }

    private fun isSocketAlive(): Boolean {
        return try {
            socket?.let {
                it.isConnected && !it.isClosed &&
                        writer != null && reader != null
            } ?: false
        } catch (e: Exception) {
            Log.w("socket", "소켓 상태 확인 오류: ${e.message}")
            false
        }
    }

    suspend fun sendMessage(message: String) = withContext(Dispatchers.IO) {
        try {
            if (!isSocketAlive()) {
                Log.w("socket", "메시지 전송 시 연결 끊김 감지, 재연결 시도")
                reconnect()
            }

            writer?.println(message)
            writer?.flush() // 즉시 전송 보장
            Log.d("socket", "전송: $message")
        } catch (e: Exception) {
            Log.e("socket", "메시지 전송 실패: ${e.message}")
            reconnect()
        }
    }

    suspend fun disconnect() = withContext(Dispatchers.IO) {
        try {
            _isConnected.value = false

            // 모든 작업 취소
            pingJob?.cancel()
            reconnectJob?.cancel()
            messageListenerJob?.cancel()

            // 리소스 정리
            try { reader?.close() } catch (e: Exception) {}
            try { writer?.close() } catch (e: Exception) {}
            try { socket?.close() } catch (e: Exception) {}

            reader = null
            writer = null
            socket = null

            Log.d("socket", "소켓 연결 해제 완료")
        } catch (e: Exception) {
            Log.e("socket", "연결 해제 중 오류: ${e.message}")
        }
    }

}