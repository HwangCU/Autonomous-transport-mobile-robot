package com.example.ssafy_pjt.network

import android.content.Context
import android.net.ConnectivityManager
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.Socket
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

class SocketService private constructor() {
    private var socket: Socket? = null
    private var reader: BufferedReader? = null
    private var writer: PrintWriter? = null
    private val _serverResponses = MutableStateFlow<List<String>>(emptyList())
    val serverResponses = _serverResponses.asStateFlow()
    private val _isConnected = MutableStateFlow(false)
    val isConnected = _isConnected.asStateFlow()
    private var reconnectJob: Job? = null
    private var messageListenerJob: Job? = null


    companion object {
        @Volatile
        private var instance: SocketService? = null

        fun getInstance(): SocketService =
            instance ?: synchronized(this) {
                instance ?: SocketService().also { instance = it }
            }
    }

    fun getLastMessage(): String? {
        return _serverResponses.value.lastOrNull()
    }

    // 특정 타입의 메시지 필터링
    fun filterMessages(predicate: (String) -> Boolean): List<String> {
        return _serverResponses.value.filter(predicate)
    }

    suspend fun connect() = withContext(Dispatchers.IO) {
        try {
            // 이미 연결된 상태라면 skip
            if (_isConnected.value) return@withContext

            socket = Socket("c103.duckdns.org", 9999)
            reader = BufferedReader(InputStreamReader(socket?.inputStream))
            writer = PrintWriter(socket?.outputStream, true)

            _isConnected.value = true
            Log.d("TAG", "Connected Successfully")

            // 메시지 리스너 시작
            startMessageListener()
        } catch (e: Exception) {
            Log.e("TAG", "Connection failed: ${e.message}", e)
            _isConnected.value = false
            disconnect()
        }
    }
    private fun startMessageListener() {
        // 기존 리스너 취소
        messageListenerJob?.cancel()

        // 새 리스너 시작
        messageListenerJob = CoroutineScope(Dispatchers.IO).launch {
            try {
                while (_isConnected.value) {
                    val message = reader?.readLine()
                    message?.let {
                        Log.d("TAG", "Received: $it")

                        // 새 메시지를 리스트에 추가
                        _serverResponses.value = _serverResponses.value + it
                    }

                    // 과도한 루프 방지
                    delay(50)
                }
            } catch (e: Exception) {
                Log.e("TAG", "Message listener error: ${e.message}")
                disconnect()
            }
        }
    }

    private fun startReconnectMonitor() {
        // 기존 작업 취소
        reconnectJob?.cancel()

        reconnectJob = CoroutineScope(Dispatchers.IO).launch {
            while (_isConnected.value) {
                delay(30000) // 30초마다 연결 상태 확인

                if (!isSocketAlive()) {
                    Log.d("TAG", "소켓 연결 끊김. 재연결 시도")
                    disconnect()
                    delay(5000) // 재연결 대기 시간
                    connect()
                }
            }
        }
    }

    private fun isSocketAlive(): Boolean {
        return try {
            socket?.let {
                // 간단한 연결 상태 확인
                socket?.isConnected == true && !socket?.isClosed!!
            } ?: false
        } catch (e: Exception) {
            false
        }
    }

    suspend fun sendMessage(message: String) = withContext(Dispatchers.IO) {
        try {
            writer?.println(message)
            Log.d("TAG", "Sent: $message")
        } catch (e: Exception) {
            Log.e("TAG", "Failed to send message", e)
            disconnect()
        }
    }

    suspend fun disconnect() = withContext(Dispatchers.IO) {
        try {
            _isConnected.value = false
            reconnectJob?.cancel()

            reader?.close()
            writer?.close()
            socket?.close()

            reader = null
            writer = null
            socket = null

            Log.d("TAG", "소켓 연결 해제")
        } catch (e: Exception) {
            Log.e("TAG", "연결 해제 중 오류: ${e.message}")
        }
    }
}