package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.ssafy_pjt.network.SocketService
import kotlinx.coroutines.launch

class socketViewModel: ViewModel() {
    private val socketService = SocketService.getInstance()
    val serverResponses = socketService.serverResponses

    fun connectToSocket() {
        viewModelScope.launch {
            try {
                // 연결 시도 로그
                Log.d("TAG", "Attempting to connect")
                socketService.connect()
            } catch (e: Exception) {
                // 연결 실패 상세 로그
                Log.e("TAG", "Connection error: ${e.message}", e)
            }
        }
    }

    fun sendSocketMessage(message: String) {
        viewModelScope.launch {
            socketService.sendMessage(message)
        }
    }

    fun sendSocketDisconnect() {
        viewModelScope.launch {
            socketService.disconnect()
        }
    }
}
