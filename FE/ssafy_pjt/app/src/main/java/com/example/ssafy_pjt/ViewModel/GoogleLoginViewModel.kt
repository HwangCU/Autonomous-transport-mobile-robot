package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.ViewModel
import com.google.android.gms.auth.api.signin.GoogleSignInAccount
import com.google.android.gms.common.api.ApiException
import com.google.android.gms.tasks.Task
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

class GoogleLoginViewModel : ViewModel() {
    private val _loginApiState = MutableStateFlow<ApiState>(ApiState.Idle)
    val loginApiState = _loginApiState.asStateFlow()

    fun handleGoogleSignInResult(task: Task<GoogleSignInAccount>?) {
        _loginApiState.value = ApiState.Loading
        if (task == null) {
            _loginApiState.value = ApiState.Error("Google 로그인 실패")
            return
        }
        try {
            val account = task.getResult(ApiException::class.java)
            account?.let {
                Log.d("LoginViewModel", "Google sign in success: ${account.id}")
                // 여기서 서버에 토큰을 전송하는 등의 추가 처리
                _loginApiState.value = ApiState.Success("Google 로그인 성공")
            } ?: run {
                _loginApiState.value = ApiState.Error("Google 로그인 실패")
            }
        } catch (e: ApiException) {
            Log.e("LoginViewModel", "Google sign in failed: ${e.statusCode}")
            _loginApiState.value = ApiState.Error("Google 로그인 실패")
        }
    }
}

sealed class ApiState {
    object Idle : ApiState()
    object Loading : ApiState()
    data class Success(val message: String) : ApiState()
    data class Error(val message: String) : ApiState()
}