package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.ViewModel
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.SnsLoginRequest
import com.example.ssafy_pjt.network.SnsLoginResponse
import com.google.android.gms.auth.api.signin.GoogleSignInAccount
import com.google.android.gms.common.api.ApiException
import com.google.android.gms.tasks.Task
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class GoogleLoginViewModel : ViewModel() {
    private val _loginApiState = MutableStateFlow<ApiState>(ApiState.Idle)
    val loginApiState = _loginApiState.asStateFlow()

    fun handleGoogleSignInResult(task: Task<GoogleSignInAccount>?) {
        _loginApiState.value = ApiState.Loading
        Log.d("TAG"," task ${task}")
        if (task == null) {
            _loginApiState.value = ApiState.Error("Google 로그인 실패")
            return
        }
        try {
            val account = task.getResult(ApiException::class.java)
            account?.let {
                Log.d("TAG", "account.id: ${account.id}")
                Log.d("TAG", "account.email: ${account.email}")
                // 여기서 서버에 토큰을 전송하는 등의 추가 처리
                googleLogin(
                    account.id?.toLongOrNull() ?: -1L,
                    account.email ?: "none"
                )

            } ?: run {
                _loginApiState.value = ApiState.Error("Google 로그인 실패")
            }
        } catch (e: ApiException) {
            Log.e("TAG", "Google sign in failed: ${e.statusCode}")
            _loginApiState.value = ApiState.Error("Google 로그인 실패")
        }
    }

    fun googleLogin(
        id: Long,
        email:String
    ){
        val result = SnsLoginRequest(userNumber = id, username = email, userLoginResource = "google")
        RetrofitClient.instance.snsLogin(result).enqueue(object : Callback<SnsLoginResponse> {
            override fun onResponse(call: Call<SnsLoginResponse>, response: Response<SnsLoginResponse>) {
                if (response.isSuccessful) {
                    val body = response.body()
                    Log.d("TAG","${body}")
                    if (body?.status == "success"){
                        _loginApiState.value = ApiState.Success("Google 로그인 성공")
                        Log.d("TAG","success")
                    } else {
                        _loginApiState.value = ApiState.Error("Google 로그인 에러")
                        Log.d("TAG","${body}")
                    }
                } else {
                    _loginApiState.value = ApiState.Error("서버 에러")
                    Log.d("TAG","server ${response} ,,, ${response}")
                }
            }

            override fun onFailure(call: Call<SnsLoginResponse>, t: Throwable) {
                Log.d("TAG","network")
                _loginApiState.value = ApiState.Error("네트워크 오류")
            }
        })
    }

}


sealed class ApiState {
    object Idle : ApiState()
    object Loading : ApiState()
    data class Success(val message: String) : ApiState()
    data class Error(val message: String) : ApiState()
}
