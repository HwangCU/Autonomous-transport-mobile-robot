package com.example.ssafy_pjt.ViewModel

import android.app.Application
import android.content.Context
import android.util.Log
import androidx.activity.result.ActivityResultLauncher
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.SnsLoginRequest
import com.example.ssafy_pjt.network.SnsLoginResponse
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.auth.api.signin.GoogleSignInAccount
import com.google.android.gms.auth.api.signin.GoogleSignInOptions
import com.google.android.gms.common.api.ApiException
import com.google.android.gms.tasks.Task
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class GoogleLoginViewModel(
    private val userViewModel: UserViewModel
) : ViewModel() {
    private val _loginApiState = MutableStateFlow<String>("")
    val loginApiState = _loginApiState.asStateFlow()

    fun handleGoogleSignInResult(task: Task<GoogleSignInAccount>?) {
        Log.d("TAG", " task ${task}")
        if (task == null) {
            _loginApiState.value = "Google 로그인 실패"
            return
        }
        try {
            val account = task.getResult(ApiException::class.java)
            account?.let {
                Log.d("TAG", "account.id: ${account.id}")
                Log.d("TAG", "account.email: ${account.email}")
                Log.d("TAG", "account.token: ${account?.idToken}")
                // 여기서 서버에 토큰을 전송하는 등의 추가 처리
                googleLogin(
                    account.id?.toLongOrNull() ?: -1L,
                    account.email ?: "none",
                    id_token=account.idToken ?: ""
                )

            } ?: run {
                _loginApiState.value = "Google 로그인 실패"
            }
        } catch (e: ApiException) {
            Log.e("TAG", "Google sign in failed: ${e.statusCode}")
            _loginApiState.value = "Google 로그인 실패"
        }
    }

    fun googleLogin(
        id: Long,
        email: String,
        id_token:String
    ) {
        val result =
            SnsLoginRequest(usernum = id, email = email, userloginresource = "google", token = id_token)
        RetrofitClient.instance.snsLogin(result).enqueue(object : Callback<SnsLoginResponse> {
            override fun onResponse(
                call: Call<SnsLoginResponse>,
                response: Response<SnsLoginResponse>
            ) {
                if (response.isSuccessful) {
                    val body = response.body()
                    Log.d("TAG", "${body}")
                    if (body?.status == "success") {
                        if (body.require_robot){
                            _loginApiState.value = "로봇 등록이 필요합니다"
                            userViewModel.setAccessToken(accessToken = "${body.access_token}", email = body.email)
                        } else {
                            _loginApiState.value = "Google 로그인 성공"
                            userViewModel.setAccessToken(accessToken = "${body.access_token}", email = body.email)
                            Log.d("TAG", "success")
                        }
                    } else {
                        _loginApiState.value = "Google 로그인 에러"
                        Log.d("TAG", "${body}")
                    }
                } else {
                    _loginApiState.value = "서버 에러"
                    Log.d("TAG", "server ${response} ,,, ${response}")
                }
            }

            override fun onFailure(call: Call<SnsLoginResponse>, t: Throwable) {
                Log.d("TAG", "network")
                _loginApiState.value = "네트워크 오류"
            }
        })
    }

    fun signInWithRevoke(context: Context, launcher: ActivityResultLauncher<Int>) {
        val gso = GoogleSignInOptions.Builder(GoogleSignInOptions.DEFAULT_SIGN_IN)
            .requestIdToken(BuildConfig.GOOGLE_OAUTH_CLIENT_ID)
            .requestId()
            .build()

        val googleSignInClient = GoogleSignIn.getClient(context, gso)
        googleSignInClient.revokeAccess().addOnCompleteListener {
            launcher.launch(1)
        }

    }
}

class GoogleLoginViewModelFactory(
    private val userViewModel: UserViewModel,
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return GoogleLoginViewModel(userViewModel) as T
    }
}

sealed class ApiState {
    object Idle : ApiState()
    object Loading : ApiState()
    data class Success(val message: String) : ApiState()
    data class Error(val message: String) : ApiState()
}

