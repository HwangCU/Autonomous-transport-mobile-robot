package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.ssafy_pjt.network.SignupRequest
import com.example.ssafy_pjt.network.SignupResponse
import com.example.ssafy_pjt.network.RetrofitClient
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class SignupViewModel : ViewModel() {
    // 입력 필드를 위한 LiveData
    private var _username = MutableLiveData("")
    val username: LiveData<String> get() = _username

    private var _userEmail = MutableLiveData("")
    val userEmail: LiveData<String> get() = _userEmail

    private var _userPassword = MutableLiveData("")
    val userPassword: LiveData<String> get() = _userPassword

    // 회원가입 결과를 저장할 LiveData
    private val _signupResult = MutableLiveData<String>()
    val signupResult: LiveData<String> get() = _signupResult

    // Setter 함수들
    fun setUsername(name: String) {
        _username.value = name
    }

    fun setUserEmail(email: String) {
        _userEmail.value = email
    }

    fun setUserPassword(password: String) {
        _userPassword.value = password
    }

    // 회원가입 함수
    fun signup() {
        val username = _username.value ?: ""
        val email = _userEmail.value ?: ""
        val password = _userPassword.value ?: ""

        val request = SignupRequest(username, email, password)

        RetrofitClient.instance.signup(request).enqueue(object : Callback<SignupResponse> {
            override fun onResponse(call: Call<SignupResponse>, response: Response<SignupResponse>) {
                if (response.isSuccessful) {
                    val body = response.body()
                    if (body?.action == "ok") {
                        _signupResult.value = "회원가입 성공"
                        Log.d("SignupViewModel", "회원가입 성공")
                    } else {
                        _signupResult.value = "${body?.message}"
                        Log.d("SignupViewModel", "회원가입 실패: ${body?.message}")
                    }
                } else {
                    _signupResult.value = "${response.code()}"
                    Log.d("SignupViewModel", "서버 오류: ${response.errorBody()?.string()}")
                }
            }

            override fun onFailure(call: Call<SignupResponse>, t: Throwable) {
                _signupResult.value = "네트워크 오류"
                Log.e("SignupViewModel", "네트워크 오류", t)
            }
        })
    }

    // 입력값 유효성 검사
    fun validateInputs(): Boolean {
        val username = _username.value ?: ""
        val email = _userEmail.value ?: ""
        val password = _userPassword.value ?: ""

        if (username.isBlank() || email.isBlank() || password.isBlank()) {
            _signupResult.value = "모든 필드를 입력해주세요"
            return false
        }

        if (!android.util.Patterns.EMAIL_ADDRESS.matcher(email).matches()) {
            _signupResult.value = "올바른 이메일 형식이 아닙니다"
            return false
        }

        if (password.length < 8) {
            _signupResult.value = "비밀번호는 8자 이상이어야 합니다"
            return false
        }

        return true
    }
}