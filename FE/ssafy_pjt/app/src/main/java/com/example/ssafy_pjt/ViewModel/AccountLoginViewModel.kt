package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.ssafy_pjt.network.LoginRequest
import com.example.ssafy_pjt.network.LoginResponse
import com.example.ssafy_pjt.network.RetrofitClient
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class AccountLoginViewModel (
    private val userViewModel: UserViewModel
): ViewModel() {

    private var _userName = MutableLiveData("")
    val userName: LiveData<String> get() = _userName

    private var _userPassword = MutableLiveData("")
    val userPassword: LiveData<String> get() = _userPassword

    private val _loginResult = MutableLiveData<String>()
    val loginResult: LiveData<String> get() = _loginResult

    fun setUserName(userName: String){
        _userName.value = userName
    }
    fun setUserPassword(password: String) {
        _userPassword.value = password
    }

    fun login(){
        val username = _userName.value ?: ""
        val password = _userPassword.value ?: ""
        val request = LoginRequest(username,password)

        RetrofitClient.instance.login(request).enqueue(object : Callback<LoginResponse> {
            override fun onResponse(call: Call<LoginResponse>, response: Response<LoginResponse>) {
                if (response.isSuccessful) {
                    val body = response.body()
                    Log.d("TAG","${body}")
                    if (body?.status == "success"){
                        if (body.require_robot){
                            userViewModel.setAccessToken(accessToken = body.access_token ?: "", username= body.username)
                            _loginResult.value = "로봇이 필요합니다"
                        } else {
                        _loginResult.value = "로그인 성공"
                        }
                        Log.d("TAG","success")
                    } else {
                        _loginResult.value = "로그인 실패"
                        Log.d("TAG","false")
                    }
                } else {
                    _loginResult.value= "서버 오류"
                    Log.d("TAG","server ${response} ,,, ${response}")
                }
            }

            override fun onFailure(call: Call<LoginResponse>, t: Throwable) {
                Log.d("TAG","network")
                _loginResult.value="네트워크 오류"
            }
        })
    }
}

class AccountLoginViewModelFactory(
    private val userViewModel: UserViewModel
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return AccountLoginViewModel(userViewModel) as T
    }
}
