package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.ssafy_pjt.network.LoginRequest
import com.example.ssafy_pjt.network.LoginResponse
import com.example.ssafy_pjt.network.RetrofitClient
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class AccountLoginViewModel: ViewModel() {
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
        Log.d("TAG",username)
        Log.d("TAG",password)
        val request = LoginRequest(username,password)

        RetrofitClient.instance.login(request).enqueue(object : Callback<LoginResponse> {
            override fun onResponse(call: Call<LoginResponse>, response: Response<LoginResponse>) {
                if (response.isSuccessful) {
                    val body = response.body()
                    Log.d("TAG","${body}")
                    if (body?.status == "success"){
                        _loginResult.value = "로그인 성공"
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
