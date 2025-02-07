package com.example.ssafy_pjt.ViewModel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

class UserViewModel:ViewModel(){
    private  var _accessToken = MutableStateFlow("")
    val accessToken: StateFlow<String> = _accessToken.asStateFlow()

    private var _email = MutableStateFlow("")
    val email: StateFlow<String> = _email.asStateFlow()

    fun setAccessToken(
        accessToken:String,
        email:String
    ){
        _accessToken.value = accessToken
        _email.value=email
    }
}