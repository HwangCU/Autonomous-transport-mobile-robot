package com.example.ssafy_pjt.ViewModel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

class UserViewModel:ViewModel(){
    private  var _accessToken = MutableStateFlow("")
    val accessToken: StateFlow<String> = _accessToken.asStateFlow()

    private var _username = MutableStateFlow("")
    val username: StateFlow<String> = _username.asStateFlow()

    fun setAccessToken(
        accessToken:String,
        username:String
    ){
        _accessToken.value = accessToken
        _username.value=username
    }
}