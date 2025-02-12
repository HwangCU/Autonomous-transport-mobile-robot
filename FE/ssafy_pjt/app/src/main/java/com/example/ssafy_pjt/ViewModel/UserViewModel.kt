package com.example.ssafy_pjt.ViewModel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

class UserViewModel:ViewModel(){
    private  val _path = MutableStateFlow<List<Pair<Double,Double>>>(emptyList())
    val path : StateFlow<List<Pair<Double,Double>>> = _path.asStateFlow()

    private val _location = MutableStateFlow(Pair(-1.0,-1.0))
    val loaction : StateFlow<Pair<Double,Double>> = _location.asStateFlow()

    private  var _accessToken = MutableStateFlow("")
    val accessToken: StateFlow<String> = _accessToken.asStateFlow()

    private  var _time = MutableStateFlow(0)
    val time: StateFlow<Int> = _time.asStateFlow()

    fun setTime(it:Int){
        _time.value=it
    }

    private var _email = MutableStateFlow("")
    val email: StateFlow<String> = _email.asStateFlow()

    private var _loginResource = MutableStateFlow("")
    val loginResource = _loginResource.asStateFlow()

    fun setPath(it:List<Pair<Double,Double>>){
        _path.value=it
    }

    fun setLoginResource(it:String){
        _loginResource.value=it
    }

    fun setAccessToken(
        accessToken:String,
        email:String
    ){
        _accessToken.value = accessToken
        _email.value=email
    }

    fun setLocati8on(
        x:Double,
        y:Double
    ) {
        _location.value = Pair(x,y)
    }
}