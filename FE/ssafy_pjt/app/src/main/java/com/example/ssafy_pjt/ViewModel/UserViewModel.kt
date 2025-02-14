package com.example.ssafy_pjt.ViewModel

import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class UserViewModel:ViewModel(){
    private  val _path = MutableStateFlow<List<Pair<Double,Double>>>(emptyList())
    val path : StateFlow<List<Pair<Double,Double>>> = _path.asStateFlow()

    private val _location = MutableStateFlow(Pair(-1.0,-1.0))
    val loaction : StateFlow<Pair<Double,Double>> = _location.asStateFlow()

    private  var _accessToken = MutableStateFlow("")
    companion object {
        // 테스트용 토큰 상수
        private const val TEST_ACCESS_TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJyZXF1ZXN0ZXJfdHlwZSI6InVzZXIiLCJ1c2VyX2lkIjo5LCJleHAiOjE3NDA3MDg1MjZ9.5xjjZPG3_x533JAeeMxaQZobkjlnEyvQqIdkDwBAE0M"
        private const val TEST_EMAIL = "a@a.a"
    }
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

    // 프로필 이미지 상태
    private val _profileImage = MutableStateFlow<Uri?>(null)
    val profileImage: StateFlow<Uri?> = _profileImage.asStateFlow()

    // 임시 프로필 이미지 (편집 중)
    private val _tempProfileImage = MutableStateFlow<Uri?>(null)
    val tempProfileImage: StateFlow<Uri?> = _tempProfileImage.asStateFlow()

    init {
        // 테스트를 위해 초기값 설정
        _accessToken.value = TEST_ACCESS_TOKEN
        _email.value = TEST_EMAIL
    }

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

    fun updateTempProfileImage(uri: Uri?) {
        _tempProfileImage.value = uri
    }

    fun confirmProfileImage() {
        _profileImage.value = _tempProfileImage.value
    }

    fun cancelProfileImageEdit() {
        _tempProfileImage.value = _profileImage.value
    }

    fun setLocati8on(
        x:Double,
        y:Double
    ) {
        _location.value = Pair(x,y)
    }
}