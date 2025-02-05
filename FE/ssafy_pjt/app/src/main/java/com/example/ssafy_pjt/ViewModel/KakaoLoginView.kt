package com.example.ssafy_pjt.ViewModel

import android.app.Application
import android.content.ContentValues.TAG
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import com.example.ssafy_pjt.network.LoginResponse
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.SnsLoginRequest
import com.example.ssafy_pjt.network.SnsLoginResponse
import com.kakao.sdk.auth.model.OAuthToken
import com.kakao.sdk.common.model.ClientError
import com.kakao.sdk.common.model.ClientErrorCause
import com.kakao.sdk.common.util.Utility
import com.kakao.sdk.user.UserApiClient
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response


class KakaoAuthViewModel(application: Application) : AndroidViewModel(application) {
    private val context = application.applicationContext
    private val _kakaologinResult = MutableLiveData("")
    val kakaologinResult: LiveData<String> get() = _kakaologinResult


    fun handleKakaoLogin() {
        // 로그인 조합 예제
        // 카카오계정으로 로그인 공통 callback 구성
        // 카카오톡으로 로그인 할 수 없어 카카오계정으로 로그인할 경우 사용됨
        val callback: (OAuthToken?, Throwable?) -> Unit = { token, error ->
            if (error != null) {
                Log.e("TAG", "카카오계정으로 로그인 실패", error)
            } else if (token != null) {
                Log.i("TAG", "카카오계정으로 로그인 성공 ${token.accessToken}")
                UserInfo()
            }
        }

        // 카카오톡이 설치되어 있으면 카카오톡으로 로그인, 아니면 카카오계정으로 로그인
        if (UserApiClient.instance.isKakaoTalkLoginAvailable(context)) {
            UserApiClient.instance.loginWithKakaoTalk(context) { token, error ->
                if (error != null) {
                    Log.e("TAG", "카카오톡으로 로그인 실패", error)

                    // 사용자가 카카오톡 설치 후 디바이스 권한 요청 화면에서 로그인을 취소한 경우,
                    // 의도적인 로그인 취소로 보고 카카오계정으로 로그인 시도 없이 로그인 취소로 처리 (예: 뒤로 가기)
                    if (error is ClientError && error.reason == ClientErrorCause.Cancelled) {
                        Log.d("TAG", "check3")
                        return@loginWithKakaoTalk
                    }

                    // 카카오톡에 연결된 카카오계정이 없는 경우, 카카오계정으로 로그인 시도
                    UserApiClient.instance.loginWithKakaoAccount(context, callback = callback)
                } else if (token != null) {
                    Log.i("TAG", "카카오톡으로 로그인 성공 ${token.accessToken}")
                }
            }
        } else {
            UserApiClient.instance.loginWithKakaoAccount(context, callback = callback)
        }
    }

    fun UserInfo(){
        // 사용자 정보 요청 (기본)
        Log.d("TAG","here")
        UserApiClient.instance.me { user, error ->
            if (error != null) {
                Log.e("TAG", "사용자 정보 요청 실패", error)
            }
            else if (user != null) {
                Log.i("TAG", "사용자 정보 요청 성공" +
                        "\n회원번호: ${user.id}" +
                        "\n이메일: ${user.kakaoAccount?.email}" +
                        "\n닉네임: ${user.kakaoAccount?.profile?.nickname}" +
                        "\n프로필사진: ${user.kakaoAccount?.profile?.thumbnailImageUrl}")
                snsLogin(
                    user.id ?: -1L,
                    user.kakaoAccount?.email ?: "none"
                )
            }
        }
    }

    fun snsLogin(
        id: Long,
        email:String
    ){
        val result = SnsLoginRequest(userNumber = id, username = email, userLoginResource = "kakao")
        RetrofitClient.instance.kakaoLogin(result).enqueue(object : Callback<SnsLoginResponse> {
            override fun onResponse(call: Call<SnsLoginResponse>, response: Response<SnsLoginResponse>) {
                if (response.isSuccessful) {
                    val body = response.body()
                    Log.d("TAG","${body}")
                    if (body?.status == "success"){
                        _kakaologinResult.value = "로그인 성공"
                        Log.d("TAG","success")
                    } else {
                        _kakaologinResult.value = "에러"
                        Log.d("TAG","${body}")
                    }
                } else {
                    _kakaologinResult.value= "서버 오류"
                    Log.d("TAG","server ${response} ,,, ${response}")
                }
            }

            override fun onFailure(call: Call<SnsLoginResponse>, t: Throwable) {
                Log.d("TAG","network")
                _kakaologinResult.value="네트워크 오류"
            }
        })
    }
}