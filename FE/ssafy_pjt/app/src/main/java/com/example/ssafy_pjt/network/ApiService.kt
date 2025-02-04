package com.example.ssafy_pjt.network

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

data class LoginRequest(
    val username: String,
    val password : String
)

data class LoginResponse(
    val status:String,
    val message: String,
    val access_token:String?,
    val refresh_token : String?
)

data class SignupRequest (
    val username:String,
    val email:String,
    val password: String
)

data class SignupResponse(
    val action: String,
    val message: String
)

data class KakaoLoginRequest(
    val username: String,
    val userNumber: Long
)

data class KakaoLoginResponse(
    val status:String,
    val message: String,
    val access_token:String?,
    val refresh_token : String?
)

interface ApiService{
    @POST("/api/login/")
    fun login(@Body request: LoginRequest): Call<LoginResponse>
    @POST("/api/register/")
    fun signup(@Body request: SignupRequest): Call<SignupResponse>
    @POST("/api/kakao_login")
    fun kakaoLogin(@Body request: KakaoLoginRequest): Call<KakaoLoginResponse>
}

