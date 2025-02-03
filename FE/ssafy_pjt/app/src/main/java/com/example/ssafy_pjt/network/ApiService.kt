package com.example.ssafy_pjt.network

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

data class LoginRequest(
    val email: String,
    val password : String
)

data class LoginResponse(
    val action: String,
    val message: String,
    val token : String?
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

interface ApiService{
    @POST("/accounts/login/")
    fun login(@Body request: LoginRequest): Call<LoginResponse>
    @POST("/accounts/signup/")
    fun signup(@Body request: SignupRequest): Call<SignupResponse>
}

