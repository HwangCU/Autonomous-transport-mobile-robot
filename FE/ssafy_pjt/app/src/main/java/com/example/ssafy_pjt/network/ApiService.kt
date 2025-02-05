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

data class SnsLoginRequest(
    val username: String,
    val userNumber: Long,
    val userLoginResource: String
)

data class SnsLoginResponse(
    val status:String,
    val message: String,
    val access_token:String?,
    val refresh_token : String?
)

data class adressRequest(
    val address:String
)

data class adressResponse(
    val status: String,
    val road_addresses: List<String>
)


interface ApiService{
    @POST("/api/login/")
    fun login(@Body request: LoginRequest): Call<LoginResponse>
    @POST("/api/register/")
    fun signup(@Body request: SignupRequest): Call<SignupResponse>
    @POST("/api/sns_login")
    fun snsLogin(@Body request: SnsLoginRequest): Call<SnsLoginResponse>
    @POST("/api/getadress")
    fun adress(@Body request: adressRequest): Call<adressResponse>
}

