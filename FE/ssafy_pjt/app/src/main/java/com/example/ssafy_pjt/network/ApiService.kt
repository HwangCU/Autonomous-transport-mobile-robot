package com.example.ssafy_pjt.network

import com.example.ssafy_pjt.ViewModel.RobotRegistrationViewModel
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

data class LoginRequest(
    val username: String,
    val password : String
)

data class LoginResponse(
    val username: String,
    val status:String,
    val message: String,
    val access_token:String?,
    val refresh_token : String?,
    val require_robot: Boolean
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
    val token: String,
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

data class updateAddressRequest (
    val access_token:String,
    val username:String,
    val address: String
)

data class updateAddressResponse (
    val status: String,
    val message: String
)

data class robotRegistRequest (
    val username: String,
    val access_token: String?,
    val robot_id: String
)
data class robotRegistResponse(
    val status: String,
    val message: String
)

interface ApiService{
    @POST("/api/login/")
    fun login(@Body request: LoginRequest): Call<LoginResponse>
    @POST("/api/register/")
    fun signup(@Body request: SignupRequest): Call<SignupResponse>
    @POST("/api/sns_login/")
    fun snsLogin(@Body request: SnsLoginRequest): Call<SnsLoginResponse>
    @POST("/api/getaddress/")
    fun adress(@Body request: adressRequest): Call<adressResponse>
    @POST("/api/putaddress/")
    fun updateAddress(@Body request: updateAddressRequest): Call<updateAddressResponse>
    @POST("/api/robot_regist/")
    fun robotRegistration(@Body request: robotRegistRequest): Call<robotRegistResponse>
}

