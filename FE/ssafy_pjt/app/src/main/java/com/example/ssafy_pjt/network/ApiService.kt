package com.example.ssafy_pjt.network

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

data class UserProfile(
    val username: String,
    val email: String,
    val address: String,
    val robot_id: String?,
    val profileImage: String? = null
)

data class LoginRequest(
    val email: String,
    val password : String
)

data class LoginResponse(
    val email: String,
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
    val email: String,
    val usernum: String,
    val userloginresource: String
)

data class SnsLoginResponse(
    val status:String,
    val message: String,
    val email: String,
    val access_token:String?,
    val refresh_token : String?,
    val require_robot: Boolean
)

data class adressRequest(
    val email: String,
    val address:String
)

data class adressResponse(
    val status: String,
    val road_addresses: List<String>
)

data class updateAddressRequest (
    val access_token:String,
    val email:String,
    val address: String
)

data class updateAddressResponse (
    val status: String,
    val message: String
)

data class robotRegistRequest (
    val email: String,
    val access_token: String?,
    val robot_id: String
)
data class robotRegistResponse(
    val status: String,
    val message: String
)

data class roadRequest(
    val destination: String,
    val access_token: String?
)

data class roadResponse(
    val status: String,
    val message: String,
    val path_list: List<List<Double>>,
    val time:Int
)

data class getProfileRequest(
    val access_token: String
)

data class getProfileResponse(
    val status: String,
    val profile: UserProfile
)

data class updateProfileRequest(
    val access_token: String,
    val username: String,
    val address: String
)

data class updateProfileResponse(
    val status: String,
    val profile: UserProfile
)

data class modeChangeReqeust(
    val robot_id:String,
    val access_token:String,
    val mode:Int
)
data class modeChangeResponse(
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
    @POST("/api/footpath/")
    fun RoadSearch(@Body request: roadRequest): Call<roadResponse>
    @POST("/api/home_sweet_home/")
    fun homeSweetHome(@Body request: roadRequest): Call<roadResponse>
    @POST("/api/get_profile/")
    fun getProfile(@Body request: getProfileRequest): Call<getProfileResponse>
    @POST("/api/edit_profile/")
    fun updateProfile(@Body request: updateProfileRequest): Call<updateProfileResponse>
    @POST("/api/robot_mode_change/")
    fun modeChange(@Body request: modeChangeReqeust): Call<modeChangeResponse>
}
