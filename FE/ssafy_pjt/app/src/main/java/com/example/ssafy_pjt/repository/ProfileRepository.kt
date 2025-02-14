package com.example.ssafy_pjt.repository

import com.example.ssafy_pjt.network.ApiService
import com.example.ssafy_pjt.network.UserProfile
import com.example.ssafy_pjt.network.getProfileRequest
import com.example.ssafy_pjt.network.updateProfileRequest
import com.example.ssafy_pjt.network.updateProfileResponse
import kotlinx.coroutines.withContext
import kotlinx.coroutines.Dispatchers

class ProfileRepository(private val apiService: ApiService) {
    suspend fun getUserProfile(accessToken: String): UserProfile {
        return withContext(Dispatchers.IO) {
            val response = apiService.getProfile(getProfileRequest(accessToken)).execute()
            if (response.isSuccessful) {
                response.body()?.profile ?: throw Exception("프로필 정보가 없습니다.")
            } else {
                throw Exception("프로필 정보를 불러오는데 실패했습니다.")
            }
        }
    }

    suspend fun updateUserProfile(
        accessToken: String,
        username: String,
        address: String
    ): Result<updateProfileResponse> {
        return withContext(Dispatchers.IO) {
            try {
                val response = apiService.updateProfile(
                    updateProfileRequest(
                        access_token = accessToken,
                        username = username,
                        address = address
                    )
                ).execute()
                
                if (response.isSuccessful) {
                    Result.success(response.body()!!)
                } else {
                    Result.failure(Exception("회원 정보 수정에 실패했습니다."))
                }
            } catch (e: Exception) {
                Result.failure(e)
            }
        }
    }
}