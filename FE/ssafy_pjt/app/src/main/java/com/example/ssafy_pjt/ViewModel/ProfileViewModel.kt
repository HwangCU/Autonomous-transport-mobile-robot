package com.example.ssafy_pjt.ViewModel

import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.network.UserProfile
import com.example.ssafy_pjt.repository.ProfileRepository
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.updateProfileResponse
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class ProfileViewModel(
    val userViewModel: UserViewModel
) : ViewModel() {
    private val repository = ProfileRepository(RetrofitClient.instance)
    
    private val _userProfile = MutableStateFlow<UserProfile?>(null)
    val userProfile: StateFlow<UserProfile?> = _userProfile.asStateFlow()

    private val _isLoading = MutableStateFlow(true)
    val isLoading: StateFlow<Boolean> = _isLoading.asStateFlow()

    private val _error = MutableStateFlow<String?>(null)
    val error: StateFlow<String?> = _error.asStateFlow()

    private val _updateStatus = MutableStateFlow<UpdateStatus>(UpdateStatus.None)
    val updateStatus: StateFlow<UpdateStatus> = _updateStatus.asStateFlow()

    // 업데이트 상태를 나타내는 sealed class
    sealed class UpdateStatus {
        object None : UpdateStatus()
        object Success : UpdateStatus()
        data class Error(val message: String) : UpdateStatus()
    }

    // 유효성 검사 결과를 나타내는 sealed class
    sealed class ValidationResult {
        object Success : ValidationResult()
        data class Error(val message: String) : ValidationResult()
    }

    init {
        fetchUserProfile()
    }

    fun fetchUserProfile() {
        viewModelScope.launch {
            try {
                _isLoading.value = true
                _error.value = null
                val result = repository.getUserProfile(
                    accessToken = userViewModel.accessToken.value
                )
                _userProfile.value = result
            } catch (e: Exception) {
                _error.value = e.message ?: "프로필 정보를 불러오는데 실패했습니다."
                _userProfile.value = null  // 에러 시 프로필 정보 초기화
            } finally {
                _isLoading.value = false
            }
        }
    }

    // 프로필 데이터 유효성 검사 함수
    private fun validateProfileData(username: String, address: String): ValidationResult {
        if (username.isBlank()) {
            return ValidationResult.Error("이름을 입력해주세요.")
        }
        if (username.length > 10) {
            return ValidationResult.Error("이름은 10자 이내로 입력해주세요.")
        }
        // 이름 유효성 검사 - 한글, 영문만 허용
        val namePattern = "^[가-힣a-zA-Z\\s]+$"
        if (!username.matches(namePattern.toRegex())) {
            return ValidationResult.Error("이름에는 한글, 영문만 사용할 수 있습니다.")
        }
        if (address.isBlank()) {
            return ValidationResult.Error("주소를 입력해주세요.")
        }
        // 주소 유효성 검사 - 한글, 영문, 숫자, 쉼표, 괄호, 하이픈만 허용
        val addressPattern = "^[가-힣a-zA-Z0-9\\s,()\\-]+$"
        if (!address.matches(addressPattern.toRegex())) {
            return ValidationResult.Error("주소에는 한글, 영문, 숫자, 쉼표, 괄호, 하이픈만 사용할 수 있습니다.")
        }

        return ValidationResult.Success
    }

    // 프로필 업데이트 함수 수정
    fun updateProfile(updatedProfile: UserProfile): ValidationResult {
        // 먼저 유효성 검사 실행
        val validationResult = validateProfileData(updatedProfile.username, updatedProfile.address)
        if (validationResult is ValidationResult.Error) {
            return validationResult
        }

        // 유효성 검사 통과 시 기존 업데이트 로직 실행
        viewModelScope.launch {
            try {
                _updateStatus.value = UpdateStatus.None
                val result = repository.updateUserProfile(
                    accessToken = userViewModel.accessToken.value,
                    username = updatedProfile.username,
                    address = updatedProfile.address
                )
                
                result.onSuccess { response ->
                    if (response.status == "success") {
                        _userProfile.value = response.profile
                        _updateStatus.value = UpdateStatus.Success
                    } else {
                        _updateStatus.value = UpdateStatus.Error("회원 정보 수정에 실패했습니다.")
                    }
                }.onFailure { e ->
                    _updateStatus.value = UpdateStatus.Error(e.message ?: "회원 정보 수정에 실패했습니다.")
                }
            } catch (e: Exception) {
                _updateStatus.value = UpdateStatus.Error(e.message ?: "회원 정보 수정에 실패했습니다.")
            }
        }
        return ValidationResult.Success
    }

    // 업데이트 상태 초기화
    fun resetUpdateStatus() {
        _updateStatus.value = UpdateStatus.None
    }

    fun onLogoutClick() {
        viewModelScope.launch {
            // 로그아웃 처리
            userViewModel.setAccessToken("", "")
        }
    }

    private fun checkAuthStatus() {
        // 로그인 상태 확인
        // 권한 확인
    }

    fun updateProfileImage(uri: Uri) {
        userViewModel.updateTempProfileImage(uri)
    }

    fun confirmProfileImageEdit() {
        userViewModel.confirmProfileImage()
    }

    fun cancelProfileImageEdit() {
        userViewModel.cancelProfileImageEdit()
    }
}

