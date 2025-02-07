package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.compose.runtime.MutableState
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.robotRegistRequest
import com.example.ssafy_pjt.network.robotRegistResponse
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class RobotRegistrationViewModel(
    private val userViewModel: UserViewModel
):ViewModel() {
    private var _robotId = MutableLiveData("")
    val robotId: LiveData<String> get() = _robotId

    private var _result = MutableStateFlow("")
    var result = _result.asStateFlow()

    fun update(it:String){
        _robotId.value = it
    }

    fun robotRegist(){
        val robotId = _robotId.value ?:""
        Log.d("TAG",userViewModel.username.value)
        val request = robotRegistRequest(username = userViewModel.username.value, robot_id = robotId, access_token =userViewModel.accessToken.value )
        RetrofitClient.instance.robotRegistration(request).enqueue(object : Callback<robotRegistResponse> {
            override fun onResponse(
                call: Call<robotRegistResponse>,
                response: Response<robotRegistResponse>
            ) {
                val body = response.body()
                if (body?.status == "success"){
                    _result.value = "success"
                    Log.d("TAG","success")
                } else {
                    _result.value = "error"
                    Log.d("TAG","${body}")
                }
            }

            override fun onFailure(call: Call<robotRegistResponse>, t: Throwable) {
                _result.value = "network error"
                Log.d("TAG","network error ${t} ")
            }

        })
    }
}

class RobotRegistViewModelFactory(
    private val userViewModel: UserViewModel
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return RobotRegistrationViewModel(userViewModel) as T
    }
}