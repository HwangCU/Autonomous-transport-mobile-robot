package com.example.ssafy_pjt.ViewModel

import android.graphics.Bitmap
import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.modeChangeReqeust
import com.example.ssafy_pjt.network.modeChangeResponse
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class RobotViewModel(
    private val userViewModel:UserViewModel
):ViewModel() {
    private val _imageBitmap = MutableStateFlow<Bitmap?>(null)
    val imageBitmap: StateFlow<Bitmap?> = _imageBitmap.asStateFlow()

    private val _robot_id = MutableStateFlow("")
    val robot_id:StateFlow<String> = _robot_id.asStateFlow()

    private val _bettery = MutableStateFlow(80)
    val bettery:StateFlow<Int> = _bettery.asStateFlow()

    private  val _weight = MutableStateFlow(0)
    val weight:StateFlow<Int> = _weight.asStateFlow()

    private val _location = MutableStateFlow(Pair(0.0,0.0))
    val location:StateFlow<Pair<Double,Double>> = _location.asStateFlow()

    fun saveBettery(it:Int){
        _bettery.value=it
    }

    fun saveWeight(it:Int){
        _weight.value=it
    }

    fun saveLocation(x:Double, y:Double) {
        Log.d("TAG","${x} ${y}")
        _location.value=Pair(x,y)
    }

    fun saveRobotId(it : String){
        _robot_id.value = it
    }

    fun saveImage(bitmap: Bitmap?) {
        _imageBitmap.value = bitmap
    }

    fun modeChange(mode:Int){
        val robotId = robot_id
        val access_token = userViewModel.accessToken.value
        Log.d("TAG","로봇아이디 : ${robotId}, 토큰 : ${access_token}, mode : ${mode}")
        val request = modeChangeReqeust(robot_id = robotId.value, access_token = access_token, mode = mode)
        RetrofitClient.instance.modeChange(request).enqueue(object : Callback<modeChangeResponse> {
            override fun onResponse(
                call: Call<modeChangeResponse>,
                response: Response<modeChangeResponse>
            ) {
                Log.d("TAG","${response.body()}")
                if (response.body()?.status == "success") {
                    Log.d("TAG","success")
                } else {
                    Log.d("TAG","fail")
                }
            }

            override fun onFailure(call: Call<modeChangeResponse>, t: Throwable) {
                Log.d("TAG","error ${t}")
            }

        })
    }
}

class RobotViewModelFactory(
    private val userViewModel: UserViewModel
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        if (modelClass.isAssignableFrom(RobotViewModel::class.java)) {
            @Suppress("UNCHECKED_CAST")
            return RobotViewModel(userViewModel) as T
        }
        throw IllegalArgumentException("Unknown ViewModel class")
    }
}