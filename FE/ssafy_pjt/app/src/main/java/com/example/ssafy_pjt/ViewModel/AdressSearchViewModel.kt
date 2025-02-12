package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.adressRequest
import com.example.ssafy_pjt.network.adressResponse
import com.example.ssafy_pjt.network.roadRequest
import com.example.ssafy_pjt.network.roadResponse
import com.example.ssafy_pjt.network.updateAddressRequest
import com.example.ssafy_pjt.network.updateAddressResponse
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.apache.commons.lang3.mutable.Mutable
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class AddressSearchViewModel(
    private  val userViewModel: UserViewModel
): ViewModel() {
    private var _prev = MutableStateFlow("")
    val prev = _prev.asStateFlow()

    private var _detail =  MutableLiveData("")
    val detail: LiveData<String> get() = _detail

    private var _address = MutableLiveData("")
    val address: LiveData<String> get() = _address

    private var _adressList = MutableStateFlow<List<String>>(emptyList())
    val addressList = _adressList.asStateFlow()

    private var _updateResult = MutableStateFlow("")
    val updateResult = _updateResult.asStateFlow()

    fun update(it: String){
        _address.value = it
    }
    fun updateDetail(it: String){
        _detail.value = it
    }

    fun updatePrev(it:String){
        _prev.value = it
    }

    fun getAdress(){
        val address = _address.value ?: ""
        val addressList = _adressList.value

        val request = adressRequest(address=address, email = userViewModel.email.value)
        RetrofitClient.instance.adress(request).enqueue(object : Callback<adressResponse> {
            override fun onResponse(call: Call<adressResponse>, response: Response<adressResponse>) {
                val body = response.body()
               if (body?.status == "success") {
                   Log.d("TAG","${body.road_addresses}")
                   _adressList.value=body.road_addresses
               } else {
                   Log.d("TAG","실패")
               }
            }

            override fun onFailure(call: Call<adressResponse>, t: Throwable) {
                Log.d("TAG","실패")
            }
        })
    }

    fun updateAddress(){
        val address = "${_address.value} ${_detail.value}"
        Log.d("TAG","${address}")
        val request = updateAddressRequest(email = userViewModel.email.value,address=address, access_token = userViewModel.accessToken.value )
        RetrofitClient.instance.updateAddress(request).enqueue(object : Callback<updateAddressResponse> {
            override fun onResponse(
                call: Call<updateAddressResponse>,
                response: Response<updateAddressResponse>
            ) {
                val body = response.body()
                if (body?.status == "success"){
                    Log.d("TAG","success")
                    _updateResult.value="success"
                } else {
                    Log.d("TAG","${response}")
                    _updateResult.value="fail"
                }
            }

            override fun onFailure(call: Call<updateAddressResponse>, t: Throwable) {
                Log.d("TAG","${t}")
                _updateResult.value="fail"
            }

        })
    }

    fun destination(){
        val request = roadRequest(_address.value ?: "", access_token = userViewModel.accessToken.value)
        Log.d("TAG","토큰 : ${userViewModel.accessToken.value} email : ${userViewModel.email.value}")
        RetrofitClient.instance.RoadSearch(request).enqueue(object : Callback<roadResponse>{
            override fun onResponse(call: Call<roadResponse>, response: Response<roadResponse>) {
                val body = response.body()
                if (body?.status == "success"){
                    val coordinates = body.path_list.map { coordinate ->
                        Pair(coordinate[0], coordinate[1])  // (경도, 위도)
                    }
                    userViewModel.setPath(coordinates) // StateFlow 업데이트
                    userViewModel.setTime(body.time)
                    Log.d("TAG", "시간: ${body.time}초")
                    Log.d("TAG", "경로 저장 성공: ${userViewModel.path.value.size}개 좌표")
                } else {
                    Log.d("TAG","bad request")
                }
            }
            override fun onFailure(call: Call<roadResponse>, t: Throwable) {
                Log.d("TAG","${t}")
            }
        })
    }
}

class addressSearchViewModelFactory(
    private  val userViewModel: UserViewModel
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return AddressSearchViewModel(userViewModel) as T
    }
}