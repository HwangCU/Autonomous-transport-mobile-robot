package com.example.ssafy_pjt.ViewModel

import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.adressRequest
import com.example.ssafy_pjt.network.adressResponse
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class AdressSearchViewModel: ViewModel() {
    private var _adress = MutableLiveData("")
    val address: LiveData<String> get() = _adress

    private var _adressList = MutableStateFlow<List<String>>(emptyList())
    val adressList = _adressList.asStateFlow()

    fun getAdress(){
        val adress = _adress.value ?: ""

        val request = adressRequest(adress)
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
}
