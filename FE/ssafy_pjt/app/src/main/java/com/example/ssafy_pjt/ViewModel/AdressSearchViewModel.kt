package com.example.ssafy_pjt.ViewModel

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.BitmapFactory
import android.util.Log
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.mutableStateOf
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.network.RetrofitClient
import com.example.ssafy_pjt.network.adressRequest
import com.example.ssafy_pjt.network.adressResponse
import com.example.ssafy_pjt.network.roadRequest
import com.example.ssafy_pjt.network.roadResponse
import com.example.ssafy_pjt.network.updateAddressRequest
import com.example.ssafy_pjt.network.updateAddressResponse
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.Priority
import com.skt.tmap.TMapPoint
import com.skt.tmap.TMapView
import com.skt.tmap.overlay.TMapMarkerItem
import com.skt.tmap.overlay.TMapPolyLine
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.apache.commons.lang3.mutable.Mutable
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

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
    fun homeSweetHome(){
        val request = roadRequest(_address.value ?: "", access_token = userViewModel.accessToken.value)
        RetrofitClient.instance.homeSweetHome(request).enqueue(object : Callback<roadResponse> {
            override fun onResponse(call: Call<roadResponse>, response: Response<roadResponse>) {
                val body = response.body()
                if (body?.status == "success") {
                    val coordinates = body.path_list.map { coordinate ->
                        Pair(coordinate[0], coordinate[1])  // (경도, 위도)
                    }
                    userViewModel.setPath(coordinates) // StateFlow 업데이트
                    userViewModel.setTime(body.time)
                    userViewModel.setHome(body.home)
                    Log.d("TAG", "출발지: ${body.home}")
                    Log.d("TAG", "시간: ${body.time}초")
                    Log.d("TAG", "경로 저장 성공: ${userViewModel.path.value.size}개 좌표")
                }
            }

            override fun onFailure(call: Call<roadResponse>, t: Throwable) {
                TODO("Not yet implemented")
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
                    userViewModel.setHome(body.home)
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

    // 현재 위치 요청
    @SuppressLint("MissingPermission")
    fun requestCurrentLocation(
        fusedLocationClient: FusedLocationProviderClient,
        onLocationReceived: (Double, Double) -> Unit
    ) {
        fusedLocationClient.getCurrentLocation(
            Priority.PRIORITY_HIGH_ACCURACY, null
        ).addOnSuccessListener { location ->
            location?.let {
                onLocationReceived(it.latitude, it.longitude)
            }
        }.addOnFailureListener { e ->
            Log.e("DeliveryScreen", "위치 가져오기 실패", e)
        }
    }


    // 지도에 현재 위치 마커 추가
    fun updateMapLocation(mapView: TMapView?, lat: Double, lng: Double) {
        mapView?.let { view ->
            // 현재 줌 레벨 저장
            val currentZoom = view.getZoomLevel()

            val tMapPoint = TMapPoint(lat, lng)

            // 마커 생성 및 추가
            val marker = TMapMarkerItem().apply {
                id = "currentLocation"
                this.tMapPoint = tMapPoint
                visible = true
                name = "현재 위치"
                icon = BitmapFactory.decodeResource(view.context.resources, R.drawable.roboticon)
            }
            try {
                view.setCenterPoint(lat, lng)
                Log.d("TAG","${lng},${lat}")
                // 기존 마커 삭제 및 새 마커 추가
                view.removeAllTMapMarkerItem()
                view.addTMapMarkerItem(marker)

                // 마커 위치로 지도 중심 이동
                // 사용자가 수동으로 줌을 조절한 경우 그 레벨 유지
                if (currentZoom == 0) {
                    // 초기 상태일 때만 기본 줌 레벨 설정
                    try {
                        view.setZoomLevel(19)
                    } catch (e: Exception) {
                        Log.e("AddressSearchViewModel", "setZoomLevel failed: ${e.message}")
                    }
                } else {}
            } catch (e: Exception) {
                Log.e("DeliveryScreen", "마커 추가 실패", e)
            }
        }
    }


    // 두 지점 사이의 거리를 계산하는 함수 (Haversine 공식 사용)
    fun calculateDistance(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val earthRadius = 6371.0 // 지구 반지름 (km)
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = sin(dLat/2) * sin(dLat/2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon/2) * sin(dLon/2)
        val c = 2 * atan2(sqrt(a), sqrt(1-a))
        return earthRadius * c * 1000 // 미터로 변환
    }

    fun drawDeliveryPath(
        mapView: TMapView?,
        userViewModel: UserViewModel,
    ):Boolean {
        return mapView?.let { view ->
            try {
                val coordinates = userViewModel.path.value

                view.removeAllTMapMarkerItem() // 모든 마커 제거
                view.removeAllTMapPolyLine() // 모든 폴리라인 제거 - 중요!
                // 출발지와 도착지 좌표
                val startPoint = coordinates.first()
                val endPoint = coordinates.last()

                // 거리 계산
                val distance = calculateDistance(
                    startPoint.second, // 위도
                    startPoint.first,  // 경도
                    endPoint.second,   // 위도
                    endPoint.first     // 경도
                )

                // 거리에 따른 줌 레벨 동적 조절
                val zoomLevel = when {
                    distance < 500 -> 16   // 500m 미만: 매우 상세한 지도
                    distance < 1000 -> 15  // 1km 미만: 상세한 지도
                    distance < 3000 -> 14  // 3km 미만: 중간 정도 확대
                    distance < 5000 -> 13  // 5km 미만: 약간 축소
                    distance < 10000 -> 12 // 10km 미만: 더 축소
                    else -> 11             // 10km 이상: 최대한 축소
                }

                // 지도 중심점 계산
                val centerLat = (startPoint.second + endPoint.second) / 2
                val centerLon = (startPoint.first + endPoint.first) / 2

                // 마커 및 경로 그리기 (기존 코드와 동일)
                val tMapPoints = ArrayList<TMapPoint>()
                coordinates.forEach { (longitude, latitude) ->
                    tMapPoints.add(TMapPoint(latitude, longitude))
                }

                // 기존 마커 제거
                view.removeAllTMapMarkerItem()

                // 출발지 마커
                val startMarker = TMapMarkerItem().apply {
                    id = "start"
                    tMapPoint = TMapPoint(startPoint.second, startPoint.first)
                    visible = true
                    name = "출발지"
                    icon = BitmapFactory.decodeResource(view.context.resources, R.drawable.roboticon)
                }
                view.addTMapMarkerItem(startMarker)

                // 도착지 마커
                val endMarker = TMapMarkerItem().apply {
                    id = "end"
                    tMapPoint = TMapPoint(endPoint.second, endPoint.first)
                    visible = true
                    name = "목적지"
                }
                view.addTMapMarkerItem(endMarker)

                // 폴리라인 생성
                val polyline = TMapPolyLine("delivery-path", tMapPoints).apply {
                    lineColor = 0xFF007AFF.toInt()
                    lineWidth = 5f
                    outLineColor = 0xFF007AFF.toInt()
                    outLineWidth = 1f
                    lineAlpha = 255
                }
                view.addTMapPolyLine(polyline)

                // 계산된 중심점과 줌 레벨 적용
                view.setCenterPoint(centerLat, centerLon)
                view.setZoomLevel(zoomLevel)

                // 거리 로깅
                Log.d("DeliveryPath", "Distance: ${distance}m, Zoom Level: $zoomLevel")

                true
            } catch (e: Exception) {
                Log.e("TAG", "경로 그리기 실패", e)
                e.printStackTrace()
                false
            }
        } ?: false
    }

}
class addressSearchViewModelFactory(
    private  val userViewModel: UserViewModel
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        return AddressSearchViewModel(userViewModel) as T
    }
}