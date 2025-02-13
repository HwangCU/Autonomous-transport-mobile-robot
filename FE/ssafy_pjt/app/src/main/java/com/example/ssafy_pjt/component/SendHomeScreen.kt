package com.example.ssafy_pjt.component

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.content.IntentSender
import android.content.pm.PackageManager
import android.location.LocationManager
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.TextFieldDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.navigation.NavController
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_red
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.nomalBold
import com.google.android.gms.common.api.ResolvableApiException
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.LocationSettingsRequest
import com.google.android.gms.location.Priority
import com.skt.tmap.TMapPoint
import com.skt.tmap.TMapView
import com.skt.tmap.overlay.TMapMarkerItem

@Composable
fun SendHomeScreen(
    modifier: Modifier,
    navController: NavController
) {
    val skKey = BuildConfig.SK_app_key
    val context = LocalContext.current
    var isPermissionGranted by remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }
    var currentLocation by remember { mutableStateOf<Pair<Double, Double>?>(null) }
    var locationCallback by remember { mutableStateOf<LocationCallback?>(null) }
    val fusedLocationClient = remember { LocationServices.getFusedLocationProviderClient(context) }
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    var targetAddress by remember { mutableStateOf("") }
    var (sendHome, setSendHome) = remember { mutableStateOf(false) }
    var (offMode, setOffMode) = remember { mutableStateOf(false) }

    // 위치 권한 요청 런처
    val locationPermissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val fineLocationGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] == true
        val coarseLocationGranted = permissions[Manifest.permission.ACCESS_COARSE_LOCATION] == true
        isPermissionGranted = fineLocationGranted || coarseLocationGranted  // 둘 중 하나만 있어도 OK

        if (isPermissionGranted) {
            requestLocationSettings(context)
        }
    }

    // 권한 확인 및 요청
    LaunchedEffect(Unit) {
        val requiredPermissions = arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )

        val allPermissionsGranted = requiredPermissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }

        if (!allPermissionsGranted) {
            locationPermissionLauncher.launch(requiredPermissions)
        } else {
            isPermissionGranted = true
            requestLocationSettings(context)
        }
    }

    // Cleanup
    DisposableEffect(Unit) {
        onDispose {
            locationCallback?.let {
                fusedLocationClient.removeLocationUpdates(it)
            }
        }
    }

    Scaffold(
        bottomBar = { CustomAppBar(navController = navController, setFollowingMode = {}, setDeliveryMode = setDeliveryMode) }
    ) { paddingValues ->
        Box(
            modifier = modifier
                .padding(paddingValues)
                .fillMaxSize()
        ) {
            // 지도 표시
            AndroidView(
                factory = { ctx ->
                    TMapView(ctx).apply {
                        setSKTMapApiKey(skKey)
                        tMapView = this
                    }
                },
                modifier = Modifier.fillMaxSize()
            )

            // 상단 바
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Color.Transparent)
                        .padding(16.dp),
                    verticalArrangement = Arrangement.SpaceBetween
                ) {
                    Row(
                        modifier = modifier.fillMaxWidth()
                            .padding(top=7.dp),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Button(
                            onClick = {},
                            modifier = Modifier.width(250.dp),
                            colors = ButtonDefaults.buttonColors(my_blue),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Row(
                                verticalAlignment = Alignment.CenterVertically,
                                horizontalArrangement = Arrangement.Center
                            ) {
                                Text(
                                    text = stringResource(R.string.comeBackHome),
                                    color = my_white,
                                    style = nomalBold
                                )
                                Icon(
                                    Icons.Default.Home,
                                    contentDescription = "home",
                                    Modifier
                                        .size(45.dp)
                                        .padding(start = 8.dp)
                                )
                            }
                        }

                        Button(
                            onClick = {
                                setOffMode(true)
                            },
                            modifier = Modifier.width(100.dp).height(60.dp),
                            colors = ButtonDefaults.buttonColors(my_red),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column {
                                Icon(
                                    painter = painterResource(R.drawable.power),
                                    contentDescription = "power",
                                    Modifier.size(24.dp),
                                    tint = my_white
                                )
                                Text(
                                    text = stringResource(R.string.end)
                                )
                            }
                        }
                    }
                    Spacer(modifier = modifier.size(15.dp))
                    Button (
                        onClick = {
                            navController.popBackStack()
                        },
                        modifier = modifier.size(80.dp)
                            .padding(bottom = 5.dp),
                        shape = RoundedCornerShape(16.dp),
                        colors = ButtonDefaults.buttonColors(my_yellow)
                    ) {
                        Text(
                            text = stringResource(R.string.back),
                            color = colorResource(R.color.black)
                        )
                        }
                    }
                }
        }
    if (deliveryMode) {
        AlertDialog(
            title = { Text(text = stringResource(R.string.deliveryMode)) },
            text = { Text(text = stringResource(R.string.deliveryModeContent)) },
            confirmButton = {
                Button(
                    colors = ButtonDefaults.buttonColors(my_blue),
                    onClick = {
                        navController.navigate("DeliverySceen")
                    }
                ) {
                    Text(text = stringResource(R.string.execution))
                }
            },
            dismissButton = {
                Button(
                    border = BorderStroke(1.dp, colorResource(R.color.black)),
                    colors = ButtonDefaults.buttonColors(my_white),
                    onClick = {
                        setDeliveryMode(false)
                    }
                ) {
                    Text(
                        text= stringResource(R.string.cancle),
                        color = colorResource(R.color.black)
                    )
                }
            },
            onDismissRequest = {
                setDeliveryMode(false)
            }
        )
    }
        if (offMode){
            AlertDialog(
                title = { Text(text= stringResource(R.string.followingModeEnd)) },
                text = { Text(text = stringResource(R.string.followingModeContentEnd)) },
                confirmButton = {
                    Button(
                        colors = ButtonDefaults.buttonColors(my_red),
                        onClick = {
                        }
                    ) {
                        Text(text= stringResource(R.string.end))
                    }
                },
                dismissButton = {
                    Button(
                        border = BorderStroke(1.dp, colorResource(R.color.black)),
                        colors = ButtonDefaults.buttonColors(my_white),
                        onClick = {
                            setOffMode(false)
                        }
                    ) {
                        Text(
                            text= stringResource(R.string.cancle),
                            color = colorResource(R.color.black)
                        )
                    }
                },
                onDismissRequest = {
                    setOffMode(false)
                }
            )
        }
    }


/**
 * 위치 서비스 활성화 여부 확인
 */
private fun isLocationEnabled(context: Context): Boolean {
    val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
            locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
}

/**
 * 위치 설정 요청
 */
private fun requestLocationSettings(context: Context) {
    val locationRequest = LocationRequest.Builder(
        Priority.PRIORITY_HIGH_ACCURACY,
        5000L
    ).setMinUpdateIntervalMillis(2000)
        .build()

    val builder = LocationSettingsRequest.Builder()
        .addLocationRequest(locationRequest)

    val client = LocationServices.getSettingsClient(context)
    val task = client.checkLocationSettings(builder.build())

    task.addOnFailureListener { exception ->
        if (exception is ResolvableApiException) {
            try {
                exception.startResolutionForResult(context as Activity, REQUEST_CHECK_SETTINGS)
            } catch (e: IntentSender.SendIntentException) {
                Log.e("DeliveryScreen", "위치 설정 요청 실패", e)
            }
        }
    }
}

/**
 * 현재 위치 요청
 */
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

/**
 * 지도에 현재 위치 마커 추가
 */
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
        }

        try {
            view.setCenterPoint(lat, lng)
            Log.d("TAG","${lng},${lat}")
            // 기존 마커 삭제 및 새 마커 추가
            view.removeAllTMapMarkerItem()
            view.addTMapMarkerItem(marker)

            // 마커 위치로 지도 중심 이동
            // 사용자가 수동으로 줌을 조절한 경우 그 레벨 유지
            if (currentZoom == 0) {  // 초기 상태일 때만 기본 줌 레벨 설정
                view.setZoomLevel(17)
            } else {}
        } catch (e: Exception) {
            Log.e("DeliveryScreen", "마커 추가 실패", e)
        }
    }
}

// 상수
private const val REQUEST_CHECK_SETTINGS = 100