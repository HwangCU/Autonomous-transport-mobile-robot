import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.navigation.NavController
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.component.CustomAppBar
import com.google.android.gms.location.*
import com.skt.tmap.TMapPoint
import com.skt.tmap.TMapView
import com.skt.tmap.overlay.TMapMarkerItem

@Composable
fun DeliverySceen(
    modifier: Modifier = Modifier,
    navController: NavController
) {
    val skKey = BuildConfig.SK_app_key
    val context = LocalContext.current
    var isPermissionGranted by remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }
    var currentLocation by remember { mutableStateOf<Pair<Double, Double>?>(null) }

    val fusedLocationClient = remember { LocationServices.getFusedLocationProviderClient(context) }

    // 위치 권한 요청 런처
    val locationPermissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        isPermissionGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] == true
                || permissions[Manifest.permission.ACCESS_COARSE_LOCATION] == true
    }

    // 권한 확인 및 요청
    LaunchedEffect(Unit) {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) {
            locationPermissionLauncher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            )
        } else {
            isPermissionGranted = true
        }
    }

    Scaffold(
        bottomBar = { CustomAppBar(navController, {}) }
    ) { paddingValues ->
        Column(
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
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
            )

            // 현재 위치 버튼
            Button(
                onClick = {
                    if (isPermissionGranted) {
                        requestCurrentLocation(fusedLocationClient) { lat, lng ->
                            currentLocation = Pair(lat, lng)
                            updateMapLocation(tMapView, lat, lng)
                        }
                    }
                },
                modifier = Modifier
                    .padding(16.dp)
                    .fillMaxWidth()
            ) {
                Text("현재 위치로 이동")
            }

            // 현재 위치 텍스트 표시
            if (isPermissionGranted) {
                currentLocation?.let { (lat, lng) ->
                    Text(
                        "현재 위치: 위도 $lat, 경도 $lng",
                        modifier = Modifier.padding(16.dp)
                    )
                }
            } else {
                Text(
                    "위치 권한이 필요합니다.",
                    modifier = Modifier.padding(16.dp)
                )
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
        val tMapPoint = TMapPoint(lat, lng)
        view.setCenterPoint(lng, lat)  // <-- 경도(lng)와 위도(lat) 순서 확인!
        view.setZoomLevel(17)  // 확대 레벨 설정

        val marker = TMapMarkerItem().apply {
            id = "currentLocation"
            this.tMapPoint = tMapPoint
            visible = true
            name = "현재 위치"
        }

        try {
            view.removeAllTMapMarkerItem()  // 기존 마커 삭제
            view.addTMapMarkerItem(marker)  // 새 마커 추가
        } catch (e: Exception) {
            Log.e("DeliveryScreen", "마커 추가 실패", e)
        }
    }
}
