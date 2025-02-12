import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.content.IntentSender
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.location.LocationManager
import android.os.Looper
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.component.CustomAppBar
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.nomalBold
import com.google.android.gms.common.api.ResolvableApiException
import com.google.android.gms.location.*
import com.skt.tmap.TMapPoint
import com.skt.tmap.TMapView
import com.skt.tmap.overlay.TMapMarkerItem
import com.skt.tmap.overlay.TMapPolyLine
import kotlinx.coroutines.awaitAll

@Composable
fun DeliverySceen(
    modifier: Modifier = Modifier,
    navController: NavController,
    viewModel: AddressSearchViewModel,
    userViewModel: UserViewModel
) {
    val time by userViewModel.time.collectAsState()
    val skKey = BuildConfig.SK_app_key
    val context = LocalContext.current
    var isPermissionGranted by remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }
    var currentLocation by remember { mutableStateOf<Pair<Double, Double>?>(null) }
    var locationCallback by remember { mutableStateOf<LocationCallback?>(null) }
    val fusedLocationClient = remember { LocationServices.getFusedLocationProviderClient(context) }
    var (followingMode, setFollowingMode) = remember { mutableStateOf(false) }
    var (start, setStart) = remember { mutableStateOf(false) }
    var targetAddress by remember { mutableStateOf("") }
    var mapInitialized by remember { mutableStateOf(false) }

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
    //path 상탱 변화 감지
    LaunchedEffect(userViewModel.path) {
        userViewModel.path.collect {pathList ->
            if (drawDeliveryPath(tMapView,userViewModel)){
                setStart(true)
            }
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
        bottomBar = { CustomAppBar(navController,{},setFollowingMode) }
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
                        // 지도 로드 완료 리스너 설정
                        setOnMapReadyListener {
                            mapInitialized = true
                        }
                    }
                },
                modifier = Modifier.fillMaxSize()
            )

            // 검색창과 버튼을 포함하는 상단 Box
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp)
                    .clip(RoundedCornerShape(16.dp))
            ) {
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .background(Color.Transparent)
                        .padding(16.dp)
                ) {
                    TextField(
                        enabled = false,
                        value = targetAddress,
                        onValueChange = { targetAddress = it },
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(bottom = 8.dp)
                            .clickable {
                                viewModel.updatePrev("search")
                                navController.navigate("homeSearch")
                            },
                        placeholder = { Text("주소를 입력하세요") },
                        colors = TextFieldDefaults.colors(
                            unfocusedContainerColor = MaterialTheme.colorScheme.surface,
                            focusedContainerColor = MaterialTheme.colorScheme.surface
                        )
                    )

                    Button(
                        onClick = {
                            if (!isPermissionGranted) {
                                locationPermissionLauncher.launch(arrayOf(
                                    Manifest.permission.ACCESS_FINE_LOCATION,
                                    Manifest.permission.ACCESS_COARSE_LOCATION
                                ))
                            } else if (isLocationEnabled(context)) {
                                requestCurrentLocation(
                                    fusedLocationClient
                                ) { lat, lng ->
                                    userViewModel.setLocati8on(lat,lng)
                                    currentLocation = Pair(lat, lng)
                                    updateMapLocation(tMapView, lat, lng)
                                }
                            }
                        },
                        modifier = Modifier.fillMaxWidth(),
                        colors = ButtonDefaults.buttonColors(my_blue)
                    ) {
                        Text(
                            if (isPermissionGranted) "현재 위치로 이동" else "위치 권한 요청",
                            color = my_white
                        )
                    }
                }
            }
        }
        if (followingMode){
            AlertDialog(
                title = { Text(text= stringResource(R.string.followingMode)) },
                text = { Text(text = stringResource(R.string.followingModeContent)) },
                confirmButton = {
                    Button(
                        colors = ButtonDefaults.buttonColors(my_blue),
                        onClick = {
                            navController.navigate("FollowingScreen")
                        }
                    ) {
                        Text(text= stringResource(R.string.execution))
                    }
                },
                dismissButton = {
                    Button(
                        colors = ButtonDefaults.buttonColors(my_blue),
                        onClick = {
                            setFollowingMode(false)
                        }
                    ) {
                        Text(text= stringResource(R.string.cancle))
                    }
                },
                onDismissRequest = {
                    setFollowingMode(false)
                }
            )
        }
        if (start){
            Column {
                Spacer(modifier = modifier.fillMaxHeight(0.85f))
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .fillMaxHeight()
                        .offset(y = (-paddingValues.calculateBottomPadding()))  // BottomBar 위로 올림
                        .clip(RoundedCornerShape(topStart = 16.dp, topEnd = 16.dp))
                        .background(my_blue)
                ){
                    Row(
                        modifier = modifier
                            .padding(start=40.dp, top = 30.dp,end=40.dp),
                        horizontalArrangement = Arrangement.spacedBy(100.dp)
                    ) {
                        Column(
                        ) {
                         Text(
                             text = stringResource(R.string.expectTime),
                             color = my_white,
                             style = modeType
                         )
                         Text(
                             text = stringResource(R.string.realTime,time/60),
                             color = my_white,
                             style = nomalBold
                         )
                        }
                        Button (
                            onClick = {
                            },
                            modifier = modifier.size(80.dp)
                                .padding(bottom = 5.dp),
                            shape = RoundedCornerShape(16.dp),
                            colors = ButtonDefaults.buttonColors(my_yellow)
                        ) {
                            Text(
                                text = stringResource(R.string.execution2),
                                color = colorResource(R.color.black)
                            )
                        }
                    }
                }
            }
        }
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
            icon = BitmapFactory.decodeResource(view.context.resources,R.drawable.roboticon)
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

fun drawDeliveryPath(
    mapView: TMapView?,
    userViewModel: UserViewModel,
):Boolean {
    return mapView?.let { view ->
        try {
            val coordinates = userViewModel.path.value
            Log.d("TAG", "list ${coordinates}")

            // 기존 마커와 폴리라인 모두 제거
            view.removeAllTMapMarkerItem()

            val tMapPoints = ArrayList<TMapPoint>()
            coordinates.forEach { (longitude, latitude) ->
                tMapPoints.add(TMapPoint(latitude, longitude))
            }

            // 출발지 마커 생성
            val startPoint = coordinates.first()
            val startMarker = TMapMarkerItem().apply {
                id = "start"
                tMapPoint = TMapPoint(startPoint.second, startPoint.first)
                visible = true
                name = "출발지"
                icon = BitmapFactory.decodeResource(view.context.resources,R.drawable.roboticon)
            }
            view.addTMapMarkerItem(startMarker)

            // 목적지 마커 생성
            val endPoint = coordinates.last()
            val endMarker = TMapMarkerItem().apply {
                id = "end"
                tMapPoint = TMapPoint(endPoint.second, endPoint.first)
                visible = true
                name = "목적지"
                // 아이콘 설정이 필요하다면 여기에 추가
            }
            view.addTMapMarkerItem(endMarker)

            // 폴리라인 생성 및 추가
            val polyline = TMapPolyLine("delivery-path", tMapPoints).apply {
                lineColor = 0xFF007AFF.toInt()
                lineWidth = 5f
                outLineColor = 0xFF007AFF.toInt()
                outLineWidth = 1f
                lineAlpha = 255
            }
            view.addTMapPolyLine(polyline)

            // 경로가 전체적으로 보이도록 중심점과 줌 레벨 조정
            view.zoomToSpan(maxOf(startPoint.second, endPoint.second) - minOf(startPoint.second, endPoint.second),
                maxOf(startPoint.first, endPoint.first) - minOf(startPoint.first, endPoint.first))
            view.setZoomLevel(16)
            true
        } catch (e: Exception) {
            Log.e("TAG", "경로 그리기 실패", e)
            e.printStackTrace()
            false
        }
    } ?: false
}

