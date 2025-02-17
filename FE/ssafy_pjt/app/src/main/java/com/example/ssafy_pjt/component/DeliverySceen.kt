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
import androidx.compose.foundation.BorderStroke
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
import com.example.ssafy_pjt.ViewModel.JourneyHistoryViewModel
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
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

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
    val historyViewModel: JourneyHistoryViewModel = viewModel()

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
            if (viewModel.drawDeliveryPath(tMapView,userViewModel)){
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
                                viewModel.requestCurrentLocation(
                                    fusedLocationClient
                                ) { lat, lng ->
                                    userViewModel.setLocati8on(lat,lng)
                                    currentLocation = Pair(lat, lng)
                                    viewModel.updateMapLocation(tMapView, lat, lng)
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
                        border = BorderStroke(1.dp, colorResource(R.color.black)),
                        colors = ButtonDefaults.buttonColors(my_white),
                        onClick = {
                            setFollowingMode(false)
                        }
                    ) {
                        Text(
                            text= stringResource(R.string.cancle),
                            color = colorResource(R.color.black)
                        )
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
                                historyViewModel.addJourney(destination = viewModel.address.value ?: "", mode ="배달 모드", startPoint = "삼성전자 광주사업장" )
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


// 상수
private const val REQUEST_CHECK_SETTINGS = 100
