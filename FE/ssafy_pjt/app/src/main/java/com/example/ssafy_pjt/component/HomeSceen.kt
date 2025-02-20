package com.example.ssafy_pjt.component

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.content.IntentSender
import android.content.pm.PackageManager
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.BottomAppBar
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonColors
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.State
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.RectangleShape
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
//import androidx.compose.ui.graphics.Color
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import coil.compose.AsyncImage
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ViewModel.RobotViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ViewModel.socketViewModel
import com.example.ssafy_pjt.network.SocketService
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.nomalBold
import com.google.android.gms.common.api.ResolvableApiException
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.LocationSettingsRequest
import com.google.android.gms.location.Priority
import com.skt.tmap.TMapView
import kotlinx.coroutines.delay

@Composable
fun HomeScreen(
    modifier: Modifier = Modifier,
    navController: NavController,
    userViewModel: UserViewModel,
    socketViewModel:socketViewModel,
    adressViewModel: AddressSearchViewModel,
    robotViewModel:RobotViewModel
) {
    val skKey = BuildConfig.SK_app_key
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    var (followingMode, setFollowingMode) = remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }
    var isPermissionGranted by remember { mutableStateOf(false) }
    val context = LocalContext.current
    val socketService = remember { SocketService.getInstance() }
    val isConnected by socketService.isConnected.collectAsState()
    val serverResponses by socketViewModel.serverResponses.collectAsState()
    val location by robotViewModel.location.collectAsState()
    val (lats, lngs) = location
    var mapReady by remember { mutableStateOf(false) }
    val weight by robotViewModel.weight.collectAsState()
    val battery by robotViewModel.bettery.collectAsState()
    val ableToMove = (battery/100.0) * 1

    LaunchedEffect(location, mapReady) {
        if (mapReady && tMapView != null) {
            Log.d("TAG", "지도 위치 업데이트 시도: lat=$lats, lng=$lngs")
            adressViewModel.updateMapLocation(tMapView, lat = lats, lng = lngs)
        }
    }

    LaunchedEffect(Unit) {
        // 지도 초기화를 위한 충분한 시간 대기
        delay(5000) // 5초 대기
        mapReady = true
        Log.d("TAG", "지도 초기화 타이머 완료")
    }

    LaunchedEffect(serverResponses) {
        if (serverResponses.isNotEmpty()) {
            val lastResponse = serverResponses.last()
            Log.d("socket", "최신 응답: $lastResponse")
        }
    }

    LaunchedEffect(Unit) {
        socketViewModel.connectToSocket()
    }

    LaunchedEffect(isConnected) {
        if (!isConnected) {
            Log.d("socket","연결 끊김")
        } else {
            Log.d("socket","연결 됨")
        }
    }

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

    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode, setFollowingMode)
        },
        containerColor = my_white,
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .padding(paddingValues)
                .fillMaxSize()
                .padding(start = 14.dp, end = 14.dp)
        ) {
            Button(
                onClick = { navController.navigate("ProfileScreen") },
                modifier = modifier.fillMaxWidth(1f),
                colors = ButtonDefaults.buttonColors(my_white)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    val profileImage by userViewModel.profileImage.collectAsState()
                    
                    Box(
                        modifier = Modifier
                            .size(45.dp)
                            .clip(CircleShape)
                            .border(2.dp, my_blue, CircleShape),
                        contentAlignment = Alignment.Center
                    ) {
                        if (profileImage != null) {
                            AsyncImage(
                                model = profileImage,
                                contentDescription = "Profile Image",
                                contentScale = ContentScale.Crop,
                                modifier = Modifier.fillMaxSize()
                            )
                        } else {
                            Icon(
                                painter = painterResource(R.drawable.usericon),
                                contentDescription = "user",
                                modifier = Modifier.fillMaxSize(),
                                tint = Color.Unspecified,

                            )
                        }
                    }
                    
                    Spacer(modifier = Modifier.width(10.dp))
                    
                    Text(
                        text = stringResource(R.string.hello),
                        color = my_blue,
                        style = loginTitle
                    )
                }
            }
            Text(
                text = stringResource(R.string.robotMode),
                color = my_blue,
                style = loginTitle
            )
            Box(
            ) {
                Row(
                    modifier = modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Button (
                        onClick = {
                            setFollowingMode(true)
                        },
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_yellow),
                        shape = RoundedCornerShape(16.dp)
                    ){
                       Column {
                           Text(
                               text = stringResource(R.string.followingModeEx),
                               style = modeType,
                               color = colorResource(R.color.black)
                           )
                           Spacer(modifier = Modifier.height(5.dp))
                           Icon(
                               painter = painterResource(R.drawable.footprint),
                               contentDescription = "팔로잉",
                               tint = colorResource(R.color.black)
                           )
                       }
                    }

                    Button (
                        onClick = {
                            setDeliveryMode(true)
                        },
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_yellow),
                        shape = RoundedCornerShape(16.dp)
                    ){
                        Column {
                            Text(
                                text = stringResource(R.string.deliveryModeEx),
                                color = colorResource(R.color.black),
                                style = modeType
                            )
                            Spacer(modifier = Modifier.height(5.dp))
                            Icon(
                                painter = painterResource(R.drawable.delivery),
                                contentDescription = "배달",
                                tint = colorResource(R.color.black)
                            )
                        }
                    }
                }
            }
            Text(
                text = stringResource(R.string.robotState),
                color = my_blue,
                style = loginTitle
            )
            Box{
                Row(
                    modifier = modifier
                        .fillMaxWidth()
                        .padding(bottom = 10.dp),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Button (
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_blue),
                        shape = RoundedCornerShape(16.dp),
                        onClick = {}
                    ){
                        Column {
                            Text(
                                text = stringResource(R.string.weight),
                                color = my_white,
                            )
                            Spacer(modifier = modifier.size(10.dp))
                            Text(
                                text = stringResource(R.string.realWeight,weight),
                                style = nomalBold,
                                color = my_white
                            )
                        }
                    }

                    Button (
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_blue),
                        onClick = {},
                        shape = RoundedCornerShape(16.dp)
                    ){
                        Column {
                            Text(
                                text = stringResource(R.string.move),
                                color = my_white,
                            )
                            Spacer(modifier = modifier.size(10.dp))
                            Text(
                                text = stringResource(R.string.realMove,ableToMove),
                                style = nomalBold,
                                color = my_white
                            )
                        }
                    }
                }
            }
            Box(
                modifier = modifier
                    .background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .height(50.dp)
                    .clip(RoundedCornerShape(16.dp)),
            ) {
                Row(
                    modifier = Modifier.fillMaxSize(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    val battery by robotViewModel.bettery.collectAsState()
                    val batteryColor = when {
                        battery > 70 -> Color.Green
                        battery > 30 -> Color.Yellow
                        else -> Color.Red
                    }

                    Text(
                        text = stringResource(R.string.bettery),
                        color = my_white
                    )

                    Spacer(modifier = Modifier.size(15.dp))

                    Box(
                        modifier = Modifier
                            .width(120.dp)
                            .height(20.dp)
                            .background(Color.DarkGray.copy(alpha = 0.5f), RoundedCornerShape(10.dp))
                            .clip(RoundedCornerShape(10.dp))
                    ) {
                        // 배터리 레벨 표시
                        Box(
                            modifier = Modifier
                                .fillMaxHeight()
                                .width(120.dp * battery / 100)
                                .background(batteryColor)
                        )
                    }

                    Spacer(modifier = Modifier.width(15.dp))

                    Text(
                        text = stringResource(R.string.realPersent, battery),
                        color = my_white
                    )
                }
            }
            Text(
                text = stringResource(R.string.robotLotation),
                color = my_blue,
                style = loginTitle
            )
            Box(
                modifier = modifier
                    .fillMaxSize(1f)
                    .padding(bottom = 5.dp)
                    .clip(RoundedCornerShape(16.dp))
            ){
                AndroidView(
                    factory = { ctx ->
                        TMapView(ctx).apply {
                            setSKTMapApiKey(skKey)
                            tMapView = this
                        }
                    },
                    modifier = Modifier.fillMaxSize()
                )
            }
        }
    if (deliveryMode){
            AlertDialog(
            title = { Text(text= stringResource(R.string.deliveryMode)) },
            text = { Text(text = stringResource(R.string.deliveryModeContent)) },
            confirmButton = {
                Button(
                    colors = ButtonDefaults.buttonColors(my_blue),
                    onClick = {
                        robotViewModel.modeChange(0)
                        robotViewModel.modeChange(3)
                        navController.navigate("DeliverySceen")
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
        if (followingMode){
            AlertDialog(
                title = { Text(text= stringResource(R.string.followingMode)) },
                text = { Text(text = stringResource(R.string.followingModeContent)) },
                confirmButton = {
                    Button(
                        colors = ButtonDefaults.buttonColors(my_blue),
                        onClick = {
                            robotViewModel.modeChange(0)
                            robotViewModel.modeChange(2)
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
    }
}

// 위치 설정
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