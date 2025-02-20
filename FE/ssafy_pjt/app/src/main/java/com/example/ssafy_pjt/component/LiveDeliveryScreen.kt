package com.example.ssafy_pjt.component

import android.Manifest
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
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Home
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.layout.ContentScale
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
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ViewModel.RobotViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_red
import com.example.ssafy_pjt.ui.theme.my_white
import com.google.android.gms.common.api.ResolvableApiException
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.LocationSettingsRequest
import com.google.android.gms.location.Priority
import com.skt.tmap.TMapView

@Composable
fun LiveDeliveryScreen(
    modifier: Modifier = Modifier,
    navController: NavController,
    userViewModel: UserViewModel,
    robotViewModel: RobotViewModel,
    addressSearchViewModel: AddressSearchViewModel
) {
    val (followingMode,setFollowingMode) = remember { mutableStateOf(false) }
    val (offMode,setOffMode) = remember { mutableStateOf(false) }
    val skKey = BuildConfig.SK_app_key
    var tMapView by remember { mutableStateOf<TMapView?>(null) }
    var mapInitialized by remember { mutableStateOf(false) }
    var isPermissionGranted by remember { mutableStateOf(false) }
    var locationCallback by remember { mutableStateOf<LocationCallback?>(null) }
    val (liveMode,setLiveMode) = remember { mutableStateOf(false) }
    val context = LocalContext.current
    val fusedLocationClient = remember { LocationServices.getFusedLocationProviderClient(context) }


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
    LaunchedEffect(userViewModel.path, mapInitialized) {
        if (mapInitialized) {
            userViewModel.path.collect {pathList ->
                addressSearchViewModel.drawDeliveryPath(tMapView,userViewModel)
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
        bottomBar = {
            CustomAppBar(navController, {},setFollowingMode)
        },
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .padding(paddingValues)
                .fillMaxSize()
                .padding()
        ) {
            Row(
                modifier = modifier.fillMaxWidth()
                    .padding(top=7.dp, start = 14.dp, end = 14.dp),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = {},
                    modifier = Modifier.width(270.dp),
                    colors = ButtonDefaults.buttonColors(my_blue),
                    shape = RoundedCornerShape(16.dp)
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically,
                        horizontalArrangement = Arrangement.Center
                    ) {
                        Text(
                            text = stringResource(R.string.deliveryMode2),
                            color = my_white,
                            style = loginTitle
                        )
                        Icon(
                            painter = painterResource(R.drawable.delivery),
                            contentDescription = "user",
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

            Box(
                modifier = modifier.fillMaxWidth()
                    .fillMaxHeight(.5f)
                    .padding(top = 5.dp)
                    .clip(RoundedCornerShape(16.dp))
                    .clickable {
                        setLiveMode(!liveMode)
                    },
            ) {
                val currentImage by robotViewModel.imageBitmap.collectAsState()
                if (liveMode && currentImage != null) {
                    Image(
                        bitmap = currentImage!!.asImageBitmap(),
                        contentDescription = "Camera Feed",
                        modifier = Modifier.fillMaxSize(),
                        contentScale = ContentScale.Fit
                    )
                } else {
                    Column(
                        modifier = Modifier.fillMaxSize(),
                        horizontalAlignment = Alignment.CenterHorizontally,
                        verticalArrangement = Arrangement.Center,

                        ) {
                        Icon(
                            painter = painterResource(R.drawable.liveicon),
                            contentDescription = "No Camera Feed",
                            tint = my_blue.copy(alpha = 0.5f),
                            modifier = Modifier.size(64.dp)
                        )
                        Spacer(modifier = Modifier.height(8.dp))
                        Text(
                            text = stringResource(R.string.touchScreen),
                            color = my_blue.copy(alpha = 0.7f)
                        )
                    }
                }
            }
            AndroidView(
                factory = { ctx ->
                    TMapView(ctx).apply {
                        setSKTMapApiKey(skKey)
                        tMapView = this
                        setOnMapReadyListener {
                            mapInitialized=true
                        }
                    }
                },
                modifier = modifier.fillMaxSize()
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

        if (offMode){
            AlertDialog(
                title = { Text(text= stringResource(R.string.followingModeEnd)) },
                text = { Text(text = stringResource(R.string.followingModeContentEnd)) },
                confirmButton = {
                    Button(
                        colors = ButtonDefaults.buttonColors(my_red),
                        onClick = {
                            robotViewModel.modeChange(0)
                            navController.navigate("HomeSceen")
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
                            robotViewModel.modeChange(0)
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
                Log.e("TAG", "위치 설정 요청 실패", e)
            }
        }
    }
}


// 상수
private const val REQUEST_CHECK_SETTINGS = 100