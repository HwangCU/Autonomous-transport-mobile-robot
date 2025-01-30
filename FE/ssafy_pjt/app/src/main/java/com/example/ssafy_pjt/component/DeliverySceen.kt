import android.Manifest
import android.content.pm.PackageManager
import android.os.Looper
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import android.content.Context
import androidx.compose.ui.platform.LocalContext
import androidx.navigation.NavController
import com.example.ssafy_pjt.component.CustomAppBar
import com.google.android.gms.location.LocationServices

@Composable
fun DeliverySceen(
    modifier: Modifier = Modifier,
    navController: NavController
) {
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    val context = LocalContext.current
    var isPermissionGranted by remember { mutableStateOf(false) }

    // 권한 요청 런처
    val locationPermissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        isPermissionGranted = isGranted
    }

    Scaffold(
        bottomBar = { CustomAppBar(navController, setDeliveryMode) }
    ) { paddingValues ->
        Column(
            modifier = modifier.padding(paddingValues).fillMaxSize()
        ) {
            Text(text = "배달 모드")

            // 권한 상태 확인 후 요청
            LaunchedEffect(Unit) {
                if (ContextCompat.checkSelfPermission(
                        context, Manifest.permission.ACCESS_FINE_LOCATION
                    ) != PackageManager.PERMISSION_GRANTED
                ) {
                    locationPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION)
                } else {
                    isPermissionGranted = true
                }
            }

            // 위치 정보 가져오기
            if (isPermissionGranted) {
                GetCurrentLocation(context)
            } else {
                Text(text = "위치 권한이 필요합니다.")
            }
        }
    }
}


@Composable
fun GetCurrentLocation(context: Context) {
    val fusedLocationClient = remember { LocationServices.getFusedLocationProviderClient(context) }
    var location by remember { mutableStateOf<Pair<Double, Double>?>(null) }

    LaunchedEffect(Unit) {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
            == PackageManager.PERMISSION_GRANTED
        ) {
            fusedLocationClient.lastLocation.addOnSuccessListener { loc ->
                loc?.let {
                    location = Pair(it.latitude, it.longitude)
                }
            }
        }
    }

    location?.let { (lat, lng) ->
        Text("현재 위치: 위도 $lat, 경도 $lng")
    } ?: Text("위치를 가져오는 중...")
}