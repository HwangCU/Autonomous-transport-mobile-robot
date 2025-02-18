import android.Manifest
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import android.bluetooth.BluetoothDevice
import android.os.Build
import android.util.Log
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.ui.Alignment
import com.example.ssafy_pjt.component.DeviceItem
import com.example.ssafy_pjt.network.BluetoothService
import androidx.navigation.NavController
import kotlinx.coroutines.delay

@Composable
fun BluetoothScreen(
    onDeviceSelected: () -> Unit,
    navController: NavController
) {
    val context = LocalContext.current
    val bluetoothService = remember { BluetoothService.getInstance(context) }
    val isConnected by bluetoothService.isConnected.collectAsState()
    val discoveredDevicesFlow by bluetoothService.discoveredDevices.collectAsState()

    var pairedDevices by remember { mutableStateOf<Set<BluetoothDevice>>(emptySet()) }
    var discoveredDevices by remember { mutableStateOf<Set<BluetoothDevice>>(emptySet()) }
    var errorMessage by remember { mutableStateOf<String?>(null) }
    var isConnecting by remember { mutableStateOf(false) }
    var startDiscovery by remember { mutableStateOf(false) }

    val launcher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.values.all { it }
        if (allGranted) {
            // 페어링된 기기 가져오기
            pairedDevices = bluetoothService.getPairedDevices()

            // 기기 검색 트리거
            startDiscovery = true
        } else {
            errorMessage = "필요한 권한이 허용되지 않았습니다"
        }
    }

    // 권한 요청 (위치 권한 포함)
    LaunchedEffect(Unit) {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
        launcher.launch(permissions)
    }

    // 검색 트리거
    LaunchedEffect(startDiscovery) {
        if (startDiscovery) {
            delay(1000) // 약간의 지연
            bluetoothService.startDiscovery()
            startDiscovery = false
        }
    }

    // discoveredDevices 업데이트
    LaunchedEffect(discoveredDevicesFlow) {
        discoveredDevices = discoveredDevicesFlow
        Log.d("BluetoothScreen", "발견된 기기 수: ${discoveredDevices.size}")
    }

    LaunchedEffect(Unit) {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
        launcher.launch(permissions)
    }

    LaunchedEffect(Unit) {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN
            )
        }
        launcher.launch(permissions)
    }

    LaunchedEffect(Unit) {
        try {
            val devices = bluetoothService.getPairedDevices()
            pairedDevices = devices
            if (devices.isEmpty()) {
                errorMessage = "페어링된 블루투스 장치를 찾을 수 없습니다"
                Log.d("TAG", "No paired devices found")
            } else {
                Log.d("TAG", "Found ${devices.size} paired devices")
            }
        } catch (e: Exception) {
            errorMessage = "페어링된 장치 검색 중 오류 발생: ${e.message}"
            Log.e("TAG", "Error getting paired devices", e)
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        // 페어링된 기기 섹션
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .height(250.dp)
                .padding(bottom = 16.dp),
            elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
        ) {
            Column(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(16.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(
                        text = "페어링된 기기",
                        style = MaterialTheme.typography.titleLarge,
                        color = MaterialTheme.colorScheme.primary
                    )
                    Text(
                        text = "(${pairedDevices.size}개)",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.secondary
                    )
                }

                if (errorMessage != null) {
                    Text(
                        text = errorMessage!!,
                        color = MaterialTheme.colorScheme.error,
                        modifier = Modifier.padding(vertical = 8.dp)
                    )
                }

                Divider(
                    modifier = Modifier.padding(vertical = 8.dp),
                    color = MaterialTheme.colorScheme.surfaceVariant
                )

                if (pairedDevices.isEmpty()) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = "페어링된 기기가 없습니다.",
                            style = MaterialTheme.typography.bodyMedium,
                            color = MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                } else {
                    LazyColumn {
                        items(pairedDevices.toList()) { device ->
                            DeviceItem(
                                device = device,
                                onClick = {
                                    isConnecting = true
                                    bluetoothService.connect(device)
                                }
                            )
                            Spacer(modifier = Modifier.height(8.dp))
                        }
                    }
                }
            }
        }

        // 검색된 기기 섹션
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .height(250.dp)
                .padding(top = 16.dp),
            elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
        ) {
            Column(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(16.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(
                        text = "검색된 기기",
                        style = MaterialTheme.typography.titleLarge,
                        color = MaterialTheme.colorScheme.primary
                    )
                    Text(
                        text = "(${discoveredDevices.size}개)",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.secondary
                    )
                }

                Divider(
                    modifier = Modifier.padding(vertical = 8.dp),
                    color = MaterialTheme.colorScheme.surfaceVariant
                )

                if (discoveredDevices.isEmpty()) {
                    Box(
                        modifier = Modifier.fillMaxSize(),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = "검색된 기기가 없습니다.",
                            style = MaterialTheme.typography.bodyMedium,
                            color = MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                } else {
                    LazyColumn {
                        items(discoveredDevices.toList()) { device ->
                            DeviceItem(
                                device = device,
                                onClick = {
                                    isConnecting = true
                                    bluetoothService.connect(device)
                                }
                            )
                            Spacer(modifier = Modifier.height(8.dp))
                        }
                    }
                }
            }
        }
    }

    if (isConnecting) {
        AlertDialog(
            onDismissRequest = { isConnecting = false },
            title = { Text("연결 중...") },
            text = { CircularProgressIndicator() },
            confirmButton = { }
        )
    }

    LaunchedEffect(isConnected) {
        if (isConnected) {
            isConnecting = false
            navController.navigate("GameController")
        }
    }

    // 연결 상태 모니터링
    LaunchedEffect(isConnected) {
        if (isConnected) {
            navController.navigate("GameController") {
                popUpTo("bluetooth") { inclusive = true }
            }
        }
    }
}