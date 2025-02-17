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
import android.util.Log
import com.example.ssafy_pjt.component.DeviceItem
import com.example.ssafy_pjt.network.BluetoothService
import androidx.compose.foundation.lazy.items
import androidx.compose.runtime.*

@Composable
fun BluetoothScreen(
    onDeviceSelected: () -> Unit
) {
    val context = LocalContext.current
    val bluetoothService = BluetoothService.getInstance(context)
    // List에서 Set으로 타입 변경
    var pairedDevices by remember { mutableStateOf<Set<BluetoothDevice>>(emptySet()) }

    LaunchedEffect(Unit) {
        // getPairedDevices()는 이미 Set을 반환하므로 그대로 할당
        pairedDevices = bluetoothService.getPairedDevices()
        try {
            pairedDevices = bluetoothService.getPairedDevices()
            if (pairedDevices.isEmpty()) {
                Log.d("TAG","페어링된 블루투스 장치를 찾을 수 없습니다")
            }
        } catch (e: Exception) {
            Log.d("TAG","페어링된 장치 검색 중 오류 발생: ${e.message}")
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        Text(
            text = "페어링된 기기",
            style = MaterialTheme.typography.titleLarge,
            modifier = Modifier.padding(bottom = 16.dp)
        )

        LazyColumn {
            // Set을 List로 변환하여 items에 전달
            items(pairedDevices.toList()) { device ->
                DeviceItem(
                    device = device,
                    onClick = {
                        bluetoothService.connect(device)
                        onDeviceSelected()
                    }
                )
            }
        }
    }
}