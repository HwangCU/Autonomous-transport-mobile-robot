import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.component.game.GameButton
import com.example.ssafy_pjt.component.game.Joystick
import com.example.ssafy_pjt.network.BluetoothPermissionHandler
import com.example.ssafy_pjt.network.BluetoothService

@Composable
fun GameController(
    modifier: Modifier = Modifier,
    navController: NavController,
    onDirectionChange: (Float, Float) -> Unit = { _, _ -> },
    onButtonClick: (String) -> Unit = { _ -> }
) {
    val context = LocalContext.current
    val bluetoothService = remember { BluetoothService.getInstance(context) }
    val isConnected by bluetoothService.isConnected.collectAsState()

    LaunchedEffect(Unit) {
        if (!isConnected) {
            navController.navigate("bluetooth")
        }
    }
    if (!isConnected){
        return
    }

    // 화면을 나갈 때 블루투스 연결 해제
    DisposableEffect(Unit) {
        onDispose {
            bluetoothService.disconnect()
        }
    }

    BluetoothPermissionHandler {

        Box(
            modifier = modifier
                .fillMaxSize()
                .background(Color(0xFFDEDEDE))
                .padding(16.dp)
        ) {
            // 게임보이 본체
            Box(
                modifier = Modifier
                    .align(Alignment.Center)
                    .fillMaxSize()
                    .background(
                        color = Color(0xFFCFD2CF),
                        shape = MaterialTheme.shapes.medium
                    )
                    .padding(16.dp)
            ) {
                // 상단 화면 부분
                Box(
                    modifier = Modifier
                        .align(Alignment.TopCenter)
                        .size(width = 500.dp, height = 500.dp)
                        .background(
                            color = Color(0xFF9CA084),  // 게임보이 LCD 색상
                            shape = MaterialTheme.shapes.small
                        )
                        .padding(8.dp)
                ) {
                    // 화면
                }

                // 조이스틱 영역
                Box(
                    modifier = Modifier
                        .align(Alignment.CenterStart)
                        .padding(start = 16.dp, top = 400.dp)
                ) {
                    Box(
                        modifier = Modifier
                            .size(140.dp)
                            .background(
                                color = Color(0xFF4A4B4A),
                                shape = MaterialTheme.shapes.medium
                            )
                            .padding(8.dp)
                    ) {
                        Joystick(
                            modifier = Modifier.fillMaxSize(),
                            onDirectionChange = onDirectionChange,
                            bluetoothService = bluetoothService
                        )
                    }
                }

                Box(
                    modifier = Modifier
                        .align(Alignment.CenterEnd)
                        .padding(top = 400.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Row(
                        horizontalArrangement = Arrangement.spacedBy(25.dp),
                        modifier = Modifier.padding(8.dp),

                        ) {
                        GameButton(

                            text = "A",
                            onClick = { onButtonClick("A") },
                            modifier = Modifier
                                .size(60.dp)
                                .offset(y = (25).dp),
                            colors = ButtonDefaults.buttonColors(
                                containerColor = Color(0xFF9C0000)
                            )
                        )
                        GameButton(
                            text = "B",
                            onClick = { onButtonClick("B") },
                            modifier = Modifier.size(60.dp)
                                .offset(y = (-10).dp),
                            colors = ButtonDefaults.buttonColors(
                                containerColor = Color(0xFF9C0000)
                            )
                        )
                    }
                }

                // START/SELECT 버튼
                Row(
                    modifier = Modifier
                        .align(Alignment.BottomCenter)
                        .padding(bottom = 32.dp),
                    horizontalArrangement = Arrangement.spacedBy(16.dp)
                ) {
                    Column(
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Button(
                            onClick = {
                                navController.navigate("")
                            },
                            modifier = Modifier.size(width = 60.dp, height = 20.dp),
                            colors = ButtonDefaults.buttonColors(
                                containerColor = Color(0xFF4A4B4A)
                            ),
                            shape = MaterialTheme.shapes.small
                        ) {
                        }
                        Text(text = stringResource(R.string.gamestart))
                    }
                    Column(
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Button(
                            onClick = {
                                navController.popBackStack()
                            },
                            modifier = Modifier.size(width = 60.dp, height = 20.dp),
                            colors = ButtonDefaults.buttonColors(
                                containerColor = Color(0xFF4A4B4A)
                            ),
                            shape = MaterialTheme.shapes.small
                        ) {
                        }
                        Text(text = stringResource(R.string.gameback))
                    }
                }
            }
        }
    }
}