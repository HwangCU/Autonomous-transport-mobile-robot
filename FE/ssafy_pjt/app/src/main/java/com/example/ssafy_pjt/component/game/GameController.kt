import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.ssafy_pjt.component.game.GameButton
import com.example.ssafy_pjt.component.game.Joystick

@Composable
fun GameController(
    modifier: Modifier = Modifier,
    navController: NavController,
    onDirectionChange: (Float, Float) -> Unit = { _, _ -> },
    onButtonClick: (String) -> Unit = { _ -> }
) {
    Box(
        modifier = modifier
            .fillMaxSize()
            .background(Color(0xFFDEDEDE))  // 게임보이 밝은 회색
    ) {
        // 게임보이 본체
        Box(
            modifier = Modifier
                .align(Alignment.Center)
                .size(width = 380.dp, height = 600.dp)
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
                    .size(width = 300.dp, height = 280.dp)
                    .background(
                        color = Color(0xFF9CA084),  // 게임보이 LCD 색상
                        shape = MaterialTheme.shapes.small
                    )
                    .padding(8.dp)
            ) {
                // Nintendo 로고
                Text(
                    text = "Nintendo®",
                    modifier = Modifier
                        .align(Alignment.TopCenter)
                        .padding(4.dp),
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF4A4B4A)
                )

                // GAME BOY 로고
                Text(
                    text = "GAME BOY",
                    modifier = Modifier
                        .align(Alignment.TopCenter)
                        .padding(top = 24.dp),
                    style = MaterialTheme.typography.headlineMedium,
                    color = Color(0xFF4A4B4A)
                )
            }

            // 조이스틱 영역
            Box(
                modifier = Modifier
                    .align(Alignment.CenterStart)
                    .padding(start = 16.dp, top = 32.dp)
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
                        onDirectionChange = onDirectionChange
                    )
                }
            }

            // 버튼 영역 - 오른쪽에 기울어진 형태로
            Box(
                modifier = Modifier
                    .align(Alignment.CenterEnd)
                    .padding(end = 16.dp, top = 32.dp)
            ) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(12.dp),
                    modifier = Modifier.padding(8.dp)
                ) {
                    GameButton(
                        text = "B",
                        onClick = { onButtonClick("B") },
                        modifier = Modifier.size(50.dp),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = Color(0xFF9C0000)
                        )
                    )
                    GameButton(
                        text = "A",
                        onClick = { onButtonClick("A") },
                        modifier = Modifier.size(50.dp),
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
                Button(
                    onClick = { },
                    modifier = Modifier.size(width = 60.dp, height = 20.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF4A4B4A)
                    ),
                    shape = MaterialTheme.shapes.small
                ) {
                    Text("SELECT", fontSize = 8.sp)
                }
                Button(
                    onClick = { },
                    modifier = Modifier.size(width = 60.dp, height = 20.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF4A4B4A)
                    ),
                    shape = MaterialTheme.shapes.small
                ) {
                    Text("START", fontSize = 8.sp)
                }
            }
        }
    }
}