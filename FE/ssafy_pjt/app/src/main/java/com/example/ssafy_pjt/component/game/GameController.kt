package com.example.ssafy_pjt.component

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember

@Composable
fun GameController(
    modifier: Modifier = Modifier,
    navController: NavController,
    onDirectionChange: (Float, Float) -> Unit = { _, _ -> },
    onButtonClick: (String) -> Unit = { _ -> }
) {
    Box(
        modifier = modifier.fillMaxSize()
    ) {
        // 뒤로가기 버튼 추가
        Button(
            onClick = { navController.popBackStack() },
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(16.dp)
        ) {
            Text("뒤로가기")
        }

        // 기존 조이스틱
        Joystick(
            modifier = Modifier
                .size(200.dp)
                .align(Alignment.BottomStart)
                .padding(16.dp),
            onDirectionChange = onDirectionChange
        )

        // 기존 버튼들
        Column(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            GameButton(
                text = "B",
                onClick = { onButtonClick("B") }
            )
            GameButton(
                text = "A",
                onClick = { onButtonClick("A") }
            )
        }
    }
}