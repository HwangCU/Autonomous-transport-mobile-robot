package com.example.ssafy_pjt.component.game

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.PointerEventPass
import androidx.compose.ui.input.pointer.PointerEventType
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Composable
fun Joystick(
    modifier: Modifier = Modifier,
    onDirectionChange: (Float, Float) -> Unit = { _, _ -> }
) {
    var center: Offset by remember { mutableStateOf(Offset.Zero) }
    var handle: Offset by remember { mutableStateOf(Offset.Zero) }
    var baseRadius: Float by remember { mutableStateOf(0f) }

    Canvas(
        modifier = modifier
            .pointerInput(Unit) {
                awaitPointerEventScope {
                    while (true) {
                        // 터치 다운/무브 이벤트
                        val event = awaitPointerEvent(PointerEventPass.Initial)
                        when (event.type) {
                            PointerEventType.Press, PointerEventType.Move -> {
                                val position = event.changes.first().position

                                val dx = position.x - center.x
                                val dy = position.y - center.y
                                val distance = sqrt(dx * dx + dy * dy)

                                handle = if (distance > baseRadius) {
                                    val angle = atan2(dy, dx)
                                    Offset(
                                        center.x + cos(angle) * baseRadius,
                                        center.y + sin(angle) * baseRadius
                                    )
                                } else {
                                    position
                                }

                                val normalizedX = (handle.x - center.x) / baseRadius
                                val normalizedY = (handle.y - center.y) / baseRadius
                                onDirectionChange(normalizedX, normalizedY)
                            }
                            // 터치 업 이벤트
                            PointerEventType.Release -> {
                                // 중앙으로 복귀
                                handle = center
                                onDirectionChange(0f, 0f)
                            }
                        }
                    }
                }
            }
    ) {
        center = Offset(size.width / 2f, size.height / 2f)
        baseRadius = minOf(size.width, size.height) / 3f

        // 베이스 원 그리기
        drawCircle(
            color = Color(0xFF2A2A2A),  // 더 어두운 회색
            radius = baseRadius,
            center = center
        )

        // 핸들 그리기
        drawCircle(
            color = Color(0xFF3A3A3A),  // 어두운 회색
            radius = baseRadius / 3f,
            center = handle.takeIf { it != Offset.Zero } ?: center
        )
    }
}