package com.example.ssafy_pjt.component

import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.BottomAppBar
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.navigation.NavController
import com.example.ssafy_pjt.R

@Composable
fun CustomAppBar(
    navController: NavController,
    setDeliveryMode: (Boolean) -> Unit
){
    BottomAppBar(
        containerColor = Color(0xFFFFE082) // 노란색 배경
    ) {
        IconButton(onClick = {
            navController.navigate("HomeSceen")
        }) {
            Icon(
                Icons.Default.Home,
                contentDescription = "홈"
            )
        }
        IconButton(onClick = { /* 팔로잉 모드 이동 */ }) {
            Icon(
                painter = painterResource(R.drawable.footprint),
                contentDescription = "팔로잉 모드"
            )
        }
        IconButton(onClick = {
            setDeliveryMode(true)
        }) {
            Icon(
                painter = painterResource(R.drawable.delivery),
                contentDescription = "배달 모드"
            )
        }
        IconButton(onClick = { /* 설정 이동 */ }) {
            Icon(
                Icons.Default.Settings,
                contentDescription = "설정"
            )
        }
    }
}
