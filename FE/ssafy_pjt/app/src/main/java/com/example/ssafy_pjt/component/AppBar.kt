package com.example.ssafy_pjt.component

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.BottomAppBar
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import com.example.ssafy_pjt.R

@Composable
fun CustomAppBar(
    navController: NavController,
    setDeliveryMode: (Boolean) -> Unit,
    setFollowingMode: (Boolean) -> Unit,
){
    BottomAppBar(
        modifier = Modifier.fillMaxHeight(0.08f),
        containerColor = Color(0xFFFFE082)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth()
                .padding(horizontal = 16.dp),
            horizontalArrangement = Arrangement.SpaceBetween  ,
        ) {
            IconButton(
                onClick = { navController.navigate("HomeSceen")  }
            ) {
                Icon(
                    Icons.Default.Home,
                    contentDescription = "홈",
                    modifier = Modifier.fillMaxSize(1f)
                )
            }

            IconButton(
                onClick = { setFollowingMode(true)}
            ) {
                Column {
                    Icon(
                        painter = painterResource(R.drawable.footprint),
                        contentDescription = "팔로잉 모드",
                        modifier = Modifier.fillMaxSize(1f)
                    )
                }
            }

            IconButton(
                onClick = { setDeliveryMode(true) }
            ) {
                Icon(
                    painter = painterResource(R.drawable.delivery),
                    contentDescription = "배달 모드",
                    modifier = Modifier.fillMaxSize(1f)
                )
            }

            IconButton(
                onClick = {
                    navController.navigate("Setting")
                }
            ) {
                Icon(
                    Icons.Default.Settings,
                    contentDescription = "설정",
                    modifier = Modifier.fillMaxSize(1f)
                )
            }
        }
    }
}