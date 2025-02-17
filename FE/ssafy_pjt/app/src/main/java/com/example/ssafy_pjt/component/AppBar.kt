package com.example.ssafy_pjt.component

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.BottomAppBar
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.menu_style


@Composable
fun CustomAppBar(
    navController: NavController,
    setDeliveryMode: (Boolean) -> Unit,
    setFollowingMode: (Boolean) -> Unit,
){
    BottomAppBar(
        modifier = Modifier.fillMaxHeight(0.09f),
        containerColor = Color(0xFFFFE082)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 16.dp),
            horizontalArrangement = Arrangement.SpaceBetween  ,
        ) {
            IconButton(
                modifier = Modifier.fillMaxHeight(),
                onClick = { navController.navigate("HomeSceen")  }
            ) {
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Icon(
                        Icons.Default.Home,
                        contentDescription = "홈",
                        modifier = Modifier.fillMaxSize(0.6f)
                    )
                    Text(
                        text= stringResource(R.string.menu_main),
                        style = menu_style
                    )
                }
            }

            IconButton(
                onClick = { setFollowingMode(true)},
                modifier = Modifier.fillMaxHeight(),
            ) {
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.footprint),
                        contentDescription = "팔로잉 모드",
                        modifier = Modifier.fillMaxSize(.6f)
                    )
                    Text(
                        text = stringResource(R.string.menu_following),
                        style = menu_style
                    )
                }
            }

            IconButton(
                modifier = Modifier.fillMaxHeight(),
                onClick = { setDeliveryMode(true) },
            ) {
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.delivery),
                        contentDescription = "배달 모드",
                        modifier = Modifier.fillMaxSize(.7f)
                    )
                    Text(
                        text= stringResource(R.string.menu_delivery),
                        style = menu_style
                    )
                }
            }

            IconButton(
                modifier = Modifier.fillMaxHeight(),
                onClick = {
                    navController.navigate("Setting")
                }
            ) {
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Icon(
                        Icons.Default.Settings,
                        contentDescription = "설정",
                        modifier = Modifier.fillMaxSize(.6f)
                    )
                    Text(
                        text = stringResource(R.string.menu_setting),
                        style = menu_style
                    )
                }
            }
        }
    }
}