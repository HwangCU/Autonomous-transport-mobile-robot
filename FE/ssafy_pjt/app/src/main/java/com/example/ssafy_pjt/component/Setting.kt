package com.example.ssafy_pjt.component

import android.widget.ToggleButton
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchColors
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.skt.tmap.TMapView

@Composable
fun Setting(
    modifier: Modifier,
    navController: NavController
) {
    val (deliveryMode,setDeliveryMode) = remember { mutableStateOf(false) }
    val (followingMode,setFollowingMode) = remember { mutableStateOf(false) }
    val check = remember { mutableStateOf(false) }
    Scaffold(
        bottomBar = {
            CustomAppBar(navController,setDeliveryMode,setFollowingMode)
        }
    ) { paddingValues ->
        Column(
            modifier = modifier.padding(paddingValues)
                .fillMaxSize()
                .padding(start = 14.dp, end = 14.dp,top=15.dp)
        ) {
            Button(
                onClick = {},
                modifier = Modifier.width(250.dp),
                colors = ButtonDefaults.buttonColors(my_blue),
                shape = RoundedCornerShape(16.dp)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Text(
                        text = stringResource(R.string.menu_setting),
                        color = my_white,
                        style = loginTitle
                    )
                    Icon(
                        Icons.Default.Settings,
                        contentDescription = "setting",
                        Modifier
                            .size(45.dp)
                            .padding(start = 8.dp)
                    )
                }
            }
            Spacer(modifier = modifier.size(40.dp))
            Box(
                modifier = modifier.background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .height(50.dp)
                    .clip(RoundedCornerShape(16.dp))
            ){
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween,
                    modifier = modifier.fillMaxSize()
                        .padding(start = 10.dp, end = 10.dp)
                ) {
                    Text(
                        text= stringResource(R.string.comeBackHomeToggle),
                        color = my_white
                    )
                    Switch(
                        checked = check.value,
                        onCheckedChange = {
                            check.value=!check.value
                        },
                        colors = SwitchDefaults.colors(
                            checkedThumbColor = my_white,  // 켜졌을 때 동그라미 색상
                            checkedTrackColor = my_yellow,  // 켜졌을 때 배경 색상
                            uncheckedThumbColor = my_white,  // 꺼졌을 때 동그라미 색상
                            uncheckedTrackColor = MaterialTheme.colorScheme.surfaceVariant
                        )
                    )
                }
            }
            Spacer(modifier = modifier.size(30.dp))
            Button(
                modifier = modifier.fillMaxWidth()
                    .height(100.dp)
                    .padding(bottom = 20.dp),
                onClick = {},
                colors = ButtonDefaults.buttonColors(my_yellow),
                shape = RoundedCornerShape(16.dp)
            ) {
                Text(
                    text = stringResource(R.string.userUpdate),
                    color = colorResource(R.color.black),
                )
            }
            Box(
                modifier = modifier.background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .fillMaxHeight(0.9f)
                    .clip(RoundedCornerShape(16.dp)),
            ) {
                Row(
                    modifier = modifier.fillMaxSize()
                        .padding(start=20.dp,end=20.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Text(
                        text = stringResource(R.string.robotStorage),
                        color = my_white,
                        )
                }

            }
        }
            if (deliveryMode){
                AlertDialog(
                    title = { Text(text= stringResource(R.string.deliveryMode)) },
                    text = { Text(text = stringResource(R.string.deliveryModeContent)) },
                    confirmButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                navController.navigate("DeliverySceen")
                            }
                        ) {
                            Text(text= stringResource(R.string.execution))
                        }
                    },
                    dismissButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                setDeliveryMode(false)
                            }
                        ) {
                            Text(text= stringResource(R.string.cancle))
                        }
                    },
                    onDismissRequest = {
                        setDeliveryMode(false)
                    }
                )
            }
            if (followingMode){
                AlertDialog(
                    title = { Text(text= stringResource(R.string.followingMode)) },
                    text = { Text(text = stringResource(R.string.followingModeContent)) },
                    confirmButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                navController.navigate("FollowingScreen")
                            }
                        ) {
                            Text(text= stringResource(R.string.execution))
                        }
                    },
                    dismissButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                setFollowingMode(false)
                            }
                        ) {
                            Text(text= stringResource(R.string.cancle))
                        }
                    },
                    onDismissRequest = {
                        setFollowingMode(false)
                    }
                )
            }
        }
    }