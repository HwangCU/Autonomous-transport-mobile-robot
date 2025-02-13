package com.example.ssafy_pjt.component

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Home
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
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
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_red
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.skt.tmap.TMapView

@Composable
fun FollowingScreen(
    modifier: Modifier,
    navController: NavController
) {
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    val (offMode,setOffMode) = remember { mutableStateOf(false) }

    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode, {})
        },
        containerColor = my_white,
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .padding(paddingValues)
                .fillMaxSize()
                .padding(start = 14.dp, end = 14.dp)
        ) {
            Row(
                modifier = modifier.fillMaxWidth()
                    .padding(top=7.dp),
                horizontalArrangement = Arrangement.SpaceBetween
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
                            text = stringResource(R.string.menu_following),
                            color = my_white,
                            style = loginTitle
                        )
                        Icon(
                            painter = painterResource(R.drawable.footprint),
                            contentDescription = "user",
                            Modifier
                                .size(45.dp)
                                .padding(start = 8.dp)
                        )
                    }
                }

                Button(
                    onClick = {
                        setOffMode(true)
                    },
                    modifier = Modifier.width(100.dp).height(60.dp),
                    colors = ButtonDefaults.buttonColors(my_red),
                    shape = RoundedCornerShape(16.dp)
                ) {
                    Column {
                        Icon(
                            painter = painterResource(R.drawable.power),
                            contentDescription = "power",
                            Modifier.size(24.dp),
                            tint = my_white
                        )
                        Text(
                            text = stringResource(R.string.end)
                        )
                    }
                }
            }
                Text(
                    text = stringResource(R.string.robotState2),
                    modifier = modifier.padding(top = 5.dp),
                    color = my_blue,
                    style = loginTitle
                )
                Box(
                    modifier = modifier.padding(top = 5.dp)
                ) {
                    Row(
                        modifier = modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Button(
                            onClick = {},
                            modifier = modifier
                                .height(140.dp)
                                .width(180.dp),
                            colors = ButtonDefaults.buttonColors(my_blue),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column {
                                Text(
                                    text = stringResource(R.string.bettery),
                                )
                            }
                        }

                        Button(
                            onClick = {},
                            modifier = modifier
                                .height(140.dp)
                                .width(180.dp),
                            colors = ButtonDefaults.buttonColors(my_blue),
                            shape = RoundedCornerShape(16.dp)
                        ) {
                            Column {
                                Text(
                                    text = stringResource(R.string.move),
                                )
                            }
                        }
                    }
                }
                Box {
                    Row(
                        modifier = modifier
                            .fillMaxWidth()
                            .padding(bottom = 10.dp, top = 20.dp),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Button(
                            modifier = modifier
                                .height(140.dp)
                                .width(180.dp),
                            colors = ButtonDefaults.buttonColors(my_blue),
                            shape = RoundedCornerShape(16.dp),
                            onClick = {}
                        ) {
                            Text(
                                text = stringResource(R.string.weight),
                                color = my_white,
                            )
                        }

                        Button(
                            modifier = modifier
                                .height(140.dp)
                                .width(180.dp),
                            colors = ButtonDefaults.buttonColors(my_yellow),
                            onClick = {
                                navController.navigate("SendHomeScreen")
                            },
                            shape = RoundedCornerShape(16.dp),
                        ) {
                            Column(
                            ) {
                                Text(
                                    text = stringResource(R.string.comeBackHome),
                                    color = colorResource(R.color.black)
                                )
                                Icon(
                                    Icons.Default.Home,
                                    contentDescription = "home",
                                    tint = colorResource(R.color.black)
                                )
                            }
                        }
                    }
                }
                Text(
                    text = stringResource(R.string.liveCamera),
                    color = my_blue,
                    style = loginTitle
                )
                Box(
                    modifier = modifier.fillMaxSize(1f)
                        .padding(bottom = 5.dp)
                        .clip(RoundedCornerShape(16.dp))
                ) {
                    Image(
                        painter = painterResource(R.drawable.ic_launcher_background),
                        contentDescription = "사진"
                    )
                }
            if (deliveryMode) {
                AlertDialog(
                    title = { Text(text = stringResource(R.string.deliveryMode)) },
                    text = { Text(text = stringResource(R.string.deliveryModeContent)) },
                    confirmButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                navController.navigate("DeliverySceen")
                            }
                        ) {
                            Text(text = stringResource(R.string.execution))
                        }
                    },
                    dismissButton = {
                        Button(
                            border = BorderStroke(1.dp, colorResource(R.color.black)),
                            colors = ButtonDefaults.buttonColors(my_white),
                            onClick = {
                                setDeliveryMode(false)
                            }
                        ) {
                            Text(
                                text= stringResource(R.string.cancle),
                                color = colorResource(R.color.black)
                            )
                        }
                    },
                    onDismissRequest = {
                        setDeliveryMode(false)
                    }
                )
            }

            if (offMode){
                AlertDialog(
                    title = { Text(text= stringResource(R.string.followingModeEnd)) },
                    text = { Text(text = stringResource(R.string.followingModeContentEnd)) },
                    confirmButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_red),
                            onClick = {
                            }
                        ) {
                            Text(text= stringResource(R.string.end))
                        }
                    },
                    dismissButton = {
                        Button(
                            border = BorderStroke(1.dp, colorResource(R.color.black)),
                            colors = ButtonDefaults.buttonColors(my_white),
                            onClick = {
                                setOffMode(false)
                            }
                        ) {
                            Text(
                                text= stringResource(R.string.cancle),
                                color = colorResource(R.color.black)
                            )
                        }
                    },
                    onDismissRequest = {
                        setOffMode(false)
                    }
                )
            }
        }
    }
}