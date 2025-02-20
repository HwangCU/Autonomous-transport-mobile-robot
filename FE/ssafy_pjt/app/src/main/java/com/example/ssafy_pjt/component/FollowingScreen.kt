package com.example.ssafy_pjt.component

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Home
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ViewModel.RobotViewModel
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_red
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.nomalBold
import com.skt.tmap.TMapView

@Composable
fun FollowingScreen(
    modifier: Modifier,
    navController: NavController,
    robotViewModel: RobotViewModel,
    addressSearchViewModel: AddressSearchViewModel
) {
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    val (offMode,setOffMode) = remember { mutableStateOf(false) }
    val (homeMode,setHomeMode) = remember { mutableStateOf(false) }
    val (liveMode,setLiveMode) = remember { mutableStateOf(false) }
    val weight by robotViewModel.weight.collectAsState()
    val bettery by robotViewModel.bettery.collectAsState()
    val ableToMove = (bettery/100.0) * 1

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
                                Spacer(modifier = modifier.size(10.dp))
                                Text(
                                    text = stringResource(R.string.realPersent,bettery),
                                    style = nomalBold,
                                    color = my_white
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
                                Spacer(modifier=modifier.size(10.dp))
                                Text(
                                    text = stringResource(R.string.realMove,ableToMove),
                                    style = nomalBold,
                                    color = my_white
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
                            Column {
                                Text(
                                    text = stringResource(R.string.weight),
                                    color = my_white,
                                )
                                Spacer(modifier = modifier.size(10.dp))
                                Text(
                                    text = stringResource(R.string.realWeight,weight),
                                    style = nomalBold,
                                    color = my_white
                                )
                            }
                        }

                        Button(
                            modifier = modifier
                                .height(140.dp)
                                .width(180.dp),
                            colors = ButtonDefaults.buttonColors(my_yellow),
                            onClick = {
                                setHomeMode(true)
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
                        .clickable {
                            setLiveMode(!liveMode)
                        }
                ) {
                    val currentImage by robotViewModel.imageBitmap.collectAsState()
                    if (liveMode && currentImage != null) {
                        Image(
                            bitmap = currentImage!!.asImageBitmap(),
                            contentDescription = "Camera Feed",
                            modifier = Modifier.fillMaxSize(),
                            contentScale = ContentScale.Fit
                        )
                    } else {
                        Column(
                            modifier = Modifier.fillMaxSize(),
                            horizontalAlignment = Alignment.CenterHorizontally,
                            verticalArrangement = Arrangement.Center,

                        ) {
                            Icon(
                                painter = painterResource(R.drawable.liveicon),
                                contentDescription = "No Camera Feed",
                                tint = my_blue.copy(alpha = 0.5f),
                                modifier = Modifier.size(64.dp)
                            )
                            Spacer(modifier = Modifier.height(8.dp))
                            Text(
                                text = stringResource(R.string.touchScreen),
                                color = my_blue.copy(alpha = 0.7f)
                            )
                        }
                    }
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
                                robotViewModel.modeChange(0)
                                navController.navigate("HomeSceen")
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
                                robotViewModel.modeChange(0)
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

            if (homeMode) {
                AlertDialog(
                    title = { Text(text = stringResource(R.string.comeBackHome)) },
                    text = { Text(text = stringResource(R.string.comeBackHome2)) },
                    confirmButton = {
                        Button(
                            colors = ButtonDefaults.buttonColors(my_blue),
                            onClick = {
                                robotViewModel.modeChange(0)
                                robotViewModel.modeChange(4)
                                addressSearchViewModel.homeSweetHome()
                                navController.navigate("SendHomeScreen")
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
                                setHomeMode(false)
                            }
                        ) {
                            Text(
                                text= stringResource(R.string.cancle),
                                color = colorResource(R.color.black)
                            )
                        }
                    },
                    onDismissRequest = {
                        setHomeMode(false)
                    }
                )
            }
        }
    }
}