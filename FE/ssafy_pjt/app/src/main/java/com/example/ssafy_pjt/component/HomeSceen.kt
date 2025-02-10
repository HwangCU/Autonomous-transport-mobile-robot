package com.example.ssafy_pjt.component

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.BottomAppBar
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonColors
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
//import androidx.compose.ui.graphics.Color
import androidx.navigation.NavController
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.skt.tmap.TMapView

@Composable
fun HomeScreen(
    modifier: Modifier = Modifier,
    navController: NavController
) {
    val skKey = BuildConfig.SK_app_key
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    var (followingMode, setFollowingMode) = remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }

    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode, setFollowingMode)
        }
    ) { paddingValues ->
        Column(modifier = Modifier.padding(paddingValues)) {
            Box {
                Row {
                    Icon(
                        Icons.Default.AccountCircle,
                        contentDescription = "user"
                    )
                    Text(text= stringResource(R.string.hello))
                }
            }
            Text(text = stringResource(R.string.robotMode))
            Box{
                Row {
                    Box(
                        modifier = modifier.background(my_yellow)
                            .size(170.dp)
                    ){
                        Text(
                            text = stringResource(R.string.followingModeEx)
                        )
                        Icon(
                            painter = painterResource(R.drawable.footprint),
                            contentDescription = "팔로잉"
                        )
                    }

                    Box(
                        modifier = modifier.background(my_yellow)
                            .size(170.dp)
                    ){
                        Text(
                            text = stringResource(R.string.deliveryModeEx)
                        )
                        Icon(
                            painter = painterResource(R.drawable.delivery),
                            contentDescription = "배달"
                        )
                    }
                }
            }
            Text(
                text = stringResource(R.string.robotState),
                color = my_blue
            )
            Box{
                Row {
                    Box(
                        modifier = modifier.background(my_blue)
                            .size(170.dp)
                    ){
                        Text(
                            text = stringResource(R.string.followingModeEx),
                            color = my_white
                        )
                    }

                    Box(
                        modifier = modifier.background(my_blue)
                            .size(170.dp)
                    ){
                        Text(
                            text = stringResource(R.string.deliveryModeEx),
                            color = my_white
                        )
                    }
                }
            }
            Box(
                modifier = modifier.background(my_blue)
                    .size(170.dp)
            ) {
                Text(
                    text = stringResource(R.string.bettery),
                    color = my_white)
            }
            Text(
                text = stringResource(R.string.robotLotation),
                color = my_blue
            )
            AndroidView(
                factory = { ctx ->
                    TMapView(ctx).apply {
                        setSKTMapApiKey(skKey)
                        tMapView = this
                    }
                },
                modifier = Modifier.fillMaxSize()
            )
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
                            navController.navigate("")
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
