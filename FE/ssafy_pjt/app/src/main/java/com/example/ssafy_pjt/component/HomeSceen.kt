package com.example.ssafy_pjt.component

import androidx.compose.foundation.background
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
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
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
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.RectangleShape
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
//import androidx.compose.ui.graphics.Color
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import coil.compose.AsyncImage
import com.example.ssafy_pjt.BuildConfig
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.skt.tmap.TMapView

@Composable
fun HomeScreen(
    modifier: Modifier = Modifier,
    navController: NavController,
    userViewModel: UserViewModel = viewModel()
) {
    val skKey = BuildConfig.SK_app_key
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    var (followingMode, setFollowingMode) = remember { mutableStateOf(false) }
    var tMapView by remember { mutableStateOf<TMapView?>(null) }

    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode, setFollowingMode)
        },
        containerColor = my_white,
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .padding(paddingValues)
                .fillMaxSize()
            .padding(start = 14.dp, end = 14.dp)
        ) {
            Button(
                onClick = { navController.navigate("ProfileScreen") },
                modifier = modifier.fillMaxWidth(1f),
                colors = ButtonDefaults.buttonColors(my_white)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    val profileImage by userViewModel.profileImage.collectAsState()
                    
                    Box(
                        modifier = Modifier
                            .size(45.dp)
                            .clip(CircleShape)
                            .background(Color(0xff5e77e1)),
                        contentAlignment = Alignment.Center
                    ) {
                        if (profileImage != null) {
                            AsyncImage(
                                model = profileImage,
                                contentDescription = "Profile Image",
                                contentScale = ContentScale.Crop,
                                modifier = Modifier.fillMaxSize()
                            )
                        } else {
                            Icon(
                                Icons.Default.AccountCircle,
                                contentDescription = "user",
                                modifier = Modifier.fillMaxSize(),
                                tint = Color.White
                            )
                        }
                    }
                    
                    Spacer(modifier = Modifier.width(10.dp))
                    
                    Text(
                        text = stringResource(R.string.hello),
                        color = my_blue,
                        style = loginTitle
                    )
                }
            }
            Text(
                text = stringResource(R.string.robotMode),
                color = my_blue,
                style = loginTitle
            )
            Box(
            ) {
                Row(
                    modifier = modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Button (
                        onClick = {},
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_yellow),
                        shape = RoundedCornerShape(16.dp)
                    ){
                       Column {
                           Text(
                               text = stringResource(R.string.followingModeEx),
                               style = modeType,
                               color = colorResource(R.color.black)
                           )
                           Icon(
                               painter = painterResource(R.drawable.footprint),
                               contentDescription = "팔로잉",
                               tint = colorResource(R.color.black)
                           )
                       }
                    }

                    Button (
                        onClick = {},
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_yellow),
                        shape = RoundedCornerShape(16.dp)
                    ){
                        Column {
                            Text(
                                text = stringResource(R.string.deliveryModeEx),
                                color = colorResource(R.color.black),
                                style = modeType
                            )
                            Icon(
                                painter = painterResource(R.drawable.delivery),
                                contentDescription = "배달",
                                tint = colorResource(R.color.black)
                            )
                        }
                    }
                }
            }
            Text(
                text = stringResource(R.string.robotState),
                color = my_blue,
                style = loginTitle
            )
            Box{
                Row(
                    modifier = modifier.fillMaxWidth()
                        .padding(bottom =10.dp),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Button (
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_blue),
                        shape = RoundedCornerShape(16.dp),
                        onClick = {}
                    ){
                        Text(
                            text = stringResource(R.string.weight),
                            color = my_white,
                        )
                    }

                    Button (
                        modifier = modifier
                            .height(140.dp)
                            .width(180.dp),
                        colors = ButtonDefaults.buttonColors(my_blue),
                        onClick = {},
                        shape = RoundedCornerShape(16.dp)
                    ){
                        Text(
                            text = stringResource(R.string.move),
                            color = my_white,
                        )
                    }
                }
            }
            Box(
                modifier = modifier.background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .height(50.dp)
                    .clip(RoundedCornerShape(16.dp)),
            ) {
                Row(
                    modifier = modifier.fillMaxSize(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Text(
                        text = stringResource(R.string.bettery),
                        color = my_white)
                     }

                }
            Text(
                text = stringResource(R.string.robotLotation),
                color = my_blue,
                style = loginTitle
            )
            Box(
                modifier = modifier.fillMaxSize(1f)
                    .padding(bottom = 5.dp)
                    .clip(RoundedCornerShape(16.dp))
            ){
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
