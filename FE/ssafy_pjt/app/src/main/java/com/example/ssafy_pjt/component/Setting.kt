package com.example.ssafy_pjt.component

import android.widget.ToggleButton
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchColors
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import coil.compose.AsyncImage
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.JourneyHistoryViewModel
import com.example.ssafy_pjt.ViewModel.ProfileViewModel
import com.example.ssafy_pjt.ViewModel.RobotViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.nomalBold
import com.skt.tmap.TMapView
import java.text.SimpleDateFormat

@Composable
fun Setting(
    modifier: Modifier,
    navController: NavController,
    profileViewModel: ProfileViewModel,
    robotViewModel: RobotViewModel

) {
    val (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    val (followingMode, setFollowingMode) = remember { mutableStateOf(false) }
    val check = remember { mutableStateOf(false) }
    val userViewModel:UserViewModel = viewModel()
    val profileImage by userViewModel.profileImage.collectAsState()
    val userProfile by profileViewModel.userProfile.collectAsState()
    val viewModel: JourneyHistoryViewModel = viewModel()
    val histories = viewModel.allHistory.collectAsState()

    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode, setFollowingMode)
        }
    ) { paddingValues ->
        Column(
            modifier = modifier.padding(paddingValues)
                .fillMaxSize()
                .padding(start = 14.dp, end = 14.dp, top = 15.dp)
        ) {
            Row(
                modifier= modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(60.dp)
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
                IconButton(
                    modifier = Modifier.size(60.dp),
                    onClick = {
                        robotViewModel.modeChange(0)
                        robotViewModel.modeChange(1)
                        navController.navigate("GameController")
                    }
                ) {
                    Icon(
                        painter = painterResource(R.drawable.controller),
                        contentDescription = "controller Icon",
                        tint = my_blue,
                        modifier = Modifier.fillMaxSize()
                        )
                }
            }
            Spacer(modifier = modifier.size(10.dp))
            Box(
                modifier = modifier.fillMaxWidth(1f)
                    .height(80.dp)
                    .clip(RoundedCornerShape(16.dp))
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Box(
                        modifier = Modifier
                            .size(45.dp)
                            .clip(CircleShape)
                            .border(2.dp, my_blue, CircleShape),
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
                                painter = painterResource(R.drawable.usericon),
                                contentDescription = "user",
                                modifier = Modifier.fillMaxSize(),
                                tint = Color.Unspecified,

                                )
                        }
                    }
                    Spacer(modifier = Modifier.width(10.dp))
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(70.dp),
                    ) {
                        Column {
                            Text(
                                text = userProfile?.username ?: "홍길동",
                                style = nomalBold,
                                color = my_blue
                            )
                            Text(
                                text = stringResource(R.string.userUpdate),
                                style = nomalBold,
                                color = my_blue,
                                modifier = modifier.clickable {
                                    navController.navigate("ProfileScreen")
                                }
                            )
                        }
                    }
                }
            }
            Box(
                modifier = modifier.background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .height(50.dp)
                    .clip(RoundedCornerShape(16.dp))
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween,
                    modifier = modifier.fillMaxSize()
                        .padding(start = 10.dp, end = 10.dp)
                ) {
                    Text(
                        text = stringResource(R.string.comeBackHomeToggle),
                        color = my_white
                    )
                    Switch(
                        checked = check.value,
                        onCheckedChange = {
                            check.value = !check.value
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
            Spacer(modifier = modifier.size(10.dp))
            Box(
                modifier = modifier.background(my_blue, shape = RoundedCornerShape(16.dp))
                    .fillMaxWidth(1f)
                    .fillMaxHeight(0.9f)
                    .clip(RoundedCornerShape(16.dp)),
            ) {
                Row(
                    modifier = modifier.fillMaxSize()
                        .padding(start = 20.dp, end = 20.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Column {
                        Text(
                            text = stringResource(R.string.robotStorage),
                            color = my_white,
                        )
                        LazyColumn(
                            modifier = modifier
                                .fillMaxWidth()
                                .fillMaxHeight(.95f)
                        ) {
                            items(histories.value.size) { index ->
                                val history = histories.value[index]
                                Card(
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .padding(vertical = 8.dp),
                                    elevation = CardDefaults.cardElevation(
                                        defaultElevation = 6.dp
                                    ),
                                    colors = CardDefaults.cardColors(
                                        containerColor = Color.White
                                    ),
                                    shape = RoundedCornerShape(12.dp)
                                ) {
                                    Row {
                                        Column(
                                            modifier = Modifier
                                                .weight(1f)
                                                .padding(16.dp),
                                            verticalArrangement = Arrangement.spacedBy(8.dp)
                                        ) {
                                            Text(
                                                text = "모드: ${history.mode}",
                                                fontSize = 20.sp,
                                                style = nomalBold,
                                                color = Color(0xFF5E77E1)  // 파란 계열 색상
                                            )
                                            Text(
                                                text = "출발지: ${history.startPoint}",
                                                fontSize = 18.sp,
                                                color = Color.DarkGray
                                            )
                                            Text(
                                                text = "목적지: ${history.destination}",
                                                fontSize = 18.sp,
                                                color = Color.DarkGray
                                            )
                                            Text(
                                                text = "시간: ${
                                                    SimpleDateFormat("yyyy-MM-dd HH:mm").format(
                                                        history.timestamp
                                                    )
                                                }",
                                                fontSize = 14.sp,
                                                color = Color.Gray,
                                                modifier = Modifier.align(Alignment.End)
                                            )
                                        }
                                        IconButton(
                                            onClick = {
                                                viewModel.deleteJourney(history)
                                            }
                                        ) {
                                            Icon(
                                                Icons.Default.Close,
                                                contentDescription = "delete"
                                            )
                                        }
                                    }
                                }
                            }
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
                                colors = ButtonDefaults.buttonColors(my_blue),
                                onClick = {
                                    setDeliveryMode(false)
                                }
                            ) {
                                Text(text = stringResource(R.string.cancle))
                            }
                        },
                        onDismissRequest = {
                            setDeliveryMode(false)
                        }
                    )
                }
                if (followingMode) {
                    AlertDialog(
                        title = { Text(text = stringResource(R.string.followingMode)) },
                        text = { Text(text = stringResource(R.string.followingModeContent)) },
                        confirmButton = {
                            Button(
                                colors = ButtonDefaults.buttonColors(my_blue),
                                onClick = {
                                    navController.navigate("FollowingScreen")
                                }
                            ) {
                                Text(text = stringResource(R.string.execution))
                            }
                        },
                        dismissButton = {
                            Button(
                                colors = ButtonDefaults.buttonColors(my_blue),
                                onClick = {
                                    setFollowingMode(false)
                                }
                            ) {
                                Text(text = stringResource(R.string.cancle))
                            }
                        },
                        onDismissRequest = {
                            setFollowingMode(false)
                        }
                    )
                }
            }
            Text(text = stringResource(R.string.copyright_company)
                    + "\n"
                    + stringResource(R.string.copyright_rights),
                fontSize = 14.sp,
                color = Color.Gray,
                modifier = modifier.padding(start=5.dp)
            )
        }
    }
}