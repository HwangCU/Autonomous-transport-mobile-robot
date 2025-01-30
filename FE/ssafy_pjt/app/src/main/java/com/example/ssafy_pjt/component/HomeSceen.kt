package com.example.ssafy_pjt.component

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
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
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
//import androidx.compose.ui.graphics.Color
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.my_blue

@Composable
fun HomeScreen(
    modifier: Modifier = Modifier,
    navController: NavController
) {
    var (deliveryMode, setDeliveryMode) = remember { mutableStateOf(false) }
    Scaffold(
        bottomBar = {
            CustomAppBar(navController, setDeliveryMode)
        }
    ) { paddingValues ->
        Column(modifier = Modifier.padding(paddingValues)) {
            Text(text="홈 화면")
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
    }
}
