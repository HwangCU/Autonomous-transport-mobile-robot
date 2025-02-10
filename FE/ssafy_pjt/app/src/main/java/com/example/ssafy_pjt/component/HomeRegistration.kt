package com.example.ssafy_pjt.component

import android.widget.EditText
import android.widget.Toast
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.focusModifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.signupTitle

@Composable
fun HomeRefistrationScreen(
    modifier: Modifier,
    navController: NavController,
    viewModel: AddressSearchViewModel
){
    val address by viewModel.address.observeAsState("")
    var detail by remember { mutableStateOf("") }
    val updateResult by viewModel.updateResult.collectAsState()
    val context = LocalContext.current

    LaunchedEffect(updateResult) {
        when ( updateResult){
            "success" -> {
                navController.navigate("HomeSceen")
            }
            "fail" -> {
                Toast.makeText(context,"실패하셨습니다", Toast.LENGTH_SHORT).show()
            }
            else -> {}
        }
    }
    Scaffold(
        modifier = Modifier.fillMaxSize(),
        contentColor = my_white
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            Text(
                text = stringResource(R.string.homeRegistration),
                color = my_blue,
                style = signupTitle,
                textAlign = TextAlign.Center
            )
            Card(
                modifier = Modifier
                    .padding(top = 35.dp)
                    .height(250.dp)
                    .width(346.dp),
                colors = CardDefaults.cardColors(my_blue)
            ) {
                Box(
                    modifier = modifier
                        .padding(start =15.dp)
                        .fillMaxSize(),
                    contentAlignment = Alignment.Center
                ) {
                    Column(
                        modifier = modifier
                            .padding(16.dp)
                            .fillMaxSize(),
                        verticalArrangement = Arrangement.Center
                    ) {
                        Text(
                            text = stringResource(R.string.roadName),
                            color = my_white,
                        )
                        TextField(
                            enabled = false,
                            modifier = Modifier.clickable {
                                viewModel.updatePrev("home")
                                navController.navigate("homeSearch")
                            },
                            value = address,
                            onValueChange = {}
                        )
                        Text(text = stringResource(R.string.detailName))
                        TextField(
                            value = detail,
                            onValueChange = { it ->
                                detail = it
                                viewModel.updateDetail(it)
                            }
                        )
                        Box(
                            modifier = modifier
                                .fillMaxSize()
                                .padding(end = 10.dp, top = 7.dp),
                            contentAlignment = Alignment.Center
                        ){
                            Button(
                                onClick = {
                                    viewModel.updateAddress()
                                },
                                colors = ButtonDefaults.buttonColors(my_yellow)
                            ) {
                                Text(text = stringResource(R.string.ok))
                            }
                        }
                    }
                }
            }
            Text(
                text = stringResource(R.string.back),
                color = my_blue,
                modifier = modifier
                    .padding(top=10.dp)
                    .clickable {
                        navController.navigate("login")
                    }
            )
        }
    }
}
