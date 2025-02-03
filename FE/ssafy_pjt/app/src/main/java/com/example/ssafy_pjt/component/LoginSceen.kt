package com.example.ssafy_pjt.component

import android.util.Log
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import androidx.activity.compose.rememberLauncherForActivityResult
import com.example.ssafy_pjt.ViewModel.ApiState
import com.example.ssafy_pjt.network.GoogleApiContract
import com.example.ssafy_pjt.ViewModel.GoogleLoginViewModel
import com.example.ssafy_pjt.ViewModel.KakaoAuthViewModel
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_yellow

@Composable
fun LoginSceen(
    viewModel: KakaoAuthViewModel,
    googleViewModel: GoogleLoginViewModel,
    modifier: Modifier = Modifier,
    navController: NavController
) {
    val loginApiState by googleViewModel.loginApiState.collectAsState()
    val authResultLauncher = rememberLauncherForActivityResult(
        contract = GoogleApiContract()
    ) { task ->
        googleViewModel.handleGoogleSignInResult(task)
    }

    LaunchedEffect(loginApiState) {
        when (loginApiState) {
            is ApiState.Success -> {
                // 로그인 성공 시 처리 (예: 메인 화면으로 이동)
                navController.navigate("main")
            }
            is ApiState.Error -> {
                // 에러 처리
                Log.e("LoginScreen", "Google login error: ${(loginApiState as ApiState.Error).message}")
            }
            else -> {}
        }
    }

    fun OnSignupclicked() {
        navController.navigate("Signup")
    }

    Scaffold(
        modifier = modifier.fillMaxSize(),
        containerColor = my_blue
    ) { innerPadding ->
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = modifier
                .padding(start = 20.dp)
                .padding(innerPadding)
        ) {
            Image(
                painter = painterResource(R.drawable.star_filled),
                modifier = modifier
                    .padding(top = 119.dp, start = 255.dp)
                    .size(75.dp),
                contentDescription = "login_title"
            )
            Text(
                text = stringResource(R.string.login_title),
                style = loginTitle,
                modifier = modifier.padding(top = 5.dp, start = 20.dp),
                color = colorResource(R.color.white)
            )
            Image(
                painter = painterResource(R.drawable.delivery_robot),
                contentDescription = "deliver_robot",
                modifier = modifier
                    .padding(top = 100.dp, start = 35.dp)
                    .size(250.dp)
            )
            Spacer(modifier = modifier.height(45.dp))
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding(start = 25.dp)
                    .fillMaxWidth(0.8f),
                onClick = {
                    navController.navigate("accountLogin")
                },
                colors = ButtonDefaults.buttonColors(
                    containerColor = my_yellow,
                    contentColor = colorResource(R.color.white)
                )
            ) {
                Text(text = stringResource(R.string.login))
            }
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding(start = 25.dp, top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = {
                    viewModel.handleKakaoLogin()
                },
                colors = ButtonDefaults.buttonColors(
                    containerColor = colorResource(R.color.yellow),
                    contentColor = colorResource(R.color.black)
                )
            ) {
                Text(text = stringResource(R.string.kaakaoLogin))
            }
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding(start = 25.dp, top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = {
                    authResultLauncher.launch(1) 
                          },
                colors = ButtonDefaults.buttonColors(
                    containerColor = colorResource(R.color.white),
                    contentColor = colorResource(R.color.black)
                )
            ) {
                Text(text = stringResource(R.string.googleLogin))
            }
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding(start = 25.dp, top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = { OnSignupclicked() },
                colors = ButtonDefaults.buttonColors(
                    containerColor = my_yellow,
                    contentColor = colorResource(R.color.white)
                )
            ) {
                Text(text = stringResource(R.string.signUp))
            }
        }
    }
}