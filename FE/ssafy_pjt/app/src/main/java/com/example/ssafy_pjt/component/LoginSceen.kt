package com.example.ssafy_pjt.component

import android.app.Activity
import android.util.Log
import android.widget.Toast
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
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.Icon
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.sp
import com.example.ssafy_pjt.ViewModel.ApiState
import com.example.ssafy_pjt.ViewModel.GoogleLoginViewModel
import com.example.ssafy_pjt.ViewModel.KakaoAuthViewModel
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.network.GoogleApiContract
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_yellow

@Composable
fun LoginSceen(
    kakaoviewModel: KakaoAuthViewModel,
    googleViewModel: GoogleLoginViewModel,
    modifier: Modifier = Modifier,
    navController: NavController
) {
    val googleloginApiState by googleViewModel.loginApiState.collectAsState()
    val KakaoLoginResult by kakaoviewModel.kakaologinResult.observeAsState()
    val context = LocalContext.current
    val authResultLauncher = rememberLauncherForActivityResult(
        contract = GoogleApiContract()
    ) { task ->
        googleViewModel.handleGoogleSignInResult(task)
    }


    LaunchedEffect(KakaoLoginResult) {
        when(KakaoLoginResult){
            "로그인 성공" -> {
                Toast.makeText(context,"로그인에 성공하셨습니다", Toast.LENGTH_SHORT).show()
                navController.navigate("HomeSceen")
            }
            "로봇 등록이 필요합니다" -> {
                navController.navigate("RobotRegistration")
            }
            else -> {
            Toast.makeText(context,KakaoLoginResult, Toast.LENGTH_SHORT).show()
            }
        }
    }

    LaunchedEffect(googleloginApiState) {
        when (googleloginApiState) {
            "Google 로그인 성공" -> {
                // 로그인 성공 시 처리 (예: 메인 화면으로 이동)
                Toast.makeText(context,"로그인에 성공하셨습니다", Toast.LENGTH_SHORT).show()
                Log.d("TAG","success")
                navController.navigate("HomeSceen")
            }
            "로봇 등록이 필요합니다" ->{
                navController.navigate("RobotRegistration")
            }
            "Google 로그인 실패" -> {
                // 에러 처리
                Toast.makeText(context,"${(googleloginApiState as ApiState.Error).message}", Toast.LENGTH_SHORT).show()
                Log.e("TAG", "Google login error: ${(googleloginApiState as ApiState.Error).message}")
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
            verticalArrangement = Arrangement.Center,
            modifier = modifier
                .fillMaxSize()
                .padding(innerPadding)
        ) {
            Image(
                painter = painterResource(R.drawable.star_filled),
                modifier = modifier
                    .padding(top = 90.dp, start = 240.dp)
                    .size(75.dp),
                contentDescription = "login_title"
            )
            Text(
                text = stringResource(R.string.login_title),
                style = loginTitle,
                modifier = modifier.padding(top = 5.dp),
                color = colorResource(R.color.white)
            )
            Image(
                painter = painterResource(R.drawable.delivery_robot),
                contentDescription = "deliver_robot",
                modifier = modifier
                    .padding(top = 80.dp)
                    .size(250.dp)
            )
            Spacer(modifier = modifier.height(25.dp))
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding()
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
                    .padding( top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = {
                    kakaoviewModel.snsLogin()
                },
                colors = ButtonDefaults.buttonColors(
                    containerColor = colorResource(R.color.yellow),
                    contentColor = colorResource(R.color.black)
                )
            ) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(10.dp)
                ) {
                    Icon(
                        painterResource(R.drawable.kakao),
                        contentDescription = "kakao",
                        tint = Color.Unspecified
                    )
                    Text(text = stringResource(R.string.kaakaoLogin))
                }
            }
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding( top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = {
                    googleViewModel.signInWithRevoke(context, authResultLauncher)
                          },
                colors = ButtonDefaults.buttonColors(
                    containerColor = colorResource(R.color.white),
                    contentColor = colorResource(R.color.black)
                )
            ) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(10.dp)
                ) {
                    Icon(
                        painter = painterResource(R.drawable.google),
                        contentDescription = "googleIcon",
                        tint = Color.Unspecified
                    )
                    Text(text = stringResource(R.string.googleLogin))
                }
            }
            Button(
                modifier = modifier
                    .height(45.dp)
                    .padding( top = 10.dp)
                    .fillMaxWidth(0.8f),
                onClick = { OnSignupclicked() },
                colors = ButtonDefaults.buttonColors(
                    containerColor = my_yellow,
                    contentColor = colorResource(R.color.white)
                )
            ) {
                Text(text = stringResource(R.string.signUp))
            }
            Spacer(modifier = modifier.size(20.dp))
            Text(text = stringResource(R.string.copyright_company)
                    + "\n"
                    + stringResource(R.string.copyright_rights),
                fontSize = 14.sp,
                color = Color.Black,
                modifier = modifier.padding()
            )
        }
    }
}