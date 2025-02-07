package com.example.ssafy_pjt.component

import android.accounts.Account
import android.widget.Toast
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Checkbox
import androidx.compose.material3.CheckboxDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AccountLoginViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.signupTitle

@Composable
fun AccountLoginSceen(
    modifier: Modifier = Modifier,
    navController: NavController,
    viewModel: AccountLoginViewModel
){
    var userName by remember { mutableStateOf(viewModel.userName.value ?: "") }
    var userPassword by remember { mutableStateOf(viewModel.userPassword.value ?: "") }
    var (autoLogin,setAutoLogin) = remember { mutableStateOf(false) }
    var (checkUser,setCheckUser) = remember { mutableStateOf(false) }
    fun autoUserLogin(){
        setAutoLogin(true)
    }
    val loginResult by viewModel.loginResult.observeAsState()
    val context = LocalContext.current
    LaunchedEffect(loginResult) {
        when(loginResult){
            "로그인 성공" -> {
                Toast.makeText(context,"로그인에 성공하셨습니다",Toast.LENGTH_SHORT).show()
                navController.navigate("HomeSceen")
            }
            "로봇이 필요합니다" -> {
                navController.navigate("RobotRegistration")
            }
            "로그인 실패" -> {
                Toast.makeText(context,"아이디 또는 비밀번호를 확인해 주세요",Toast.LENGTH_SHORT).show()
            }
            "서버 오류" -> {
                Toast.makeText(context,"서버 오류 잠시만 있다가 시도해 주세요",Toast.LENGTH_SHORT).show()
            }
            else -> {}
        }
    }

    Scaffold(
        modifier = modifier.fillMaxSize(),
        containerColor = my_white
    ) { paddingValues ->
        Column(
            modifier = modifier
                .padding(top = 137.dp)
                .fillMaxWidth()
                .padding(paddingValues),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            Text(
                text = stringResource(R.string.login),
                style = signupTitle,
                color = my_blue
            )

            Card(
                modifier = modifier
                    .padding(top = 35.dp)
                    .height(350.dp)
                    .width(346.dp),
                colors = CardDefaults.cardColors(my_blue),
            ) {
                Box(
                    modifier = modifier.fillMaxSize(),
                    contentAlignment = Alignment.Center
                ) {
                    Column(
                        modifier = Modifier
                            .fillMaxSize()
                            .padding(16.dp),  // 여백 추가
                        verticalArrangement = Arrangement.Center,  // 세로 중앙 정렬
                    ) {
                        // 사용자 이메일 입력란
                        Text(
                            text = stringResource(R.string.userEmail),
                            color = my_white,
                            modifier = modifier.padding(top = 30.dp)
                        )
                        TextField(
                            value = userName,
                            onValueChange = { newName->
                                userName = newName
                                viewModel.setUserName(newName)
                            },
                            modifier = Modifier.fillMaxWidth()
                        )
                        // 비밀번호 입력란
                        Text(
                            text = stringResource(R.string.userPassword),
                            color = my_white,
                            modifier = modifier.padding(top = 15.dp)
                        )
                        TextField(
                            value = userPassword,
                            onValueChange = { newPassword->
                                userPassword = newPassword
                                viewModel.setUserPassword(newPassword)
                            },
                            modifier = Modifier.fillMaxWidth(),
                            visualTransformation = PasswordVisualTransformation()
                        )
                        Row(
                            modifier = modifier
                                .fillMaxWidth()
                                .padding(top = 1.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Checkbox(
                                colors = CheckboxDefaults.colors(
                                    uncheckedColor = my_white,
                                    checkedColor = my_yellow,
                                    checkmarkColor = my_white
                                ),
                                checked = autoLogin,
                                onCheckedChange = { isChecked -> setAutoLogin(isChecked) }
                            )
                            Text(
                                text = stringResource(R.string.autoLogin),
                                color = my_white
                            )

                        }
                        Box(
                            modifier = modifier
                                .fillMaxSize()
                                .padding(top = 12.dp),
                            contentAlignment = Alignment.Center,
                        ){
                            Button(
                                onClick = {
                                    viewModel.login()
                                },
                                colors = ButtonDefaults.buttonColors(my_yellow),
                            ) {
                                Text(
                                    text= stringResource(R.string.login),
                                    color = colorResource(R.color.black)
                                )
                            }
                        }
                    }
                }
            }
            Text(
                text= stringResource(R.string.back),
                color = my_blue,
                modifier = modifier
                    .padding(top = 10.dp)
                    .clickable(onClick = {
                        navController.navigate("login")
                    })
            )
        }
    }
}