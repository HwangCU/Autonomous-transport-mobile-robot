package com.example.ssafy_pjt.component

import android.util.Log
import android.widget.EditText
import android.widget.Toast
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonColors
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ui.theme.my_blue
import com.example.ssafy_pjt.ui.theme.my_white
import com.example.ssafy_pjt.ui.theme.my_yellow
import com.example.ssafy_pjt.ui.theme.signupTitle
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.platform.LocalContext
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.ssafy_pjt.ViewModel.SignupViewModel


@Composable
fun SignupSceen(
    modifier: Modifier = Modifier,
    navController: NavController
) {
    val viewModel: SignupViewModel = viewModel()  // ViewModel 초기화

    var userName by remember { mutableStateOf("") }
    var userEmail by remember { mutableStateOf("") }
    var userPassword by remember { mutableStateOf("") }
    var userPasswordAgain by remember { mutableStateOf("") }
    var (checkUser,setCheckUser) = remember { mutableStateOf(false) }

    // LiveData를 State로 변환
    val signupResult by viewModel.signupResult.observeAsState()
    val context = LocalContext.current

    // signupResult 변화 감지
    LaunchedEffect(signupResult) {
        signupResult?.let { result->
            when(result){
                "User registered successfully" -> {
                    Toast.makeText(context,"회원가입이 완료되었습니다", Toast.LENGTH_SHORT).show()
                    setCheckUser(false)
                    navController.navigate("AccountLogin")
                }
                else -> {
                    Toast.makeText(context,"${result}", Toast.LENGTH_SHORT).show()
                }
            }
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
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = stringResource(R.string.signUp),
                style = signupTitle,
                color = my_blue
            )

            Card(
                modifier = modifier
                    .padding(top = 35.dp)
                    .height(444.dp)
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
                            .padding(16.dp),
                        verticalArrangement = Arrangement.Center,
                    ) {
                        // 사용자 이름 입력란
                        Text(
                            text = stringResource(R.string.userName),
                            color = my_white,
                        )
                        TextField(
                            value = userName,
                            onValueChange = {
                                userName = it
                                viewModel.setUsername(it)
                            },
                            modifier = Modifier.fillMaxWidth()
                        )
                        // 사용자 이메일 입력란
                        Text(
                            text = stringResource(R.string.userEmail),
                            color = my_white,
                            modifier = modifier.padding(top = 15.dp)
                        )
                        TextField(
                            value = userEmail,
                            onValueChange = {
                                userEmail = it
                                viewModel.setUserEmail(it)
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
                            onValueChange = {
                                userPassword = it
                                viewModel.setUserPassword(it)
                            },
                            modifier = Modifier.fillMaxWidth(),
                            visualTransformation = PasswordVisualTransformation()
                        )
                        // 비밀번호 확인 입력란
                        Text(
                            text = stringResource(R.string.userPasswordAgain),
                            color = my_white,
                            modifier = modifier.padding(top = 15.dp)
                        )
                        TextField(
                            value = userPasswordAgain,
                            onValueChange = { userPasswordAgain = it },
                            modifier = Modifier.fillMaxWidth(),
                            visualTransformation = PasswordVisualTransformation()
                        )
                        Box(
                            modifier = modifier
                                .fillMaxSize()
                                .padding(top = 12.dp),
                            contentAlignment = Alignment.Center,
                        ){
                            Button(
                                onClick = {
                                    if (userPassword == userPasswordAgain) {
                                            setCheckUser(true)
                                    }
                                },
                                colors = ButtonDefaults.buttonColors(my_yellow),
                            ) {
                                Text(
                                    text = stringResource(R.string.signUp),
                                    color = colorResource(R.color.black)
                                )
                            }
                            if (checkUser) {
                                AlertDialog(
                                    title = { Text(text = stringResource(R.string.infoCheck)) },
                                    text = { Text(text = stringResource(R.string.infoCheckDetail)) },
                                    confirmButton = {
                                        Button(
                                            colors = ButtonDefaults.buttonColors(my_blue),
                                            onClick = {
                                                viewModel.signup()
                                                Log.d("TAG","${signupResult}")

                                            }
                                        ) {
                                            Text(text = stringResource(R.string.signUp))
                                        }
                                    },
                                    dismissButton = {
                                        Button(
                                            colors = ButtonDefaults.buttonColors(my_blue),
                                            onClick = {
                                                setCheckUser(false)
                                            }
                                        ) {
                                            Text(text = stringResource(R.string.cancle))
                                        }
                                    },
                                    onDismissRequest = {
                                        setCheckUser(false)
                                    },
                                )
                            }
                        }
                    }
                }
            }
            Text(
                text = stringResource(R.string.back),
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