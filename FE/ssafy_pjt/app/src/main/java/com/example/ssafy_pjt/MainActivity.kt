package com.example.ssafy_pjt

import DeliverySceen
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.example.ssafy_pjt.component.AccountLoginSceen
import com.example.ssafy_pjt.component.HomeScreen
import com.example.ssafy_pjt.component.LoginSceen
import com.example.ssafy_pjt.component.SignupSceen
import com.example.ssafy_pjt.ui.theme.Ssafy_pjtTheme

class MainActivity : ComponentActivity() {

    private val kakaoAuthViewModel: KakaoAuthViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            Ssafy_pjtTheme {
                val navController = rememberNavController()
                NavHost(
                    navController = navController,
                    startDestination = "login"
                ){
                    composable("login") {
                        LoginSceen(
                            viewModel = kakaoAuthViewModel,
                            modifier = Modifier,
                            navController = navController
                            )
                    }
                    composable("signup") {
                        SignupSceen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("AccountLogin") {
                        AccountLoginSceen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("HomeSceen") {
                        HomeScreen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("DeliverySceen") {
                        DeliverySceen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                }
            }
        }
    }
}
