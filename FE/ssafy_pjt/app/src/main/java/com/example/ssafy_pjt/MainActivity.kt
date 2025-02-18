package com.example.ssafy_pjt

import BluetoothScreen
import DeliverySceen
import GameController
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.example.ssafy_pjt.ViewModel.AccountLoginViewModel
import com.example.ssafy_pjt.ViewModel.AccountLoginViewModelFactory
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ViewModel.GoogleLoginViewModel
import com.example.ssafy_pjt.ViewModel.GoogleLoginViewModelFactory
import com.example.ssafy_pjt.ViewModel.KakaoAuthViewModel
import com.example.ssafy_pjt.ViewModel.KakaoLoginViewModelFactory
import com.example.ssafy_pjt.ViewModel.RobotRegistViewModelFactory
import com.example.ssafy_pjt.ViewModel.RobotRegistrationViewModel
import com.example.ssafy_pjt.ViewModel.UserViewModel
import com.example.ssafy_pjt.ViewModel.addressSearchViewModelFactory
import com.example.ssafy_pjt.component.AccountLoginSceen
import com.example.ssafy_pjt.component.FollowingScreen
import com.example.ssafy_pjt.component.HomeRefistrationScreen
import com.example.ssafy_pjt.component.HomeScreen
import com.example.ssafy_pjt.component.HomeSearchScreen
import com.example.ssafy_pjt.component.LoginSceen
import com.example.ssafy_pjt.component.ProfileScreen
import com.example.ssafy_pjt.component.RobotRegistration
import com.example.ssafy_pjt.component.SendHomeScreen
import com.example.ssafy_pjt.component.Setting
import com.example.ssafy_pjt.component.SignupSceen
import com.example.ssafy_pjt.ui.theme.Ssafy_pjtTheme
import com.example.ssafy_pjt.ViewModel.ProfileViewModel
import com.kakao.sdk.common.util.Utility
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.File
import com.example.ssafy_pjt.network.RetrofitClient
import androidx.activity.result.ActivityResultLauncher
import com.example.ssafy_pjt.ViewModel.socketViewModel
import com.example.ssafy_pjt.component.DeviceItem

class MainActivity : ComponentActivity() {
    private val socketViewModel: socketViewModel by viewModels()

    private val kakaoAuthViewModel: KakaoAuthViewModel by viewModels{
        KakaoLoginViewModelFactory(userViewModel,application)
    }
    private val GoogleLoginViewModel: GoogleLoginViewModel by viewModels{
        GoogleLoginViewModelFactory(userViewModel)
    }
    private val userViewModel: UserViewModel by viewModels()
    private val accountLoginViewModel: AccountLoginViewModel by viewModels {
        AccountLoginViewModelFactory(userViewModel)
    }
    private val addressSearchViewModel: AddressSearchViewModel by viewModels{
        addressSearchViewModelFactory(userViewModel)
    }
    private val robotRegistrationViewModel : RobotRegistrationViewModel by viewModels{
        RobotRegistViewModelFactory(userViewModel)
    }

    private val profileViewModel: ProfileViewModel by viewModels {
        object : ViewModelProvider.Factory {
            override fun <T : ViewModel> create(modelClass: Class<T>): T {
                return ProfileViewModel(userViewModel) as T
            }
        }
    }

    private lateinit var imagePickerLauncher: ActivityResultLauncher<String>

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        if (isGranted) {
            imagePickerLauncher.launch("image/*")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        
        imagePickerLauncher = registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
            uri?.let { selectedImageUri ->
                profileViewModel.updateProfileImage(selectedImageUri)
            }
        }
        
        setContent {
            Ssafy_pjtTheme {
                val navController = rememberNavController()
                NavHost(
                    navController = navController,
                    startDestination = "Setting"
                ){
                    composable("login") {
                        LoginSceen(
                            kakaoviewModel = kakaoAuthViewModel,
                            modifier = Modifier,
                            navController = navController,
                            googleViewModel= GoogleLoginViewModel,
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
                            navController = navController,
                            viewModel= accountLoginViewModel
                        )
                    }
                    composable("HomeSceen") {
                        HomeScreen(
                            modifier = Modifier,
                            navController = navController,
                            userViewModel = userViewModel,
                            socketViewModel = socketViewModel
                        )
                    }
                    composable("DeliverySceen") {
                        DeliverySceen(
                            modifier = Modifier,
                            navController = navController,
                            viewModel = addressSearchViewModel,
                            userViewModel = userViewModel
                        )
                    }
                    composable("RobotRegistration"){
                        RobotRegistration(
                            modifier = Modifier,
                            navController = navController,
                            robotRegistrationViewModel = robotRegistrationViewModel
                        )
                    }
                    composable("homeRegisration") {
                        HomeRefistrationScreen(
                            modifier= Modifier,
                            navController = navController,
                            viewModel = addressSearchViewModel
                        )
                    }
                    composable("homeSearch") {
                        HomeSearchScreen(
                            modifier = Modifier,
                            navController = navController,
                            viewModel = addressSearchViewModel
                        )
                    }
                    composable("FollowingScreen") {
                        FollowingScreen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("Setting") {
                        Setting(
                            modifier = Modifier,
                            navController = navController,
                            profileViewModel=profileViewModel
                        )
                    }
                    composable("SendHomeScreen") {
                        SendHomeScreen(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("GameController") {
                        GameController(
                            modifier = Modifier,
                            navController = navController
                        )
                    }
                    composable("ProfileScreen") {
                        ProfileScreen(
                            navController = navController,
                            viewModel = profileViewModel,
                            onImageClick = {
                                checkAndRequestPermission()
                            }
                        )
                    }
                    composable("bluetooth") {
                        BluetoothScreen(
                            navController = navController,
                            onDeviceSelected = {
                            navController.navigate("GameController")
                            }
                        )
                    }
                }
            }
        }
    }

    private fun checkAndRequestPermission() {
        when {
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU -> {
                requestPermissionLauncher.launch(android.Manifest.permission.READ_MEDIA_IMAGES)
            }
            else -> {
                requestPermissionLauncher.launch(android.Manifest.permission.READ_EXTERNAL_STORAGE)
            }
        }
    }
}

