package com.example.ssafy_pjt.component

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.ui.unit.Dp
import com.example.ssafy_pjt.ViewModel.ProfileViewModel
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.network.UserProfile
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.TextFieldDefaults
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.draw.clip
import androidx.compose.material3.Icon
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Edit
import androidx.compose.material.icons.filled.AccountCircle
import coil.compose.AsyncImage

@Composable
private fun ProfileContent(
    userProfile: UserProfile,
    modifier: Modifier = Modifier,
    navController: NavController,
    viewModel: ProfileViewModel,
    onImageClick: () -> Unit
) {
    val isEditing = remember { mutableStateOf(false) }
    var editedUsername by remember { mutableStateOf(userProfile.username) }
    var editedAddress by remember { mutableStateOf(userProfile.address) }
    val updateStatus by viewModel.updateStatus.collectAsState()
    val showErrorDialog = remember { mutableStateOf(false) }
    val errorMessage = remember { mutableStateOf("") }
    val showCancelConfirmDialog = remember { mutableStateOf(false) }

    fun hasChanges(): Boolean {
        return editedUsername != userProfile.username || editedAddress != userProfile.address
    }

    fun handleCloseClick() {
        if (isEditing.value) {
            if (hasChanges() || viewModel.userViewModel.tempProfileImage.value != viewModel.userViewModel.profileImage.value) {
                showCancelConfirmDialog.value = true
            } else {
                isEditing.value = false
            }
        } else {
            navController.popBackStack()
        }
    }

    // 업데이트 상태 처리
    LaunchedEffect(updateStatus) {
        when (updateStatus) {
            is ProfileViewModel.UpdateStatus.Success -> {
                isEditing.value = false
                viewModel.resetUpdateStatus()
            }
            is ProfileViewModel.UpdateStatus.Error -> {
                errorMessage.value = (updateStatus as ProfileViewModel.UpdateStatus.Error).message
                showErrorDialog.value = true
                viewModel.resetUpdateStatus()
            }
            else -> {}
        }
    }

    Box(
        contentAlignment = Alignment.TopStart,
        modifier = modifier
            .fillMaxSize()
            .background(Color(0xffe5ecf0))
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            // Top section with close button
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(8.dp)
            ) {
                Box(
                    modifier = Modifier
                        .align(Alignment.TopEnd)
                        .size(48.dp)
                        .background(Color(0xffe23c3c), RoundedCornerShape(5.dp))
                        .clickable { handleCloseClick() },
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "X",
                        color = Color.White,
                        fontSize = 24.sp,
                        modifier = Modifier.border(1.dp, Color(0xffe23c3c))
                    )
                }
            }

            Spacer(modifier = Modifier.height(24.dp))

            // Welcome text
            Text(
                text = "${userProfile.username}님 안녕하세요",
                color = Color(0xff5e77e1),
                fontSize = 24.sp,
                fontWeight = FontWeight.Bold,
                textAlign = TextAlign.Center,
                modifier = Modifier.padding(vertical = 16.dp)
            )

            // Profile image
            Column(
                modifier = Modifier.padding(vertical = 16.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(120.dp)
                        .clip(CircleShape)
                        .background(Color(0xff5e77e1))
                        .clickable(enabled = isEditing.value, onClick = onImageClick),
                    contentAlignment = Alignment.Center
                ) {
                    val profileImage by viewModel.userViewModel.profileImage.collectAsState()
                    val tempProfileImage by viewModel.userViewModel.tempProfileImage.collectAsState()
                    val displayImage = if (isEditing.value) tempProfileImage else profileImage
                    
                    if (displayImage != null) {
                        AsyncImage(
                            model = displayImage,
                            contentDescription = "Profile Image",
                            contentScale = ContentScale.Crop,
                            modifier = Modifier.fillMaxSize()
                        )
                    } else {
                        Icon(
                            Icons.Default.AccountCircle,
                            contentDescription = "Profile Image",
                            modifier = Modifier.fillMaxSize(),
                            tint = Color.White
                        )
                    }
                    
                    if (isEditing.value) {
                        Box(
                            modifier = Modifier
                                .fillMaxSize()
                                .background(Color(0x66000000)),
                            contentAlignment = Alignment.Center
                        ) {
                            Icon(
                                imageVector = Icons.Default.Edit,
                                contentDescription = "Edit Image",
                                tint = Color.White
                            )
                        }
                    }
                }
            }

            // Profile information section
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 16.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp)
            ) {
                if (isEditing.value) {
                    // 편집 모드 UI
                    EditableProfileInfoItem(
                        label = "이름",
                        value = editedUsername,
                        onValueChange = { editedUsername = it }
                    )
                    ProfileInfoItem(label = "이메일", value = userProfile.email)
                    EditableProfileInfoItem(
                        label = "집주소",
                        value = editedAddress,
                        onValueChange = { editedAddress = it },
                        height = 120.dp
                    )
                    ProfileInfoItem(label = "로봇ID", value = userProfile.robotId)
                } else {
                    // 기존 표시 UI
                    ProfileInfoItem(label = "이름", value = userProfile.username)
                    ProfileInfoItem(label = "이메일", value = userProfile.email)
                    ProfileInfoItem(
                        label = "집주소",
                        value = userProfile.address,
                        height = 120.dp
                    )
                    ProfileInfoItem(label = "로봇ID", value = userProfile.robotId)
                }
            }

            Spacer(modifier = Modifier.weight(1f))

            // Bottom buttons
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 16.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                if (isEditing.value) {
                    // 편집 모드일 때는 완료 버튼만 표시
                    ActionButton(text = "완료") {
                        val updatedProfile = userProfile.copy(
                            username = editedUsername,
                            address = editedAddress
                        )
                        when (val result = viewModel.updateProfile(updatedProfile)) {
                            is ProfileViewModel.ValidationResult.Success -> {
                                viewModel.confirmProfileImageEdit()
                            }
                            is ProfileViewModel.ValidationResult.Error -> {
                                // 유효성 검사 실패 시 에러 메시지 표시
                                errorMessage.value = result.message
                                showErrorDialog.value = true
                            }
                        }
                    }
                } else {
                    // 일반 모드일 때는 정보수정과 로그아웃 버튼 표시
                    ActionButton(text = "정보수정") {
                        isEditing.value = true
                        // 편집 시작할 때 현재 값으로 초기화
                        editedUsername = userProfile.username
                        editedAddress = userProfile.address
                    }
                    ActionButton(text = "로그아웃") {
                        viewModel.onLogoutClick()
                    }
                }
            }
        }
    }

    // 편집 취소 확인 팝업
    if (showCancelConfirmDialog.value) {
        Box(
            contentAlignment = Alignment.Center,
            modifier = Modifier
                .fillMaxSize()
                .background(Color(0x33000000))
                .clickable(enabled = false) { }
        ) {
            Column(
                verticalArrangement = Arrangement.Top,
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .wrapContentSize()
                    .clip(RoundedCornerShape(14.dp))
                    .background(Color.White)
                    .border(
                        width = 1.dp,
                        color = Color(0x8c808080),
                        shape = RoundedCornerShape(14.dp)
                    )
            ) {
                Column(
                    verticalArrangement = Arrangement.Top,
                    horizontalAlignment = Alignment.CenterHorizontally,
                    modifier = Modifier
                        .width(270.dp)
                        .padding(start = 16.dp, top = 19.dp, end = 16.dp, bottom = 15.dp)
                ) {
                    Text(
                        text = "회원 정보 수정 취소",
                        color = Color(0xff000000),
                        fontSize = 17.sp,
                        fontWeight = FontWeight.Normal,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.fillMaxWidth()
                    )
                    Text(
                        text = "변경된 내용을 저장하지 않고\n나가시겠습니까?",
                        color = Color(0xff000000),
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Normal,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
                
                Row(
                    modifier = Modifier.width(270.dp)
                ) {
                    Box(
                        contentAlignment = Alignment.Center,
                        modifier = Modifier
                            .weight(1f)
                            .height(44.dp)
                            .border(
                                width = 1.dp,
                                color = Color(0x8c808080)
                            )
                            .clickable { showCancelConfirmDialog.value = false }
                    ) {
                        Text(
                            text = "취소",
                            color = Color(0xffe23c3c),
                            fontSize = 17.sp,
                            fontWeight = FontWeight.Normal,
                            textAlign = TextAlign.Center
                        )
                    }
                    Box(
                        contentAlignment = Alignment.Center,
                        modifier = Modifier
                            .weight(1f)
                            .height(44.dp)
                            .border(
                                width = 1.dp,
                                color = Color(0x8c808080)
                            )
                            .clickable {
                                showCancelConfirmDialog.value = false
                                isEditing.value = false
                                editedUsername = userProfile.username
                                editedAddress = userProfile.address
                                viewModel.cancelProfileImageEdit()
                            }
                    ) {
                        Text(
                            text = "확인",
                            color = Color(0xff007aff),
                            fontSize = 17.sp,
                            fontWeight = FontWeight.Normal,
                            textAlign = TextAlign.Center
                        )
                    }
                }
            }
        }
    }

    // 팝업창 UI
    if (showErrorDialog.value) {
        Box(
            contentAlignment = Alignment.Center,
            modifier = Modifier
                .fillMaxSize()
                .background(Color(0x33000000))
                .clickable(enabled = false) { }
        ) {
            Column(
                verticalArrangement = Arrangement.Top,
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .wrapContentSize()
                    .clip(RoundedCornerShape(14.dp))
                    .background(Color.White)
                    .border(
                        width = 1.dp,
                        color = Color(0x8c808080),
                        shape = RoundedCornerShape(14.dp)
                    )
            ) {
                Column(
                    verticalArrangement = Arrangement.Top,
                    horizontalAlignment = Alignment.CenterHorizontally,
                    modifier = Modifier
                        .width(270.dp)
                        .padding(start = 16.dp, top = 19.dp, end = 16.dp, bottom = 15.dp)
                ) {
                    Text(
                        text = "회원 정보 수정 실패",
                        color = Color(0xff000000),
                        fontSize = 17.sp,
                        fontWeight = FontWeight.Normal,
                        textAlign = TextAlign.Center,
                        modifier = Modifier
                            .fillMaxWidth()
                    )
                    Text(
                        text = errorMessage.value,
                        color = Color(0xff000000),
                        fontSize = 13.sp,
                        fontWeight = FontWeight.Normal,
                        textAlign = TextAlign.Center,
                        modifier = Modifier
                            .fillMaxWidth()
                    )
                }
                
                Box(
                    contentAlignment = Alignment.Center,
                    modifier = Modifier
                        .width(270.dp)
                        .height(44.dp)
                        .border(
                            width = 1.dp,
                            color = Color(0x8c808080)
                        )
                        .clickable { showErrorDialog.value = false }
                ) {
                    Text(
                        text = "확인",
                        color = Color(0xff007aff),
                        fontSize = 17.sp,
                        fontWeight = FontWeight.Normal,
                        textAlign = TextAlign.Center
                    )
                }
            }
        }
    }
}

@Composable
private fun ProfileInfoItem(
    label: String,
    value: String?,
    height: Dp = 63.dp
) {
    Row(
        modifier = Modifier
            .fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(
            text = label,
            color = Color(0xff636363),
            fontSize = 20.sp,
            modifier = Modifier
                .weight(0.25f)
                .padding(top = 8.dp)
        )

        Box(
            modifier = Modifier
                .weight(0.75f)
                .height(height)
                .background(Color(0xff5e77e1), RoundedCornerShape(15.dp))
                .padding(16.dp),
            contentAlignment = Alignment.CenterStart
        ) {
            Text(
                text = value ?: "미설정",
                color = Color.White,
                fontSize = 20.sp
            )
        }
    }
}

@Composable
private fun EditableProfileInfoItem(
    label: String,
    value: String,
    onValueChange: (String) -> Unit,
    height: Dp = 63.dp
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(
            text = label,
            color = Color(0xff636363),
            fontSize = 20.sp,
            modifier = Modifier
                .weight(0.25f)
                .padding(top = 8.dp)
        )

        Box(
            modifier = Modifier
                .weight(0.75f)
                .height(height)
                .background(Color(0xff5e77e1), RoundedCornerShape(15.dp))
                .padding(4.dp),
            contentAlignment = Alignment.Center
        ) {
            OutlinedTextField(
                value = value,
                onValueChange = onValueChange,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(height - 8.dp),
                colors = TextFieldDefaults.colors(
                    focusedContainerColor = Color.White,
                    unfocusedContainerColor = Color.White,
                    focusedTextColor = Color(0xff5e77e1),
                    unfocusedTextColor = Color(0xff5e77e1),
                    focusedIndicatorColor = Color.Transparent,
                    unfocusedIndicatorColor = Color.Transparent,
                    disabledIndicatorColor = Color.Transparent,
                    errorIndicatorColor = Color.Transparent
                ),
                shape = RoundedCornerShape(10.dp),
                keyboardOptions = KeyboardOptions.Default.copy(
                    imeAction = ImeAction.Done
                ),
                textStyle = androidx.compose.ui.text.TextStyle(
                    fontSize = 20.sp
                )
            )
        }
    }
}

@Composable
private fun ActionButton(
    text: String,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(41.dp)
            .background(Color(0xfff6de79), RoundedCornerShape(5.dp))
            .clickable(onClick = onClick),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            color = Color.Black,
            fontSize = 12.sp
        )
    }
}

@Composable
fun ProfileScreen(
    navController: NavController,
    viewModel: ProfileViewModel = viewModel(),
    onImageClick: () -> Unit
) {
    val userProfile by viewModel.userProfile.collectAsState()
    val isLoading by viewModel.isLoading.collectAsState()
    val error by viewModel.error.collectAsState()
    
    // 화면에 진입할 때마다 프로필 정보 다시 불러오기
    LaunchedEffect(Unit) {
        viewModel.fetchUserProfile()
    }
    
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        when {
            isLoading -> {
                CircularProgressIndicator()
            }
            error != null -> {
                // 에러 화면 표시
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.Center
                ) {
                    Text(
                        text = error ?: "알 수 없는 오류가 발생했습니다.",
                        color = Color.Red
                    )
                    Spacer(modifier = Modifier.height(16.dp))
                    Button(onClick = { 
                        // 다시 시도 버튼 추가
                        viewModel.fetchUserProfile()
                    }) {
                        Text("다시 시도")
                    }
                    Button(onClick = { navController.popBackStack() }) {
                        Text("돌아가기")
                    }
                }
            }
            userProfile != null -> {
                ProfileContent(
                    userProfile = userProfile!!,
                    modifier = Modifier,
                    navController = navController,
                    viewModel = viewModel,
                    onImageClick = onImageClick
                )
            }
        }
    }
}