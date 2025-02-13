package com.example.ssafy_pjt.component

import android.util.Log
import android.widget.Toast
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel
import com.example.ssafy_pjt.ui.theme.loginTitle
import com.example.ssafy_pjt.ui.theme.modeType
import com.example.ssafy_pjt.ui.theme.nomalBold

@Composable
fun HomeSearchScreen(
    modifier: Modifier,
    navController: NavController,
    viewModel: AddressSearchViewModel,
){
    var address by remember { mutableStateOf("") }
    val addressList by viewModel.addressList.collectAsState()
    val prev by viewModel.prev.collectAsState()

    Scaffold { it->
        Column(
            modifier = modifier.padding(it)
        ) {
            Row (
                modifier = modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically,
            ){
                IconButton(
                    onClick = {
                        navController.popBackStack()
                    }
                ) {
                    Icon(Icons.Default.ArrowBack,
                        contentDescription = "뒤로가기")
                }
                Box(
                    modifier = modifier.fillMaxWidth(0.85f),
                    contentAlignment = Alignment.Center
                ){
                    Text(
                        text= stringResource(R.string.homeSearchTitle),
                        style = loginTitle
                    )
                }
            }
            Column(
                modifier = modifier.padding(start = 15.dp, end = 15.dp)
            ) {
                Text(
                    text = stringResource(R.string.homeSearch),
                    style = nomalBold
                )
                Spacer(modifier = modifier.size(15.dp))
                TextField(
                    modifier = modifier.fillMaxWidth(),
                    keyboardOptions = KeyboardOptions(
                        imeAction = ImeAction.Search
                    ),
                    keyboardActions = KeyboardActions(
                        onSearch = {
                            viewModel.getAdress()
                        }
                    ),
                    value = address,
                    onValueChange = {it ->
                        address = it
                        viewModel.update(it)
                    }
                )
                LazyColumn(
                    modifier = modifier.fillMaxSize()
                ) {
                    items(addressList) { address ->
                        Text(
                            style = modeType,
                            text = address,
                            modifier = modifier.clickable {
                                viewModel.update(address)
                                Log.d("TAG", "Selected address: $address")
                                Log.d("TAG", "prev: $prev")
                                if (prev=="home") {
                                    navController.navigate("homeRegisration")
                                } else if (prev=="search") {
                                    viewModel.destination()
                                    navController.navigate("DeliverySceen")
                                }
                            }
                                .padding(bottom = 10.dp, top = 10.dp)
                        )
                        Box(
                            modifier = Modifier
                                .fillMaxWidth()
                                .height(1.dp)
                                .background(Color.LightGray)
                        )
                    }
                }
            }
        }
    }
}