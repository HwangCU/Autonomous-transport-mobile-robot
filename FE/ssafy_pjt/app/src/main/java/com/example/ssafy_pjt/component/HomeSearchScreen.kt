package com.example.ssafy_pjt.component

import android.util.Log
import android.widget.Toast
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
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
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.ImeAction
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AddressSearchViewModel

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
            Text(text= stringResource(R.string.homeSearchTitle))
            Text(text = stringResource(R.string.homeSearch))
            TextField(
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
            LazyColumn {
                items(addressList) { address ->
                    Text(
                        text = address,
                        modifier = modifier.clickable {
                            viewModel.update(address)
                            Log.d("TAG", "Selected address: $address")
                            Log.d("TAG", "prev: $prev")
                            if (prev=="home") {
                                navController.navigate("homeRegisration")
                            } else if (prev=="search") {
                                navController.navigate("DeliverySceen")
                            }
                        }
                    )
                }
            }
        }
    }
}