package com.example.ssafy_pjt.component

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.ImeAction
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavController
import com.example.ssafy_pjt.R
import com.example.ssafy_pjt.ViewModel.AdressSearchViewModel

@Composable
fun HomeSearchScreen(
    modifier: Modifier,
    navController: NavController,
    viewModel: AdressSearchViewModel = viewModel()
){
    var searchAdress by remember { mutableStateOf("") }
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
                value = searchAdress,
                onValueChange = {it ->
                    searchAdress = it}
            )
        }
    }
}