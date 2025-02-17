package com.example.ssafy_pjt.ViewModel

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

class JourneyHistoryViewModel(application: Application) : AndroidViewModel(application) {
    private val historyDataStore = JourneyHistoryDataStore(application)

    val allHistory = historyDataStore.journeyHistories
        .stateIn(
            viewModelScope,
            SharingStarted.Lazily,
            emptyList()
        )

    fun addJourney(mode: String, destination: String, startPoint:String) {
        viewModelScope.launch {
            historyDataStore.addJourney(
                JourneyHistory(
                    mode = mode,
                    destination = destination,
                    startPoint = startPoint
                )
            )
        }
    }

    fun deleteJourney(journey: JourneyHistory) {
        viewModelScope.launch {
            historyDataStore.deleteJourney(journey)
        }
    }
}