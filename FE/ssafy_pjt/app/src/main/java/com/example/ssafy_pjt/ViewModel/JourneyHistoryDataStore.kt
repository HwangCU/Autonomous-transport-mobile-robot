package com.example.ssafy_pjt.ViewModel

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.google.gson.Gson
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

// companion object로 DataStore 인스턴스 생성
private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "journey_history")

class JourneyHistoryDataStore(private val context: Context) {
    private val gson = Gson()

    val journeyHistories: Flow<List<JourneyHistory>> = context.dataStore.data
        .map { preferences ->
            val historyJson = preferences[stringPreferencesKey("journeys")] ?: "[]"
            gson.fromJson(historyJson, Array<JourneyHistory>::class.java).toList()
        }

    suspend fun addJourney(journey: JourneyHistory) {
        context.dataStore.edit { preferences ->
            val currentHistoryJson = preferences[stringPreferencesKey("journeys")] ?: "[]"
            val currentHistory = gson.fromJson(currentHistoryJson, Array<JourneyHistory>::class.java).toList()
            val updatedHistory = (currentHistory + journey).takeLast(20)
            preferences[stringPreferencesKey("journeys")] = gson.toJson(updatedHistory)
        }
    }

    suspend fun deleteJourney(journey: JourneyHistory) {
        context.dataStore.edit { preferences ->
            val currentHistoryJson = preferences[stringPreferencesKey("journeys")] ?: "[]"
            val currentHistory = gson.fromJson(currentHistoryJson, Array<JourneyHistory>::class.java).toList()
            val updatedHistory = currentHistory.filter { it.timestamp != journey.timestamp }
            preferences[stringPreferencesKey("journeys")] = gson.toJson(updatedHistory)
        }
    }
}
data class JourneyHistory(
    val mode: String,
    val startPoint:String,
    val destination: String,
    val timestamp: Long = System.currentTimeMillis()
)