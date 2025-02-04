package com.example.ssafy_pjt.network

import android.app.Activity
import android.content.Context
import android.content.Intent
import android.util.Log
import androidx.activity.result.contract.ActivityResultContract
import com.example.ssafy_pjt.BuildConfig
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.auth.api.signin.GoogleSignInAccount
import com.google.android.gms.auth.api.signin.GoogleSignInOptions
import com.google.android.gms.tasks.Task

class GoogleApiContract : ActivityResultContract<Int, Task<GoogleSignInAccount>?>() {
    override fun createIntent(context: Context, input: Int): Intent {
        val gso = GoogleSignInOptions.Builder(GoogleSignInOptions.DEFAULT_SIGN_IN)
            .requestIdToken(BuildConfig.GOOGLE_OAUTH_CLIENT_ID)
            .requestId()
            .build()

        val intent = GoogleSignIn.getClient(context, gso)
        Log.d("TAG", "gso: $gso ")

        Log.d("TAG", "intent:  $intent")
        return intent.signInIntent
    }

    override fun parseResult(resultCode: Int, intent: Intent?): Task<GoogleSignInAccount>? {
        Log.d("TAG", "resultCode: $resultCode ")
        intent?.extras?.keySet()?.forEach { key ->
            Log.d("TAG", "Key: $key, Value: ${intent.extras?.get(key)}")
        }
        return when (resultCode) {
            Activity.RESULT_OK -> {
                GoogleSignIn.getSignedInAccountFromIntent(intent)
            }
            else -> null
        }
    }
}
