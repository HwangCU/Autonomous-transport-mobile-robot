package com.example.ros2gpspublisher;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Bundle;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.location.Priority;
import com.google.android.material.textfield.TextInputEditText;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONObject;

import java.net.URI;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private static final int LOCATION_PERMISSION_REQUEST_CODE = 1234;
    private static final String TAG = "MainActivity";

    // Location related variables
    private FusedLocationProviderClient fusedLocationClient;
    private LocationCallback locationCallback;

    // WebSocket related variables
    private WebSocketClient webSocketClient;

    // UI elements
    private TextView locationText;
    private TextView connectionStatus;
    private Button reconnectButton;
    private TextInputEditText ipInput;
    private TextInputEditText portInput;
    private Button disconnectButton;
    private TextView imuText;

    // Sensor related variables
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private Sensor magnetometer;
    private Sensor rotationVector;

    // Sensor data storage
    private float[] accelerometerValues = new float[3];
    private float[] gyroscopeValues = new float[3];
    private float[] magneticValues = new float[3];
    private float[] rotationValues = new float[4];  // w, x, y, z

    // Matrices for orientation calculation
    private float[] rotationMatrix = new float[9];
    private float[] orientationAngles = new float[3];

    // Variables for velocity calculation
    private long lastUpdateTime = 0;
    private float x_velocity = 0;
    private float y_velocity = 0;
    private float z_velocity = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        try {
            super.onCreate(savedInstanceState);
            setContentView(R.layout.activity_main);

            // UI 초기화
            locationText = findViewById(R.id.locationText);
            connectionStatus = findViewById(R.id.connectionStatus);
            ipInput = findViewById(R.id.ipInput);
            portInput = findViewById(R.id.portInput);
            imuText = findViewById(R.id.imuText);

            // 버튼 초기화 추가
            Button reconnectButton = findViewById(R.id.reconnectButton);
            Button disconnectButton = findViewById(R.id.disconnectButton);

            if (reconnectButton != null) {
                reconnectButton.setOnClickListener(v -> {
                    String ip = ipInput.getText().toString();
                    String port = portInput.getText().toString();

                    if (ip.isEmpty() || port.isEmpty()) {
                        Toast.makeText(this, "IP와 포트를 입력해주세요", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    String websocketUrl = "ws://" + ip + ":" + port;
                    initWebSocket(websocketUrl);
                });
            }

            if (disconnectButton != null) {
                disconnectButton.setOnClickListener(v -> {
                    if (webSocketClient != null) {
                        webSocketClient.close();
                        updateConnectionStatus("연결이 해제되었습니다");
                        Toast.makeText(this, "웹소켓 연결이 해제되었습니다", Toast.LENGTH_SHORT).show();
                    }
                });
            }

            // GPS 클라이언트 초기화
            fusedLocationClient = LocationServices.getFusedLocationProviderClient(this);

            // 위치 업데이트 콜백 설정
            locationCallback = new LocationCallback() {
                @Override
                public void onLocationResult(LocationResult locationResult) {
                    if (locationResult == null) {
                        updateLocationUI("위치 정보를 받아올 수 없습니다");
                        return;
                    }

                    for (Location location : locationResult.getLocations()) {
                        updateLocationUI(location);
                        sendLocationData(location);
                    }
                }
            };

            // IMU 센서 초기화
            sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

            // 센서 리스너 등록
            if (accelerometer != null) {
                sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
            }
            if (gyroscope != null) {
                sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_NORMAL);
            }
            if (magnetometer != null) {
                sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
            }
            if (rotationVector != null) {
                sensorManager.registerListener(this, rotationVector, SensorManager.SENSOR_DELAY_NORMAL);
            }

            // GPS 권한 확인
            checkLocationPermission();

        } catch (Exception e) {
            Log.e(TAG, "onCreate 에러", e);
            Toast.makeText(this, "초기화 중 오류 발생: " + e.getMessage(), Toast.LENGTH_LONG).show();
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, accelerometerValues, 0, 3);
                calculateVelocity(event.timestamp);
                break;

            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(event.values, 0, gyroscopeValues, 0, 3);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values, 0, magneticValues, 0, 3);
                calculateOrientation();
                break;

            case Sensor.TYPE_ROTATION_VECTOR:
                System.arraycopy(event.values, 0, rotationValues, 0,
                        Math.min(event.values.length, 4));
                break;
        }

        updateIMUUI();
        sendIMUData();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 센서 정확도 변경 시 처리
    }

    private void calculateVelocity(long currentTime) {
        if (lastUpdateTime != 0) {
            float dt = (currentTime - lastUpdateTime) * 1.0f / 1000000000.0f; // 나노초를 초로 변환

            // 중력 가속도 제거 (간단한 방법)
            float linearX = accelerometerValues[0];
            float linearY = accelerometerValues[1];
            float linearZ = accelerometerValues[2] - 9.81f;

            // 속도 적분
            x_velocity += linearX * dt;
            y_velocity += linearY * dt;
            z_velocity += linearZ * dt;
        }
        lastUpdateTime = currentTime;
    }

    private void calculateOrientation() {
        SensorManager.getRotationMatrix(rotationMatrix, null,
                accelerometerValues, magneticValues);
        SensorManager.getOrientation(rotationMatrix, orientationAngles);
    }
    private void sendIMUData() {
        try {
            if (webSocketClient != null && webSocketClient.isOpen()) {
                JSONObject imuData = new JSONObject();
                imuData.put("type", "imu");

                // 가속도
                JSONObject accel = new JSONObject();
                accel.put("x", accelerometerValues[0]);
                accel.put("y", accelerometerValues[1]);
                accel.put("z", accelerometerValues[2]);
                imuData.put("accelerometer", accel);

                // 자이로스코프
                JSONObject gyro = new JSONObject();
                gyro.put("x", gyroscopeValues[0]);
                gyro.put("y", gyroscopeValues[1]);
                gyro.put("z", gyroscopeValues[2]);
                imuData.put("gyroscope", gyro);

                // 자기장
                JSONObject magnetic = new JSONObject();
                magnetic.put("x", magneticValues[0]);
                magnetic.put("y", magneticValues[1]);
                magnetic.put("z", magneticValues[2]);
                imuData.put("magnetic", magnetic);

                // 회전 벡터
                JSONObject rotation = new JSONObject();
                rotation.put("x", rotationValues[0]);
                rotation.put("y", rotationValues[1]);
                rotation.put("z", rotationValues[2]);
                if (rotationValues.length > 3) {
                    rotation.put("w", rotationValues[3]);
                }
                imuData.put("rotation", rotation);

                // 계산된 값들
                JSONObject calculated = new JSONObject();
                calculated.put("x_velocity", x_velocity);
                calculated.put("y_velocity", y_velocity);
                calculated.put("z_velocity", z_velocity);
                calculated.put("yaw", orientationAngles[0]);
                calculated.put("pitch", orientationAngles[1]);
                calculated.put("roll", orientationAngles[2]);
                imuData.put("calculated", calculated);

                webSocketClient.send(imuData.toString());
            }
        } catch (Exception e) {
            Log.e(TAG, "IMU 데이터 전송 오류", e);
        }
    }

    private void updateIMUUI() {
        String imuInfo = String.format(Locale.getDefault(),
                "가속도(m/s²):\nX: %.2f  Y: %.2f  Z: %.2f\n\n" +
                        "자이로(rad/s):\nX: %.2f  Y: %.2f  Z: %.2f\n\n" +
                        "자기장(µT):\nX: %.2f  Y: %.2f  Z: %.2f\n\n" +
                        "방향(rad):\nYaw: %.2f\nPitch: %.2f\nRoll: %.2f\n\n" +
                        "속도(m/s):\nX: %.2f  Y: %.2f  Z: %.2f",
                accelerometerValues[0], accelerometerValues[1], accelerometerValues[2],
                gyroscopeValues[0], gyroscopeValues[1], gyroscopeValues[2],
                magneticValues[0], magneticValues[1], magneticValues[2],
                orientationAngles[0], orientationAngles[1], orientationAngles[2],
                x_velocity, y_velocity, z_velocity
        );
        runOnUiThread(() -> imuText.setText(imuInfo));
    }

    private void sendLocationData(Location location) {
        try {
            if (webSocketClient != null && webSocketClient.isOpen()) {
                JSONObject locationData = new JSONObject();
                locationData.put("type", "gps");  // 메시지 타입 추가
                locationData.put("latitude", location.getLatitude());
                locationData.put("longitude", location.getLongitude());
                locationData.put("altitude", location.getAltitude());
                locationData.put("accuracy", location.getAccuracy());
                locationData.put("time", location.getTime());

                webSocketClient.send(locationData.toString());
            }
        } catch (Exception e) {
            Log.e(TAG, "위치 데이터 전송 오류", e);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (fusedLocationClient != null) {
            fusedLocationClient.removeLocationUpdates(locationCallback);
        }
        if (webSocketClient != null) {
            webSocketClient.close();
        }
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }
    private void initWebSocket(String websocketUrl) {
        try {
            // 기존 연결이 있으면 종료
            if (webSocketClient != null) {
                webSocketClient.close();
            }

            URI uri = new URI(websocketUrl);
            webSocketClient = new WebSocketClient(uri) {
                @Override
                public void onOpen(ServerHandshake handshakedata) {
                    Log.d(TAG, "WebSocket 연결됨");
                    updateConnectionStatus("서버에 연결되었습니다");
                }

                @Override
                public void onMessage(String message) {
                    Log.d(TAG, "메시지 수신: " + message);
                }

                @Override
                public void onClose(int code, String reason, boolean remote) {
                    Log.d(TAG, "WebSocket 연결 종료: " + reason);
                    updateConnectionStatus("서버 연결이 종료되었습니다");
                }

                @Override
                public void onError(Exception ex) {
                    Log.e(TAG, "WebSocket 에러", ex);
                    updateConnectionStatus("연결 오류: " + ex.getMessage());
                }
            };
            webSocketClient.connect();
        } catch (Exception e) {
            Log.e(TAG, "WebSocket 초기화 에러", e);
            updateConnectionStatus("초기화 오류: " + e.getMessage());
        }
    }

    private void checkLocationPermission() {
        try {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                        LOCATION_PERMISSION_REQUEST_CODE);
                Log.d(TAG, "위치 권한 요청");
            } else {
                startLocationUpdates();
                Log.d(TAG, "이미 위치 권한 있음");
            }
        } catch (Exception e) {
            Log.e(TAG, "권한 체크 에러", e);
            Toast.makeText(this, "GPS 권한 체크 중 오류 발생", Toast.LENGTH_SHORT).show();
        }
    }

    private void startLocationUpdates() {
        try {
            LocationRequest locationRequest = new LocationRequest.Builder(1000)  // 1초마다 업데이트
                    .setPriority(Priority.PRIORITY_HIGH_ACCURACY)
                    .build();

            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    != PackageManager.PERMISSION_GRANTED) {
                Log.d(TAG, "위치 권한 없음");
                return;
            }

            fusedLocationClient.requestLocationUpdates(locationRequest,
                    locationCallback,
                    null);
            Log.d(TAG, "위치 업데이트 시작됨");
        } catch (Exception e) {
            Log.e(TAG, "위치 업데이트 시작 에러", e);
            Toast.makeText(this, "위치 업데이트 시작 중 오류 발생", Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == LOCATION_PERMISSION_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startLocationUpdates();
                Toast.makeText(this, "GPS 권한이 허용되었습니다", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "GPS 권한이 필요합니다", Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void updateLocationUI(Location location) {
        String locationInfo = String.format(
                "위도: %.6f\n경도: %.6f\n고도: %.2f m\n정확도: %.1f m",
                location.getLatitude(),
                location.getLongitude(),
                location.getAltitude(),
                location.getAccuracy()
        );
        runOnUiThread(() -> locationText.setText(locationInfo));
    }

    private void updateLocationUI(String message) {
        runOnUiThread(() -> locationText.setText(message));
    }

    private void updateConnectionStatus(String status) {
        runOnUiThread(() -> connectionStatus.setText(status));
    }
}