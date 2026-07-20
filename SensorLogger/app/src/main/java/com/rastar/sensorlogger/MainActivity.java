package com.rastar.sensorlogger;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.os.Build;
import android.os.Bundle;
import android.view.Gravity;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.rastar.vslam.msgs.ImuSample;
import com.rastar.vslam.msgs.Mat4f;
import com.rastar.vslam.msgs.PoseSample;
import com.rastar.vslam.msgs.Vec3f;

import java.util.Locale;

/**
 * Live IMU readout that can stream samples to a UDP-broadcast consumer.
 *
 * Accelerometer (TYPE_ACCELEROMETER) reports specific force in m/s^2 along the
 * device axes, including gravity. Gyroscope (TYPE_GYROSCOPE) reports angular
 * velocity in rad/s about the same axes. The gyro event drives the sample
 * cadence; each emitted {@link ImuSample} pairs that gyro reading with the most
 * recent accelerometer reading, stamped with the sensor's monotonic clock.
 *
 * All network work happens off-thread inside {@link ImuPublisher}; here we only
 * build protobuf messages and enqueue them.
 */
public class MainActivity extends AppCompatActivity implements SensorEventListener {

    // UDP broadcast port. Every host on the 192.168.1.* subnet that binds this
    // port receives the stream — no consumer IP required.
    private static final int  PORT             = 5555;          // ImuBatch  (body.jl)
    private static final int  PORT_POSE        = 5556;          // PoseBatch (world.jl)
    private static final long RENDER_PERIOD_NS = 66_000_000L;   // ~15 Hz UI

    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor gyroscope;

    private Button toggleButton;
    private TextView readout;

    private final float[] accel = {Float.NaN, Float.NaN, Float.NaN};
    private final float[] gyro  = {Float.NaN, Float.NaN, Float.NaN};

    private ImuPublisher publisher;      // non-null exactly while streaming
    private PosePublisher posePublisher; // ΔT stream (world.jl), parallel to publisher
    private final Integrator integrator = new Integrator();   // ΔPose since stream start
    private long lastRenderNs = 0;
    private long lastGyroTs = 0;       // for the Hz estimate
    private double hzEma = 0.0;        // EMA of the gyro sample rate

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        // targetSdk 35 renders edge-to-edge: without insetting, the top control
        // sits under the status bar / camera cutout and its taps are swallowed
        // by the system bar. Pad below the system bars + display cutout.
        ViewCompat.setOnApplyWindowInsetsListener(root, (v, insets) -> {
            Insets bars = insets.getInsets(
                    WindowInsetsCompat.Type.systemBars()
                            | WindowInsetsCompat.Type.displayCutout());
            v.setPadding(bars.left + 32, bars.top + 24, bars.right + 32, bars.bottom + 32);
            return insets;
        });

        // --- streaming toggle + readout -------------------------------------
        toggleButton = new Button(this);
        toggleButton.setText("Start streaming");
        toggleButton.setOnClickListener(v -> toggleStreaming());
        root.addView(toggleButton, matchWidth());
        readout = new TextView(this);
        readout.setTextSize(16f);
        readout.setGravity(Gravity.CENTER);
        readout.setPadding(0, 48, 0, 0);
        root.addView(readout, matchWidth());

        setContentView(root);

        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope     = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        render();
    }

    private LinearLayout.LayoutParams matchWidth() {
        return new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private void toggleStreaming() {
        if (publisher == null) {
            // Hard rule: Wi-Fi LAN only, never cellular. Refuse to start off Wi-Fi,
            // and pin the whole process to the Wi-Fi network so no socket can
            // egress over mobile data.
            Network wifi = findWifiNetwork();
            if (wifi == null) {
                readout.setText("Refusing to stream: not on Wi-Fi.\n"
                        + "This app never uses cellular data.\n\n" + sensorLines());
                return;
            }
            ConnectivityManager cm =
                    (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
            cm.bindProcessToNetwork(wifi);

            publisher = new ImuPublisher(PORT, deviceId());
            publisher.start();
            posePublisher = new PosePublisher(PORT_POSE, deviceId());
            posePublisher.start();
            integrator.start();            // zero ΔPose when streaming begins
            toggleButton.setText("Stop streaming");
        } else {
            stopStreaming();
        }
        render();
    }

    private void stopStreaming() {
        if (publisher != null) {
            publisher.stop();
            publisher = null;
        }
        if (posePublisher != null) {
            posePublisher.stop();
            posePublisher = null;
        }
        // Release the Wi-Fi pin so the rest of the system routes normally again.
        ConnectivityManager cm =
                (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        if (cm != null) cm.bindProcessToNetwork(null);
        toggleButton.setText("Start streaming");
    }

    /** A bound Wi-Fi {@link Network}, or null if the device isn't on Wi-Fi. */
    private Network findWifiNetwork() {
        ConnectivityManager cm =
                (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        if (cm == null) return null;
        for (Network n : cm.getAllNetworks()) {
            NetworkCapabilities c = cm.getNetworkCapabilities(n);
            if (c != null && c.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                return n;
            }
        }
        return null;
    }

    private static String deviceId() {
        return Build.MANUFACTURER + " " + Build.MODEL;
    }

    @Override
    protected void onResume() {
        super.onResume();
        lastGyroTs = 0;   // reset Hz estimate; ignore the pause gap
        // IMU-rate sampling (~50 Hz on GAME) — much faster than the old NORMAL.
        int rate = SensorManager.SENSOR_DELAY_GAME;
        if (accelerometer != null) sensorManager.registerListener(this, accelerometer, rate);
        if (gyroscope != null)     sensorManager.registerListener(this, gyroscope, rate);
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
        stopStreaming();   // don't hold a socket open while backgrounded
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, accel, 0, 3);
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(event.values, 0, gyro, 0, 3);
                updateRate(event.timestamp);
                // Gyro drives the cadence; emit only once accel is populated.
                if (publisher != null && isValid(accel)) {
                    ImuSample sample = buildSample(event.timestamp);
                    publisher.offer(sample);
                    integrator.update(sample);      // dead-reckon ΔPose from the same sample
                    // Broadcast the integrated pose ΔT once integration has begun.
                    if (posePublisher != null && integrator.isRunning()) {
                        posePublisher.offer(buildPose(event.timestamp));
                    }
                }
                break;
            default:
                return;
        }
        maybeRender(event.timestamp);
    }

    private ImuSample buildSample(long tMonoNs) {
        Vec3f g = Vec3f.newBuilder().setX(gyro[0]).setY(gyro[1]).setZ(gyro[2]).build();
        Vec3f a = Vec3f.newBuilder().setX(accel[0]).setY(accel[1]).setZ(accel[2]).build();
        return ImuSample.newBuilder()
                .setTMonoNs(tMonoNs)
                .setTUtcNs(System.currentTimeMillis() * 1_000_000L)
                .setGyro(g)
                .setAccel(a)
                .build();
    }

    /** Pack the integrator's ΔT (4×4, row-major) into a Mat4f PoseSample. */
    private PoseSample buildPose(long tMonoNs) {
        double[][] T = integrator.getDeltaPose();
        Mat4f.Builder m = Mat4f.newBuilder();
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                m.addM((float) T[r][c]);          // row-major: m[4*r + c]
        return PoseSample.newBuilder()
                .setTMonoNs(tMonoNs)
                .setTUtcNs(System.currentTimeMillis() * 1_000_000L)
                .setPose(m)
                .build();
    }

    private static boolean isValid(float[] v) {
        return !Float.isNaN(v[0]) && !Float.isNaN(v[1]) && !Float.isNaN(v[2]);
    }

    /** EMA of the gyro sample rate (gyro drives the IMU cadence). */
    private void updateRate(long tNs) {
        if (lastGyroTs != 0) {
            long dt = tNs - lastGyroTs;
            if (dt > 0) {
                double inst = 1e9 / dt;
                hzEma = (hzEma == 0.0) ? inst : 0.9 * hzEma + 0.1 * inst;
            }
        }
        lastGyroTs = tNs;
    }

    /** Throttle the TextView update; we sample far faster than we can read. */
    private void maybeRender(long tNs) {
        if (tNs - lastRenderNs > RENDER_PERIOD_NS) {
            lastRenderNs = tNs;
            // Wi-Fi dropped out mid-stream? Stop rather than risk cellular.
            // (Presence check — Wi-Fi need not be the default network.)
            if (publisher != null && findWifiNetwork() == null) {
                stopStreaming();
            }
            render();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // No-op.
    }

    private void render() {
        // Detect Wi-Fi by PRESENCE among all networks, not the default network:
        // this phone keeps Cellular as default while Wi-Fi (no internet) stays
        // connected, so getActiveNetwork() would wrongly report Cellular.
        boolean wifi = findWifiNetwork() != null;
        String wifiLine = "Wi-Fi:  " + (wifi ? "available" : "not available") + "\n";

        String streamLine;
        if (publisher != null) {
            streamLine = String.format(Locale.US,
                    "Stream: ON — UDP broadcast → %s\n        sent %d   dropped %d   %.1f Hz\n\n",
                    publisher.target(),
                    publisher.sentCount(), publisher.droppedCount(), hzEma);
        } else {
            streamLine = String.format(Locale.US,
                    "Stream: idle (not sending)   %.1f Hz\n\n", hzEma);
        }
        readout.setText(wifiLine + streamLine + sensorLines() + poseLines());
    }

    /**
     * Integrated ΔPose since stream start, laid out like the gyro/accel blocks:
     * aeronautical yaw/pitch/roll (deg) then translation x/y/z (m). Naked
     * open-loop strapdown — no bias correction, no ZUPT — so this is the honest,
     * unbounded drift.
     */
    private String poseLines() {
        if (!integrator.isRunning()) {                 // not integrating yet — no 0,0,0
            return "\n\nPress Start to begin dead-reckoning";
        }
        double[] rpy = Utils.rollPitchYawDeg(integrator.getDeltaPose());   // {roll, pitch, yaw} deg
        double[] t   = integrator.translation();       // {x, y, z} m
        String zupt = integrator.isStationary() ? "  [ZUPT]" : "";
        double elapsed = integrator.elapsedSeconds();
        double zuptS   = integrator.zuptSeconds();
        double zuptPct = elapsed > 0.0 ? 100.0 * zuptS / elapsed : 0.0;
        return String.format(Locale.US,
                "\n\nRunning %.1f s  (open-loop drift)\n" +
                "ZUPT %.1f s of %.1f s  (%.0f%%)\n" +
                "\nRotation (deg)\n" +
                "yaw:   %+7.2f\npitch: %+7.2f\nroll:  %+7.2f" +
                "\n\nTranslation (m)%s\n" +
                "x: %+7.3f\ny: %+7.3f\nz: %+7.3f",
                elapsed, zuptS, elapsed, zuptPct,
                rpy[2], rpy[1], rpy[0],      // yaw, pitch, roll
                zupt, t[0], t[1], t[2]);
    }

    private String sensorLines() {
        return "Accelerometer (m/s^2)\n" + axisLines(accel) +
                "\n\nGyroscope (rad/s)\n" + axisLines(gyro);
    }

    private static String axisLines(float[] v) {
        return String.format(Locale.US,
                "X: %+8.3f\nY: %+8.3f\nZ: %+8.3f", v[0], v[1], v[2]);
    }
}
