package com.rastar.sensorlogger;

import android.util.Log;

import com.rastar.vslam.msgs.ImuBatch;
import com.rastar.vslam.msgs.ImuSample;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Streams IMU samples as protobuf {@link ImuBatch} datagrams over <b>UDP
 * broadcast</b> on the local subnet. No consumer IP is needed: the datagrams go
 * to the subnet broadcast address (e.g. 192.168.1.255), and any host on
 * 192.168.1.* that binds the port receives them.
 *
 * <p><b>Connectionless and lossy by design.</b> UDP gives no delivery, ordering,
 * or loss notification. That's acceptable here because every {@link ImuSample}
 * self-timestamps ({@code t_mono_ns}), so a dropped datagram merely widens an
 * integration interval rather than corrupting state. Consumers should reorder by
 * timestamp.
 *
 * <p><b>MTU.</b> Batches are kept small (≤ {@link #BATCH_SIZE} samples, well
 * under ~1 KB) so a datagram is never IP-fragmented — a fragment loss would lose
 * the whole datagram.
 *
 * <p><b>Threading.</b> The socket lives on a private sender thread; sensor
 * callbacks only {@link #offer} samples.
 */
public final class ImuPublisher {
    private static final String TAG = "ImuPublisher";

    // Worst-case ImuSample is ~80 B, so ≤10 samples keeps each datagram < 1 KB.
    private static final int  BATCH_SIZE = 10;
    private static final long FLUSH_MS   = 50;

    private final int port;
    private final String deviceId;
    private final BlockingQueue<ImuSample> queue = new LinkedBlockingQueue<>();
    private final AtomicLong sent    = new AtomicLong();
    private final AtomicLong dropped = new AtomicLong();

    // Monotonic batch counter for the *raw IMU stream*, shared across all
    // publisher instances so it NEVER resets on stream stop/start — the sensor
    // feed is continuous. (Only a full process restart zeroes it.) Contrast the
    // per-session PosePublisher counter, which resets because ΔT resets.
    private static final AtomicLong SEQ = new AtomicLong();

    private volatile boolean running = false;
    private volatile String target = "?";
    private Thread thread;

    public ImuPublisher(int port, String deviceId) {
        this.port = port;
        this.deviceId = deviceId;
    }

    public void start() {
        if (running) return;
        running = true;
        thread = new Thread(this::run, "imu-udp-sender");
        thread.start();
    }

    /** Non-blocking enqueue; safe to call from the sensor (main) thread. */
    public void offer(ImuSample s) {
        if (running) queue.offer(s);
    }

    public void stop() {
        running = false;
        if (thread != null) {
            thread.interrupt();
            try {
                thread.join(1000);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
            thread = null;
        }
        queue.clear();
    }

    public long sentCount()    { return sent.get(); }
    public long droppedCount() { return dropped.get(); }

    /** Broadcast target being used, e.g. "192.168.1.255:5555" — for display. */
    public String target() { return target + ":" + port; }

    private void run() {
        try (DatagramSocket sock = new DatagramSocket()) {
            sock.setBroadcast(true);
            InetAddress dst = broadcastAddress();
            target = (dst != null) ? dst.getHostAddress() : "255.255.255.255";
            Log.i(TAG, "UDP broadcast to " + target + ":" + port);

            ImuBatch.Builder batch = ImuBatch.newBuilder().setDeviceId(deviceId);
            int n = 0;
            long lastFlush = System.nanoTime();

            while (running && !Thread.currentThread().isInterrupted()) {
                ImuSample s;
                try {
                    s = queue.poll(FLUSH_MS, TimeUnit.MILLISECONDS);
                } catch (InterruptedException e) {
                    break;
                }
                if (s != null) {
                    batch.addSamples(s);
                    n++;
                }

                long ageMs = (System.nanoTime() - lastFlush) / 1_000_000L;
                boolean flush = n >= BATCH_SIZE || (n > 0 && ageMs >= FLUSH_MS);
                if (flush) {
                    batch.setSeq(SEQ.getAndIncrement());   // monotonic, never resets
                    byte[] bytes = batch.build().toByteArray();
                    try {
                        sock.send(new DatagramPacket(bytes, bytes.length, dst, port));
                        sent.addAndGet(n);
                    } catch (Exception e) {
                        dropped.addAndGet(n);   // interface down / no route
                    }
                    batch = ImuBatch.newBuilder().setDeviceId(deviceId);
                    n = 0;
                    lastFlush = System.nanoTime();
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "sender thread died", e);
        }
    }

    /** Subnet-directed IPv4 broadcast (e.g. 192.168.1.255), else limited bcast. */
    private static InetAddress broadcastAddress() {
        try {
            for (NetworkInterface ni :
                    Collections.list(NetworkInterface.getNetworkInterfaces())) {
                if (ni.isLoopback() || !ni.isUp()) continue;
                for (InterfaceAddress ia : ni.getInterfaceAddresses()) {
                    InetAddress bc = ia.getBroadcast();
                    if (bc != null && ia.getAddress() instanceof Inet4Address) {
                        return bc;
                    }
                }
            }
        } catch (Exception ignored) {
            // fall through to limited broadcast
        }
        try {
            return InetAddress.getByName("255.255.255.255");
        } catch (Exception e) {
            return null;
        }
    }
}
