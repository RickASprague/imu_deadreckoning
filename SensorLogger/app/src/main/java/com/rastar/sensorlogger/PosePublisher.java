package com.rastar.sensorlogger;

import android.util.Log;

import com.rastar.vslam.msgs.PoseBatch;
import com.rastar.vslam.msgs.PoseSample;

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
 * Streams integrated poses as protobuf {@link PoseBatch} datagrams over UDP
 * broadcast — the sibling of {@link ImuPublisher}, on a separate port so the raw
 * IMU stream (body-frame viewer) and the pose stream (world-frame viewer) are
 * independent subscriptions.
 *
 * <p>Each {@link PoseSample} carries the integrator's ΔT = [R p; 0 1] as a
 * row-major {@code Mat4f}, stamped on the same monotonic clock as the IMU. Same
 * connectionless/lossy contract as {@link ImuPublisher}: a dropped datagram just
 * skips a pose frame; the next one is absolute (ΔT from start), not incremental,
 * so there is nothing to resync.
 */
public final class PosePublisher {
    private static final String TAG = "PosePublisher";

    // A PoseSample is ~80 B (16 floats + 2 longs), so ≤10 keeps a datagram <1 KB.
    private static final int  BATCH_SIZE = 10;
    private static final long FLUSH_MS   = 50;

    private final int port;
    private final String deviceId;
    private final BlockingQueue<PoseSample> queue = new LinkedBlockingQueue<>();
    private final AtomicLong sent    = new AtomicLong();
    private final AtomicLong dropped = new AtomicLong();
    private long seq = 0;   // per-session batch counter (sender thread only); 0 = first

    private volatile boolean running = false;
    private volatile String target = "?";
    private Thread thread;

    public PosePublisher(int port, String deviceId) {
        this.port = port;
        this.deviceId = deviceId;
    }

    public void start() {
        if (running) return;
        running = true;
        thread = new Thread(this::run, "pose-udp-sender");
        thread.start();
    }

    /** Non-blocking enqueue; safe to call from the sensor (main) thread. */
    public void offer(PoseSample s) {
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

    /** Broadcast target being used, e.g. "192.168.1.255:5556" — for display. */
    public String target() { return target + ":" + port; }

    private void run() {
        try (DatagramSocket sock = new DatagramSocket()) {
            sock.setBroadcast(true);
            InetAddress dst = broadcastAddress();
            target = (dst != null) ? dst.getHostAddress() : "255.255.255.255";
            Log.i(TAG, "UDP broadcast to " + target + ":" + port);

            PoseBatch.Builder batch = PoseBatch.newBuilder().setDeviceId(deviceId);
            int n = 0;
            long lastFlush = System.nanoTime();

            while (running && !Thread.currentThread().isInterrupted()) {
                PoseSample s;
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
                    batch.setSeq(seq++);   // stamp + advance the session batch counter
                    byte[] bytes = batch.build().toByteArray();
                    try {
                        sock.send(new DatagramPacket(bytes, bytes.length, dst, port));
                        sent.addAndGet(n);
                    } catch (Exception e) {
                        dropped.addAndGet(n);   // interface down / no route
                    }
                    batch = PoseBatch.newBuilder().setDeviceId(deviceId);
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
