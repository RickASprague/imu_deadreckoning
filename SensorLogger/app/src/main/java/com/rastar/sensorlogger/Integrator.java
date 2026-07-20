package com.rastar.sensorlogger;

import com.rastar.vslam.msgs.ImuSample;
import com.rastar.vslam.msgs.Vec3f;

import java.util.Locale;

/**
 * Open-loop strap-down IMU integrator. Accumulates a relative pose ΔT ∈ SE(3)
 * from the frame captured at {@link #start()} to the current body frame, by
 * integrating the gyroscope (orientation) and accelerometer (translation).
 *
 * <p><b>No calibration, no bias estimation.</b> The gyro is integrated raw (no
 * gyro-bias removal); gravity is not measured but assumed constant,
 * g^w = (0,0,−g) with g = {@link #G} (start frame taken level, z up); the
 * accelerometer bias is left in (unobservable from a static pose anyway). The
 * one thing that still bounds the drift is the ZUPT below.
 *
 * <p><b>State.</b> ΔT = [ R^w_b  p^w ; 0 1 ], with R^w_b = R_start←body (maps a
 * body-frame vector into the start/world frame) and p^w the body origin in the
 * start frame. Velocity v^w (start frame) is carried internally for the double
 * integration; it is not part of the returned pose.
 *
 * <p><b>Orientation.</b> Raw gyro ω^b_wb (rad/s), NO bias removal, on SO(3):
 * <pre>   R^w_b ← R^w_b · Exp(ω^b_wb·Δt),   Exp(φ) = I + (sinθ/θ)[φ]× + ((1−cosθ)/θ²)[φ]×²,
 *                            θ = ‖φ‖,  φ = ω^b_wb·Δt.</pre>
 * The right multiply applies the body-frame increment; the closed-form Rodrigues
 * rotation is exact (small-angle series near θ = 0 for stability).
 *
 * <p><b>Translation.</b> The accelerometer's specific force is rotated to the
 * world frame and the assumed constant gravity added:
 * <pre>   a^w = R^w_b·f^b + g^w,   g^w = (0,0,−G),</pre>
 * with a^w_n at R^w_b(t_n) and a^w_{n+1} at R^w_b(t_{n+1}). First-order hold
 * (a^w linear over the interval), exact integrals:
 * <pre>   p^w ← p^w + v^w·Δt + (2·a^w_n + a^w_{n+1})/6·Δt²,
 *   v^w ← v^w + ½·(a^w_n + a^w_{n+1})·Δt.        [trapezoidal ≡ velocity-Verlet]</pre>
 *
 * <p><b>Δt</b> comes from consecutive {@code t_mono_ns} (hardware monotonic
 * clock). Samples with non-positive or implausibly large Δt (dropped packets)
 * are skipped for integration but still advance the clock. Integration begins on
 * the first sample after {@link #start()} — there is no calibration hold.
 *
 * <p><b>ZUPT (zero-velocity update).</b> Open-loop, position drifts as O(t²): an
 * accel bias or a tilt leak g·δθ ramps velocity, and p^w = ∬a^w runs away. The
 * cheap fix exploits the one thing we know when the device is <i>still</i>:
 * v^w = 0. A stationarity detector fires when the current sample shows no
 * rotation (‖ω^b_wb‖ &lt; {@link #GYRO_STATIC}) and specific force ≈ gravity
 * (|‖f‖ − G| &lt; {@link #ACCEL_STATIC}). While stationary we clamp v^w ← 0 and
 * hold p^w, severing the bias/tilt → velocity → position drift ramp — so on a
 * table position stops wandering. This is the velocity-only ZUPT; it is the only
 * thing bounding the drift, since no bias is estimated.
 */
public final class Integrator {

    private static final double MAX_DT = 0.5;   // s; larger ⇒ treat as a gap, skip
    private static final double G      = 9.81;  // assumed gravity magnitude (m/s²)

    // ZUPT stationarity gates, tested on the current sample.
    private static final double GYRO_STATIC  = 0.05;   // rad/s  (~2.9°/s)
    private static final double ACCEL_STATIC = 0.5;    // m/s²   |‖f‖ − G|

    /** Accumulated relative pose ΔT (4×4 homogeneous), identity at start. */
    private final double[][] deltaPose = Utils.identity4();
    private final double[]   velocity  = new double[3];   // v^w, start frame

    private ImuSample last;              // previous sample; its ω,f drive the interval
    private boolean   started    = false;
    private boolean   stationary = false;   // last ZUPT decision (for display)

    // Monotonic-clock stamp of the first integrated sample. Elapsed time since
    // this is the window over which ΔT — and its drift — has accumulated.
    private long integStartNs = 0;

    // Cumulative wall time (s) spent stationary (ZUPT engaged). zuptSeconds /
    // elapsedSeconds is "how much of the run was ZUPT".
    private double zuptSeconds = 0.0;

    /** Reset ΔT to identity; integration begins on the next sample (no hold). */
    public void start() {
        Utils.setIdentity4(deltaPose);
        Utils.zero(velocity);
        last         = null;
        started      = true;
        stationary   = false;
        integStartNs = 0;
        zuptSeconds  = 0.0;
    }

    /**
     * Integrate one IMU sample and return the accumulated ΔPose so far (4×4
     * homogeneous, R^w_b in the upper-left block, p^w in the last column). The
     * returned array is the live internal buffer — copy it to retain a snapshot.
     *
     * <p>Zero-order hold: the <b>previous</b> sample's ω, f were in effect over
     * the interval Δt = t_now − t_prev that this sample closes; we integrate
     * those over Δt, then remember this sample.
     */
    public double[][] update(ImuSample s) {
        if (!started) start();

        if (last == null) {                 // first sample: t = 0, start the clock
            last = s;
            integStartNs = s.getTMonoNs();
            return deltaPose;
        }

        final double dt = (s.getTMonoNs() - last.getTMonoNs()) * 1e-9;
        if (dt <= 0.0 || dt > MAX_DT) {     // out-of-order or gap: skip, but advance
            last = s;
            return deltaPose;
        }

        // Raw previous-sample gyro drives the interval rotation (no bias removal);
        // the two accelerometer readings bracket the interval for a trapezoid.
        final Vec3f w   = last.getGyro();
        final double wx = w.getX();
        final double wy = w.getY();
        final double wz = w.getZ();
        final Vec3f fn  = last.getAccel();      // specific force at t_n
        final Vec3f fn1 = s.getAccel();         // specific force at t_{n+1}

        final double[][] Rn  = Utils.rotationOf(deltaPose);               // R^w_b(t_n)
        final double[][] Rn1 = Utils.matMul3(Rn,                          // R^w_b(t_{n+1})
                Utils.expSO3(wx * dt, wy * dt, wz * dt));

        // ---- ZUPT: zero-velocity update ------------------------------------
        // If the current sample says we're stationary, assert v^w = 0 and hold
        // p^w (skip translation) — this cuts the drift ramp. Otherwise integrate.
        stationary = isStationary(s);
        if (stationary) {
            Utils.zero(velocity);               // v^w ← 0; position held
            zuptSeconds += dt;                  // tally this interval as ZUPT time
        } else {
            // World proper acceleration at both endpoints: a^w = R^w_b·f^b + g^w,
            // with the assumed constant gravity g^w = (0,0,−G).
            final double[] an  = new double[3];
            final double[] an1 = new double[3];
            Utils.matVec3(Rn,  fn.getX(),  fn.getY(),  fn.getZ(),  an);
            Utils.matVec3(Rn1, fn1.getX(), fn1.getY(), fn1.getZ(), an1);
            an[2]  -= G;                         // + g^w_z, and g^w_z = −G
            an1[2] -= G;

            // First-order hold (a^w linear over the interval), exact integrals:
            //   p^w += v^w·Δt + (2·a^w_n + a^w_{n+1})/6·Δt² ;  v^w += ½·(a^w_n + a^w_{n+1})·Δt
            // (p^w uses the pre-update velocity, so update p^w before v^w.)
            for (int i = 0; i < 3; i++) {
                deltaPose[i][3] += velocity[i] * dt
                                 + (2.0 * an[i] + an1[i]) / 6.0 * dt * dt;
                velocity[i]     += 0.5 * (an[i] + an1[i]) * dt;
            }
        }

        Utils.setRotation(deltaPose, Rn1);      // commit the updated orientation
        last = s;
        return deltaPose;
    }

    /**
     * ZUPT stationarity test on a sample: no rotation and specific force ≈ the
     * assumed gravity magnitude G ⇒ the device is (near) still.
     */
    private boolean isStationary(ImuSample s) {
        final Vec3f w = s.getGyro();            // raw rate (no bias to subtract)
        final Vec3f f = s.getAccel();
        final double wn = Math.sqrt(w.getX()*w.getX() + w.getY()*w.getY() + w.getZ()*w.getZ());
        final double fn = Math.sqrt(f.getX()*f.getX() + f.getY()*f.getY() + f.getZ()*f.getZ());
        return wn < GYRO_STATIC && Math.abs(fn - G) < ACCEL_STATIC;
    }

    /** Whether the last {@link #update} applied a ZUPT (device judged stationary). */
    public boolean isStationary() { return stationary; }

    /** True once integration has begun (first sample seen after {@link #start()}). */
    public boolean isRunning() { return integStartNs != 0; }

    /**
     * Seconds of pose integration elapsed — from the first sample (t = 0,
     * ΔT = identity) to the most recent, on the monotonic sensor clock. This is
     * the window the displayed ΔT drifted over. 0 before the first sample.
     */
    public double elapsedSeconds() {
        if (integStartNs == 0 || last == null) return 0.0;
        return (last.getTMonoNs() - integStartNs) * 1e-9;
    }

    /**
     * Cumulative seconds held stationary (ZUPT) since integration began. Divided
     * by {@link #elapsedSeconds()} this is the fraction of the run spent still.
     */
    public double zuptSeconds() { return zuptSeconds; }

    public double[][] getDeltaPose() { return deltaPose; }
    public double[]   getVelocity()  { return velocity; }

    /** Translation p^w of ΔT, in metres (start-frame). */
    public double[] translation() {
        return new double[] { deltaPose[0][3], deltaPose[1][3], deltaPose[2][3] };
    }

    /** Compact one-line pose summary for logging/GUI. */
    public String poseString() {
        return String.format(Locale.US,
                "p=(%+.3f,%+.3f,%+.3f) |v|=%.3f",
                deltaPose[0][3], deltaPose[1][3], deltaPose[2][3], Utils.norm(velocity));
    }
}
