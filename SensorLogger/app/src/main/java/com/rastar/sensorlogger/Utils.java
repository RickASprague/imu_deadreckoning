package com.rastar.sensorlogger;

/**
 * Small, dependency-free linear-algebra helpers shared across the app —
 * 3×3 / 4×4 matrix ops, SO(3) exp, vector utilities, Euler extraction.
 * Pure functions, no state.
 */
public final class Utils {

    private Utils() {}   // static-only

    /**
     * Skew-symmetric ("hat") matrix [φ]× of a 3-vector φ = (x,y,z), the 𝔰𝔬(3)
     * generator such that [φ]×·v = φ × v:
     * <pre>   [ 0  −z   y ;  z   0  −x ;  −y   x   0 ].</pre>
     */
    public static double[][] skew(double x, double y, double z) {
        return new double[][] {
            {  0.0,  -z,   y  },
            {   z,  0.0,  -x  },
            {  -y,   x,  0.0  },
        };
    }

    /**
     * Rodrigues: exp of the rotation vector φ = (x,y,z) ∈ 𝔰𝔬(3) as a 3×3
     * rotation matrix. Exact closed form
     * <pre>   R = I + (sinθ/θ)[φ]× + ((1−cosθ)/θ²)[φ]×²,   θ = ‖φ‖,</pre>
     * with the small-angle series near θ = 0 to avoid the 0/0.
     */
    public static double[][] expSO3(double x, double y, double z) {
        final double t2 = x * x + y * y + z * z;
        final double t  = Math.sqrt(t2);
        final double a, b;                      // a = sinθ/θ, b = (1−cosθ)/θ²
        if (t < 1e-8) {                         // series near 0 (avoid 0/0)
            a = 1.0 - t2 / 6.0;
            b = 0.5 - t2 / 24.0;
        } else {
            a = Math.sin(t) / t;
            b = (1.0 - Math.cos(t)) / t2;
        }
        final double[][] K  = skew(x, y, z);    // [φ]×
        final double[][] K2 = matMul3(K, K);    // [φ]×²
        final double[][] R  = identity3();      // R = I + a·K + b·K²
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R[i][j] += a * K[i][j] + b * K2[i][j];
        return R;
    }

    /** 3×3 matrix product C = A·B. */
    public static double[][] matMul3(double[][] A, double[][] B) {
        final double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                double s = 0.0;
                for (int k = 0; k < 3; k++) s += A[i][k] * B[k][j];
                C[i][j] = s;
            }
        return C;
    }

    /** 3×3 mat-vec: out = A·(x,y,z). */
    public static void matVec3(double[][] A, double x, double y, double z, double[] out) {
        out[0] = A[0][0] * x + A[0][1] * y + A[0][2] * z;
        out[1] = A[1][0] * x + A[1][1] * y + A[1][2] * z;
        out[2] = A[2][0] * x + A[2][1] * y + A[2][2] * z;
    }

    /** Extract the upper-left 3×3 rotation block from a 4×4 homogeneous T. */
    public static double[][] rotationOf(double[][] T) {
        return new double[][] {
            { T[0][0], T[0][1], T[0][2] },
            { T[1][0], T[1][1], T[1][2] },
            { T[2][0], T[2][1], T[2][2] },
        };
    }

    /** Write a 3×3 rotation R into the upper-left block of a 4×4 T. */
    public static void setRotation(double[][] T, double[][] R) {
        for (int i = 0; i < 3; i++)
            System.arraycopy(R[i], 0, T[i], 0, 3);
    }

    /**
     * Aeronautical Tait-Bryan angles of the rotation in the upper-left 3×3 of
     * {@code T}, in <b>degrees</b>: {roll, pitch, yaw}. Intrinsic Z-Y-X sequence
     * R = Rz(ψ)·Ry(θ)·Rx(φ) — yaw ψ about z, pitch θ about y, roll φ about x.
     * Gimbal-lock (θ ≈ ±90°) folds roll into yaw.
     */
    public static double[] rollPitchYawDeg(double[][] T) {
        final double r20   = clamp(T[2][0], -1.0, 1.0);
        final double pitch = Math.asin(-r20);                 // θ = asin(−R₂₀)
        final double roll, yaw;
        if (Math.abs(Math.cos(pitch)) > 1e-6) {
            roll = Math.atan2(T[2][1], T[2][2]);              // φ = atan2(R₂₁, R₂₂)
            yaw  = Math.atan2(T[1][0], T[0][0]);              // ψ = atan2(R₁₀, R₀₀)
        } else {                                              // pitch ≈ ±90°
            roll = 0.0;
            yaw  = Math.atan2(-T[0][1], T[1][1]);
        }
        final double d = 180.0 / Math.PI;
        return new double[] { roll * d, pitch * d, yaw * d };
    }

    public static double[][] identity3() {
        return new double[][] { {1,0,0}, {0,1,0}, {0,0,1} };
    }

    public static double[][] identity4() {
        final double[][] T = new double[4][4];
        setIdentity4(T);
        return T;
    }

    public static void setIdentity4(double[][] T) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = (i == j) ? 1.0 : 0.0;
    }

    public static void zero(double[] v) { v[0] = v[1] = v[2] = 0.0; }

    public static double norm(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
