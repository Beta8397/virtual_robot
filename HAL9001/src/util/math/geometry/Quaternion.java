package util.math.geometry;

import org.ejml.simple.SimpleMatrix;

import static java.lang.Math.PI;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * A Quaternion class.
 * <p>
 * Creation Date:
 *
 * @author Jack Kinney, Level Up
 * @version 1.0.0
 * @since 1.1.0
 */
public class Quaternion {
    //The Quaternion's components.
    private double x, y, z, w;

    /**
     * A private constructor for Quaternion. Used for cloning.
     *
     * @param q The quaternion to clone.
     */
    private Quaternion(Quaternion q) {
        this(q.x, q.y, q.z, q.w);
    }

    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Quaternion(Axis3D axis, double angle) {
        double s = sin(angle / 2);
        w = cos(angle / 2);
        x = axis.getX() * s;
        y = axis.getY() * s;
        z = axis.getZ() * s;
    }

    public static double norm(Quaternion q) {
        return sqrt(q.dot(q));
    }

    public static Quaternion multiply(Quaternion q1, Quaternion q2) {
        return new Quaternion(q1).multiply(q2);
    }

    public static Quaternion multiply(Quaternion q, double scale) {
        return new Quaternion(q).multiply(scale);
    }

    public static Quaternion divide(Quaternion q, double scale) {
        return new Quaternion(q).divide(scale);
    }

    public static double dot(Quaternion q1, Quaternion q2) {
        return q1.dot(q2);
    }

    public static Quaternion interpolate(Quaternion q1, Quaternion q2, double t) {
        return new Quaternion(q1).interpolate(q2, t);
    }

    public static Quaternion normalize(Quaternion q) {
        return new Quaternion(q).normalize();
    }

    /**
     * Converts the given quaternion to Euler angles.
     *
     * @param q The quaternion to convert to Euler angles.
     * @return The Euler angles representation of the quaternion.
     */
    public static double[] toEulerAngles(Quaternion q) {
        return new Quaternion(q).toEulerAngles();
    }

    /**
     * Converts a set of Euler angles to a quaternion.
     *
     * @param yaw   The yaw angle.
     * @param pitch The pitch angle.
     * @param roll  The roll angle.
     * @return The quaternion representation of the given 3D orientation.
     */
    public static Quaternion ToQuaternion(double yaw, double pitch, double roll) { // yaw (Z), pitch (Y), roll (X)
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        return new Quaternion(
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy
        );
    }

    public double norm() {
        return sqrt(this.dot(this));
    }

    /**
     * Multiplies this quaternion by another quaternion
     *
     * @param q Thw quaternion to multiply by.
     * @return This quaternion.
     */
    public Quaternion multiply(Quaternion q) {
        //matrixs = null;
        double nw = w * q.w - x * q.x - y * q.y - z * q.z;
        double nx = w * q.x + x * q.w + y * q.z - z * q.y;
        double ny = w * q.y + y * q.w + z * q.x - x * q.z;
        z = w * q.z + z * q.w + x * q.y - y * q.x;
        w = nw;
        x = nx;
        y = ny;
        return this;
    }

    /**
     * Multiplies this quaternion by a constant.
     *
     * @param scale The constant to multiply by.
     * @return This quaternion.
     */
    public Quaternion multiply(double scale) {
        if (scale != 1) {
            //matrixs = null;
            w *= scale;
            x *= scale;
            y *= scale;
            z *= scale;
        }
        return this;
    }

    /**
     * Divides this quaternion by a constant.
     *
     * @param scale The constant to divide by.
     * @return This quaternion.
     */
    public Quaternion divide(double scale) {
        if (scale != 1) {
            //matrixs = null;
            w /= scale;
            x /= scale;
            y /= scale;
            z /= scale;
        }
        return this;
    }

    public double dot(Quaternion q) {
        return x * q.x + y * q.y + z * q.z + w * q.w;
    }

    public Quaternion interpolate(Quaternion q, double t) {
        if (!equals(q)) {
            double d = dot(q);
            double qx, qy, qz, qw;

            if (d < 0) {
                qx = -q.x;
                qy = -q.y;
                qz = -q.z;
                qw = -q.w;
                d = -d;
            } else {
                qx = q.x;
                qy = q.y;
                qz = q.z;
                qw = q.w;
            }

            double f0, f1;

            if ((1 - d) > 0.1f) {
                double angle = acos(d);
                double s = sin(angle);
                double tAngle = t * angle;
                f0 = sin(angle - tAngle) / s;
                f1 = sin(tAngle) / s;
            } else {
                f0 = 1 - t;
                f1 = t;
            }

            x = f0 * x + f1 * qx;
            y = f0 * y + f1 * qy;
            z = f0 * z + f1 * qz;
            w = f0 * w + f1 * qw;
        }

        return this;
    }

    public Quaternion normalize() {
        return divide(this.norm());
    }

    /**
     * Converts this Quaternion into a matrix.
     */
    public SimpleMatrix toMatrix() {
        return new SimpleMatrix(new double[][]{
                {1.0 - (2.0 * ((y * y) + (z * z))), 2.0 * ((x * y) - (z * w)), 2.0 * ((x * z) + (y * w)), 0},
                {2.0 * ((x * y) + (z * w)), 1.0 - (2.0 * ((x * x) + (z * z))), 2.0 * ((y * z) - (x * w)), 0},
                {2.0 * ((x * z) - (y * w)), 2.0 * ((y * z) + (x * w)), 1.0 - (2.0 * ((x * x) + (y * y))), 0},
                {0, 0, 0, 0}
        });
    }

    /**
     * Converts this quaternion to its Euler angle representation.
     *
     * @return The Euler angles representation of this orientation.
     */
    public double[] toEulerAngles() {
        double sqw = w * w;
        double sqx = x * x;
        double sqy = y * y;
        double sqz = z * z;

        // If quaternion is normalised the unit is one, otherwise it is the correction factor
        double unit = sqx + sqy + sqz + sqw;
        double test = x * y + y * w;

        // Store the Euler angles in radians
        double[] pitchYawRoll;
        if (test > 0.4999 * unit) {                             // 0.4999f OR 0.5f - EPSILON
            // Singularity at north pole
            pitchYawRoll = new double[]{
                    2 * atan2(x, w), // Yaw
                    PI * 0.5,        // Pitch
                    0                // Roll
            };
            return pitchYawRoll;
        } else if (test < -0.4999 * unit) {                       // -0.4999f OR -0.5f + EPSILON
            // Singularity at south pole
            pitchYawRoll = new double[]{
                    -2 * atan2(x, w), // Yaw
                    -PI * 0.5,        // Pitch
                    0                 // Roll
            };
            return pitchYawRoll;
        } else {
            pitchYawRoll = new double[]{
                    atan2(2 * y * w - 2 * x * z, sqx - sqy - sqz + sqw),  // Yaw
                    asin(2 * test / unit),                                // Pitch
                    atan2(2 * x * w - 2 * y * z, -sqx + sqy - sqz + sqw)  // Roll
            };
        }

        return pitchYawRoll;
    }

    public double getW() {
        return w;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Quaternion) {
            Quaternion q = (Quaternion) obj;
            return x == q.x && y == q.y && z == q.z && w == q.w;
        }
        return false;
    }

    @Override
    public Quaternion clone() {
        return new Quaternion(this);
    }
}
