package roboter;

public class RotateAmount {
    private final int degrees;

    private RotateAmount(int degrees) {
        this.degrees = degrees;
    }

    public static RotateAmount rotations(float rotations) {
        return new RotateAmount(Math.round(rotations * 360));
    }

    public static RotateAmount degrees(int degrees) {
        return new RotateAmount(degrees);
    }

    public int getDegrees() {
        return degrees;
    }

    public float getRotations() {
        return (float) degrees / 360F;
    }

    boolean isPositive() {
        return degrees > 0;
    }

    int signum() {
        return Integer.signum(degrees);
    }

    RotateAmount mult(int factor) {
        return RotateAmount.degrees(degrees * factor);
    }
}