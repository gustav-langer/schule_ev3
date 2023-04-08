package roboter;

/**
 * This class is for working with speeds. The original graphical UI for the EV3 calculated speed in rotations per minute,
 * while functions like {@link lejos.robotics.RegulatedMotor#setSpeed(int) setSpeed(int)} calculate in degrees per second.
 * This class supports both and can be used to easily convert between the units.
 */
class Speed {
    private static final int JAVA_EV3_CONVERSION_FACTOR = 6; // (360 degrees / 1 rotation) * (1 minute / 60 seconds)
    private final int javaSpeed; //Geschwindigkeit in Grad/Sekunde

    private Speed(int javaSpeed) {
        this.javaSpeed = javaSpeed;
    }

    /**
     * @param degreesPerSecond The speed in degrees per second
     * @return A {@link Speed} instance with the desired values
     */
    static Speed javaSpeed(int degreesPerSecond) {
        return new Speed(degreesPerSecond);
    }

    /**
     * @param rotationsPerMinute The speed in rotations per minute
     * @return A {@link Speed} instance with the desired values
     */
    static Speed ev3Speed(int rotationsPerMinute) {
        return new Speed(rotationsPerMinute * JAVA_EV3_CONVERSION_FACTOR);
    }

    /**
     * @return The speed in degrees per second
     */
    int getJavaSpeed() {
        return javaSpeed;
    }

    /**
     * @return The speed in rotations per minute
     */
    int getEv3Speed() {
        return javaSpeed / JAVA_EV3_CONVERSION_FACTOR;
    }

    /**
     * @return true if the stored value is greater than 0
     */
    @SuppressWarnings("unused")
    boolean isPositive() {
        return javaSpeed > 0;
    }

    int signum() {
        return Integer.signum(javaSpeed);
    }

    /**
     * @param factor The factor to multiply the speed by
     * @return A new {@link Speed} with the value of the current one multiplied by {@code factor}
     */
    @SuppressWarnings("SameParameterValue")
    Speed mult(float factor) {
        return Speed.javaSpeed(Math.round(this.getJavaSpeed() * factor));
    }

    /**
     * @return A new {@link Speed} with the same absolute value as this one but a reversed direction.
     * Equivalent to {@code mult(-1);}
     */
    Speed neg() {
        return mult(-1);
    }
}
