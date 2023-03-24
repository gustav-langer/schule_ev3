package roboter;

/**
 * This class is for working with speeds. The original graphical UI for the EV3 calculated speed in rotations per minute,
 * while functions like {@link lejos.robotics.RegulatedMotor#setSpeed(int) setSpeed(int)} calculate in degrees per second.
 * This class supports both and can be used to easily convert between the units.
 */
class Speed {
    private static final int JAVA_EV3_CONVERSION_FACTOR = 6;
    private final int javaSpeed; //Geschwindigkeit in Grad/Sekunde

    private Speed(int javaSpeed) {
        this.javaSpeed = javaSpeed;
    }

    /**
     * @param value The speed in degrees per second
     * @return A {@link Speed} instance with the desired values
     */
    static Speed java(int value) {
        return new Speed(value);
    }

    /**
     * @param value The speed in rotations per minute
     * @return A {@link Speed} instance with the desired values
     */
    static Speed ev3(int value) {
        return new Speed(value * JAVA_EV3_CONVERSION_FACTOR);
    }

    /**
     * @return The speed in degrees per second
     */
    int getJava() {
        return javaSpeed;
    }

    /**
     * @return The speed in rotations per minute
     */
    int getEv3() {
        return javaSpeed / JAVA_EV3_CONVERSION_FACTOR;
    }

    /**
     * @return true iff the stored value is greater than 0
     */
    boolean isPositive() {
        return javaSpeed > 0;
    }

    /**
     * @param offset The value (in degrees per second) to offset the speed by
     * @return A new {@link Speed} with the value of the current one offset by {@code offset}
     */
    @SuppressWarnings("SameParameterValue")
    Speed javaOffset(int offset) {
        return Speed.java(this.getJava() + offset);
    }

    /**
     * @param offset The value (in rotations per minute) to offset the speed by
     * @return A new {@link Speed} with the value of the current one offset by {@code offset}
     */
    @SuppressWarnings({"SameParameterValue", "unused"})
    Speed ev3Offset(int offset) {
        return Speed.ev3(this.getEv3() + offset);
    }

    /**
     * @param factor The factor to multiply the speed by
     * @return A new {@link Speed} with the value of the current one multiplied by {@code factor}
     */
    @SuppressWarnings("SameParameterValue")
    Speed mult(int factor) {
        return Speed.java(this.getJava() * factor);
    }

    /**
     * @return A new {@link Speed} with the same absolute value as this one but a reversed direction.
     * Equivalent to {@code mult(-1);}
     */
    Speed neg() {
        return mult(-1);
    }
}
