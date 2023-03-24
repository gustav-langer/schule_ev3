package roboter;

class Speed {
    // Funktionen wie setSpeed() messen Geschwindigkeit in Grad/Sekunde. Der grafische Programmeditor
    // misst Geschwindigkeit in Umdrehungen/Minute, deshalb diese Klasse zum einfachen Umrechnen
    private final int javaSpeed; //Geschwindigkeit in Grad/Sekunde

    private Speed(int javaSpeed) {
        this.javaSpeed = javaSpeed;
    }

    static Speed java(int value) {
        return new Speed(value);
    }

    static Speed ev3(int value) {
        return new Speed(value * 6);
    }

    int getJava() {
        return javaSpeed;
    }

    int getEv3() {
        return javaSpeed / 6;
    }

    boolean isPositive() {
        return javaSpeed > 0;
    }

    @SuppressWarnings("SameParameterValue")
    Speed offset(int offset) {
        return Speed.java(this.javaSpeed + offset);
    }

    @SuppressWarnings("SameParameterValue")
    Speed mult(int factor) {
        return Speed.java(this.javaSpeed * factor);
    }

    Speed neg() {
        return mult(-1);
    }
}
