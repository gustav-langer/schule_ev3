package roboter;

import ev3dev.actuators.LCD;
import ev3dev.actuators.Sound;
import ev3dev.actuators.lego.motors.NXTRegulatedMotor;
import ev3dev.robotics.tts.Espeak;
import ev3dev.sensors.ev3.EV3TouchSensor;
import ev3dev.utils.JarResource;
import ev3dev.utils.Sysfs;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.Touch;
import lejos.utility.Delay;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

/*
Was der Roboter können muss:
-Motoren müssen benutzt werden
-Er muss sich bewegen können (Motoren benutzen)
-Etwas auf dem Bildschirm zeigen


Ideen:
-Zu Musik tanzen
 */

public class Main {

    private static final Logger LOGGER = LoggerFactory.getLogger(Main.class);

    public static void main(String[] args) {
        Speed speed = Speed.ev3(45);
        Robot robot = new Robot(true);
        LOGGER.info("Roboter erzeugt!");
        try {
            robot.lcd.drawImage(JarResource.loadImage(JarResource.JAVA_DUKE_IMAGE_NAME), 35, 10, 0);
            robot.lcd.refresh();
        } catch (IOException e) {
            //couldn't load image
        }
        robot.move(Speed.ev3(45), 7);
        robot.espeak.setMessage("Hello");
        robot.espeak.say();
        robot.move(Speed.ev3(-45), 7);
        robot.turn(Speed.ev3(50), -90);
        robot.sound.playSample(new File("song.wav"));
    }
}

class Robot {

    private static final Logger LOGGER = LoggerFactory.getLogger(Robot.class);
    RegulatedMotor rightMotor;
    Touch leftSensor;
    Touch rightSensor;
    GraphicsLCD lcd;
    Espeak espeak = new Espeak();
    Sound sound;
    boolean isInitiallyCalibrated = false;
    int leftCalibratedAngle;
    int rightCalibratedAngle;
    private RegulatedMotor leftMotor;

    Robot(boolean doImmediateCalibration) {
        Runtime.getRuntime().addShutdownHook(new Thread(this::stop));
        if (true) { //true für asynchron, false für synchron
            try {
                initAsync(doImmediateCalibration);
            } catch (Exception e) {
                e.printStackTrace();
                System.err.flush();
                System.exit(-2);
            }
        } else {
            init(doImmediateCalibration);
        }
    }

    private void initAsync(boolean doImmediateCalibration) throws ExecutionException, InterruptedException {
        List<File> motoren = Sysfs.getElements("/sys/class/tacho-motor");
        LOGGER.info(motoren.toString());
        for (File motor : motoren) {
            LOGGER.info(Sysfs.getElements(motor.getPath()).toString());
        }
        CompletableFuture<RegulatedMotor> leftFuture = CompletableFuture.<RegulatedMotor>supplyAsync(
                        () -> new NXTRegulatedMotor(MotorPort.C))
                .exceptionally((e) -> {
                    LOGGER.info("Motor C not found");
                    // Immer wieder versuchen, den Motor zu finden, da der Roboter manchmal beim ersten oder zweiten Aufruf zu langsam ist.
                    while (true) {
                        try {
                            return new NXTRegulatedMotor(MotorPort.C);
                        } catch (RuntimeException f) {
                            LOGGER.info("Motor C not found");
                        }
                    }
                });
        CompletableFuture<RegulatedMotor> rightFuture = CompletableFuture.<RegulatedMotor>supplyAsync(
                        () -> new NXTRegulatedMotor(MotorPort.B))
                .exceptionally((e) -> {
                    LOGGER.info("Motor B not found");
                    while (true) {
                        try {
                            return new NXTRegulatedMotor(MotorPort.B);
                        } catch (RuntimeException f) {
                            LOGGER.info("Motor B not found");
                        }
                    }
                });
        CompletableFuture<Touch> leftSensorFuture = CompletableFuture.<Touch>supplyAsync(() -> new EV3TouchSensor(SensorPort.S2))
                .exceptionally((e) -> {
                    LOGGER.info("Sensor S2 not found");
                    // Wenn das passiert, wurde der Sensor nicht erkannt. Mit Sysfs.writeString kann man den Sensormodus manuell setzen
                    Sysfs.writeString("/sys/class/lego-port/port1/set_device", "lego-nxt-touch");
                    return new EV3TouchSensor(SensorPort.S2);
                });
        CompletableFuture<Touch> rightSensorFuture = CompletableFuture.<Touch>supplyAsync(() -> new EV3TouchSensor(SensorPort.S1))
                .exceptionally((e) -> {
                    LOGGER.info("Sensor S1 not found");
                    Sysfs.writeString("/sys/class/lego-port/port0/set_device", "lego-nxt-touch");
                    return new EV3TouchSensor(SensorPort.S1);
                });
        CompletableFuture<GraphicsLCD> lcdFuture = CompletableFuture.supplyAsync(LCD::getInstance);
        CompletableFuture<Sound> soundFuture = CompletableFuture.supplyAsync(Sound::getInstance);

        leftMotor = leftFuture.get();
        rightMotor = rightFuture.get();
        leftSensor = leftSensorFuture.get();
        rightSensor = rightSensorFuture.get();
        CompletableFuture<Void> calibrateFuture = CompletableFuture.runAsync(() -> {
            if (doImmediateCalibration) calibrate(true);
        });
        lcd = lcdFuture.get();
        sound = soundFuture.get();
        calibrateFuture.get(); // Warten, bis das Kalibrieren abgeschlossen ist
    }

    private void init(boolean doImmediateCalibration) {
        leftMotor = new NXTRegulatedMotor(MotorPort.C);
        rightMotor = new NXTRegulatedMotor(MotorPort.B);
        try {
            leftSensor = new EV3TouchSensor(SensorPort.S2);
            rightSensor = new EV3TouchSensor(SensorPort.S1);
        } catch (RuntimeException e) {
            Sysfs.writeString("/sys/class/lego-port/port0/set_device", "lego-nxt-touch");
            Sysfs.writeString("/sys/class/lego-port/port1/set_device", "lego-nxt-touch");
            leftSensor = new EV3TouchSensor(SensorPort.S2);
            rightSensor = new EV3TouchSensor(SensorPort.S1);
        }
        lcd = LCD.getInstance();
        sound = Sound.getInstance();
        if (doImmediateCalibration) {
            calibrate(true);
        }
    }

    void calibrate() {
        calibrate(false);
    }

    void calibrate(boolean forceCalibrate) {
        if (forceCalibrate || !isInitiallyCalibrated
                || (leftCalibratedAngle != leftMotor.getTachoCount() % 360)
                || (rightCalibratedAngle != rightMotor.getTachoCount() % 360)) {
            leftMotor.setSpeed(270);
            if (leftSensor.isPressed()) leftMotor.rotate(180);
            leftMotor.forward();
            while (!leftSensor.isPressed()) Delay.msDelay(10);
            leftMotor.stop();
            rightMotor.setSpeed(270);
            if (rightSensor.isPressed()) rightMotor.rotate(180);
            rightMotor.forward();
            while (!rightSensor.isPressed()) Delay.msDelay(10);
            rightMotor.stop();
            leftMotor.rotate(180);
            leftCalibratedAngle = leftMotor.getTachoCount() % 360;
            rightCalibratedAngle = rightMotor.getTachoCount() % 360;
            isInitiallyCalibrated = true;
        }
    }

    void forward(Speed s) {
        leftMotor.setSpeed(s.getJava());
        rightMotor.setSpeed(s.getJava());
        leftMotor.forward();
        rightMotor.forward();
    }

    void backward(Speed speed) {
        leftMotor.setSpeed(speed.getJava());
        rightMotor.setSpeed(speed.getJava());
        leftMotor.backward();
        rightMotor.backward();
    }

    void stop() {
        leftMotor.stop(true);
        rightMotor.stop();
    }

    void move(Speed s, int steps) {
        move(s, steps, false);
    }

    void move(Speed s, int steps, boolean immediateReturn) {
        leftMotor.setSpeed(s.getJava());
        rightMotor.setSpeed(s.getJava());
        if (s.isPositive()) {
            leftMotor.rotate(steps * 360, true);
            rightMotor.rotate(steps * 360, immediateReturn);
        } else {
            leftMotor.rotate(steps * -360, true);
            rightMotor.rotate(steps * -360, immediateReturn);
        }
    }

    void turn(Speed s, int degrees) {
        // Die Motoren müssen sich um degrees * rotationFactor Grad drehen, damit sich der Roboter um degrees dreht
        //Herausgefunden durch Trial-and-Error
        final int rotationFactor = 60;
        if (degrees < 0) {
            calibrate();
            leftMotor.rotate(180);
            rightMotor.rotate(36 + 180);
            leftMotor.setSpeed(s.getJava());
            leftMotor.rotate(rotationFactor * degrees);
        } else {
            calibrate();
            leftMotor.rotate(36);
            rightMotor.setSpeed(s.getJava());
            rightMotor.rotate(rotationFactor * degrees);
        }
    }
}

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
}

/*
 Keep 'Em Coming - Jules Gaia
 https://youtu.be/3bd1pJEZjvE
 BPM ~ 137,6
*/