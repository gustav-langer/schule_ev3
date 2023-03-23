package roboter;

import ev3dev.actuators.LCD;
import ev3dev.actuators.Sound;
import ev3dev.actuators.lego.motors.NXTRegulatedMotor;
import ev3dev.hardware.EV3DevPlatforms;
import ev3dev.robotics.tts.Espeak;
import ev3dev.sensors.EV3Key;
import ev3dev.sensors.ev3.EV3TouchSensor;
import ev3dev.utils.Shell;
import ev3dev.utils.Sysfs;
import lejos.hardware.Key;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.Touch;
import lejos.utility.Delay;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.awt.*;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

class Robot {
    static final boolean USE_INIT_ASYNC = false;
    private static final Logger LOGGER = LoggerFactory.getLogger(Robot.class);
    Key any = new EV3Key(EV3Key.BUTTON_ALL);
    Sound sound = Sound.getInstance();
    private RegulatedMotor rightMotor;
    private RegulatedMotor leftMotor;
    private Touch leftSensor;
    private Touch rightSensor;
    private GraphicsLCD lcd;
    private Espeak espeak = new Espeak();
    private boolean isInitiallyCalibrated = false; // Wird beim ersten Kalibrieren auf true gesetzt
    private int leftCalibratedAngle;
    private int rightCalibratedAngle;

    Robot(boolean doImmediateCalibration) {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            stop();
            stopAudio();
        })); // Roboter stoppen, wenn das Programm endet
        if (USE_INIT_ASYNC) {
            try {
                initAsync(doImmediateCalibration);
            } catch (Exception e) {
                e.printStackTrace();
                System.err.flush();
                throw new RuntimeException();
            }
        } else {
            init(doImmediateCalibration);
        }
    }

    private void initAsync(boolean doImmediateCalibration) throws ExecutionException, InterruptedException {
        EV3DevPlatforms.getInstance(); //Einmal initialisieren, damit das Asynchrone nicht kaputtgeht
        CompletableFuture<RegulatedMotor> leftFuture = CompletableFuture.<RegulatedMotor>supplyAsync(() -> new NXTRegulatedMotor(MotorPort.C))
                .exceptionally((e) -> {
                    LOGGER.info("Motor C not found");
                    // Immer wieder versuchen, den Motor zu finden, da der Roboter manchmal beim ersten oder zweiten Aufruf zu langsam ist.
                    while (true) {
                        try {
                            return new NXTRegulatedMotor(MotorPort.C);
                        } catch (RuntimeException f) {
                            LOGGER.info("Motor C not found");
                            Delay.msDelay((long) (Math.random() * 5000));
                        }
                    }
                });
        LOGGER.info("Starting left (C), sleeping");
        //Thread.sleep(10000);
        //LOGGER.info("Sleep done");
        CompletableFuture<RegulatedMotor> rightFuture = CompletableFuture.<RegulatedMotor>supplyAsync(() -> new NXTRegulatedMotor(MotorPort.B))
                .exceptionally((e) -> {
                    LOGGER.info("Motor B not found");
                    while (true) {
                        try {
                            return new NXTRegulatedMotor(MotorPort.B);
                        } catch (RuntimeException f) {
                            LOGGER.info("Motor B not found");
                            Delay.msDelay((long) (Math.random() * 5000));
                        }
                    }
                });
        LOGGER.info("Starting right (B), sleeping");
        //Thread.sleep(10000);
        //LOGGER.info("Sleep done");
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
        try {
            leftMotor = leftFuture.get(1, TimeUnit.MINUTES);
            rightMotor = rightFuture.get(1, TimeUnit.MINUTES);
        } catch (TimeoutException e) {
            LOGGER.error("Motors timed out");
            throw new RuntimeException(e);
        }
        leftSensor = leftSensorFuture.get();
        rightSensor = rightSensorFuture.get();
        CompletableFuture<Void> calibrateFuture = CompletableFuture.runAsync(() -> {
            if (doImmediateCalibration) calibrate(true);
        });
        lcd = lcdFuture.get();
        espeak = new Espeak();
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
        espeak = new Espeak();
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
            int calibrateSpeed = 270;
            leftMotor.setSpeed(calibrateSpeed);
            if (leftSensor.isPressed()) leftMotor.rotate(180);
            leftMotor.forward();
            while (!leftSensor.isPressed()) Delay.msDelay(5);
            leftMotor.stop();
            rightMotor.setSpeed(calibrateSpeed);
            if (rightSensor.isPressed()) rightMotor.rotate(180);
            rightMotor.forward();
            while (!rightSensor.isPressed()) Delay.msDelay(5);
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

    void backward(Speed s) {
        leftMotor.setSpeed(s.getJava());
        rightMotor.setSpeed(s.getJava());
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

    GraphicsLCD getLcd() {
        return this.lcd;
    }

    void drawImageAndRefresh(Image src, int x, int y, int anchor) {
        lcd.drawImage(src, x, y, anchor);
        lcd.refresh();
    }

    void say(String message) {
        espeak.setMessage(message);
        espeak.say();
    }

    CompletableFuture<Void> startPlayingFile(String path) {
        return CompletableFuture.runAsync(() -> playFile(path));
    }

    void playFile(String path) {
        Shell.execute("aplay " + path);
    }

    void stopAudio() {
        Shell.execute("killall aplay");
    }

    void beep() {
        sound.beep();
    }
}
