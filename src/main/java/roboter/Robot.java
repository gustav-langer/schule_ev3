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

/**
 * Represents the robot.
 * All the low-level code like accessing the hardware and basic movement is here.
 */
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
    private int leftCalibratedAngle; // In diesem Winkel ist der linke Fuß kalibriert. Kann unnötiges Kalibrieren vermeiden
    private int rightCalibratedAngle; // Dasselbe wie leftCalibratedAngle

    Robot(boolean doImmediateCalibration) {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            stop();
            stopAudio();
        })); // Roboter stoppen, wenn das Programm endet
        if (USE_INIT_ASYNC) {
            try {
                initAsync(doImmediateCalibration);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        } else {
            init(doImmediateCalibration);
        }
    }

    /**
     * Initializes this robot's hardware and optionally calibrates the motors.
     * Uses an asynchronous approach.
     *
     * @param doImmediateCalibration If the robot should be immediately calibrated
     * @throws ExecutionException   if any hardware piece encountered an error while initializing
     * @throws InterruptedException if one of the asynchronous threads was interrupted
     */
    private void initAsync(boolean doImmediateCalibration) throws ExecutionException, InterruptedException {
        EV3DevPlatforms.getInstance(); //Einmal initialisieren, weil EV3DevPlatforms nicht thread-sicher ist
        CompletableFuture<RegulatedMotor> leftFuture = CompletableFuture.<RegulatedMotor>
                        supplyAsync(() -> new NXTRegulatedMotor(MotorPort.C))
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
        CompletableFuture<RegulatedMotor> rightFuture = CompletableFuture.<RegulatedMotor>
                        supplyAsync(() -> new NXTRegulatedMotor(MotorPort.B))
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
        CompletableFuture<Touch> leftSensorFuture = CompletableFuture.<Touch>
                        supplyAsync(() -> new EV3TouchSensor(SensorPort.S2))
                .exceptionally((e) -> {
                    LOGGER.info("Sensor S2 not found");
                    // Wenn das passiert, wurde der Sensor nicht erkannt. Mit Sysfs.writeString kann man den Sensormodus manuell setzen
                    Sysfs.writeString("/sys/class/lego-port/port1/set_device", "lego-nxt-touch");
                    return new EV3TouchSensor(SensorPort.S2);
                });
        CompletableFuture<Touch> rightSensorFuture = CompletableFuture.<Touch>
                        supplyAsync(() -> new EV3TouchSensor(SensorPort.S1))
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
        calibrateFuture.get(); // Warten, bis das Kalibrieren abgeschlossen ist
    }

    /**
     * Initializes this robot's hardware and optionally calibrates the motors.
     * Uses synchronous approach.
     *
     * @param doImmediateCalibration If the robot should be immediately calibrated
     */
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
        if (doImmediateCalibration) {
            calibrate(true);
        }
    }

    void calibrate() {
        calibrate(false);
    }

    void calibrate(boolean forceCalibrate) { //FIXME
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

    /**
     * Moves the robot forward at a constant speed, assuming it is calibrated.
     *
     * @param s The speed to move at
     */
    @SuppressWarnings("unused")
    void forward(Speed s) {
        leftMotor.setSpeed(s.getJava());
        rightMotor.setSpeed(s.getJava());
        leftMotor.forward();
        rightMotor.forward();
    }

    /**
     * Moves the robot backward at a constant speed, assuming it is calibrated.
     *
     * @param s The speed to move at
     */
    @SuppressWarnings("unused")
    void backward(Speed s) {
        leftMotor.setSpeed(s.getJava());
        rightMotor.setSpeed(s.getJava());
        leftMotor.backward();
        rightMotor.backward();
    }

    /**
     * Stops the robot from moving any further.
     */
    void stop() {
        leftMotor.stop(true);
        rightMotor.stop();
    }

    /**
     * Moves the robot a fixed amount of steps.
     *
     * @param s     The speed to move at
     * @param steps How much to move by
     */
    void move(Speed s, int steps) {
        move(s, steps, false);
    }

    /**
     * Moves the robot a fixed amount of steps. Also supports not waiting for the motors to finish turning.
     *
     * @param s               The speed to move at
     * @param steps           How much to move by
     * @param immediateReturn Iff true, method returns immediately
     */
    @SuppressWarnings("SameParameterValue")
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

    /**
     * Turns the robot.
     *
     * @param s       The speed to turn at
     * @param degrees How many degrees to turn by, counted clockwise
     */
    @SuppressWarnings("SameParameterValue")
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

    /**
     * Draws an image on the screen.
     * Equivalent to {@code getLcd().drawImage(...);
     * getLcd().refresh();}.
     *
     * @param src    Image to draw
     * @param x      Destination
     * @param y      Destination
     * @param anchor Location of the anchor point
     * @see GraphicsLCD#drawImage(Image, int, int, int)
     */
    @SuppressWarnings("SameParameterValue")
    void drawImageAndRefresh(Image src, int x, int y, int anchor) {
        lcd.drawImage(src, x, y, anchor);
        lcd.refresh();
    }

    /**
     * Says a message using Espeak.
     *
     * @param message The message
     * @see Espeak
     */
    @SuppressWarnings("SameParameterValue")
    void say(String message) {
        espeak.setMessage(message);
        espeak.say();
    }

    /**
     * Starts playing an audio file. Returns immediately to allow for other things while the song plays.
     * <em>WARNING! Not protected against code injections through </em>{@code path}!
     *
     * @param path The path to the audio file
     * @return A CompletableFuture playing the file. May be discarded without interrupting the song.
     */
    @SuppressWarnings("SameParameterValue")
    CompletableFuture<Void> startPlayingFile(String path) {
        return CompletableFuture.runAsync(() -> playFile(path));
    }

    /**
     * Plays an audio file. Waits for it to be finished before returning.
     * <em>WARNING! Not protected against code injections through </em>{@code path}!
     *
     * @param path The path to the audio file
     */
    void playFile(String path) {
        Shell.execute("aplay " + path);
    }

    /**
     * Stops all sounds.
     */
    void stopAudio() {
        Shell.execute("killall aplay");
    }

    /**
     * Beeps once.
     */
    void beep() {
        sound.beep();
    }
}
