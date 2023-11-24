package roboter;

import ev3dev.actuators.LCD;
import ev3dev.actuators.Sound;
import ev3dev.actuators.lego.motors.NXTRegulatedMotor;
import ev3dev.hardware.EV3DevPlatforms;
import ev3dev.robotics.tts.Espeak;
import ev3dev.sensors.ev3.EV3TouchSensor;
import ev3dev.utils.Shell;
import ev3dev.utils.Sysfs;
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
public class Robot {
    static final boolean USE_INIT_ASYNC = false;
    private static final Logger LOGGER = LoggerFactory.getLogger(Robot.class);
    private RegulatedMotor leftMotor;
    private RegulatedMotor rightMotor;
    private RegulatedMotor armsMotor;
    private Touch leftSensor;
    private Touch rightSensor;
    private GraphicsLCD lcd;
    private Sound sound = Sound.getInstance();
    private Buttons buttons;
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
        CompletableFuture<RegulatedMotor> leftFuture = CompletableFuture.supplyAsync(() -> {
            // Immer wieder versuchen, den Motor zu finden, da der Roboter manchmal beim ersten oder zweiten Aufruf zu langsam ist.
            while (true) try {
                return new NXTRegulatedMotor(MotorPort.C);
            } catch (RuntimeException f) {
                LOGGER.info("Motor C not found");
                Delay.msDelay((long) (Math.random() * 5000));
            }
        });
        CompletableFuture<RegulatedMotor> rightFuture = CompletableFuture.supplyAsync(() -> {
            while (true) {
                try {
                    return new NXTRegulatedMotor(MotorPort.B);
                } catch (RuntimeException f) {
                    LOGGER.info("Motor B not found");
                    Delay.msDelay((long) (Math.random() * 5000));
                }
            }
        });
        CompletableFuture<Touch> leftSensorFuture = CompletableFuture.
                <Touch>supplyAsync(() -> new EV3TouchSensor(SensorPort.S2))
                .exceptionally((e) -> {
                    LOGGER.info("Sensor S2 not found");
                    // Wenn das passiert, wurde der Sensor nicht erkannt. Mit Sysfs.writeString kann man den Sensormodus manuell setzen
                    Sysfs.writeString("/sys/class/lego-port/port1/set_device", "lego-nxt-touch");
                    return new EV3TouchSensor(SensorPort.S2);
                });
        CompletableFuture<Touch> rightSensorFuture = CompletableFuture.
                <Touch>supplyAsync(() -> new EV3TouchSensor(SensorPort.S1))
                .exceptionally((e) -> {
                    LOGGER.info("Sensor S1 not found");
                    Sysfs.writeString("/sys/class/lego-port/port0/set_device", "lego-nxt-touch");
                    return new EV3TouchSensor(SensorPort.S1);
                });
        CompletableFuture<GraphicsLCD> lcdFuture = CompletableFuture.supplyAsync(LCD::getInstance);
        CompletableFuture<Sound> soundFuture = CompletableFuture.supplyAsync(Sound::getInstance);
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
        sound = soundFuture.get();
        espeak = new Espeak();
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
        armsMotor = new NXTRegulatedMotor(MotorPort.A);
        try {
            leftSensor = new EV3TouchSensor(SensorPort.S2);
            rightSensor = new EV3TouchSensor(SensorPort.S1);
        } catch (RuntimeException e) {
            Sysfs.writeString("/sys/class/lego-port/port0/set_device", "lego-nxt-touch");
            Sysfs.writeString("/sys/class/lego-port/port1/set_device", "lego-nxt-touch");
            leftSensor = new EV3TouchSensor(SensorPort.S2); // Possible error intentionally not caught
            rightSensor = new EV3TouchSensor(SensorPort.S1);
        }
        lcd = LCD.getInstance();
        sound = Sound.getInstance();
        espeak = new Espeak();
        buttons = new Buttons();
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
            calibrateSingleMotor(leftMotor, leftSensor);
            calibrateSingleMotor(rightMotor, rightSensor);
            leftMotor.rotate(180);
            leftCalibratedAngle = leftMotor.getTachoCount() % 360;
            rightCalibratedAngle = rightMotor.getTachoCount() % 360;
            isInitiallyCalibrated = true;
        }
    }

    private void calibrateSingleMotor(RegulatedMotor motor, Touch sensor) {
        int calibrateSpeed = 140; // TODO was originally 270
        motor.setSpeed(calibrateSpeed);
        if (sensor.isPressed()) {
            //beep();
            motor.rotate(180);
            //Delay.msDelay(1000);
        }
        //beep();
        motor.forward();
        while (!sensor.isPressed()) {
            Delay.msDelay(50);
        }
        motor.stop();
        //Delay.msDelay(1000);
        //beep();
        //beep();
        //Delay.msDelay(1000);
    }

    /**
     * Moves the robot forward at a constant speed, assuming it is calibrated.
     *
     * @param s The speed to move at
     */
    void forward(Speed s) {
        leftMotor.setSpeed(s.getJavaSpeed());
        rightMotor.setSpeed(s.getJavaSpeed());
        leftMotor.forward();
        rightMotor.forward();
    }

    /**
     * Moves the robot backward at a constant speed, assuming it is calibrated.
     *
     * @param s The speed to move at
     */
    void backward(Speed s) {
        leftMotor.setSpeed(s.getJavaSpeed());
        rightMotor.setSpeed(s.getJavaSpeed());
        leftMotor.backward();
        rightMotor.backward();
    }

    /**
     * Stops the robot from moving any further.
     */
    void stop() {
        leftMotor.stop(true);
        rightMotor.stop(true);
        armsMotor.stop();
    }

    public void rotateSingleMotor(RegulatedMotor motor, Speed speed, RotateAmount amount, boolean immediateReturn) {
        int degrees = amount.getDegrees() * speed.signum();
        motor.setSpeed(speed.getJavaSpeed());
        motor.rotate(degrees, immediateReturn);
    }

    public void rotateSingleMotor(RegulatedMotor motor, Speed speed, RotateAmount amount) {
        rotateSingleMotor(motor, speed, amount, false);
    }

    public void rotateLeftMotor(Speed speed, RotateAmount amount) {
        rotateSingleMotor(leftMotor, speed, amount);
    }

    public void rotateRightMotor(Speed speed, RotateAmount amount) {
        rotateSingleMotor(rightMotor, speed, amount);
    }

    public void rotateArmsMotor(Speed speed, RotateAmount amount) {
        rotateSingleMotor(armsMotor, speed, amount);
    }

    public void startArms(Speed speed) {
        armsMotor.setSpeed(speed.getJavaSpeed());
        armsMotor.forward();
    }

    public void stopArms() {
        armsMotor.stop();
    }

    /**
     * Moves the robot a fixed amount of steps. Also supports not waiting for the motors to finish turning.
     *
     * @param speed           The speed to move at
     * @param steps           How much to move by
     * @param immediateReturn Iff true, method returns immediately
     */
    void move(Speed speed, int steps, boolean immediateReturn) {
        RotateAmount amount = RotateAmount.rotations((float) steps / 2);
        rotateSingleMotor(leftMotor, speed, amount, true);
        rotateSingleMotor(rightMotor, speed, amount, immediateReturn);
    }

    /**
     * Moves the robot a fixed amount of steps.
     *
     * @param speed The speed to move at
     * @param steps How much to move by
     */
    void move(Speed speed, int steps) {
        move(speed, steps, false);
    }

    /**
     * Turns the robot.
     *
     * @param turningSpeed How fast the <b>robot</b> turns (not the motors)
     * @param amount       How much to turn by, counted clockwise
     */
    void turn(Speed turningSpeed, RotateAmount amount) {
        // Die Motoren müssen sich um degrees * rotationFactor Grad drehen, damit sich der Roboter um degrees dreht
        // Herausgefunden durch Trial-and-Error
        int rotationFactor = 45;
        Speed actualSpeed = turningSpeed.mult(rotationFactor);
        RotateAmount actualAmount = amount.mult(rotationFactor);
        if (amount.isPositive()) {
            rotateSingleMotor(leftMotor, actualSpeed, RotateAmount.degrees(36));
            rotateSingleMotor(rightMotor, actualSpeed, actualAmount);
            rotateSingleMotor(leftMotor, actualSpeed, RotateAmount.degrees(-36));
        } else {
            rotateSingleMotor(leftMotor, actualSpeed, RotateAmount.degrees(-180)); // Reverse 180° from calibrate()
            rotateSingleMotor(rightMotor, actualSpeed, RotateAmount.degrees(180 + 36));
            rotateSingleMotor(leftMotor, actualSpeed, actualAmount);
            rotateSingleMotor(leftMotor, actualSpeed, RotateAmount.degrees(180)); // Revert everything from before
            rotateSingleMotor(rightMotor, actualSpeed, RotateAmount.degrees(-180 - 36));
        }
    }

    public RegulatedMotor getRightMotor() {
        return rightMotor;
    }

    public RegulatedMotor getLeftMotor() {
        return leftMotor;
    }

    public RegulatedMotor getArmsMotor() {
        return armsMotor;
    }

    public Buttons getButtons() {
        return buttons;
    }

    public GraphicsLCD getLcd() {
        return lcd;
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
    void say(String message) {
        espeak.setMessage(message);
        espeak.say();
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
     * Starts playing an audio file. Returns immediately to allow for other things while the song plays.
     * <em>WARNING! Not protected against code injections through </em>{@code path}!
     *
     * @param path The path to the audio file
     * @return A CompletableFuture for further manipulation.
     */
    CompletableFuture<Void> startPlayingFile(String path) {
        return CompletableFuture.runAsync(() -> playFile(path));
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
