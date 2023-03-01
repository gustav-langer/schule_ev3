package roboter;

import ev3dev.actuators.lego.motors.NXTRegulatedMotor;
import ev3dev.robotics.tts.Espeak;
import ev3dev.sensors.ev3.EV3TouchSensor;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

import java.io.IOException;

/*
Was der Roboter können muss:
-Motoren müssen benutzt werden
-Er muss sich bewegen können (Motoren benutzen)
-Etwas auf dem Bildschirm zeigen


Ideen:
-Zu Musik tanzen
 */

public class Main {
    public static void main(String[] args) throws IOException {
        Robot robot = new Robot();
        /*//lcd.setFont(lcd.getFont().deriveFont((float)lcd.getFont().getSize()*10));
        robot.lcd.setColor(255, 255, 255);
        robot.lcd.drawRect(0, 0, robot.lcd.getWidth(), robot.lcd.getHeight());
        robot.lcd.setColor(0);
        robot.lcd.drawString("Please wait...", 35, 10, 0);
        robot.lcd.getFont();
        //lcd.drawImage(JarResource.loadImage(JarResource.JAVA_DUKE_IMAGE_NAME), 35, 10, 0);
        robot.lcd.refresh();*/

        robot.left.rotate(90);
        robot.right.rotate(90);
        robot.calibrate();
        /*robot.move(45 * 6, 7);
        robot.espeak.setMessage("Hello");
        robot.espeak.say();
        robot.move(-45 * 6, 7);
        robot.turnLeft(270);
        Delay.msDelay(2000);
        robot.stop();*/
        //for (int i = 0; i < 4; i++) {
        //    robot.move(45 * 6, 3);
        Delay.msDelay(2000);
        robot.turn(50 * 6, 90);
        //}

        //tanzen1(linkesBein, arme);
        //tanzen1(rechtesBein, arme);
    }
}

class Robot {
    RegulatedMotor left;
    RegulatedMotor right;
    EV3TouchSensor leftSensor;
    EV3TouchSensor rightSensor;
    GraphicsLCD lcd;
    Espeak espeak;

    Boolean isCalibrated;

    Robot() {
        left = new NXTRegulatedMotor(MotorPort.C);
        right = new NXTRegulatedMotor(MotorPort.B);
        leftSensor = new EV3TouchSensor(SensorPort.S2);
        rightSensor = new EV3TouchSensor(SensorPort.S1);
        isCalibrated = false;
        //lcd = LCD.getInstance();
        //espeak = new Espeak();
    }

    void calibrate() {
        calibrate(true);
    }

    void calibrate(Boolean extraTurnLeft) {
        if (!isCalibrated) {
            left.setSpeed(270);
            left.forward();
            while (!leftSensor.isPressed()) Delay.msDelay(100);
            left.stop();
            right.setSpeed(270);
            right.forward();
            while (!rightSensor.isPressed()) Delay.msDelay(100);
            right.stop();
            if (extraTurnLeft) left.rotate(180);
            else right.rotate(180);
            isCalibrated = true;
        }
    }

    void forward(int speed) {
        left.setSpeed(speed);
        right.setSpeed(speed);
        left.forward();
        right.forward();
    }

    void backward(int speed) {
        left.setSpeed(speed);
        right.setSpeed(speed);
        left.backward();
        right.backward();
    }

    void stop() {
        left.stop(true);
        right.stop();
    }

    void move(int speed, int steps) {
        move(speed, steps, false);
    }

    void move(int speed, int steps, boolean immediateReturn) {
        left.setSpeed(speed);
        right.setSpeed(speed);
        if (speed > 0) {
            left.rotate(steps * 360, true);
            right.rotate(steps * 360, immediateReturn);
        } else {
            left.rotate(steps * -360, true);
            right.rotate(steps * -360, immediateReturn);
        }
    }

    void turn(int speed, int degrees) {
        if (degrees < 0) {
            calibrate(false);
            right.rotate(36);
            left.setSpeed(speed);
            left.rotate(40 * degrees);
        } else {
            calibrate();
            left.rotate(36);
            right.setSpeed(speed);
            right.rotate(60 * degrees);
        }
    }
}