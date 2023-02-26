package roboter;

import ev3dev.actuators.LCD;
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
        Roboter robo = new Roboter();
        //lcd.setFont(lcd.getFont().deriveFont((float)lcd.getFont().getSize()*10));
        robo.lcd.setColor(255, 255, 255);
        robo.lcd.drawRect(0, 0, robo.lcd.getWidth(), robo.lcd.getHeight());
        robo.lcd.setColor(0);
        robo.lcd.drawString("Please wait...", 35, 10, 0);
        robo.lcd.getFont();
        //lcd.drawImage(JarResource.loadImage(JarResource.JAVA_DUKE_IMAGE_NAME), 35, 10, 0);
        robo.lcd.refresh();

        robo.calibrate();
        robo.move(45 * 6, 7);
        robo.espeak.setMessage("Hello");
        robo.espeak.say();
        robo.move(-45 * 6, 7);
        robo.turnLeft(270);
        Delay.msDelay(2000);
        robo.stop();

        //tanzen1(linkesBein, arme);
        //tanzen1(rechtesBein, arme);
    }
}

class Roboter {
    RegulatedMotor links;
    RegulatedMotor rechts;
    EV3TouchSensor linksSensor;
    EV3TouchSensor rechtsSensor;
    GraphicsLCD lcd;
    Espeak espeak;

    Roboter() {
        links = new NXTRegulatedMotor(MotorPort.C);
        rechts = new NXTRegulatedMotor(MotorPort.B);
        linksSensor = new EV3TouchSensor(SensorPort.S2);
        rechtsSensor = new EV3TouchSensor(SensorPort.S1);
        lcd = LCD.getInstance();
        espeak = new Espeak();
    }

    void calibrate() {
        links.setSpeed(270);
        links.forward();
        while (!linksSensor.isPressed()) Delay.msDelay(100);
        links.stop();
        rechts.setSpeed(270);
        rechts.forward();
        while (!rechtsSensor.isPressed()) Delay.msDelay(100);
        rechts.stop();
        links.rotate(180);
    }

    void forward(int speed) {
        links.setSpeed(speed);
        rechts.setSpeed(speed);
        links.forward();
        rechts.forward();
    }

    void backward(int speed) {
        links.setSpeed(speed);
        rechts.setSpeed(speed);
        links.backward();
        rechts.backward();
    }

    void stop() {
        links.stop(true);
        rechts.stop();
    }

    void move(int speed, int steps) {
        move(speed, steps, false);
    }

    void move(int speed, int steps, boolean immediateReturn) {
        links.setSpeed(speed);
        rechts.setSpeed(speed);
        if (speed > 0) {
            links.rotate(steps * 360, true);
            rechts.rotate(steps * 360, immediateReturn);
        } else {
            links.rotate(steps * -360, true);
            rechts.rotate(steps * -360, immediateReturn);
        }
    }

    void turnLeft(int speed) {
        links.setSpeed(speed);
        rechts.setSpeed(speed);
        links.forward();
        rechts.backward();
    }
}