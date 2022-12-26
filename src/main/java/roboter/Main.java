package roboter;

import ev3dev.actuators.*;
import ev3dev.actuators.lego.motors.*;
import lejos.hardware.lcd.*;
import lejos.hardware.port.*;

/*
Was der Roboter können muss:
-Motoren müssen benutzt werden
-Er muss sich bewegen können (Motoren benutzen)
-Etwas auf dem Bildschirm zeigen


Ideen:
-Zu Musik tanzen
 */

public class Main {
    public static void main(String[] args) {
        NXTRegulatedMotor leftLeg = new NXTRegulatedMotor(MotorPort.B);
        NXTRegulatedMotor rightLeg = new NXTRegulatedMotor(MotorPort.C);
        GraphicsLCD screen = LCD.getInstance();
        String data = "Hello World!";
        screen.drawChars(data.toCharArray(),0,data.length(),0,0,0);
    }
}
