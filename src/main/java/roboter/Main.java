package roboter;

import ev3dev.actuators.lego.motors.*;
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
        System.out.println("Done");
    }
}
