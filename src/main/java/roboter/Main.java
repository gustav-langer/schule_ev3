package roboter;

import ev3dev.actuators.lego.motors.*;
import lejos.hardware.port.*;
import lejos.robotics.*;
import lejos.utility.*;

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
        NXTRegulatedMotor linkesBein = new NXTRegulatedMotor(MotorPort.B);
        NXTRegulatedMotor rechtesBein = new NXTRegulatedMotor(MotorPort.C);
        NXTRegulatedMotor arme = new NXTRegulatedMotor(MotorPort.A);
        tanzen1(linkesBein, arme);
        tanzen1(rechtesBein, arme);
    }

    static void tanzen1(RegulatedMotor bein, RegulatedMotor arm) {
        bein.setSpeed(200);
        bein.rotate(70);
        Delay.msDelay(100);
        bein.rotate(-70);
    }
}
