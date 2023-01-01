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

    static void spagat(RegulatedMotor bein1, RegulatedMotor bein2) {
        bein1.setSpeed(200);
        bein2.setSpeed(200);
        bein1.rotate(90,true);
        bein2.rotate(-90,true);
        bein1.waitComplete();
        bein2.waitComplete();
    }
}

/*
import lejos.nxt.*;

public class Spagatt {

  public static void main(String[] args) {
    Motor.A.setSpeed(900);
    Motor.C.setSpeed(900);

    Motor.A.forward();
    Motor.C.backward();

    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // Nichts tun
    }

    Motor.A.stop();
    Motor.C.stop();
  }

}



 */