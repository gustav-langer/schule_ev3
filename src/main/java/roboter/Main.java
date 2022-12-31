package roboter;

import ev3dev.actuators.lego.motors.*;
import ev3dev.robotics.tts.*;
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
        //NXTRegulatedMotor leftLeg = new NXTRegulatedMotor(MotorPort.B);
        //NXTRegulatedMotor rightLeg = new NXTRegulatedMotor(MotorPort.C);
        Espeak espeak = new Espeak();
        espeak.setVoice(Espeak.VOICE_ENGLISH);
        espeak.setMessage("Hello World!");
        espeak.say();
    }
}
