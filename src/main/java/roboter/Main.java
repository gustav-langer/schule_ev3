package roboter;

import ev3dev.utils.JarResource;
import lejos.utility.Delay;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.List;

import static roboter.DanceMoves.ONE_STEP;

/*
Was der Roboter können muss:
-Motoren müssen benutzt werden
-Er muss sich bewegen können (Motoren benutzen)
-Etwas auf dem Bildschirm zeigen


Ideen:
-Zu Musik tanzen
 */

//todo: Annotations hinkriegen

/**
 * The main class of this project.
 * Contains all the business logic.
 */
public class Main {
    private static final Logger LOGGER = LoggerFactory.getLogger(Main.class);

    public static void main(String[] args) {
        Robot robot = new Robot(true);
        LOGGER.info("Roboter erzeugt!");
        robot.beep();
        robot.any.waitForPressAndRelease();
        dance(robot, 120);
    }

    @SuppressWarnings("SameParameterValue")
    static void dance(Robot robot, int bpm) {
        var msPerBeat = 60000 / bpm;
        Speed baseSpeed = Speed.ev3(bpm / 4);// baseSpeed ~= 1 Umdrehung/Takt
        List<DanceMove> dance = List.of(ONE_STEP,
                ONE_STEP,
                ONE_STEP,
                ONE_STEP,
                ONE_STEP);
        try {
            robot.getLcd().drawImage(JarResource.loadImage("java_logo_2.png"), 35, 10, 0);
            robot.getLcd().refresh();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.startPlayingFile("song.wav");
        Delay.msDelay(Math.round(msPerBeat * 16));
        runDance(robot, baseSpeed, dance);
        robot.stopAudio();
    }

    /**
     * Executes a given dance.
     *
     * @param robot The robot executing the dance
     * @param speed The optimal speed (in most cases, the BPM of the song)
     * @param dance The dance, represented as a List of {@link DanceMove}
     */
    static void runDance(Robot robot, Speed speed, List<DanceMove> dance) {
        dance.forEach((move) -> move.execute(robot, speed));
    }

    @SuppressWarnings("unused")
    static void demo(Robot robot) {
        Speed baseSpeed = Speed.ev3(45);
        try {
            robot.drawImageAndRefresh(JarResource.loadImage("ev3_logo.png"), 35, 10, 0);
        } catch (IOException e) {
            LOGGER.info("Couldn't load image");
        }
        robot.move(baseSpeed, 7);
        robot.say("Hello");
        robot.move(baseSpeed.neg(), 7);
        robot.turn(baseSpeed.javaOffset(5), -90);
        robot.startPlayingFile("song.wav").whenComplete((v, e) -> {
            if ((e) != null) {
                throw new RuntimeException(e);
            } else System.exit(0);
        });
        LOGGER.info("startPlayingFile returned");
        robot.any.waitForPressAndRelease();
        robot.stopAudio();
        LOGGER.info("stopped");
    }
}

/*
 Keep 'Em Coming - Jules Gaia
 https://youtu.be/3bd1pJEZjvE
 BPM ~ 137,6
*/