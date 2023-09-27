package roboter;

import ev3dev.sensors.Button;
import ev3dev.utils.JarResource;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.time.Duration;
import java.util.List;

import static roboter.DanceMoves.*;

/*
Was der Roboter können muss:
-Motoren müssen benutzt werden
-Er muss sich bewegen können (Motoren benutzen)
-Etwas auf dem Bildschirm zeigen


Ideen:
-Zu Musik tanzen
 */

/*
 * Erst nur ein Bein bewegen, dann nur das andere
 * Dann die Arme bewegen
 * 0:19 2 Schritte nach vorne, 2 zurück
 */

//todo: Annotations hinkriegen

/**
 * The main class of this project.
 * Contains all the business logic.
 */
public class Main {
    private static final Logger LOGGER = LoggerFactory.getLogger(Main.class);

    /**
     * nicht in die präsentation!
     */
    private static KeyListener onKeyPressed(Runnable run) {
        return new KeyListener() {
            @Override
            public void keyPressed(Key key) {
                run.run();
            }

            @Override
            public void keyReleased(Key key) {
            }
        };
    }

    /**
     * (noch) nicht in die präsentation
     */
    public static void main(String[] args) {
        String action = "dance";
        Robot robot = new Robot(true);
        drawJavaLogo(robot);
        LOGGER.info("Roboter erzeugt!");
        while (true) {
            robot.beep();
            Button.UP.addKeyListener(onKeyPressed(robot::calibrate));
            Button.DOWN.addKeyListener(onKeyPressed(() -> {
                robot.getLeftMotor().rotate(30);
                robot.getRightMotor().rotate(-40);
            }));
            Button.ENTER.waitForPressAndRelease();
            switch (action) {
                case "dance":
                    dance(robot, 128);
                    break;
                case "demo":
                    demo(robot);
                    break;
            }
        }
    }

    @SuppressWarnings("SameParameterValue")
    static void dance(Robot robot, int bpm) {
        List<DanceMove> dance = List.of(
                WAIT_ONE_MEASURE,WAIT_ONE_MEASURE,
                FOUR_STEPS
        );
        runDance(robot, dance, "song.wav", bpm, Duration.ofSeconds(2 * 60 + 18)); //Song length: 2 minutes 18
        robot.stopAudio();
    }

    /**
     * Executes a given dance.
     *
     * @param robot             The robot executing the dance
     * @param dance             The dance, represented as a List<{@link DanceMove}>
     * @param songFileName      The file name of the song to play
     * @param bpm               The BPM of the song
     * @param ignoredSongLength The length of the song, as a {@link Duration}
     */
    @SuppressWarnings("SameParameterValue")
    static void runDance(Robot robot, List<DanceMove> dance, String songFileName, int bpm, Duration ignoredSongLength) {
        Speed baseSpeed = Speed.ev3Speed(bpm / 4); // Base speed is one rotation per measure
        robot.startPlayingFile(songFileName);
        long startTimeMillis = System.currentTimeMillis();
        dance.forEach((move) -> {
            LOGGER.debug("Now executing move " + move.toString());
            move.execute(robot, baseSpeed);
        });
    }

    /**
     * For given song metadata, calculates the speed the robot has to move at to not be too fast or slow.
     * Assumes the error between the actual pace and the expected one is at most one measure.
     *
     * @param baseSpeed        The speed that the robot should be moving at if there was no error. One rotation per measure.
     * @param startTimeMillis  The time in milliseconds where the song was started
     * @param songLengthMillis The length of the song in milliseconds
     * @return A new {@link Speed} which will bring the robot back to normal pace at the start of the next measure
     */
    static Speed syncSpeed(Speed baseSpeed, long startTimeMillis, long songLengthMillis) {
        /*float currentMeasureTime = (float) (System.currentTimeMillis() - startTimeMillis) % (msPerBeat(baseSpeed) * 4);
        // How much of the current measure has been completed
        float offset = currentMeasureTime < 0.5 ? currentMeasureTime : currentMeasureTime - 1;
        // How many milliseconds the robot is offset by from the perfect timing
        int rpm = Math.round(15000 / (msPerBeat(baseSpeed) - offset)); // The new rpm to catch up at the next measure
        return Speed.ev3(rpm);*/
        var currentTime = System.currentTimeMillis();
        var elapsedTime = currentTime - startTimeMillis;
        var remainingTime = songLengthMillis - (currentTime - startTimeMillis);
        var syncUpIn = remainingTime / msPerBeat(baseSpeed);
        double positionInMeasure = elapsedTime % (msPerBeat(baseSpeed) * 4L);
        var offset = positionInMeasure;
        if (positionInMeasure > msPerBeat(baseSpeed) * 2L) {
            offset = positionInMeasure - msPerBeat(baseSpeed) * 4L;
        }
        var offsetPerBeat = offset / syncUpIn;
        var newMsPerBeat = msPerBeat(baseSpeed) - offsetPerBeat;
        return Speed.ev3Speed((int) (15000 / newMsPerBeat));
    }

    /**
     * For a given {@link Speed} (where one motor rotation equals one measure),
     * calculates how many milliseconds each beat takes.
     *
     * @param speed The motor speed (one rotation = one measure)
     * @return The amount of milliseconds per beat
     */
    static int msPerBeat(Speed speed) {
        return 15000 / speed.getEv3Speed(); //60 seconds/minute * 1000 ms/second : 4 beats/measure = 15000
    }

    @SuppressWarnings("unused")
    static void demo(Robot robot) {
        Speed speed = Speed.ev3Speed(45);
        try {
            robot.drawImageAndRefresh(JarResource.loadImage("ev3_logo.png"), 35, 10, 0);
        } catch (IOException e) {
            LOGGER.info("Couldn't load image");
        }
        robot.move(speed, 7);
        robot.say("Hello");
        robot.rotateSingleMotor(robot.getArmsMotor(), speed, RotateAmount.rotations(7), true);
        robot.move(speed.negate(), 7);
        robot.turn(speed, RotateAmount.degrees(-90));
        robot.startPlayingFile("song.wav").whenComplete((v, e) -> {
            if ((e) != null) {
                throw new RuntimeException(e);
            } else System.exit(0);
        });
        LOGGER.info("startPlayingFile returned");
        Button.ENTER.waitForPressAndRelease();
        robot.stopAudio();
        LOGGER.info("stopped");
    }

    /*static void getMotorSpeed(Robot robot) {
        var m = robot.getArmsMotor();
        m.setSpeed(100);
        m.resetTachoCount();
        var t = Delay
    }*/

    static void manualControl(Robot robot) {
        robot.getLeftMotor().setSpeed(30 * 6);
        robot.getRightMotor().setSpeed(30 * 6);
        Button.ENTER.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                robot.getArmsMotor().forward();
            }

            @Override
            public void keyReleased(Key k) {
                robot.getArmsMotor().stop();
            }
        });
        Button.UP.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                robot.getLeftMotor().forward();
            }

            @Override
            public void keyReleased(Key k) {
                robot.getLeftMotor().stop();
            }
        });
        Button.DOWN.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                robot.getLeftMotor().backward();
            }

            @Override
            public void keyReleased(Key k) {
                robot.getLeftMotor().stop();
            }
        });
        Button.LEFT.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                robot.getRightMotor().forward();
            }

            @Override
            public void keyReleased(Key k) {
                robot.getRightMotor().stop();
            }
        });
        Button.RIGHT.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                robot.getRightMotor().backward();
            }

            @Override
            public void keyReleased(Key k) {
                robot.getRightMotor().stop();
            }
        });
        Button.ESCAPE.waitForPress();
    }

    static void drawJavaLogo(Robot robot) {
        try {
            robot.drawImageAndRefresh(JarResource.loadImage("java_logo_2.png"), 35, 10, 0);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

/*
 Keep 'Em Coming - Jules Gaia
 https://youtu.be/3bd1pJEZjvE
 BPM: 128
 Length = 2:18,75
*/