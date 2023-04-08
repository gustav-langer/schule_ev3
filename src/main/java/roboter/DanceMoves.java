package roboter;

import lejos.utility.Delay;

/**
 * Contains all dance moves the robot can do.
 * The moves are represented as {@link DanceMove} implementations.
 */
class DanceMoves {
    static final DanceMove EXAMPLE1 = (robot, speed) -> {
        robot.move(speed, 1);
        robot.move(speed, -1);
    };
    static final DanceMove EXAMPLE2 = (robot, speed) -> {
        robot.turn(speed, RotateAmount.degrees(90));
        robot.turn(speed.mult(3), RotateAmount.degrees(270));
    };

    static final DanceMove WAIT_ONE_BEAT = (robot, speed) -> Delay.msDelay(Main.msPerBeat(speed));

    static final DanceMove WAIT_ONE_MEASURE = (robot, speed) -> Delay.msDelay(Main.msPerBeat(speed) * 4L);

    static final DanceMove FOUR_STEPS = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getArmsMotor(), speed.mult(1), RotateAmount.rotations(2), true);
        robot.move(speed.mult(2), 4);
        robot.move(speed.mult(2), -4);
    };

    static final DanceMove MOVE_RIGHT_MOTOR = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getLeftMotor(),speed,RotateAmount.rotations(-0.15F-0.5F));
        robot.rotateSingleMotor(robot.getRightMotor(),speed.mult(2),RotateAmount.rotations(8-0.6F));
        robot.rotateSingleMotor(robot.getLeftMotor(),speed.neg(),RotateAmount.rotations(-0.15F-0.5F));
    };

    static final DanceMove MOVE_LEFT_MOTOR = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getRightMotor(), speed, RotateAmount.rotations(-0.15F - 0.5F));
        robot.rotateSingleMotor(robot.getLeftMotor(), speed.mult(2), RotateAmount.rotations(8 - 0.6F));
        robot.rotateSingleMotor(robot.getRightMotor(), speed.neg(), RotateAmount.rotations(-0.15F - 0.5F));
    };
    static final DanceMove MOVE_ARMS = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getArmsMotor(),speed,RotateAmount.rotations(4));
    };
}
