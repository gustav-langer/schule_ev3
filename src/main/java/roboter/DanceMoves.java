package roboter;

import lejos.utility.Delay;

/**
 * Contains all dance moves the robot can do.
 * The moves are represented as {@link DanceMove} implementations.
 */
class DanceMoves {
    static final DanceMove WAIT_1 = (robot, speed) -> Delay.msDelay(Main.msPerBeat(speed));
    static final DanceMove WAIT_4 = (robot, speed) -> Delay.msDelay(Main.msPerBeat(speed) * 4L);

    static final DanceMove SINGLE_FORWARD_STEP_2 = steps(1);
    static final DanceMove SINGLE_BACKWARD_STEP_2 = steps(-1);

    static final DanceMove SINGLE_FORWARD_STEP_4 = SINGLE_FORWARD_STEP_2.repeat(2);
    static final DanceMove SINGLE_BACKWARD_STEP_4 = SINGLE_BACKWARD_STEP_2.repeat(2);

    static final DanceMove TWO_FORWARD_STEPS_4 = steps(2);

    static final DanceMove TWO_BACKWARD_STEPS_4 = steps(-2);

    static final DanceMove FOUR_STEPS_FORWARD_4 = (robot, speed) -> robot.move(speed.mult(2), 4);

    static final DanceMove FOUR_STEPS_BACKWARD_4 = (robot, speed) -> robot.move(speed.mult(2), -4);

    static final DanceMove MOVE_ARMS_4 = (robot, speed) ->
            robot.rotateArmsMotor(speed.mult(2), RotateAmount.rotations(2));

    static final DanceMove MOVE_ARMS_ALT_4 = (robot, speed) ->
            robot.rotateArmsMotor(speed.mult(2), RotateAmount.rotations(-2));

    static final DanceMove ARMS_ON_0 = (robot, speed) -> robot.startArms(speed.mult(2));

    static final DanceMove ARMS_OFF_0 = (robot, speed) -> robot.stopArms();
    static final DanceMove MOVE_RIGHT_MOTOR_20 = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getLeftMotor(), speed, RotateAmount.rotations(-0.15F - 0.5F));
        robot.rotateSingleMotor(robot.getRightMotor(), speed.mult(2), RotateAmount.rotations(8 - 0.6F));
        robot.rotateSingleMotor(robot.getLeftMotor(), speed.negate(), RotateAmount.rotations(-0.15F - 0.5F));
    };
    static final DanceMove MOVE_LEFT_MOTOR_20 = (robot, speed) -> {
        robot.rotateSingleMotor(robot.getRightMotor(), speed, RotateAmount.rotations(-0.15F - 0.5F));
        robot.rotateSingleMotor(robot.getLeftMotor(), speed.mult(2), RotateAmount.rotations(8 - 0.6F));
        robot.rotateSingleMotor(robot.getRightMotor(), speed.negate(), RotateAmount.rotations(-0.15F - 0.5F));
    };

    private DanceMoves() {
    }

    static DanceMove steps(int count) {
        return (robot, speed) -> {
            robot.move(speed.mult(2), count);
            robot.move(speed.mult(2), count * -1);
        };
    }
}
