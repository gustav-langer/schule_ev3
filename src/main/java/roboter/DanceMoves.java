package roboter;

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
        robot.turn(speed, 90);
        robot.turn(speed.mult(3),270);
    };
}
