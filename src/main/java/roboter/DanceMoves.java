package roboter;

/**
 * Contains all dance moves the robot can do.
 * The moves are represented as {@link DanceMove} implementations.
 */
class DanceMoves {
    static final DanceMove ONE_STEP = (robot, speed) -> {
        robot.move(speed, 1);
        robot.move(speed, -1);
    };
}
