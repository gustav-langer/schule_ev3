package roboter;

class DanceMoves {
    static final DanceMove ONE_STEP = (robot, speed) -> {
        robot.move(speed, 1);
        robot.move(speed, -1);
    };
}
