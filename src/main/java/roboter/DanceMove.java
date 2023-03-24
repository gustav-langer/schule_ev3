package roboter;

/**
 * Represents the different moves the robot can do
 *
 * @see DanceMoves
 */
interface DanceMove {
    /**
     * Executes the move.
     *
     * @param robot The robot to execute the move on
     * @param speed The speed at which the move should be executed
     */
    void execute(Robot robot, Speed speed);
}
