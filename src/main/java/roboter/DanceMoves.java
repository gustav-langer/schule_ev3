package roboter;

class DanceMoves {
    static final DanceMove ONE_STEP = (r, s) -> {
        r.move(s, 1);
        r.move(s, -1);
    };
}
