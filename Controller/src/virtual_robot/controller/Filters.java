package virtual_robot.controller;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.collision.Filter;

public class Filters {

    //Masks

    public static long MASK_ALL = -1;  // Binary representation of -1 is all 1s

    //Categories

    public static long WALL = 0b1;

    public static final CategoryFilter WALL_FILTER = new CategoryFilter(WALL, MASK_ALL);
}
