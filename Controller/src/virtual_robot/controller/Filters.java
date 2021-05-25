package virtual_robot.controller;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.collision.Filter;

public class Filters {

    //Masks

    public static long MASK_ALL = -1;  // Binary representation of -1 is all 1s

    //Categories

    public static long WALL = 0b1;
    public static long CHASSIS = 0b10;
    public static long ARM = 0b100;

    // Filter for the field walls: collides with anything that includes WALL in its mask
    public static final CategoryFilter WALL_FILTER = new CategoryFilter(WALL, MASK_ALL);

    // Filter for the robot chassis: collides with anything that includes CHASSIS in its mask
    public static final CategoryFilter CHASSIS_FILTER = new CategoryFilter(CHASSIS, MASK_ALL);
}
