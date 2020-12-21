package sim_config;

/**
 * A config class used for setting configuration settings for the adapted version of HAL used in the simulation.
 *
 * Creation Date: 12/20/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.0.0
 */
public class SimConfig {

    /**
     * Private constructor used to make class static.
     */
    private SimConfig() {}

    /**
     * The root directory of HAL's filesystem. Used to store config files.
     * If you don't want this to be stored in your root computer directory
     * (or if you are running this on a mac) you should change this.
     */
    public static String HAL_FILESYSTEM_ROOT = "C://";
}
