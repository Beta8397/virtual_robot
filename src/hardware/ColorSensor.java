package hardware;

/**
 * An abridged version of the FTC ColorSensor interface.
 */
public interface ColorSensor {

    /**
     * @return Red channel value, 0..255
     */
    public int red();

    /**
     * @return Green channel value, 0..255
     */
    public int green();

    /**
     * @return Blue channel value, 0..255
     */
    public int blue();

}
