package util.math;

import util.math.units.HALDistanceUnit;

/**
 * A calculator that calculates encoder ticks per meter and uses that to find.
 * <p>
 * Creation Date: 7/19/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.0.0
 * @see HALDistanceUnit
 * @since 1.0.0
 */
public class EncoderToDistanceProcessor {

    //Conversion of 1 encoder per meter.
    private double encoderPerMeter;

    /**
     * Constructor that sets theoretical encoderPerMeter using default encoderPerRotation(less accurate).
     *
     * @param diameter Diameter of wheel.
     * @param unit     Units of the diameter.
     */
    public EncoderToDistanceProcessor(double diameter, HALDistanceUnit unit) {
        encoderPerMeter = 1440 / (Math.PI * (diameter * unit.conversionFactor));
    }

    /**
     * Constructor that sets theoretical encoderPerMeter using given encoderPerRotation(less accurate).
     *
     * @param diameter           Diameter of wheel.
     * @param encoderPerRotation number of encoders per one rotation of the wheel.
     * @param unit               Units of the diameter.
     */
    public EncoderToDistanceProcessor(double diameter, int encoderPerRotation, HALDistanceUnit unit) {
        encoderPerMeter = encoderPerRotation / (Math.PI * (diameter * unit.conversionFactor));
    }

    /**
     * Constructor that uses encoderPerMeter given by calibration program.
     *
     * @param encoderPerMeter Experimentally gotten value.
     * */
    public EncoderToDistanceProcessor(double encoderPerMeter){
        this.encoderPerMeter = encoderPerMeter;
    }

    /**
     * Returns encoder amount to go given distance.
     *
     * @param distance Distance wished to travel.
     * @param unit     Units of distance.
     */
    public int getEncoderAmount(double distance, HALDistanceUnit unit) {
        return (int) Math.round(unit.conversionFactor * distance * encoderPerMeter);
    }

    /**
     * returns distance from encoder amount.
     *
     * @param encoderAmount Amount of encoder.
     * @param unit          Units of returned distance.
     */
    public double getDistanceFromEncoders(double encoderAmount, HALDistanceUnit unit) {
        return (encoderAmount / encoderPerMeter) * unit.conversionFactor;
    }
}
