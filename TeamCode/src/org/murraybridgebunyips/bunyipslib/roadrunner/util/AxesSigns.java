package org.murraybridgebunyips.bunyipslib.roadrunner.util;

/**
 * IMU axes signs in the order XYZ (after remapping).
 */
public enum AxesSigns {
    /**
     * Positive X, Positive Y, Positive Z.
     */
    PPP(0b000),
    /**
     * Positive X, Positive Y, Negative Z.
     */
    PPN(0b001),
    /**
     * Positive X, Negative Y, Positive Z.
     */
    PNP(0b010),
    /**
     * Positive X, Negative Y, Negative Z.
     */
    PNN(0b011),
    /**
     * Negative X, Positive Y, Positive Z.
     */
    NPP(0b100),
    /**
     * Negative X, Positive Y, Negative Z.
     */
    NPN(0b101),
    /**
     * Negative X, Negative Y, Positive Z.
     */
    NNP(0b110),
    /**
     * Negative X, Negative Y, Negative Z.
     */
    NNN(0b111);

    /**
     * Binary value of the axes signs.
     */
    public final int bVal;

    AxesSigns(int bVal) {
        this.bVal = bVal;
    }

    /**
     * Converts the axes signs to a binary value.
     *
     * @param bVal binary value
     * @return axes signs
     */
    public static AxesSigns fromBinaryValue(int bVal) {
        int maskedVal = bVal & 0x07;
        switch (maskedVal) {
            case 0b000:
                return PPP;
            case 0b001:
                return PPN;
            case 0b010:
                return PNP;
            case 0b011:
                return PNN;
            case 0b100:
                return NPP;
            case 0b101:
                return NPN;
            case 0b110:
                return NNP;
            case 0b111:
                return NNN;
            default:
                throw new IllegalStateException("Unexpected value for maskedVal: " + maskedVal);
        }
    }
}
