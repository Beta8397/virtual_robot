package org.murraybridgebunyips.bunyipslib.roadrunner.util;

/**
 * IMU axes signs in the order XYZ (after remapping).
 */
public enum AxesSigns {
    PPP(0b000),
    PPN(0b001),
    PNP(0b010),
    PNN(0b011),
    NPP(0b100),
    NPN(0b101),
    NNP(0b110),
    NNN(0b111);

    public final int bVal;

    AxesSigns(int bVal) {
        this.bVal = bVal;
    }

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
