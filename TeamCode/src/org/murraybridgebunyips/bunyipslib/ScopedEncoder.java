package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Marker interface for encoders that are scoped to relative or absolute positions.
 *
 * @author Lucas Bubner, 2024
 */
public interface ScopedEncoder {
    /**
     * Enable encoder and start tracking in the selected mode, which will also save a snapshot of the encoder position for relative tracking.
     *
     * @param mode the mode to track the encoder in
     */
    void track(@NonNull DcMotor.RunMode mode);

    /**
     * Reset encoder positions to zero. Useful when saved state is not needed or can be discarded.
     */
    void reset();

    /**
     * Get a movement reading in ticks from the encoder as per the scope.
     *
     * @param scope the scope to read the encoder in
     * @return encoder value relative to last track() call, or since the last reset() call
     */
    double position(@NonNull Scope scope);

    /**
     * Get a movement reading in ticks from the encoder since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     *
     * @return encoder value relative to last track() call, or since the last reset() call
     */
    default double position() {
        return position(Scope.RELATIVE);
    }

    /**
     * Get the number of revolutions the encoder has travelled as per the scope.
     *
     * @param scope the scope to read the encoder in
     * @return revolutions indicating how far the encoder has travelled
     */
    double travelledRevolutions(@NonNull Scope scope);

    /**
     * Get the number of revolutions the encoder has travelled since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     *
     * @return revolutions indicating how far the encoder has travelled
     */
    default double travelledRevolutions() {
        return travelledRevolutions(Scope.RELATIVE);
    }

    /**
     * Get the number of millimeters the encoder has travelled as per the scope.
     *
     * @param scope the scope to read the encoder in
     * @return millimeters indicating how far the encoder has travelled
     */
    double travelledMM(@NonNull Scope scope);

    /**
     * Get the number of millimeters the encoder has travelled since the last track()
     * Can use an optional parameter to use since reset() position instead of track()
     *
     * @return millimeters indicating how far the encoder has travelled
     */
    default double travelledMM() {
        return travelledMM(Scope.RELATIVE);
    }

    /**
     * Scope of the encoder tracking.
     */
    enum Scope {
        /**
         * Relative scope will return the encoder value since the last track() call.
         */
        RELATIVE,

        /**
         * Global scope will return the encoder value since the last reset() call.
         */
        GLOBAL
    }
}
