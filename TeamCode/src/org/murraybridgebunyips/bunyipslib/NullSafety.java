package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.getCallingUserCodeFunction;

import java.util.List;

/**
 * Null safety utilities for robot components.
 *
 * @author Lucas Bubner, 2023
 * @see Exceptions
 */
public final class NullSafety {

    private NullSafety() {
    }

    /**
     * Ensure that all objects are not null.
     *
     * @param objs Objects to check for null
     * @return Whether all objects are not null
     */
    public static boolean assertNotNull(Object... objs) {
        for (Object o : objs) {
            if (o == null) {
                Dbg.warn(getCallingUserCodeFunction(), "Assertion by NullSafety.assertNotNull() failed.");
                return false;
            }
        }
        return true;
    }

    /**
     * Ensure that all objects are not null.
     *
     * @param objs Objects to check for null
     * @return Whether all objects are not null
     */
    public static boolean assertNotNull(List<Object> objs) {
        return assertNotNull(objs.toArray());
    }

    /**
     * Ensure a component is safe to instantiate by checking for null objects.
     * Errors caused by null objects are logged and the component is added to the unusable components list.
     * Components in the unusable components list will not have their errors logged.
     * Telemetry will be added to a BunyipsOpMode if it is running.
     *
     * @param T    Class of the component (e.g. Cannon.class)
     * @param objs Objects to check for null
     * @return Whether the component is safe to instantiate
     */
    public static boolean assertComponentArgs(Class<?> T, Object... objs) {
        for (Object o : objs) {
            if (o == null) {
                if (BunyipsOpMode.isRunning()) {
                    BunyipsOpMode opMode = BunyipsOpMode.getInstance();
                    opMode.addRetainedTelemetry("<font color='red'><b>! COM_FAULT</b></font>: %", T.getSimpleName());
                    opMode.log("<font color='yellow'><b>warning!</b> % was disabled due to a null assertion fault.</font>", T.getSimpleName());
                }
                Dbg.warn(getCallingUserCodeFunction(), "Null object passed to % failed assertion, adding to unusable components...", T.getSimpleName());
                if (!Storage.unusableComponents.contains(T.getSimpleName()))
                    Storage.unusableComponents.add(T.getSimpleName());
            }
        }
        return true;
    }
}
