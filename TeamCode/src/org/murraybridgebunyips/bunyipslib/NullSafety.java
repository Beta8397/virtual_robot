package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.getCallingUserCodeFunction;

import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

import java.util.ArrayList;
import java.util.List;

/**
 * Null safety utilities for robot components.
 *
 * @author Lucas Bubner, 2023
 * @see Exceptions
 */
public final class NullSafety {
    /**
     * Components that are unusable and should not have their errors logged.
     */
    public static final List<String> unusableComponents = new ArrayList<>();

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
     * <p>
     * This function may only be called within a BunyipsOpMode, and only during OpMode runtime.
     *
     * @param T    Class of the component (e.g. Cannon.class)
     * @param objs Objects to check for null
     * @return Whether the component is safe to instantiate
     */
    public static boolean assertComponentArgs(Class<?> T, Object... objs) {
        for (Object o : objs) {
            if (o == null) {
                return reportUnusable(T);
            } else if (o instanceof Encoder) {
                if (((Encoder) o).isNull()) {
                    return reportUnusable(T);
                }
            }
        }
        return true;
    }

    private static boolean reportUnusable(Class<?> component) {
        BunyipsOpMode opMode = BunyipsOpMode.getInstance();
        opMode.addRetainedTelemetry("! COM_FAULT: %", component.getSimpleName());
        opMode.log("error: % was disabled due to a null assertion fault.", component.getSimpleName());
        Dbg.warn(getCallingUserCodeFunction(), "Null object passed to % failed assertion, adding to unusable components...", component.getSimpleName());
        if (!unusableComponents.contains(component.getSimpleName()))
            unusableComponents.add(component.getSimpleName());
        return false;
    }
}
