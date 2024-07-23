package org.murraybridgebunyips.bunyipslib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.google.gson.Gson;
//import com.google.gson.GsonBuilder;
//import com.google.gson.JsonDeserializationContext;
//import com.google.gson.JsonDeserializer;
//import com.google.gson.JsonElement;
//import com.google.gson.JsonParseException;
//import com.google.gson.JsonPrimitive;
//import com.google.gson.JsonSerializationContext;
//import com.google.gson.JsonSerializer;
//import com.google.gson.JsonSyntaxException;
//import com.google.gson.reflect.TypeToken;

//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

//import java.io.Closeable;
//import java.io.File;
//import java.io.FileReader;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Global filesystem and volatile storage utilities for robot operation.
 *
 * @author Lucas Bubner, 2024
 */
public final class Storage {
    private static Memory memory = null;
//    private static Filesystem filesystem = null;

    private Storage() {
    }

    /**
     * Global method to clear all static state for an OpMode.
     * Called internally by {@link BunyipsOpMode}.
     */
    public static void resetAllStaticFieldsForOpMode() {
        // All static field resets go here
        RoadRunner.resetForOpMode();
        BunyipsSubsystem.resetForOpMode();
        Tasks.resetForOpMode();
    }

    /**
     * Get the global volatile (cleared after restart) memory storage for the robot.
     *
     * @return Instance for volatile memory storage
     */
    public static Memory memory() {
        if (memory == null)
            memory = new Memory();
        return memory;
    }

//    /**
//     * Get the global persistent (saved after restart) filesystem storage for the robot.
//     * Storage information is saved in the robot controller internal storage.
//     *
//     * @return Instance for persistent storage
//     */
//    public static Filesystem filesystem() {
//        if (filesystem == null)
//            filesystem = new Filesystem();
//        return filesystem;
//    }

    /**
     * Represents in-memory storage for the robot.
     */
    public static class Memory {
        /**
         * Static array of hardware errors stored via hardware name.
         *
         * @see RobotConfig
         */
        public final ArrayList<String> hardwareErrors = new ArrayList<>();
        /**
         * Components that are unusable and should not have their errors logged.
         *
         * @see NullSafety
         */
        public final List<String> unusableComponents = new ArrayList<>();
        private final HashMap<String, Object> store = new HashMap<>();
        /**
         * The last known player Alliance.
         *
         * @see StartingPositions
         */
        public StartingPositions lastKnownAlliance = null;
        /**
         * The last known position of the robot from odometry.
         *
         * @see RoadRunnerDrive
         */
        public Pose2d lastKnownPosition = null;

        private Memory() {
        }

        /**
         * Clear all volatile memory related to the robot.
         */
        public void clear() {
            store.clear();
            hardwareErrors.clear();
            unusableComponents.clear();
            lastKnownAlliance = null;
            lastKnownPosition = null;
        }

        /**
         * Clear the volatile HashMap.
         */
        public void clearVolatile() {
            store.clear();
        }

        /**
         * Get a volatile value from memory stored by key in {@link #setVolatile(String, Object)}.
         *
         * @param key the key to search for
         * @return the value associated with the key
         * @throws IllegalArgumentException if key not found
         */
        public Object getVolatile(String key) throws IllegalArgumentException {
            if (!store.containsKey(key))
                throw new IllegalArgumentException("Key not found in memory: " + key);
            return store.get(key);
        }

        /**
         * Set a volatile value in memory stored by key.
         *
         * @param key   the key to store the value under
         * @param value the value to store
         */
        public void setVolatile(String key, Object value) {
            store.put(key, value);
        }
    }

//    /**
//     * Represents persistent, file-saved storage for the robot.
//     */
//    public static class Filesystem implements Closeable {
//        private final HashMap<String, Object> store = new HashMap<>();
//        private final File file = new File(AppUtil.ROBOT_DATA_DIR, "bunyipslib_storage.json");
//        private final Gson gson = new GsonBuilder()
//                .registerTypeAdapter(Class.class, new ClassTypeAdapter())
//                .setPrettyPrinting()
//                .create();
//        private final Type storeType = new TypeToken<HashMap<String, Object>>() {
//        }.getType();
//
//        private Filesystem() {
//            File dir = file.getParentFile();
//            AppUtil.getInstance().ensureDirectoryExists(dir);
//
//            if (!file.exists())
//                return;
//
//            try (FileReader reader = new FileReader(file)) {
//                store.putAll(gson.fromJson(reader, storeType));
//            } catch (IOException | JsonSyntaxException e) {
//                Dbg.error("Failed to load storage file: " + e.getMessage());
//            }
//        }
//
//        /**
//         * Access the HashMap of all stored values in the filesystem.
//         * When this resource is closed, the values are saved to the file, so ensure to use
//         * a try-with-resources block or call {@link #close()} when done.
//         * <b>Warning</b>: Trying to add objects to the HashMap that are not serializable by Gson will throw an exception on write,
//         * and may cause in the corruption of the storage file (other valid objects may be lost)
//         *
//         * @return the stored values
//         */
//        public HashMap<String, Object> access() {
//            return store;
//        }
//
//        /**
//         * Delete all persistent storage related to the robot.
//         * This will close the stream and save the empty store to the file.
//         *
//         * @throws RuntimeException if the storage file cannot be deleted
//         */
//        public void delete() {
//            store.clear();
//            close();
//            filesystem = null;
//        }
//
//        @Override
//        public void close() {
//            try (FileWriter writer = new FileWriter(file)) {
//                gson.toJson(store, writer);
//            } catch (IOException e) {
//                Dbg.error("Failed to save storage file: " + e.getMessage());
//            }
//        }
//
//        private static class ClassTypeAdapter implements JsonSerializer<Class<?>>, JsonDeserializer<Class<?>> {
//            @Override
//            public JsonElement serialize(Class<?> src, Type typeOfSrc, JsonSerializationContext context) {
//                return new JsonPrimitive(src.getName());
//            }
//
//            @Override
//            public Class<?> deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context)
//                    throws JsonParseException {
//                try {
//                    return Class.forName(json.getAsString());
//                } catch (ClassNotFoundException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//        }
//    }
}
