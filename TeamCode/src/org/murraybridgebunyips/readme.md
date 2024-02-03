## TeamCode

This is a reflected copy of the core components of https://github.com/Murray-Bridge-Bunyips/BunyipsFTC.<br><br>
BunyipsLib is under `bunyipslib/`, default testing virtual_robot OpModes are under `virtual_robot/samples/`, and there is
a bot under `imposter/` to test a virtual robot configuration, as other robots won't work with this simulator due to
configuration differences. Features from the SDK that aren't supported have been replicated to return null or similar,
and may cause null exceptions if used. This includes all the Vision systems (VisionPortal will not build any cameras).

### Porting BunyipsLib
When copying BunyipsLib to org.murraybridgebunyips, you will need to do the following:
1. Copy the `bunyipslib` folder to `TeamCode/src/org/murraybridgebunyips/`
2. Import `deps/BuildConfig` in `bunyipslib/BunyipsOpMode.kt`
3. Update `fun log(Class<*>, String)` in `bunyipslib/BunyipsOpMode.kt` changing `lowercase()` to `toLowerCase()`
4. Delete `bunyipslib/roadrunner/util/android/`, ignore warnings
5. Remove deleted import from `bunyipslib/roadrunner/drive/tuning/AutomaticFeedforwardTuner.java` and import `deps/LoggingUtil`
6. Remove deleted import from `bunyipslib/roadrunner/trajectorysequence/TrajectorySequenceRunner.java` and import `deps/LogFiles`

BunyipsLib will now compile, although missing some features.
