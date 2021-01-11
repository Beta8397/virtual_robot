package system.robot.roadrunner_util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import util.math.geometry.Vector2D;

import java.util.function.Function;

import static java.lang.Math.PI;

public enum CoordinateMode {
    ROADRUNNER((Pose2d pose) -> {
        Vector2D positionVector = new Vector2D(pose.getX(), pose.getY()).rotate(PI/2);
        return new Pose2d(positionVector.getX(), positionVector.getY(), pose.getHeading());
    }),
    HAL((Pose2d pose) -> {
        Vector2D positionVector = new Vector2D(pose.getX(), pose.getY()).rotate(-PI/2);
        return new Pose2d(positionVector.getX(), positionVector.getY(), pose.getHeading());
    });

    private final Function<Pose2d, Pose2d> converter;
    CoordinateMode(Function<Pose2d, Pose2d> converter) {
        this.converter = converter;
    }

    public Function<Pose2d, Pose2d> convertTo(CoordinateMode coordinateMode) {
        switch (this) {
            default:
            case HAL:
                if(coordinateMode == HAL) return (Pose2d pose) -> pose;
                else return converter;
            case ROADRUNNER:
                if(coordinateMode == HAL) return converter;
                else return (Pose2d pose) -> pose;
        }
    }
}
