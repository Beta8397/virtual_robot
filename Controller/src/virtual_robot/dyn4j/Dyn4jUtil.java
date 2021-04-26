package virtual_robot.dyn4j;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Ellipse;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Shape;
import org.dyn4j.collision.Filter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.VirtualField;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 *  A dyn4j Body that can have a parent object.
 *
 */
public class Dyn4jUtil {

    /**
     * Return the position (meters, Y-up) of the center of the specified Shape object, relative to the
     * (xOffset, yOffset) position (inches, Y-down).
     * @param shape             The Rectangle, Circle, or Ellipse in question
     * @param xOffsetInches     Reference X-value (inches)
     * @param yOffsetInches     Reference Y-value (inches - in a Y-down system)
     * @return                  Offset (meters, Y-up, of center of shape relative to reference point)
     *
     * The most common use would be to determine the position of a shape in the robot fxml file relative to
     * the center of the robot, in meters, world coordinates (i.e., Y-up). In that case, use xOffset = 9
     * and yOffset = 9 (for 18 inch robot).
     */
    public static Vector2 getCenterMeters(Shape shape, double xOffsetInches, double yOffsetInches){
        double pixelsPerMeter = VirtualField.getInstance().PIXELS_PER_METER;
        double xOffsetMeters = xOffsetInches / VirtualField.INCHES_PER_METER;
        double yOffsetMeters = yOffsetInches / VirtualField.INCHES_PER_METER;
        double xCenterMeters = 0;
        double yCenterMeters = 0;
        if (shape instanceof Rectangle){
            Rectangle r = (Rectangle)shape;
            double widthMeters = r.getWidth() / pixelsPerMeter;
            double heightMeters = r.getHeight() / pixelsPerMeter;
            xCenterMeters = (r.getX() + r.getTranslateX()) / pixelsPerMeter + widthMeters / 2.0 - xOffsetMeters;
            yCenterMeters = yOffsetMeters - (r.getY() + r.getTranslateY()) / pixelsPerMeter - heightMeters / 2.0;
        } else if (shape instanceof Circle) {
            Circle c = (Circle)shape;
            xCenterMeters = (c.getCenterX() + c.getTranslateX()) / pixelsPerMeter - xOffsetMeters;
            yCenterMeters = yOffsetMeters - (c.getCenterY() + c.getTranslateY()) / pixelsPerMeter;
        } else if (shape instanceof Ellipse) {
            Ellipse e = (Ellipse)shape;
            xCenterMeters = (e.getCenterX() + e.getTranslateX()) / pixelsPerMeter - xOffsetMeters;
            yCenterMeters = yOffsetMeters - (e.getCenterY() + e.getTranslateY()) / pixelsPerMeter;
        } else {
            throw new IllegalArgumentException("Argument must be Rectangle, Circle, or Ellipse.");
        }
        return new Vector2(xCenterMeters, yCenterMeters);
    }

    public Vector2 getCenterMeters(Shape shape){
        return getCenterMeters(shape, 9, 9);
    }


    public static BodyFixture createFixture(Shape shape, double xOffsetInches, double yOffsetInches,
                                            boolean applyTransforms, FixtureData fixtureData){
        Convex convex = null;
        double pixelsPerMeter = VirtualField.getInstance().PIXELS_PER_METER;

        if (shape instanceof Rectangle) {
            Rectangle r = (Rectangle) shape;
            double widthMeters = r.getWidth() / pixelsPerMeter;
            double heightMeters = r.getHeight() / pixelsPerMeter;
            convex = new org.dyn4j.geometry.Rectangle(widthMeters, heightMeters);
        } else if (shape instanceof Circle) {
            Circle c = (Circle) shape;
            double radMeters = c.getRadius() / pixelsPerMeter;
            convex = new org.dyn4j.geometry.Circle(radMeters);
        } else if (shape instanceof Ellipse) {
            Ellipse e = (Ellipse) shape;
            double widthMeters = 2.0 * e.getRadiusX() / pixelsPerMeter;
            double heightMeters = 2.0 * e.getRadiusY() / pixelsPerMeter;
            convex = new org.dyn4j.geometry.Ellipse(widthMeters, heightMeters);
        } else {
            throw new IllegalArgumentException("Shape must be Rectangle, Circle, or Ellipse.");
        }

        if (applyTransforms){
            double radians = -Math.toRadians(shape.getRotate());
            convex.rotate(radians);
            Vector2 translate = getCenterMeters(shape, xOffsetInches, yOffsetInches);
            convex.translate(translate);
        }

        BodyFixture bodyFixture = new BodyFixture(convex);
        if (fixtureData.filter != null){
            bodyFixture.setFilter(fixtureData.filter);
        }
        bodyFixture.setRestitution(fixtureData.restitution);
        bodyFixture.setFriction(fixtureData.friction);
        bodyFixture.setDensity(fixtureData.density);
        bodyFixture.setSensor(fixtureData.isSensor);

        return bodyFixture;
    }

    public static List<BodyFixture> createFixtures(Group group, double xOffsetInches, double yOffsetInches,
                                                   Map<String,FixtureData> map){
        List<BodyFixture> list = new ArrayList<>();
        FixtureData defaultData = new FixtureData();

        for (Node n: group.getChildren()){
            if (n instanceof Shape){
                FixtureData data = defaultData;
                if (map != null && n.getId() != null && map.get(n.getId()) != null){
                    data = map.get(n.getId());
                }
                BodyFixture fixture = createFixture((Shape)n, xOffsetInches, yOffsetInches, true, data);
                fixture.setUserData(n.getId());
                list.add(fixture);
            }else {
                throw new IllegalArgumentException("Group children must be Shapes.");
            }
        }
        return list;
    }




}
