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
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.controller.Wall;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 *  A dyn4j Body that can have a parent object.
 *
 */
public class VRBody extends Body {

    private Object parent = null;

    public VRBody() {
        super();
    }

    public VRBody(Object gameObject){
        super();
        parent = gameObject;
    }

    public void setParent(Object gameObject) { parent = gameObject; }

    public Object getParent() { return parent; }

    public boolean hasParent() { return parent != null; }

    public boolean parentIsBot() { return parent != null && parent instanceof VirtualBot; }

    public boolean parentIsGameElement() { return parent != null && parent instanceof VirtualGameElement; }

    public boolean parentIsWall() { return parent != null && parent instanceof Wall; }

    public static Vector2 getCenterMeters(Shape shape){
        double pixelsPerMeter = VirtualField.getInstance().PIXELS_PER_METER;
        double halfBotWidthMeters = 9.0 / VirtualField.INCHES_PER_METER;
        double xCenterMeters = 0;
        double yCenterMeters = 0;
        if (shape instanceof Rectangle){
            Rectangle r = (Rectangle)shape;
            double widthMeters = r.getWidth() / pixelsPerMeter;
            double heightMeters = r.getHeight() / pixelsPerMeter;
            xCenterMeters = (r.getX() + r.getTranslateX()) / pixelsPerMeter + widthMeters / 2.0 - halfBotWidthMeters;
            yCenterMeters = halfBotWidthMeters - (r.getY() + r.getTranslateY()) / pixelsPerMeter - heightMeters / 2.0;
        } else if (shape instanceof Circle) {
            Circle c = (Circle)shape;
            xCenterMeters = (c.getCenterX() + c.getTranslateX()) / pixelsPerMeter - halfBotWidthMeters;
            yCenterMeters = halfBotWidthMeters - (c.getCenterY() + c.getTranslateY()) / pixelsPerMeter;
        } else if (shape instanceof Ellipse) {
            Ellipse e = (Ellipse)shape;
            xCenterMeters = (e.getCenterX() + e.getTranslateX()) / pixelsPerMeter - halfBotWidthMeters;
            yCenterMeters = halfBotWidthMeters - (e.getCenterY() + e.getTranslateY()) / pixelsPerMeter;
        } else {
            throw new IllegalArgumentException("Argument must be Rectangle, Circle, or Ellipse.");
        }
        return new Vector2(xCenterMeters, yCenterMeters);
    }

    public static BodyFixture createFixture(Shape shape, boolean applyTransforms, Filter filter, double density,
                                            double restitution, double friction, boolean isSensor){
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
            Vector2 translate = getCenterMeters(shape);
            convex.translate(translate);
        }

        BodyFixture bodyFixture = new BodyFixture(convex);
        if (filter != null){
            bodyFixture.setFilter(filter);
        }
        bodyFixture.setRestitution(restitution);
        bodyFixture.setFriction(friction);
        bodyFixture.setDensity(density);
        bodyFixture.setSensor(isSensor);

        return bodyFixture;
    }

    public static BodyFixture createFixture(Shape shape, boolean applyTransforms){
        return createFixture(shape, applyTransforms, null, 1.0, 0.2, 0.0, false);
    }

    public static BodyFixture createFixture(Shape shape, boolean applyTransforms, Filter filter,
                                            boolean isSensor){
        return createFixture(shape, applyTransforms, filter,
                isSensor? 0.0: 1.0,
                isSensor? 0.0 : 0.2,
                0, isSensor);
    }


    public static List<BodyFixture> createFixtures(Group group){
        List<BodyFixture> list = new ArrayList<>();

        for (Node n: group.getChildren()){
            if (n instanceof Shape){
                list.add(createFixture((Shape)n, true));
            }else {
                throw new IllegalArgumentException("Group children must be Shapes.");
            }
        }
        return list;
    }




}
