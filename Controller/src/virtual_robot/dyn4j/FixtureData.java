package virtual_robot.dyn4j;

import org.dyn4j.collision.Filter;

public class FixtureData {
    public final Filter filter;
    public final double density;
    public final double restitution;
    public final double friction;
    public final boolean isSensor;
    public final double xMag;
    public final double yMag;

    public FixtureData(Filter filter, double density, double restitution, double friction, boolean isSensor){
        this.filter = filter;
        this.density = density;
        this.restitution = restitution;
        this.friction = friction;
        this.isSensor = isSensor;
        xMag = 1.0;
        yMag = 1.0;
    }

    public FixtureData(){
        this(null, 1.0, 0.0, 0.0, false);
    }

    public FixtureData(double density, double restitution, double friction){
        this(null, density, restitution, friction, false);
    }

    public FixtureData(Filter filter, double density, double restitution, double friction){
        this(filter, density, restitution, friction, false);
    }

    public FixtureData(Filter filter, boolean isSensor){
        this(filter, 1.0, 0.0, 0.0, isSensor);
    }

    public FixtureData(Filter filter, double density, double restitution, double friction, boolean isSensor,
                       double xMag, double yMag){
        this.filter = filter;
        this.density = density;
        this.restitution = restitution;
        this.friction = friction;
        this.isSensor = isSensor;
        this.xMag = xMag;
        this.yMag = yMag;
    }

    public FixtureData(double density, double restitution, double friction, double xMag, double yMag){
        this(null, density, restitution, friction, false, xMag, yMag);
    }

    public FixtureData(Filter filter, double density, double restitution, double friction, double xMag, double yMag){
        this(filter, density, restitution, friction, false, xMag, yMag);
    }

    public FixtureData(Filter filter, boolean isSensor, double xMag, double yMag){
        this(filter, 1.0, 0.0, 0.0, isSensor, xMag, yMag);
    }

}
