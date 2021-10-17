package virtual_robot.game_elements.classes;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.joint.WeldJoint;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.List;

/**
 * Freight is the abstract base class for BoxFreight, CargoFreight, and DuckFreight
 */
public abstract class Freight extends VirtualGameElement {

    public static final List<Freight> FREIGHT_ITEMS = new ArrayList<>();

    /*
     * Collision Filters for freight include:
     *   NORMAL_FILTER (collides with everything that is capable of colliding with FREIGHT_CATEGORY;
     *   OWNED_FILTER (for freight owned by robot -- does not collide with HUB_CATEGORY)
     *   ON_CAROUSEL_FILTER (for duck on carousel -- does not collide with anything)
     */
    public static long FREIGHT_CATEGORY = 2048;
    public static CategoryFilter NORMAL_FILTER = new CategoryFilter(FREIGHT_CATEGORY, Filters.MASK_ALL);
    public static CategoryFilter OWNED_FILTER = new CategoryFilter(FREIGHT_CATEGORY,
            Filters.MASK_ALL & ~ShippingHub.HUB_CATEGORY);
    public static CategoryFilter ON_CAROUSEL_FILTER = new CategoryFilter(FREIGHT_CATEGORY, ~Filters.MASK_ALL);

    // The shipping hub that owns this freight, if any (and the corresponding weld joint)
    private ShippingHub owningShippingHub = null;
    private WeldJoint shippingHubJoint = null;

    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body.
         */
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0,
                new FixtureData(NORMAL_FILTER, 1, 0, 0));
        elementBody.setLinearDamping(100);
        elementBody.setAngularDamping(100);
    }

    /*
     * State handling for freight is very basic; just get new pose from the physics engine.
     */
    @Override
    public void updateState(double millis) {
        x = elementBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = elementBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = elementBody.getTransform().getRotationAngle();
    }

    public ShippingHub getOwningShippingHub() {
        return owningShippingHub;
    }

    /**
     * Attach/Detach freight object to/from a shipping hub.
     * This method also sets the category filter appropriately.
     * @param hub attach to this shipping hub (or if null, detach from the owning hub, if any)
     */
    public void setOwningShippingHub(ShippingHub hub) {
        if (hub == null){
            if (owningShippingHub != null){
                world.removeJoint(shippingHubJoint);
                owningShippingHub = null;
                shippingHubJoint = null;
                setCategoryFilter(NORMAL_FILTER);
            }
        } else {
            if (owningShippingHub == hub) return;
            if (owningShippingHub != null){
                world.removeJoint(shippingHubJoint);
            }
            shippingHubJoint = new WeldJoint(elementBody, hub.getElementBody(),
                    hub.getElementBody().getTransform().getTranslation());
            world.addJoint(shippingHubJoint);
            owningShippingHub = hub;
            setCategoryFilter(OWNED_FILTER);
        }
    }

    /**
     * Set the category filter for this Freight object. This method will set all BodyFixtures to the
     * same category filter.
     * @param f
     */
    public void setCategoryFilter(CategoryFilter f){
        for (BodyFixture fixture: elementBody.getFixtures()) {
            fixture.setFilter(f);
        }
    }

}
