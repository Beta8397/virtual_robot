package virtual_robot.game_elements.classes;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.joint.WeldJoint;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

import java.util.ArrayList;
import java.util.List;

public abstract class Freight extends VirtualGameElement {

    public static final List<Freight> freightItems = new ArrayList<>();

    public static long FREIGHT_CATEGORY = 2048;
    public static CategoryFilter NORMAL_FILTER = new CategoryFilter(FREIGHT_CATEGORY, Filters.MASK_ALL);
    public static CategoryFilter OWNED_FILTER = new CategoryFilter(FREIGHT_CATEGORY,
            Filters.MASK_ALL & ~ShippingHub.HUB_CATEGORY);
    public static CategoryFilter ON_CAROUSEL_FILTER = new CategoryFilter(FREIGHT_CATEGORY, ~Filters.MASK_ALL);

    private ShippingHub owningShippingHub = null;
    private WeldJoint shippingHubJoint = null;

    @Override
    public void setUpBody(){
        /*
         * Use Dyn4jUtil.createBody to create a Body. Infinite mass (i.e., the barrier is fixed in position)
         */
        elementBody = Dyn4jUtil.createBody(displayGroup, this, 0, 0,
                new FixtureData(NORMAL_FILTER, 1, 0, 0));
        elementBody.setLinearDamping(100);
        elementBody.setAngularDamping(100);
    }

    @Override
    public void updateState(double millis) {
        x = elementBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = elementBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = elementBody.getTransform().getRotationAngle();
    }

    public ShippingHub getOwningShippingHub() {
        return owningShippingHub;
    }

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

    public void setCategoryFilter(CategoryFilter f){
        this.elementBody.getFixture(0).setFilter(f);
    }

}
