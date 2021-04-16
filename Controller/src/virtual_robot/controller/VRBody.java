package virtual_robot.controller;

import org.dyn4j.dynamics.Body;
import virtual_robot.config.GameObject;

/**
 *  A dyn4j Body that can have a GameObject parent.
 *
 */
public class VRBody extends Body {

    private GameObject parent = null;

    public VRBody() {
        super();
    }

    public VRBody(GameObject gameObject){
        super();
        parent = gameObject;
    }

    public void setParent(GameObject gameObject) { parent = gameObject; }

    public GameObject getParent() { return parent; }

    public boolean hasParent() { return parent != null; }

    public boolean parentIsBot() { return parent != null && parent instanceof VirtualBot; }

    public boolean parentIsGameElement() { return parent != null && parent instanceof VirtualGameElement; }

    public boolean parentIsWall() { return parent != null && parent instanceof Wall; }

}
