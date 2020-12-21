package system.gui.menus.examplemenu;

import system.gui.HALMenu;
import system.gui.Payload;
import system.gui.event.DataPacket;
import system.gui.viewelement.TextElement;
import system.gui.viewelement.eventlistener.EntireViewButton;
import util.control.Button;

public class ExampleMenu3 extends HALMenu {
    @Override
    protected void init(Payload payload) {
        addItem(new TextElement("Hey, you're not supposed to be here!"));
        addItem(new EntireViewButton()
                .onClick(new Button<>(1, Button.BooleanInputs.b), (DataPacket packet) -> {
                    gui.back();
                }));
    }
}
