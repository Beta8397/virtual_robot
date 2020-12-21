package system.gui.menus.examplemenu;

import system.gui.HALMenu;
import system.gui.Payload;
import system.gui.SelectionZone;
import system.gui.event.DataPacket;
import system.gui.viewelement.TextElement;
import system.gui.viewelement.eventlistener.ViewButton;
import util.control.Button;

public class ExampleMenu2 extends HALMenu {
    @Override
    protected void init(Payload payload) {
        selectionZone = new SelectionZone(2,2);
        addItem(new ViewButton("## | Sorry Mario, the princess is in another castle.")
                    .onClick(new Button<>(1, Button.BooleanInputs.y), (DataPacket packet) -> gui.back())
                    .onClick(new Button<>(1, Button.BooleanInputs.a), (DataPacket packet) -> gui.inflate(new ExampleMenu3())));
        addItem(new TextElement("#"));
    }
}
